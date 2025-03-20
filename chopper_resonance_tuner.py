# TMC drivers registers analysing tool
#
# Copyright (C) 2025  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import logging
import math
import numpy as np
import multiprocessing
import plotly.graph_objects as go
import json
import time
from datetime import datetime
from . import resonance_tester
from . import shaper_calibrate

RESULTS_FOLDER = os.path.expanduser('~/printer_data/config/adxl_results/chopper_resonance_tuner/')
SAMPLES_FOLDER = RESULTS_FOLDER + 'tmp_samples/'
TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660",
    "tmc5160"]
FCLK = 12 # MHz

def check_export_path(path):
    if not os.path.exists(path):
        try:
            os.makedirs(path)
        except Exception as e:
            return f'Error generate path {path}: {e}'

def remove_export_path(path):
    if not os.path.exists(path):
        return
    try:
        for filename in os.listdir(path):
            filepath = os.path.join(path, filename)
            os.remove(filepath)
        os.rmdir(path)
    except Exception as e:
        return f'Error remove path {path}: {e}'


class AccelHelper:
    def __init__(self, printer, chip_name):
        self.chip_name = chip_name
        self.chip_type = 'accelerometer'
        self.dim_type = 'magnitude'
        self.printer = printer
        self.init_chip_config(chip_name)
        self.is_finished = False
        self.samples = []
        self.request_start_time = None
        self.request_timings = []
        self.max_freq = 0
        self.samples_trim_size = 0
        self.static_noise = []
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.reactor = self.printer.get_reactor()
        self.sh_helper = shaper_calibrate.ShaperCalibrate(printer=None)

    def init_chip_config(self, chip_name):
        self.accel_config = self.printer.lookup_object(chip_name)

    def set_samples_trim_size(self, s):
        self.samples_trim_size = s

    def handle_batch(self, batch):
        if self.is_finished:
            return False
        samples = batch['data']
        self.samples.extend(samples)
        return True

    def flush_data(self):
        self.is_finished = False
        self.samples.clear()
        self.request_timings.clear()

    def update_start_time(self):
        if self.request_start_time is not None:
            raise self.gcode.error('Error in accelerometer handler')
        self.request_start_time = self.toolhead.get_last_move_time()

    def update_end_time(self):
        end_time = self.toolhead.get_last_move_time()
        self.request_timings.append([self.request_start_time, end_time])
        self.request_start_time = None

    def _init_static(self):
        self.toolhead.dwell(1)
        self.static_noise = np.mean(self.samples, axis=0)

    def start_measurements(self):
        self.flush_data()
        self.accel_config.batch_bulk.add_client(self.handle_batch)
        self._init_static()

    def finish_measurements(self):
        self.toolhead.wait_moves()
        self.is_finished = True

    def _wait_samples(self):
        while True:
            last_mcu_time = self.samples[-1][0]
            req_end_time = self.request_timings[0][1]
            if last_mcu_time > req_end_time:
                request_timings = self.request_timings[0][:]
                del self.request_timings[0]
                return request_timings
            elif (last_mcu_time < req_end_time
                  and len(self.request_timings) > 3):
                raise self.gcode.error('chopper_resonance_tuner: '
                                       'No data from accelerometer')
            now = self.reactor.monotonic()
            self.reactor.pause(now + 0.1)

    def get_samples(self, start_t, stop_t):
        raw_data = np.array(self.samples)
        st_idx = np.searchsorted(raw_data[:, 0], start_t, side='left')
        end_idx = np.searchsorted(raw_data[:, 0], stop_t, side='right')
        time_accels = raw_data[st_idx:end_idx]
        return np.array(time_accels), end_idx - st_idx

    def set_max_freq(self, max_freq):
        self.max_freq = max_freq

    def samples_to_freq(self, data):
        if self.samples_trim_size:
            trim = int(data.shape[0] // self.samples_trim_size)
            data = data[trim:-trim]
        cal_data = self.sh_helper.process_accelerometer_data(data)
        cal_data.normalize_to_frequencies()
        freqs = cal_data.freq_bins
        psd = cal_data.psd_sum[freqs <= self.max_freq]
        px = cal_data.psd_x[freqs <= self.max_freq]
        py = cal_data.psd_y[freqs <= self.max_freq]
        pz = cal_data.psd_z[freqs <= self.max_freq]
        freqs = freqs[freqs <= self.max_freq]
        max_power_index = np.argmax(psd)
        fr = freqs[max_power_index]
        max_power = psd[max_power_index]
        return max_power, freqs, psd, px, py, pz

    def samples_to_vects(self, data):
        if self.samples_trim_size:
            trim = int(data.shape[0] // self.samples_trim_size)
            data = data[trim:-trim]
        data = data - self.static_noise
        times = data[:, 0]
        data = np.array(data[:, 1:])
        vects = np.linalg.norm(data, axis=1)
        md_magnitude = np.median(vects)
        return md_magnitude, times, vects


class SamplesCollector:
    def __init__(self, printer, chip_helper):
        self.printer = printer
        self.chip_helper = chip_helper
        self.gcode = printer.lookup_object('gcode')
        self.reactor = printer.get_reactor()
        self.collector_timer = None
        self.names = []
        self.procs = []
        self.error = None

    def set_names(self, names):
        self.names = names

    def check_error(self):
        return self.error

    def _check_procs(self):
        while self.procs:
            proc, p_conn = self.procs[0]
            if proc.is_alive():
                return
            err, res = p_conn.recv()
            if err:
                self.error = res
                return
            del self.chip_helper.samples[:res]
            del self.procs[0]

    def _collect_sample(self, eventtime):
        self._check_procs()
        delay = 1.0
        if not self.chip_helper.request_timings:
            return eventtime + delay
        try:
            t_range = self.chip_helper._wait_samples()
        except Exception as e:
            self.error = e
            return self.reactor.NEVER
        def save_sample():
            try:
                samples, samples_ct = \
                    self.chip_helper.get_samples(*t_range)
                mp, fq, psd, px, py, pz = \
                    self.chip_helper.samples_to_freq(samples)
                filepath = os.path.join(SAMPLES_FOLDER, filename)
                np.savez_compressed(filepath, mp=mp, fq=fq,
                                    psd=psd, px=px, py=py, pz=pz)
                c_conn.send((False, samples_ct))
                c_conn.close()
            except Exception as e:
                c_conn.send((True, e))
                c_conn.close()
        filename = self.names.pop(0)
        p_conn, c_conn = multiprocessing.Pipe()
        proc = multiprocessing.Process(target=save_sample)
        proc.daemon = True
        proc.start()
        self.procs.append((proc, p_conn))
        if len(self.chip_helper.request_timings) > 1:
            delay = 0.1
        now = self.reactor.monotonic()
        return now + delay

    def start_collector(self):
        if self.collector_timer is None:
            self.error = None
            if err := check_export_path(SAMPLES_FOLDER):
                self.gcode.error(str(err))
            self.chip_helper.start_measurements()
            now = self.reactor.monotonic()
            self.collector_timer = self.reactor.register_timer(
                self._collect_sample, now + 1.0)

    def stop_collector(self):
        if self.collector_timer is not None:
            while ((self.chip_helper.request_timings or self.procs)
                   and not self.error):
                self._check_procs()
                now = self.reactor.monotonic()
                self.reactor.pause(now + 0.1)
            self.chip_helper.finish_measurements()
            self.reactor.unregister_timer(self.collector_timer)
            self.collector_timer = None
            self.names.clear()
            self.procs.clear()


class LinearTest:
    def __init__(self, tuner, *args):
        self.printer = tuner.printer
        self.min_speed, self.max_speed = args
        tuner.chip_helper.set_samples_trim_size(5)
        self.reactor = self.printer.get_reactor()
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2
        self.travel_distance = self.max_speed * 2.0

    def run_movement(self, speed):
        x, *args = self.toolhead.get_position()
        travel_distance = (self.travel_distance / self.max_speed) * speed
        position = travel_distance + x
        self.toolhead.manual_move([position], speed)
        # self.reactor.pause(self.reactor.monotonic() + 0.01)
        self.toolhead.manual_move([x], self.travel_speed)


class ResonanceTest:
    def __init__(self, tuner, *args):
        self.printer = tuner.printer
        gn, self.axis = args
        tuner.chip_helper.set_samples_trim_size(0)
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2
        self.test_seq = gn.gen_test()

    def run_movement(self, speed):
        X, Y, Z, E = old_pos = self.toolhead.get_position()
        # Override maximum acceleration and acceleration to
        # deceleration based on the maximum test frequency
        systime = self.reactor.monotonic()
        toolhead_info = self.toolhead.get_status(systime)
        old_max_accel = toolhead_info['max_accel']
        max_accel = max([abs(a) for _, a, _ in self.test_seq])
        self.gcode.run_script_from_command(
            "SET_VELOCITY_LIMIT ACCEL=%.3f MINIMUM_CRUISE_RATIO=0"
            % (max_accel,))
        last_v = last_t = last_accel = last_freq = 0.
        for next_t, accel, freq in self.test_seq:
            t_seg = next_t - last_t
            self.toolhead.cmd_M204(self.gcode.create_gcode_command(
                "M204", "M204", {"S": abs(accel)}))
            v = last_v + accel * t_seg
            abs_v = abs(v)
            if abs_v < 0.000001:
                v = abs_v = 0.
            abs_last_v = abs(last_v)
            v2 = v * v
            last_v2 = last_v * last_v
            half_inv_accel = .5 / accel
            d = (v2 - last_v2) * half_inv_accel
            dX, dY = self.axis.get_point(d)
            nX = X + dX
            nY = Y + dY
            self.toolhead.limit_next_junction_speed(abs_last_v)
            if v * last_v < 0:
                # The move first goes to a complete stop, then changes direction
                d_decel = -last_v2 * half_inv_accel
                decel_X, decel_Y = self.axis.get_point(d_decel)
                self.toolhead.move(
                    [X + decel_X, Y + decel_Y, Z, E], abs_last_v)
                self.toolhead.move([nX, nY, Z, E], abs_v)
            else:
                self.toolhead.move([nX, nY, Z, E], max(abs_v, abs_last_v))
            if math.floor(freq) > math.floor(last_freq):
                # self.gcode.respond_info(f"Testing frequency {freq} Hz")
                self.reactor.pause(self.reactor.monotonic() + 0.01)
            X, Y = nX, nY
            last_t = next_t
            last_v = v
            last_accel = accel
            last_freq = freq
        if last_v:
            d_decel = -.5 * last_v2 / old_max_accel
            decel_X, decel_Y = self.axis.get_point(d_decel)
            self.toolhead.move([X + decel_X, Y + decel_Y, Z, E], abs(last_v))
            self.toolhead.cmd_M204(self.gcode.create_gcode_command(
                "M204", "M204", {"S": old_max_accel}))
        self.toolhead.move(old_pos, speed=self.travel_speed)


class ChopperResonanceTuner:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.lookup_object(config, 'gcode_move')
        self.force_move = self.printer.lookup_object(config, 'force_move')
        self.stepper_en = self.printer.lookup_object(config, 'stepper_enable')
        self.input_shaper = self.printer.load_object(config, 'input_shaper')
        self.res_tester = self.printer.load_object(config, 'resonance_tester')
        self.plt_helper = PlotterHelper(self.printer)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self._init_axes()
        # Read config
        self.accel_chip = self.config.get('accel_chip')
        self.debug = self.config.getboolean('debug', default=False)
        # Register commands
        self.gcode.register_command('CHOPPER_RESONANCE_TUNER',
                                    self.cmd_RUN_TUNER,
                                    desc=self.cmd_RUN_TUNER_help)
        # Variables
        self.ask_user = False

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2
        self.travel_accel = self.toolhead.max_accel / 2
        self.kin = self.toolhead.get_kinematics()
        self._init_steppers()
        self.chip_helper = AccelHelper(self.printer, self.accel_chip)
        self.collector = SamplesCollector(self.printer, self.chip_helper)

    def lookup_config(self, section, entry, default=None):
        section = self.config.getsection(section)
        try:
            return section.getfloat(entry, default)
        except:
            return section.get(entry, default)

    def _init_axes(self):
        kin = self.lookup_config('printer', 'kinematics')
        if kin != 'corexy':
            self.config.error(f"chopper_resonance_tuner: "
                              f"unsupported kinematics '{kin}'")
        self.axes = ['x', 'y']
        self.initial_pos = []
        for axis in self.axes:
            stepper = 'stepper_' + axis
            min_pos = self.lookup_config(stepper, 'position_min', 0)
            max_pos = self.lookup_config(stepper, 'position_max')
            self.initial_pos.append(min_pos + (max_pos - min_pos) / 2)

    def _get_tmc_driver(self, stepper_name):
        for driver in TRINAMIC_DRIVERS:
            driver_name = f"{driver} {stepper_name}"
            module = self.printer.lookup_object(driver_name, None)
            if module is not None:
                return driver_name, module
        else:
            raise self.config.error(
                f"chopper_resonance_tuner: unable to find "
                f"TMC driver for '{stepper_name}' stepper")

    def get_reg(self, tmc, field):
        if tmc.fields.lookup_register(field, None) is None:
            raise self.gcode.error(f'Unknown field: {field}')
        reg = tmc.fields.lookup_register(field)
        val = tmc.mcu_tmc.get_register(reg)
        val = tmc.fields.get_field(field, val)
        return val

    def set_reg(self, tmc, field, val, ptime):
        bit = tmc.fields.set_field(field, val)
        reg_name = tmc.fields.lookup_register(field)
        if self.debug: self.gcode.respond_info(
                f'Setting field: {field.upper()}={val}')
        tmc.mcu_tmc.set_register(reg_name, bit, ptime)

    def _init_def_registers(self):
        tmc_name = list(self.kin_steppers.values())[0]['tmc_name']
        configfile = self.printer.lookup_object('configfile')
        sconfig = configfile.get_status(None)['settings']
        tcf = sconfig.get(tmc_name)
        self.def_fields = {
            'current': int(tcf['run_current'] * 1000),
            'tbl': tcf['driver_tbl'],
            'toff': tcf['driver_toff'],
            'hstrt': tcf['driver_hstrt'],
            'hend': tcf['driver_hend'],
            'tpfd': tcf['driver_tpfd'],
        }

    def _init_steppers(self):
        self.kin_steppers = {}
        for s in self.kin.get_steppers():
            for a in self.axes:
                kin_stepper = s.get_name()
                if 'stepper_' + a in kin_stepper:
                    name, module = self._get_tmc_driver(kin_stepper)
                    self.kin_steppers[kin_stepper] = {
                        'stepper': s,
                        'tmc_name': name,
                        'tmc': module,
                    }
        self._init_def_registers()

    def gsend(self, params):
        self.gcode._process_commands([params], False)

    def stepper_enable(self, stepper, mode, ontime, offtime):
        self.toolhead.dwell(ontime)
        print_time = self.toolhead.get_last_move_time()
        el = self.stepper_en.enable_lines[stepper]
        el.motor_enable(print_time) if mode \
            else el.motor_disable(print_time)
        self.toolhead.dwell(offtime)

    def stepper_move(self, mcu_stepper, dist):
        self.force_move.manual_move(mcu_stepper,
            dist, self.travel_speed, self.travel_accel)

    def check_recovery(self, gcmd):
        recovery = gcmd.get('RECOVERY', None)
        if os.path.exists(SAMPLES_FOLDER):
            if self.ask_user and recovery is None:
                self.ask_user = False
                return None
            if recovery is not None:
                return True
            gcmd.respond_info(
                'Uncompleted test was found, to continue it '
                'enter: CHOPPER_RESONANCE_TUNER RECOVERY=1')
            self.ask_user = True
            return False

    def prestart(self):
        # Going to initial position
        now = self.reactor.monotonic()
        kin_status = self.kin.get_status(now)
        if ''.join(self.axes)not in kin_status['homed_axes']:
            raise self.gcode.error("Must home axes first")
        # self.toolhead.manual_move(self.initial_pos, speed=self.travel_speed)
        self.toolhead.wait_moves()

    def generate_plot(self, axis_name):
        def run():
            plot = self.plt_helper.generate_html()
            if plot is None:
                return
            path = self.plt_helper.save_plot(
                plot, axis_name, self.chip_helper.chip_name)
            self.gcode.respond_info(f'Access to interactive '
                                    f'plot at: {path}')
        run()

    def enable_extra_steppers(self, mode):
        ptime = self.toolhead.get_last_move_time()
        for n, p in self.kin_steppers.items():
            if n not in self.steppers.keys():
                s = 'Enabling' if mode else 'Disabling'
                self.gcode.respond_info(f'{s} control of {n}')
                self.set_reg(p['tmc'], 'toff', mode, ptime)

    def finish_test(self, axis, last_fields, toolhead_info):
        self.gsend('M84')
        self.enable_extra_steppers(1)
        # Restore the original fields
        ptime = self.toolhead.get_last_move_time()
        for field, val in self.def_fields.items():
            if last_fields[field] == val:
                continue
            for st in self.kin_steppers.values():
                if field == 'current':
                    current = val / 1000
                    st_name = st['stepper'].get_name()
                    self.gsend(f"SET_TMC_CURRENT "
                               f"STEPPER={st_name} "
                               f"CURRENT={current} "
                               f"HOLDCURRENT={current}")
                    continue
                self.set_reg(st['tmc'], field, val, ptime)
        # Restore the original acceleration values
        old_ma = toolhead_info['max_accel']
        old_mcr = toolhead_info['minimum_cruise_ratio']
        self.gcode.run_script_from_command(
            f'SET_VELOCITY_LIMIT ACCEL={old_ma} '
            f'MINIMUM_CRUISE_RATIO={old_mcr}')
        self.input_shaper.enable_shaping()
        self.collector.stop_collector()
        self.generate_plot(axis)
        remove_export_path(SAMPLES_FOLDER)

    cmd_RUN_TUNER_help = 'Start chopper resonance analyzing'
    def cmd_RUN_TUNER(self, gcmd):
        # status = self.check_recovery(gcmd)
        # if status is False:
        #     return
        # elif status is None
        # Live variables
        axis_name = 'x'
        # axis_name = gcmd.get('AXIS', self.axes[0]).lower()
        axis = resonance_tester._parse_axis(gcmd, axis_name)
        _steppers = gcmd.get('STEPPER',).split(',')
        if not _steppers[0]:
            _steppers = list(self.kin_steppers.keys())
        self.steppers = {str(name): st for name, st in
                         self.kin_steppers.items()
                         for _name in _steppers if name == _name}
        freq_start = gcmd.get_int(
            'FREQ_START', 1, minval=1.)
        freq_end = gcmd.get_int(
            'FREQ_END', 300, minval=freq_start, maxval=1000.)
        gn = self.res_tester.generator
        vgn = self.res_tester.generator.vibration_generator
        test_accel_per_hz = gcmd.get_float(
            'ACCEL_PER_HZ', vgn.accel_per_hz, above=0.)
        test_hz_per_sec = gcmd.get_float(
            'HZ_PER_SEC', vgn.hz_per_sec, above=0., maxval=100.)
        test_sweeping_accel = gcmd.get_float(
            'SWEEPING_ACCEL', gn.sweeping_accel, above=0.)
        test_sweeping_period = gcmd.get_float(
            'SWEEPING_PERIOD', gn.sweeping_period, minval=0.)
        # Manual gn.prepare_test() with increased limits
        gn.vibration_generator.freq_start = freq_start
        gn.vibration_generator.freq_end = freq_end
        gn.vibration_generator.test_accel_per_hz = test_accel_per_hz
        gn.vibration_generator.test_hz_per_sec = test_hz_per_sec
        gn.test_sweeping_accel = test_sweeping_accel
        gn.test_sweeping_period = test_sweeping_period
        # Fields
        curr_min_ma = gcmd.get_int('CURRENT_MIN_MA',
                                   self.def_fields['current'])
        curr_max_ma = gcmd.get_int('CURRENT_MAX_MA',
                                   self.def_fields['current'])
        curr_change_step = gcmd.get_int('CURRENT_CHANGE_STEP',
                                        100, minval=10, maxval=1000)
        tbl_min = gcmd.get_int('TBL_MIN', self.def_fields['tbl'],
                               minval=0, maxval=3)
        tbl_max = gcmd.get_int('TBL_MAX', self.def_fields['tbl'],
                               minval=0, maxval=3)
        toff_min = gcmd.get_int('TOFF_MIN', self.def_fields['toff'],
                                minval=1, maxval=15)
        toff_max = gcmd.get_int('TOFF_MAX', self.def_fields['toff'],
                                minval=1, maxval=15)
        hstrt_hend_max = gcmd.get_int('HSTRT_HEND_MAX',
                                      15, minval=1, maxval=15)
        hstrt_min = gcmd.get_int('HSTRT_MIN', self.def_fields['hstrt'],
                                 minval=0, maxval=7)
        hstrt_max = gcmd.get_int('HSTRT_MAX', self.def_fields['hstrt'],
                                 minval=0, maxval=7)
        hend_min = gcmd.get_int('HEND_MIN', self.def_fields['hend'],
                                minval=0, maxval=15)
        hend_max = gcmd.get_int('HEND_MAX', self.def_fields['hend'],
                                minval=0, maxval=15)
        tpfd_min = gcmd.get_int('TPFD_MIN', self.def_fields['tpfd'],
                                minval=0, maxval=15)
        tpfd_max = gcmd.get_int('TPFD_MAX', self.def_fields['tpfd'],
                                minval=0, maxval=15)
        min_speed = gcmd.get_int('MIN_SPEED', 1, minval=0, maxval=250)
        max_speed = gcmd.get_int('MAX_SPEED', 1, minval=0, maxval=250)
        speed_change_step = gcmd.get_int('SPEED_CHANGE_STEP',
                                         1, minval=0, maxval=100)
        # iterations = gcmd.get_int('ITERATIONS', 1, minval=0, maxval=100)
        methods = {'linear': LinearTest(self, min_speed, max_speed),
                   'resonance': ResonanceTest(self, gn, axis)}
        method_str = gcmd.get('METHOD', 'linear').lower()
        method = methods.get(method_str, None)
        if method is None:
            raise self.gcode.error(f'Invalid method: {method_str}')
        # For future restore
        now = self.reactor.monotonic()
        toolhead_info = self.toolhead.get_status(now)
        # Variables
        test_seq = []
        for current in range(curr_min_ma, curr_max_ma + 1, curr_change_step):
         for tbl in range(tbl_min, tbl_max + 1):
          for toff in range(toff_min, toff_max + 1):
           for hstrt in range(hstrt_min, hstrt_max + 1):
            for hend in range(hend_min, hend_max + 1):
             if hstrt + hend <= hstrt_hend_max:
              for tpfd in range(tpfd_min, tpfd_max + 1):
               for speed in range(min_speed, max_speed + 1, speed_change_step):
                freq = float(round(1/(2*(12+32*toff)*1/(1000000*FCLK)+2*1/(
                             1000000*FCLK)*16*(1.5**tbl))/1000, 1))
                n = (f'current={current}_tbl={tbl}_toff={toff}_hstrt={hstrt}'
                     f'_hend={hend}_tpfd={tpfd}_speed={speed}_freq={freq}kHz')
                test_seq.append({
                    'fields': {
                        'current': current,
                        'tbl': tbl,
                        'toff': toff,
                        'hstrt': hstrt,
                        'hend': hend,
                        'tpfd': tpfd,
                    },
                    'params': {
                        'speed': speed,
                        'name': n,
                    },
                })
        last_fields = self.def_fields.copy()
        self.collector.set_names([s['params']['name'] for s in test_seq])
        # Run
        # self.prestart()
        self.toolhead.set_position([50,50,0,0],homing_axes='xyz')
        self.toolhead.max_accel = 5000
        self.enable_extra_steppers(0)
        self.input_shaper.disable_shaping()
        self.chip_helper.set_max_freq(freq_end)
        self.collector.start_collector()
        self.toolhead.wait_moves()

        for seq in test_seq:
            # now = self.reactor.monotonic()
            # ptime = max(self.toolhead.get_last_move_time() - 0.3,
            #             self.toolhead.mcu.estimated_print_time(now))
            ptime = self.toolhead.get_last_move_time()
            for field, val in seq['fields'].items():
                if last_fields[field] != val:
                    last_fields[field] = val
                    for st in self.steppers.values():
                        if field == 'current':
                            current = val / 1000
                            st_name = st['stepper'].get_name()
                            self.gsend(f"SET_TMC_CURRENT "
                                       f"STEPPER={st_name} "
                                       f"CURRENT={current} "
                                       f"HOLDCURRENT={current}")
                            continue
                        self.set_reg(st['tmc'], field, val, ptime)
            speed = seq['params']['speed']
            name = seq['params']['name']
            # Run
            self.gcode.respond_info(f'Testing: {name}')
            try:
                err = self.collector.check_error()
                if err is not None:
                    raise self.gcode.error(err)
                self.chip_helper.update_start_time()
                method.run_movement(speed)
                self.chip_helper.update_end_time()
            except self.gcode.error as e:
                self.finish_test(axis_name, last_fields, toolhead_info)
                raise self.gcode.error(str(e))
        self.finish_test(axis_name, last_fields, toolhead_info)


class PlotterHelper:
    colors = ['', '#2F4F4F', '#12B57F', '#9DB512', '#DF8816',
              '#1297B5', '#5912B5', '#B51284', '#127D0C']

    def __init__(self, printer):
        if err := check_export_path(RESULTS_FOLDER):
            printer.error(str(err))

    def add_bar(self, name, max_power, freqs, psd, px, py, pz):
        dx = freqs.tolist()
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=dx, y=psd.tolist(), mode='lines', name='X+Y+Z',
            line=dict(color='red')))
        fig.add_trace(go.Scatter(
            x=dx, y=px.tolist(), mode='lines', name='X',
            line=dict(color='green')))
        fig.add_trace(go.Scatter(
            x=dx, y=py.tolist(), mode='lines', name='Y',
            line=dict(color='yellow')))
        fig.add_trace(go.Scatter(
            x=dx, y=pz.tolist(), mode='lines', name='Z',
            line=dict(color='blue')))
        fig.update_layout(
            title='Frequency response', template='plotly_dark',
            xaxis_title='Frequency (Hz)',
            yaxis_title='Power spectral density',
            yaxis_tickformat=".2e", autosize=True,
            hovermode='x unified', margin=dict(l=40, r=40, t=80, b=40))
        return [[name, float(max_power)], fig.to_dict()]

    def generate_bars(self):
        subplots = []
        data_folder = sorted(os.listdir(SAMPLES_FOLDER), key=lambda x:
            os.path.getmtime(os.path.join(SAMPLES_FOLDER, x)))
        for filename in data_folder:
            if not filename.endswith('.npz'):
                continue
            filepath = os.path.join(SAMPLES_FOLDER, filename)
            data = [np.load(filepath)[key] for key in
                    ['mp', 'fq', 'psd', 'px', 'py', 'pz']]
            subplots.append(self.add_bar(filename, *data))
        return subplots

    def generate_html(self):
        subplots = self.generate_bars()
        if not subplots:
            return
        fig = go.Figure()
        for p, _ in subplots:
            toff = int(p[0].split('_')[2].split('=')[1])
            color = self.colors[toff if toff <= 8 else toff - 8]
            fig.add_trace(go.Bar(x=[p[1]], y=[p[0]], orientation='h',
                                 marker_color=color, showlegend=False))
        fig.update_layout(
            title='Magnitude vs Parameters', template='plotly_dark',
            xaxis_title="Magnitude", yaxis_title="Parameters",
            autosize=True, margin=dict(l=40, r=40, t=80, b=40))

        main_graph = fig.to_dict()
        sec_graphs = {e[0][1]: e[1] for e in subplots}

        html_template = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
            <style>
                body {{
                    margin: 0;
                    padding: 0;
                    background-color: black;
                    color: white;
                    overflow: hidden;
                }}
                #plot {{
                    width: 100vw;
                    height: 100vh;
                }}
                #back {{
                    display: none;
                    position: absolute;
                    top: 10px;
                    left: 10px;
                    padding: 10px 20px;
                    background: rgba(255, 255, 255, 0.2);
                    color: white;
                    border: none;
                    cursor: pointer;
                    font-size: 14px;
                    z-index: 1000;
                }}
                #back:hover {{
                    background: rgba(255, 255, 255, 0.4);
                }}
            </style>
        </head>
        <body>
            <button id="back" onclick="showMain()">Back</button>
            <div id="plot"></div>

            <script>
                const mainGraph = {json.dumps(main_graph)};
                const detailedGraphs = {json.dumps(sec_graphs)};
                const plotDiv = document.getElementById('plot');
                const backButton = document.getElementById('back');

                function showMain() {{
                    Plotly.newPlot(plotDiv, mainGraph.data,
                     mainGraph.layout, {{responsive: true}});
                    backButton.style.display = 'none';

                    plotDiv.on('plotly_click', function(data) {{
                        const label = data.points[0].x;
                        if (detailedGraphs[label]) {{
                            const detail = detailedGraphs[label];
                            Plotly.newPlot(
                             plotDiv, detail, {{responsive: true}});
                            backButton.style.display = 'block';
                            plotDiv.removeAllListeners('plotly_click');
                        }}
                    }});
                }}

                showMain();
            </script>
        </body>
        </html>
        """
        return html_template

    def save_plot(self, template, axis, chip_name):
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(RESULTS_FOLDER,
            f'resonances_{axis}_{now}_{chip_name}.html')
        with open(filename, "w", encoding="utf-8") as f:
            f.write(template)
        return filename


def load_config(config):
    return ChopperResonanceTuner(config)
