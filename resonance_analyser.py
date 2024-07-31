# Script to match the shapers frequency to the resonance of the printer mechanics
#
# Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, time, itertools
import numpy as np
import matplotlib.pyplot as plt, matplotlib.ticker as ticker
from textwrap import wrap
from datetime import datetime
from . import input_shaper

MEASURE_DELAY = 0.10        # Delay between damped oscillations and measurement
MEASURE_SPEED = 200         # mm/s
MEDIAN_FILTER_WINDOW = 3    # Number of window lines
RESULTS_FOLDER = os.path.expanduser('~/printer_data/config/adxl_results/resonance_analyser')

def check_export_path(path):
    if not os.path.exists(path):
        try:
            os.makedirs(path)
        except OSError as e:
            print(f'Error generate path {path}: {e}')

class ResonanceAnalyser:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.force_move = self.printer.load_object(config, 'force_move')
        self.stepper_en = self.printer.load_object(config, 'stepper_enable')
        self.input_shaper = self.printer.load_object(config, 'input_shaper')
        # self.input_shaper_conf = input_shaper.AxisInputShaper(self.config)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        # Read config
        self.accel_chip = self.config.get('accel_chip')
        self.chip_config = self.printer.lookup_object(self.accel_chip)
        self.repeats = self.config.getint('repeats', default=1, minval=1, maxval=100)
        self.debug = self.config.getboolean('debug', default=False)
        # Register commands
        self.gcode.register_command('RESONANCE_ANALYSE', self.cmd_RUN_ANALYZE, desc='Start resonance analyzing')
        # Variables

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2
        self.travel_accel = self.toolhead.max_accel
        self.travel_dist = MEASURE_SPEED ** 2 / self.travel_accel + 20
        # self.input_shaper = self.printer.lookup_object('input_shaper')

    def lookup_config(self, section, section_entries, force_back=None):
        out = []
        section = self.config.getsection(section)
        for value in section_entries:
            try:
                out.append(float(section.get(value, force_back)))
            except:
                out.append(section.get(value, force_back))
        if None in out: raise
        return out[0] if len(out) == 1 else out
    
    def _send(self, params):
        self.gcode._process_commands([params], False)

    def _stepper_switch(self, stepper, mode):
        self.stepper_en.motor_debug_enable(stepper, mode)

    def _stepper_move(self, stepper, dist):
        self.force_move.manual_move(stepper, dist, 100, self.travel_accel)

    def _init_axes(self):
        axes_columns = {}
        self.toolhead.dwell(1.00)
        for axis in ['Z', 'X', 'Y']:
            aclient = self.chip_config.start_internal_client()
            self.toolhead.dwell(0.025)
            if axis != 'Z':
                self._send(f'G0 {axis}{self.center[axis] + 1} F{MEASURE_SPEED * 60}')
                self.toolhead.dwell(0.025)
            else:
                self.toolhead.dwell(0.1)
            aclient.finish_measurements()
            vect = np.array([[sample.accel_x, sample.accel_y, sample.accel_z]
                             for sample in aclient.get_samples()])
            # with open(f'{axis}.txt', 'w') as file:
            #     for i in vect:
            #             msg = f'[{i[0]}, {i[1]}, {i[2]}, {i[3]}],'
            #             file.write(str(msg) + '\n')
            # vect[:, 0] = 0
            for colum in axes_columns.values():
                vect[:, colum] = 0
            sort_vect = np.apply_along_axis(np.sort, axis=0, arr=vect)
            # Write avg of 5 max magnitudes
            nn = np.mean(np.abs(sort_vect[-5:]), axis=0)
            exc = nn.argmax()
            self.gcode.respond_info(f'Axes column: {axis} - {exc}, {nn}')
            axes_columns[axis] = exc
        return axes_columns

    def _resonance_move(self):
        # extra_axis = (ax if ax != axis else '' for ax in self.axes)
        # x, y = self.axes
        # ctx = self.center[x]
        # cty = self.center[y]
        # inset = self.travel_dist / 2
        # speed = MEASURE_SPEED * 60
        # moves = [
        #     f'G0 {x}{ctx - inset} {y}{cty - inset} F{speed}',
        #     f'G0 {x}{ctx + inset} {y}{cty - inset} F{speed}',
        #     f'G0 {x}{ctx + inset} {y}{cty + inset} F{speed}',
        #     f'G0 {x}{ctx - inset} {y}{cty + inset} F{speed}']
        for i in range(self.repeats):
            # self.moves.reverse()
            for move in self.moves:
                self._send(str(move))

    def _peak_rangecut(self, xy_vect, size=50, tolerance=5000):
        # cut = np.mean(vect[0:10, 1:], axis=0)
        # z_axis = np.abs(cut).argmax() + 1
        # xy_vect = np.delete(vect, z_axis, axis=1)
        tolerance = np.mean(np.abs(xy_vect[:, 1:])) * 4
        peak_pos = np.argmax(np.abs(xy_vect[:, 1:]).max(axis=1))
        fil_vect = []
        # fil_vect.append(vect[peak_pos - 100:peak_pos])
        for start in range(peak_pos, xy_vect.shape[0], size):
            window = xy_vect[start:start + size]
            fil_vect.append(window)
            if np.all(np.ptp(abs(window), axis=0) < tolerance):
                break
        for start in range(peak_pos, 0, -size // 4):
            low = max(0, start - size // 4)
            window = xy_vect[low:start]
            fil_vect.insert(0, window)
            if np.all(np.ptp(abs(window), axis=0) < tolerance):
                break
        fil_vect = np.vstack(fil_vect)
        start_time = float(fil_vect[0][0])
        for i in range(0, len(fil_vect)):
            fil_vect[i][0] -= start_time
        return fil_vect

    def _measure(self):
        # Measure the impact
        self._send(f'G0 X67.76 Y167.24 F{MEASURE_SPEED * 60}')
        self.toolhead.dwell(self.delay)
        aclient = self.chip_config.start_internal_client()
        self.toolhead.dwell(self.delay)
        self._resonance_move()
        self.toolhead.dwell(self.delay)
        aclient.finish_measurements()
        # return self._filter(aclient)
        return aclient

    def _homing(self):
        # Homing and going to center
        now = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(now)
        self.center = {}
        for axis in self.axes:
            stepper = 'stepper_' + axis.lower()
            min_pos, max_pos = self.lookup_config(stepper, ['position_min', 'position_max'], 0)
            self.center[axis] = min_pos + (max_pos - min_pos) / 2
        if ''.join(self.axes).lower() not in kin_status['homed_axes']:
            self._send(f"G28 {' '.join(self.axes)}")
        self._send(f"G0 {' '.join(f'{axis}{pos + self.travel_dist}' for axis, pos in self.center.items())} "
                   f"F{self.travel_speed * 60}")
        self._send(f'SET_VELOCITY_LIMIT ACCEL={self.travel_accel} MINIMUM_CRUISE_RATIO=0 SQUARE_CORNER_VELOCITY=5')
        self.toolhead.wait_moves()
    
    def cmd_RUN_ANALYZE(self, gcmd):
        # type = gcmd.get('SHAPER_TYPE', '')
        #
        # for shaper in self.input_shaper.shapers:
        #     shaper.shaper_type = type
        # # self.input_shaper_conf.report(gcmd)
        #
        # return
        self.axes = ['X', 'Y']
        self.motion = {}
        # Live variables
        self.axes = gcmd.get('AXES', self.axes)
        self.repeats = gcmd.get_int('REPEATS', self.repeats, minval=1, maxval=100)
        self.travel_dist = gcmd.get_int('DIST', self.travel_dist, minval=1, maxval=100)
        self.delay = gcmd.get_float('DELAY', MEASURE_DELAY, minval=0, maxval=100)
        freq_start = gcmd.get_int("FREQ_START", 20, minval=1.)
        freq_end = gcmd.get_int("FREQ_END", 150, minval=freq_start, maxval=200.)
        step = gcmd.get_float('STEP', 5, minval=1, maxval=10)
        self.travel_accel = gcmd.get_int('ACCEL', self.travel_accel, minval=1000, maxval=999999)
        self.travel_dist = MEASURE_SPEED ** 2 / self.travel_accel + 20
        shaper = gcmd.get('SHAPER_TYPE', '')
        # Run
        self._homing()
        axes_columns = self._init_axes()
        # self.gcode.respond_info('Started measurements')
        data = {}
        for axis in self.axes:
            data[axis] = {}
        coeff = 10

        if not shaper:
            for type in ['shaper_type_x', 'shaper_type']:
                shaper = self.lookup_config('input_shaper', [type], 0)
                if shaper: break
        if not shaper:
            raise self.gcode.error('Unknown shaper type')

        #################################################
        # aclient = self.chip_config.start_internal_client()
        # self.toolhead.dwell(self.delay)
        self.moves = [
            'G0 X67.76 Y167.24',
            'G0 X67.76 Y143.732',
            'G0 X67.918 Y143.732',
            'G0 X72.76 Y147.122',
            'G0 X72.76 Y139.672',
            'G0 X67.918 Y143.062',
            'G0 X67.76 Y143.062',
            'G0 X67.76 Y104.109',
            'G0 X72.76 Y100.608',
            'G0 X72.76 Y92.838',
            'G0 X67.76 Y89.337',
            'G0 X67.76 Y67.76',
            'G0 X91.268 Y67.76',
            'G0 X91.268 Y67.918',
            'G0 X87.921 Y72.699',
            'G0 X87.878 Y72.76',
            'G0 X95.328 Y72.76',
            'G0 X91.938 Y67.918',
            'G0 X91.938 Y67.76',
            'G0 X130.891 Y67.76',
            'G0 X134.392 Y72.76',
            'G0 X142.162 Y72.76',
            'G0 X145.663 Y67.76',
            'G0 X167.24 Y67.76',
        ]
        # for move in moves:
        #     self._send(str(move))
        # self.toolhead.dwell(self.delay)
        # aclient.finish_measurements()
        # vect = np.array([[sample.time, sample.accel_x, sample.accel_y, sample.accel_z]
        #                  for sample in aclient.get_samples()])
        # with open('log.txt', 'w') as file:
        #     for i in vect:
        #             msg = f'[{i[0]}, {i[1]}, {i[2]}],'
        #             file.write(str(msg) + '\n')
        # return
        #################################################

        freq_range = range(freq_start * coeff, int(freq_end * coeff + step * coeff), int(step * coeff))
        for freq in freq_range:
            freq /= coeff
            self._send(f'SET_INPUT_SHAPER SHAPER_TYPE={shaper} SHAPER_FREQ_{self.axes[0]}={freq} SHAPER_FREQ_{self.axes[1]}={freq}')
            aclient = self._measure()
            vect = np.array([[sample.time, sample.accel_x, sample.accel_y, sample.accel_z]
                             for sample in aclient.get_samples()])
            # with open('log.txt', 'w') as file:
            #     for i in vect:
            #             msg = f'[{i[0]}, {i[1]}, {i[2]}],'
            #             file.write(str(msg) + '\n')
            #     return
            for axis in self.axes:
                column = axes_columns[axis] + 1 # Skip time column
                # peak_point = np.mean(np.sort(np.abs(vect[:, column]))[-self.repeats * 4 * 2 * 2:]) # square + decel * 2
                peak_point = np.mean(np.sort(np.abs(vect[:, column]))[-self.repeats * 5 * 2:])
                fil_xy_vect = self._peak_rangecut(vect[:, [0, column]])
                # peak_point = np.abs(fil_xy_vect[:, 1:]).max()
                oscill_time = float(fil_xy_vect[-1, 0])
                self.gcode.respond_info(f'{axis}-Peak point: {peak_point:.2f}, time: {oscill_time:.5f} on {freq} Hz')
                data[axis][freq] = [peak_point, oscill_time]

        for axis in data:
            self.gcode.respond_info(f'//')
            out = sorted(data[axis].items(), key=lambda x: x[1][0])[:3]
            for freq, vals in out:
                self.gcode.respond_info(f'{axis}-Best peak: {vals[0]:.2f}, time: {vals[1]:.5f} on {freq} Hz')

        x_data = np.array(list(next(iter(data.values()))))
        y_data = np.array([list(val[0] for val in data[axis].values()) for axis in data]).T

        self.gcode.respond_info(str(y_data))
        self.gcode.respond_info(str(x_data))

        # Add window mean filter
        half_window = MEDIAN_FILTER_WINDOW // 2
        y_data = np.array(np.mean([y_data[max(i - half_window, 0):min(i + half_window + 1, len(y_data))]
                                        for i in range(half_window, len(y_data) - half_window)], axis=1))
        y_data = np.insert(y_data, range(half_window), y_data[:half_window], axis=0)
        y_data = np.append(y_data, y_data[-half_window:], axis=0)

        msg = plotter(x_data, y_data, self.axes, self.accel_chip, shaper)
        self.gcode.respond_info(msg)

def load_config(config):
    return ResonanceAnalyser(config)

def plotter(x_data, y_data, axes, accel_chip, shaper, debug=False):
    fig, ax = plt.subplots()
    # accel_y = vect[:, 2]
    # accel_z = vect[:, 3]
    axes = [f'Accel_{axis}' for axis in axes]
    colors = ['r', 'g', 'b', 'yellow', 'black']

    for i in range(y_data.shape[1]):
        ax.plot(x_data, y_data[:, i], label=axes[i], color=colors[i])

    # ax.plot(x_data, y_data, label=axes[0], color=colors[0])
    title = f'Toolhead inertial coasting over frequency'
    ax.set_title("\n".join(wrap(title, 66)), fontsize=10)
    ax.legend(loc='upper right', fontsize=10, framealpha=1, ncol=1)

    ax.set_xlabel(f'Shaper {shaper} frequency (Hz)')
    ax.set_xlim(0, 200)
    # ax.set_xlim(x_data.min(), x_data.max())
    ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
    # ax.ticklabel_format(axis='x', style='scientific', scilimits=(0, 0))

    ax.set_ylabel('Power spectral density')
    # max_abs_accel = np.abs(vect[:, 1:3]).max()
    # accel_range = max_abs_accel + (max_abs_accel / 10)
    # ax.set_ylim(-accel_range, accel_range)
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.ticklabel_format(axis='y', style='scientific', scilimits=(0, 0))
    ax.grid(which='major', color='grey')
    ax.grid(which='minor', color='lightgrey')
    if debug:
        plt.show()
    else:
        check_export_path(RESULTS_FOLDER)
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        png_path = os.path.join(RESULTS_FOLDER, f'interactive_plot_{shaper}_{accel_chip}_{now}.png')
        plt.savefig(png_path, dpi=500)
        return f'Access to interactive plot at: {png_path}'
