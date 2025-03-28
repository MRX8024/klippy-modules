# Registers changer script
#
# Copyright (C) 2025  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660",
    "tmc5160"]
TRINAMIC_DELAY_TIME = 0.001

class RegistersChanger:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self._init_axes()
        # Read config
        self.debug = self.config.getboolean('debug', default=False)
        # Register commands
        self.gcode.register_command('REGISTER_CHANGER',
                                    self.cmd_RUN_CHANGER,
                                    desc=self.cmd_RUN_CHANGER_help)
        # Variables
        self.is_running = False
        self.segments = []
        # self.reg_thr = {
        #     45: {
        #         'tbl': 2,
        #         'toff': 4,
        #         'hstrt': 6,
        #         'hend': 0,
        #     },
        #     90: {
        #         'tbl': 0,
        #         'toff': 1,
        #         'hstrt': 7,
        #         'hend': 5,
        #     },
        #     150: {
        #         'tbl': 0,
        #         'toff': 10,
        #         'hstrt': 7,
        #         'hend': 2,
        #     },
        #     9999: {
        #         'tbl': 0,
        #         'toff': 1,
        #         'hstrt': 5,
        #         'hend': 1,
        #     },
        # }
        self.reg_thr = {
            100: {
                'current': 0.75,
            },
            200: {
                'current': 1.0,
            },
            250: {
                'current': 1.5,
            },
            500: {
                'current': 2.0,
            },
            1000: {
                'current': 2.0,
            },
        }
        self.min_time = 0.1
        self.timer = None
        self.tasks = []

    def _registers_timer(self, eventtime):
        delay = 0.05
        while self.tasks:
            self.tasks.pop(0)()
        return eventtime + delay

    def register_timer(self):
        if self.timer is None:
            now = self.reactor.monotonic()
            self.timer = self.reactor.register_timer(
                self._registers_timer, now + 0.05)

    def unregister_timer(self):
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        if not hasattr(self.toolhead, 'reg_chelper'):
            raise self.config.error('register_changer: '
                                    'Unsupported toolhead module')
        self.toolhead.add_reg_chelper(self)
        self.travel_speed = self.toolhead.max_velocity / 2
        self.travel_accel = self.toolhead.max_accel / 2
        self.kin = self.toolhead.get_kinematics()
        self._init_steppers()

    def lookup_config(self, section, entry, default=None):
        section = self.config.getsection(section)
        try:
            return section.getfloat(entry, default)
        except:
            return section.get(entry, default)

    def _init_axes(self):
        kin = self.lookup_config('printer', 'kinematics')
        if kin != 'corexy':
            self.config.error(f"register_changer: "
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
                f"register_changer: unable to find TMC "
                f"driver for '{stepper_name}' stepper")

    def _init_def_registers(self):
        stepper = list(self.kin_steppers.values())[0]
        tmc_name = stepper['tmc_name']
        tmc = stepper['tmc']
        configfile = self.printer.lookup_object('configfile')
        sconfig = configfile.get_status(None)['settings']
        tcf = sconfig.get(tmc_name)
        self.def_fields = {
            'current': float(tcf['run_current']),
            'tbl': tcf.get('driver_tbl',
                self.get_field(tmc, 'tbl')),
            'toff': tcf.get('driver_toff',
                self.get_field(tmc, 'toff')),
            'hstrt': tcf.get('driver_hstrt',
                self.get_field(tmc, 'hstrt')),
            'hend': tcf.get('driver_hend',
                self.get_field(tmc, 'hend')),
            'tpfd': tcf.get('driver_tpfd',
                self.get_field(tmc, 'tpfd')),
        }

    def _init_steppers(self):
        self.kin_steppers = {}
        key, value = self.gcode.mux_commands.get('SET_TMC_FIELD')
        for s in self.kin.get_steppers():
            for a in self.axes:
                kin_stepper = s.get_name()
                if 'stepper_' + a in kin_stepper:
                    name, module = self._get_tmc_driver(kin_stepper)
                    cmdhelp = value.get(kin_stepper).__self__
                    self.kin_steppers[kin_stepper] = {
                        'stepper': s,
                        'tmc_name': name,
                        'tmc': module,
                        'tmc_cmdhelp': cmdhelp,
                    }
        self._init_def_registers()

    def get_field(self, tmc, field):
        if tmc.fields.lookup_register(field, None) is None:
            raise self.gcode.error(f'Unknown field: {field}')
        reg_name = tmc.fields.lookup_register(field)
        reg = tmc.mcu_tmc.get_register(reg_name)
        val = tmc.fields.get_field(field, reg)
        return val

    def set_reg(self, tmc, field, val, ptime):
        reg_name = tmc.fields.lookup_register(field)
        bit = tmc.fields.set_field(field, val, reg_name=reg_name)
        if self.debug: self.gcode.respond_info(
                f'Setting field: {field.upper()}={val}')
        tmc.mcu_tmc.set_register(reg_name, bit, ptime)

    def _set_current(self, stepper, val, ptime):
        ch = stepper['tmc_cmdhelp'].current_helper
        prev_cur, prev_hold_cur, req_hold_cur, max_cur = ch.get_current()
        run_current = hold_current = min(val, max_cur)
        ch.set_current(run_current, hold_current, ptime)
        # if self.debug: self.gcode.respond_info(
        #         f'Setting current: {val}')
        prev_cur, prev_hold_cur, req_hold_cur, max_cur = ch.get_current()
        if self.debug: self.gcode.respond_info(
            f'Setting run: {prev_cur:.2f}A hold: {prev_hold_cur:.2f}A')

    def set_field(self, stepper, field, val, ptime):
        if field == 'current':
            self._set_current(stepper, val, ptime)
        else:
            self.set_reg(stepper['tmc'], field, val, ptime)

    def set_fields(self, regs, next_move_t):
        ptime = next_move_t - TRINAMIC_DELAY_TIME
        now = self.reactor.monotonic()
        est_ptime = self.toolhead.mcu.estimated_print_time(now)
        delta_ptime = ptime - (est_ptime + 0.01)
        if delta_ptime < 0:
            self.gcode.respond_info(f'Change time is in the past '
                                    f'on {delta_ptime/1000} ms')
            return
        for reg, val in regs.items():
            for st in self.kin_steppers.values():
                self.set_field(st, reg, val, ptime)

    def gsend(self, params):
        self.gcode._process_commands([params], False)
        
    def get_thr_regs(self, velocity):
        for speed, regs in sorted(self.reg_thr.items()):
            if velocity <= speed:
                return speed, regs
        return list(self.reg_thr.items())[-1]

    def add_segment(self, seg_v, seg_t, next_move_t):
        speed, regs = self.get_thr_regs(seg_v)
        if self.segments and self.segments[-1][0] == speed:
            self.segments[-1][1] += seg_t
            self.gcode.respond_info(f"Expand {speed}: {regs} on {seg_t}s")
        elif seg_t >= self.min_time:
            self.segments.append([speed, seg_t])
            self.gcode.respond_info(f"Change to {speed}: {regs} on min {seg_t} s")
            self.tasks.append(lambda: self.set_fields(regs, next_move_t))

    def parse_move(self, move, next_move_t):
        # todo: complete the full calculations
        self.add_segment(move.start_v, move.accel_t, next_move_t)
        next_move_t += move.accel_t
        self.add_segment(move.cruise_v, move.cruise_t, next_move_t)
        next_move_t += move.cruise_t
        self.add_segment(move.end_v, move.decel_t, next_move_t)
        if len(self.segments) > 10:
            del self.segments[:5]

    def add_move(self, move, next_move_t):
        if self.is_running:
            self.parse_move(move, next_move_t)

    def on_reg_changer(self):
        if not self.is_running:
            self.is_running = True
            self.register_timer()
            self.gcode.respond_info('Enabling register changer')
            self.toolhead.wait_moves()

    def off_reg_changer(self):
        if self.is_running:
            self.is_running = False
            self.unregister_timer()
            ptime = self.toolhead.get_last_move_time()
            self.set_fields(self.def_fields, ptime)
            self.gcode.respond_info('Disabling register changer')

    cmd_RUN_CHANGER_help = 'ENABLE=0/1'
    def cmd_RUN_CHANGER(self, gcmd):
        en = gcmd.get('ENABLE', None)
        if en is None:
            return
        try:
            en = int(en)
        except:
            self.gcode.respond_info(f'Invalid enable value: {en}')
            return
        if en:
            self.on_reg_changer()
        elif not en:
            self.off_reg_changer()


def load_config(config):
    return RegistersChanger(config)
