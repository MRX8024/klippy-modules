# Script for diagonal parking on corexy kinematics
#
# Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660",
    "tmc5160"]

class DiagonalHoming:
    def __init__(self, config):
        self.config = config
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.gcode = printer.lookup_object('gcode')
        self._allow_multi_use_pin(config)
        # self.stepper_en = printer.lookup_object(config, 'stepper_enable')
        self.stepper_en = printer.load_object(config, 'stepper_enable')
        logging.info(f'diag_homing.py load stepper_enable')
        printer.register_event_handler("klippy:connect",
                                       self._handle_connect)
        printer.register_event_handler("homing:home_rails_begin",
                                       self.handle_home_rails_begin)
        printer.register_event_handler("homing:home_rails_end",
                                       self.handle_home_rails_end)
        self.endstops = []
        self.tmcs = []
        self.stepper_lines = []
        self.toff_vals = []

    def _allow_multi_use_pin(self, config):
        ppins = self.printer.lookup_object('pins')
        for axis_n in ['x', 'y']:
            for i in range(0, 99):
                if i == 0: i = ''
                section_n = f'stepper_{axis_n}{i}'
                if not config.has_section(section_n):
                    break
                sconfig = config.getsection(section_n)
                endstop_pin = sconfig.get('endstop_pin', None)
                if endstop_pin is not None:
                    ppins.allow_multi_use_pin(endstop_pin)

    def _get_tmc_driver(self, stepper_name):
        for driver in TRINAMIC_DRIVERS:
            driver_name = f"{driver} {stepper_name}"
            module = self.printer.lookup_object(driver_name, None)
            if module is not None:
                return module
        else:
            raise self.config.error(
                f"register_changer: unable to find TMC "
                f"driver for '{stepper_name}' stepper")

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        kin = self.toolhead.get_kinematics()
        self.endstops = [es for rail in kin.rails[0:2]
                         for es, name in rail.get_endstops()]
        key, value = self.gcode.mux_commands.get('SET_TMC_FIELD')
        for mcu_stepper in kin.rails[1].get_steppers():
            cmdhelp = value.get(mcu_stepper.get_name()).__self__
            self.tmcs.append(self._get_tmc_driver(mcu_stepper.get_name()))
            el = self.stepper_en.enable_lines[mcu_stepper.get_name()]
            self.stepper_lines.append(el)

    def get_field(self, tmc, field):
        if tmc.fields.lookup_register(field, None) is None:
            raise self.gcode.error(f'Unknown field: {field}')
        reg_name = tmc.fields.lookup_register(field)
        reg = tmc.mcu_tmc.get_register(reg_name)
        val = tmc.fields.get_field(field, reg)
        return val

    def set_field(self, tmc, field, val, ptime):
        reg_name = tmc.fields.lookup_register(field)
        bit = tmc.fields.set_field(field, val, reg_name=reg_name)
        tmc.mcu_tmc.set_register(reg_name, bit, ptime)

    def handle_home_rails_begin(self, homing, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if not any(en in endstops for en in self.endstops):
            return
        self.gcode.respond_info('handle_home_rails_begin')
        ptime = self.toolhead.get_last_move_time()
        now = self.reactor.monotonic()
        est_ptime = self.toolhead.mcu.estimated_print_time(now)
        # for en in self.stepper_lines:
        #     self.gcode.respond_info(f'Disabling: {en}')
        #     en.motor_disable(est_ptime + 0.05)
        for tmc in self.tmcs:
            # tmc._do_disable(est_ptime + 0.05)
            self.toff_vals.append(self.get_field(tmc, 'toff'))
            self.set_field(tmc, 'toff', 0, est_ptime + 0.05)

    def handle_home_rails_end(self, homing, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if not any(en in endstops for en in self.endstops):
            return
        self.gcode.respond_info('handle_home_rails_end')
        ptime = self.toolhead.get_last_move_time()
        # for en in self.stepper_lines:
        #     en.motor_enable(ptime)
        for tmc in self.tmcs:
            # tmc._do_enable(ptime)
            self.set_field(tmc, 'toff', self.toff_vals.pop(0), ptime)


def load_config(config):
    return DiagonalHoming(config)
