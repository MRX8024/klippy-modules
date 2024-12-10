# Z offsets calibration for toolchanger script
#
# Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import numpy as np
from mcu import MCU_endstop

class EndstopWrapper:
    def __init__(self, endstop):
        self.mcu_endstop = endstop
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop

class ZProbe:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.query_endstops = self.printer.load_object(config, 'query_endstops')
        self._setup_pin()
        self.probing_pos = config.getfloatlist('probing_position', count=3)
        self.probing_speed = config.getfloat('probing_speed', above=0.)
        self.samples = config.getint('samples', minval=1)
        self.retract_dist = config.getfloat('sample_retract_dist', above=0.)
        self.lift_speed = config.getfloat('lift_speed', above=0.)
        self.tolerance = config.getfloat('samples_tolerance', above=0.)
        self.retries = config.getint('samples_tolerance_retries', minval=0)
        # self.safe_z_height = config.getfloat('safe_z_height', above=0.)


        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.gcode.register_command('Z_PROBE', self.cmd_Z_PROBE,
                                    desc=self.cmd_Z_PROBE_help)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.travel_speed = self.toolhead.max_velocity / 2
        self.travel_accel = min(self.toolhead.max_accel, 5000)
        self.kin = self.toolhead.get_kinematics()
        self.probe = self.printer.lookup_object('probe')
        # get endstop config
        ct = 0
        for endstop, name in self.query_endstops.endstops:
            if name == 'z_probe_endstop':
                self.z_endstop = EndstopWrapper(endstop)
                ct += 1
            elif name == 'z':
                self.z_endstop.mcu_endstop._dispatch = endstop.mcu_endstop._dispatch
                ct += 1
        if ct != 2:
            raise self.config.error('Error in z_probe')

    def _setup_pin(self):
        ppins = self.printer.lookup_object('pins')
        endstop_pin = self.config.get('pin')
        # pin_params = ppins.parse_pin(endstop_pin, True, True)
        # pin_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
        mcu_endstop = ppins.setup_pin('endstop', endstop_pin)
        self.query_endstops.register_endstop(mcu_endstop, 'z_probe_endstop')

    def _move(self, coord, speed):
        if not speed:
            speed = self.travel_speed
        self.toolhead.manual_move(coord, speed)

    def _probe(self, gcmd, mcu_endstop, speed, wiggle=False):
            pos = self.toolhead.get_position()
            pos[2] = self.probing_pos[2] - 10
            phoming = self.printer.lookup_object('homing')
            curpos = phoming.probing_move(mcu_endstop, pos, speed)
            # retract
            self._move([None, None, curpos[2] + self.retract_dist],
                       self.lift_speed)
            # if wiggle and self.wiggle_offsets is not None:
            #     self._move([curpos[0] + self.wiggle_offsets[0],
            #                 curpos[1] + self.wiggle_offsets[1],
            #                 None],
            #                 self.speed)
            #     self._move([curpos[0], curpos[1], None], self.speed)
            self.gcode.respond_info(f"Probe at {curpos[2]:.6f}")
            return curpos

    def _probe_run(self, gcmd, mcu_endstop, wiggle=False):
        pos = self.toolhead.get_position()
        if pos[2] < self.probing_pos[2] + 10:
            self._move([None, None, self.probing_pos[2] + 10], self.lift_speed)
        self._move(self.probing_pos, self.travel_speed)
        self.toolhead.wait_moves()

        # check if switch is closed
        time = self.toolhead.get_last_move_time()
        if self.z_endstop.query_endstop(time):
            raise self.gcode.error(f"Z_probe switch not closed!")

        retries = 0
        poss = []
        while len(poss) < self.samples:
            # probe with second probing speed
            curpos = self._probe(gcmd, mcu_endstop, self.probing_speed)
            poss.append(curpos[:3])
            # check tolerance
            z_poss = [p[2] for p in poss]
            if max(z_poss) - min(z_poss) > self.tolerance:
                if retries >= self.retries:
                    raise gcmd.error(f"{gcmd.get_command()}: probe samples "
                                     f"exceed tolerance")
                gcmd.respond_info(f"{gcmd.get_command()}: probe samples"
                                  f" exceed tolerance. Retrying...")
                retries += 1
                poss.clear()

        return np.mean(poss, axis=0)

    cmd_Z_PROBE_help = ''
    def cmd_Z_PROBE(self, gcmd):
        # self.gcode.respond_info(f'{self.z_endstop.mcu_endstop.steppers}')
        # pst = self.probe.mcu_probe.get_steppers()
        # self.gcode.respond_info(f'Probe steppers {len(pst)}')
        # zst = self.z_endstop.get_steppers()
        # self.gcode.respond_info(f'z_probe steppers {len(zst)}')
        # return
        now = self.printer.get_reactor().monotonic()
        if 'xyz' not in self.kin.get_status(now)['homed_axes']:
            raise gcmd.error(f"{gcmd.get_command()}: must home axes first")

        xyz_pos = self._probe_run(gcmd, self.z_endstop)
        self.gcode.respond_info(f'Z mean value: {xyz_pos[2]}')


def load_config(config):
    return ZProbe(config)
