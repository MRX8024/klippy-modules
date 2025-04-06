# Script for tracking stepper position with encoder
#
# Copyright (C) 2025 Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

class PositionTracker:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        # Register event handlers
        printer.register_event_handler("klippy:connect",
                                       self._handle_connect)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        printer.register_event_handler("stepper_enable:motor_off",
                                       self._motor_off)
        printer.register_event_handler("homing:home_rails_begin",
                                       self._handle_home_rails_begin)
        printer.register_event_handler("homing:home_rails_end",
                                       self._handle_home_rails_end)
        # Read config
        self.sensor_name = config.get('sensor')
        self.max_difference = config.getfloat('max_difference', 1.0)
        self.runout_gcode = config.get('runout_gcode', None)
        if self.runout_gcode is not None:
            gm = printer.load_object(config, 'gcode_macro')
            self.runout_gcode = gm.load_template(config, 'runout_gcode')
        # Register commands
        gcode.register_mux_command("POSITION_TRACKER", "CHIP",
                                   self.sensor_name,
                                   self.cmd_POSITION_TRACKER,
                                   desc=self.cmd_POSITION_TRACKER_help)
        # Variables
        self.toolhead = None
        self.sensor = None
        self.stepper = None
        self.axis = None
        self.is_running = False
        self.is_paused = False
        self.resp_clock = 0

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.sensor = self.printer.lookup_object(self.sensor_name)
        stepper_name = self.sensor.calibration.stepper_name
        if stepper_name is None:
            raise self.printer.config_error(
                f'Option stepper in section '
                f'{self.sensor_name} must be specified')
        self.kin = self.toolhead.get_kinematics()
        self.mcu_stepper = [s for s in self.kin.get_steppers()
                        if stepper_name == s.get_name()][0]
        for i, rail in enumerate(self.kin.rails):
            if self.mcu_stepper in rail.get_steppers():
                self.axis = 'xyz'[i]
                break

    def normalize_encoder_pos(self, pos, offset):
        rd = self.mcu_stepper.get_rotation_distance()[0]
        return rd / ((1 << 16) / pos) + offset

    def _batch_handle(self, samples):
        if not self.is_running:
            return False
        if self.is_paused:
            return True
        enc_ptime, enc_raw_pos = samples['data'][-1]
        enc_pos = self.normalize_encoder_pos(
            enc_raw_pos, samples['position_offset'])
        mcu_pos = self.mcu_stepper.get_past_mcu_position(enc_ptime)
        req_pos = self.mcu_stepper.mcu_to_commanded_position(mcu_pos)
        self.resp_clock += 1
        if self.resp_clock == 200:
            self.resp_clock = 0
            self.gcode.respond_info(
                f"{self.sensor_name}: "
                f"delta_p: {req_pos-enc_pos:.3f}, "
                f"req_pos: {req_pos:.3f}, "
                f"enc_pos: {enc_pos:.3f}, "
                f"enc_t: {enc_ptime:.3f}")
        if (req_pos-self.max_difference
                < enc_pos < req_pos+self.max_difference):
            return True
        reactor = self.printer.get_reactor()
        now = reactor.monotonic()
        est_print_time = self.toolhead.mcu.estimated_print_time(now)
        msg = (f'Skipped steps detected on {self.sensor_name}: '
               f'required_pos={req_pos:.3f} sensor_pos={enc_pos:.3f} '
               f'ptime={est_print_time:.3f} sensor_time={enc_ptime:.3f}')
        if self.runout_gcode:
            self.gcode.respond_info(msg)
            self.gcode.run_script(self.runout_gcode.render() + "\nM400")
            return True
        else:
            self.printer.invoke_shutdown(msg)

    def start_tracker(self):
        self.is_running = True
        self.is_paused = False
        self.sensor.batch_bulk.batch_interval = 0.005
        self.sensor.add_client(self._batch_handle)

    def stop_tracker(self):
        self.is_running = False
        self.is_paused = False

    def pause_tracker(self):
        self.is_paused = True

    def resume_tracker(self):
        self.is_paused = False
        homed = self.axis in self.kin.get_status(None)['homed_axes']
        if not self.is_running and homed:
            self.start_tracker()

    def _handle_shutdown(self):
        self.stop_tracker()

    def _motor_off(self, ptime):
        self.stop_tracker()

    def _handle_home_rails_begin(self, homing, rails):
        self.pause_tracker()

    def _handle_home_rails_end(self, homing, rails):
        self.resume_tracker()

    def get_status(self, ptime):
        if not self.is_running:
            msg = 'disabled'
        elif self.is_paused:
            msg = 'paused'
        else:
            msg = 'running'
        return {'status': msg}

    cmd_POSITION_TRACKER_help = 'Position tracker'
    def cmd_POSITION_TRACKER(self, gcmd):
        enable = gcmd.get_int('ENABLE', -1)
        if enable == 0:
            self.pause_tracker()
        elif enable == 1:
            self.resume_tracker()
        self.gcode.respond_info(f"Position tracker {self.sensor_name}: "
                                f"{self.get_status(None)['status']}")


def load_config(config):
    return PositionTracker(config)

def load_config_prefix(config):
    return PositionTracker(config)
