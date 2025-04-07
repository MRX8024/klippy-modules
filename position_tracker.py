# Script for tracking stepper position with encoder
#
# Copyright (C) 2025 Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math

class PositionTracker:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.gcode = gcode = printer.lookup_object('gcode')
        self.force_move = self.printer.load_object(config, 'force_move')
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
        # self.timer = self.reactor.register_timer(self._resume_tracker,
        #                                          self.reactor.NEVER)
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
        self.enc_offset = 0.
        self.mcu_offset = 0.
        self.last_position = 0.
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
        configfile = self.printer.lookup_object('configfile')
        st_sc = configfile.get_status(None)['settings'][stepper_name]
        self.stepper_fspr = int(st_sc.get('full_steps_per_rotation'))
        for i, rail in enumerate(self.kin.rails):
            if self.mcu_stepper in rail.get_steppers():
                self.axis = 'xyz'[i]
                break

    def normalize_encoder_pos(self, pos):
        rd, _ = self.mcu_stepper.get_rotation_distance()
        return rd / ((1 << 16) / (pos - self.enc_offset + 1.))

    def stepper_move(self, dist):
        self.force_move.manual_move(self.mcu_stepper, dist, 500, 20000)

    def _wait_moves(self, time):
        self.is_paused = True
        while time > 0:
            now = self.reactor.monotonic()
            self.reactor.pause(now + 0.010)
            time -= 0.010
        self.is_paused = False

    def set_mcu_offset(self):
        now = self.reactor.monotonic()
        etime = self.toolhead.mcu.estimated_print_time(now)
        mcu_pos = self.mcu_stepper.get_past_mcu_position(etime)
        self.mcu_offset = self.mcu_stepper.mcu_to_commanded_position(mcu_pos)

    def _batch_handle(self, samples):
        enc_ptime, self.last_position = samples['data'][-1]
        if not self.is_running:
            return False
        if self.is_paused:
            return True
        enc_pos = self.normalize_encoder_pos(self.last_position)
        mcu_pos = self.mcu_stepper.get_past_mcu_position(enc_ptime) #- self.mcu_offset
        req_pos = self.mcu_stepper.mcu_to_commanded_position(mcu_pos) - self.mcu_offset
        self.resp_clock += 1
        if self.resp_clock == 4:
            self.resp_clock = 0
            self.gcode.respond_info(
                f"{self.sensor_name}: "
                f"delta_p: {req_pos-enc_pos:.3f}, "
                f"req_pos: {req_pos:.3f}, "
                f"enc_pos: {enc_pos:.3f}, "
                f"enc_t: {enc_ptime:.3f}")
        # return True
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
            # self.gcode.respond_info(msg)
            # self.gcode.run_script(self.runout_gcode.render() + "\nM400")
            # dir = 1
            # if enc_pos < req_pos:
            #     dir = -1
            # c_pos = self.toolhead.get_position()
            # # c_pos[0:2] *= 0.1*dir
            # c_pos[0] += 0.1*dir
            # c_pos[1] += 0.1*dir
            # self.toolhead.move(c_pos, 10)
            diff_d = req_pos - enc_pos
            rd, _ = self.mcu_stepper.get_rotation_distance()
            single_d = rd / self.stepper_fspr * 4
            aligned_diff_d = math.ceil(diff_d / single_d) * single_d
            self.gcode.respond_info(f"Move {self.sensor_name} on {aligned_diff_d}")
            # c_pos = self.toolhead.get_position()
            # c_pos[0] += delta_pos
            # c_pos[1] += delta_pos
            # self.toolhead.move(c_pos, 10)
            # self.toolhead.set_position(c_pos, homing_axes="xyz")
            self.stepper_move(aligned_diff_d)
            # self.toolhead.set_position(c_pos, homing_axes="xyz")
            # self.toolhead.wait_moves()
            # self._wait_moves(5.5)
            return True
        else:
            self.printer.invoke_shutdown(msg)

    def start_client(self):
        if not self.is_running:
            self.is_running = True
            self.sensor.batch_bulk.batch_interval = 0.05
            self.sensor.add_client(self._batch_handle)

    def stop_client(self):
        # self.reactor.update_timer(self.timer, self.reactor.NEVER)
        self.is_running = False
        self.is_paused = False

    def pause_tracker(self):
        # self.reactor.update_timer(self.timer, self.reactor.NEVER)
        self.is_paused = True

    # def _resume_tracker(self, etime):
    #     self.toolhead.wait_moves()
    #     self.is_paused = False
    #     homed = self.axis in self.kin.get_status(None)['homed_axes']
    #     if not self.is_running and homed:
    #         self.start_tracker()
    #     return self.reactor.NEVER

    def resume_tracker(self):
        self.is_paused = False
        # homed = self.axis in self.kin.get_status(None)['homed_axes']
        # if not self.is_running and homed:
        #     self.start_tracker()
        # self.toolhead.wait_moves()
        # waketime = self.reactor.monotonic() + 1.0
        # self.reactor.update_timer(self.timer, waketime)

    def _handle_shutdown(self):
        self.stop_client()
        # self.reactor.unregister_timer(self.timer)

    def _motor_off(self, ptime):
        self.stop_client()

    def _handle_home_rails_begin(self, homing, rails):
        if not self.is_running:
            self.start_client()
        self.pause_tracker()
        self.offset = 0.
        self.mcu_offset = 0.

    def _handle_home_rails_end(self, homing, rails):
        self.toolhead.wait_moves()
        self.enc_offset = self.last_position
        self.set_mcu_offset()
        self.gcode.respond_info(f"Offset {self.sensor_name}: "
                                f"enc_offset={self.enc_offset} "
                                f"mcu_offset={self.mcu_offset}")
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
