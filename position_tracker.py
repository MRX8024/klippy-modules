# Script for tracking stepper position with encoder
#
# Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import numpy as np

class AngleTracker:
    def __init__(self, config):
        self.config = config
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.gcode = printer.lookup_object('gcode')
        self.motion_report = printer.load_object(config, 'motion_report')
        printer.register_event_handler("klippy:connect", self._handle_connect)
        self.gcode.register_command('ANGLE_RUN', self.cmd_ANGLE_RUN,
                                    desc=self.cmd_ANGLE_RUN_help)
        self.time_pos = []

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        kin = self.toolhead.get_kinematics()
        self.steppers = [s for s in kin.get_steppers() for ax in 'xy'
                         if 'stepper_' + ax in s.get_name()]

    def _dump_stepper_handle(self, msg):
        start_position = msg['start_position']
        first_step_time = msg['first_step_time']
        self.time_pos.append((first_step_time, start_position))
        return True

    def _find_closest(self, samples, t_thresh=0.001):
        arr = np.array(samples['data'])
        times = arr[:, 0]
        for i, time_pos in enumerate(reversed(self.time_pos)):
            diffs = times - time_pos[0]
            positive_diffs = diffs[diffs >= 0]
            if len(positive_diffs) > 0:
                min_diff_idx = np.argmin(positive_diffs)
                min_diff = positive_diffs[min_diff_idx]
                if min_diff <= t_thresh:
                    ind = diffs.tolist().index(min_diff)
                    del self.time_pos[:-i+1]
                    return *time_pos, *arr[ind]
        return None, None, None, None

    def normalize_encoder_pos(self, pos, offset):
        pos = 40 / ((1 << 16) / pos)
        return pos + offset

    def _angle_handle(self, samples):
        if not self.time_pos:
            return True
        timing, pred_pos, enc_timing, enc_raw_pos = self._find_closest(samples)
        if timing is None:
            return True
        offset = samples['position_offset']
        enc_pos = self.normalize_encoder_pos(enc_raw_pos, offset)
        self.gcode.respond_info(
            f"delta_p: {pred_pos-enc_pos:.3f}, delta_t: {timing-enc_timing:.3f} "
            f"pred_pos: {pred_pos:.3f}, enc_pos: {enc_pos:.3f}, "
            f"pred_t: {timing:.3f}, enc_t: {enc_timing:.3f}")
        if not (pred_pos -1 < enc_pos < pred_pos + 1):
            self.gcode.respond_info(f'skipped steps detected!')
            # self.printer.invoke_shutdown(f'Skipped steps detected!')
        return True

    cmd_ANGLE_RUN_help = 'cmd_ANGLE_RUN_help'
    def cmd_ANGLE_RUN(self, gcmd):
        # st = self.steppers[0]
        # st_mcu = st.get_mcu()
        # now = self.reactor.monotonic()
        # est_ptime = self.toolhead.mcu.estimated_print_time(now)
        # est_clock = st_mcu.seconds_to_clock(est_ptime)
        # clock_100ms = st_mcu.seconds_to_clock(0.100)
        # start_clock = max(0, est_clock - clock_100ms)
        # end_clock = est_clock + clock_100ms
        # steps = st.dump_steps(128, start_clock, end_clock)
        # self.gcode.respond_info(str(steps))
        st = self.steppers[0]
        mot_st = self.motion_report.steppers.get(st.get_name(), None)
        if mot_st is None:
            self.gcode.error("Unknown stepper '%s'" % st.get_name())
        mot_st.batch_bulk.add_client(self._dump_stepper_handle)
        mot_st.batch_bulk.batch_interval = 0.1
        chip_name = 'angle ' + st.get_name()
        angle_cfg = self.printer.lookup_object(chip_name)
        angle_cfg.add_client(self._angle_handle)


def load_config(config):
    return AngleTracker(config)