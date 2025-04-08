# Script for tracking stepper position with encoder
#
# Copyright (C) 2025 Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math
import chelper
from . import force_move

class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        self.junction_deviation = toolhead.junction_deviation
        self.timing_callbacks = []
        velocity = min(speed, toolhead.max_velocity)
        self.is_kinematic_move = True
        self.axes_d = axes_d = [end_pos[i] - start_pos[i] for i in (0, 1, 2, 3)]
        self.move_d = move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        if move_d < .000000001:
            # Extrude only move
            self.end_pos = (start_pos[0], start_pos[1], start_pos[2],
                            end_pos[3])
            axes_d[0] = axes_d[1] = axes_d[2] = 0.
            self.move_d = move_d = abs(axes_d[3])
            inv_move_d = 0.
            if move_d:
                inv_move_d = 1. / move_d
            self.accel = 99999999.9
            velocity = speed
            self.is_kinematic_move = False
        else:
            inv_move_d = 1. / move_d
        self.axes_r = [d * inv_move_d for d in axes_d]
        self.min_move_t = move_d / velocity
        # Junction speeds are tracked in velocity squared.  The
        # delta_v2 is the maximum amount of this squared-velocity that
        # can change in this move.
        self.max_start_v2 = 0.
        self.max_cruise_v2 = velocity**2
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel
        self.next_junction_v2 = 999999999.9
    def limit_speed(self, speed, accel):
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)
    def limit_next_junction_speed(self, speed):
        self.next_junction_v2 = min(self.next_junction_v2, speed**2)
    def move_error(self, msg="Move out of range"):
        ep = self.end_pos
        m = "%s: %.3f %.3f %.3f [%.3f]" % (msg, ep[0], ep[1], ep[2], ep[3])
        return self.toolhead.printer.command_error(m)
    def calc_junction(self, prev_move):
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        # Allow extruder to calculate its maximum junction
        extruder_v2 = self.toolhead.extruder.calc_junction(prev_move, self)
        # Find max velocity using "approximated centripetal velocity"
        axes_r = self.axes_r
        prev_axes_r = prev_move.axes_r
        junction_cos_theta = -(axes_r[0] * prev_axes_r[0]
                               + axes_r[1] * prev_axes_r[1]
                               + axes_r[2] * prev_axes_r[2])
        if junction_cos_theta > 0.999999:
            return
        # Apply limits
        self.max_start_v2 = min(
            extruder_v2, self.max_cruise_v2, prev_move.max_cruise_v2,
            self.next_junction_v2, prev_move.max_start_v2 + prev_move.delta_v2)
        if junction_cos_theta >= -0.999999:
            if (math.acos(junction_cos_theta) >
                    math.radians(self.toolhead.scv_angle_threshold)):
                sin_theta_d2 = min(max(math.sqrt(0.5 * (1. - round(
                    junction_cos_theta * self.toolhead.scv_boost_coeff, 5))
                                                 ), 0.000001), 0.999999)
            else:
                sin_theta_d2 = math.sqrt(0.5 * (1.0 - junction_cos_theta))
            R_jd = sin_theta_d2 / (1. - sin_theta_d2)
            # Approximated circle must contact moves no further than mid-move
            tan_theta_d2 = sin_theta_d2 / math.sqrt(.5*(1.0+junction_cos_theta))
            move_centripetal_v2 = .5 * self.move_d * tan_theta_d2 * self.accel
            prev_move_centripetal_v2 = (.5 * prev_move.move_d * tan_theta_d2
                                        * prev_move.accel)
            self.max_start_v2 = min(
                self.max_start_v2,
                R_jd * self.junction_deviation * self.accel,
                R_jd * prev_move.junction_deviation * prev_move.accel,
                move_centripetal_v2, prev_move_centripetal_v2)
        self.max_smoothed_v2 = min(self.max_start_v2,
            prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2)
    def set_junction(self, start_v2, cruise_v2, end_v2):
        # Determine accel, cruise, and decel portions of the move distance
        half_inv_accel = .5 / self.accel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_accel
        cruise_d = self.move_d - accel_d - decel_d
        # Determine move velocities
        self.start_v = start_v = math.sqrt(start_v2)
        self.cruise_v = cruise_v = math.sqrt(cruise_v2)
        self.end_v = end_v = math.sqrt(end_v2)
        # Determine time spent in each portion of move (time is the
        # distance divided by average velocity)
        self.accel_t = accel_d / ((start_v + cruise_v) * 0.5)
        self.cruise_t = cruise_d / cruise_v
        self.decel_t = decel_d / ((end_v + cruise_v) * 0.5)
        full_t = self.accel_t + self.cruise_t + self.decel_t
        self.avgspeed = self.move_d / full_t

class CorexyMoveChelper:
    def __init__(self, printer):
        self.printer = printer
        self.toolhead = printer.lookup_object('toolhead')
        self.rails = self.toolhead.get_kinematics().rails[:2]
        self.gcode = printer.lookup_object('gcode')
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.corexy_kins = None
        self._setup_itersolve(ffi_main, ffi_lib)

    def _setup_itersolve(self, ffi_main, ffi_lib):
        self.corexy_kins = kins = {'extra': {}, 'orig': {}}
        p = [b'+', b'-']
        for i, rail in enumerate(self.rails):
            for st in rail.get_steppers():
                name = st.get_name()
                kins['extra'][name] = ffi_main.gc(
                    ffi_lib.corexy_stepper_alloc(p[i]), ffi_lib.free)

    def dump_kinematics(self):
        for i, rail in enumerate(self.rails):
            for st in rail.get_steppers():
                name = st.get_name()
                self.corexy_kins['orig'][name] = st.get_stepper_kinematics()

    def set_kinematics(self, type='extra'):
        for i, rail in enumerate(self.rails):
            for st in rail.get_steppers():
                st.set_stepper_kinematics(
                    self.corexy_kins[type][st.get_name()])

    def set_extra_kinematics(self):
        for i, rail in enumerate(self.rails):
            for st in rail.get_steppers():
                name = st.get_name()
                self.corexy_kins['orig'][name] = \
                    st.set_stepper_kinematics(
                        self.corexy_kins['extra'][name])

    def set_orig_kinematics(self):
        for i, rail in enumerate(self.rails):
            for st in rail.get_steppers():
                st.set_stepper_kinematics(
                    self.corexy_kins['orig'][st.get_name()])

    def set_trapq(self, trapq):
        for rail in self.rails:
            rail.set_trapq(trapq)

    def set_position(self, coord):
        for rail in self.rails:
            rail.set_position(coord)

    def generate_steps(self, flush_time):
        for rail in self.rails:
            rail.generate_steps(flush_time)

    def debug_set_kin(self):
        self.dump_kinematics()
        # self.set_kinematics('extra')
        # self.set_trapq(self.trapq)
        # self.set_position((0., 0., 0.))
        # self.set_trapq(self.toolhead.get_trapq())
        # self.set_kinematics('orig')
        self._manual_move(25)
        self._manual_move(-25)

    def _manual_move(self, dist, speed=10., accel=5000.):
        toolhead = self.toolhead
        stepper = self.rails[0].get_steppers()[0]
        stepper1 = self.rails[1].get_steppers()[0]
        toolhead.flush_step_generation()
        prev_sk = stepper.set_stepper_kinematics(self.corexy_kins['extra'][stepper.get_name()])
        prev_sk1 = stepper1.set_stepper_kinematics(self.corexy_kins['extra'][stepper1.get_name()])
        # self.set_kinematics('extra')
        self.set_trapq(self.trapq)
        stepper.set_position((0., 0., 0.))
        stepper1.set_position((0., 0., 0.))
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(dist, speed, accel)
        axis_r1, *_ = force_move.calc_move_time(dist, speed, accel)
        print_time = toolhead.get_last_move_time()
        self.trapq_append(self.trapq, print_time, accel_t, cruise_t, accel_t,
                          0., 0., 0., axis_r, axis_r1, 0., 0., cruise_v, accel)
        print_time = print_time + accel_t + cruise_t + accel_t
        stepper.generate_steps(print_time)
        stepper1.generate_steps(print_time)
        self.trapq_finalize_moves(self.trapq, print_time + 99999.9,
                                  print_time + 99999.9)
        self.set_trapq(toolhead.get_trapq())
        # stepper.set_stepper_kinematics(prev_sk)
        # stepper1.set_stepper_kinematics(prev_sk1)
        self.set_kinematics('orig')
        toolhead.note_mcu_movequeue_activity(print_time)
        toolhead.dwell(accel_t + cruise_t + accel_t)
        toolhead.flush_step_generation()

    def multi_move(self, dists, speed=50., accel=5000.):
        toolhead = self.toolhead
        toolhead.flush_step_generation()
        self.set_extra_kinematics()
        self.set_trapq(self.trapq)
        self.set_position((0., 0., 0.))
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(dists[0], speed, accel)
        axis_r1, *_ = force_move.calc_move_time(dists[1], speed, accel)
        # axes_d = [dists[i] - 0 for i in (0, 1)]
        # inv_move_d = 1. / math.sqrt(sum([d * d for d in axes_d]))
        # axes_r = [d * inv_move_d for d in axes_d]
        print_time = toolhead.get_last_move_time()
        ##############################################################################################
        # now = self.printer.get_reactor().monotonic()
        # etime = toolhead.mcu.estimated_print_time(now)
        # print_time = etime + 0.2
        # self.gcode.respond_info(f"multi_move: axes_r={axes_r} delta_t={print_time-etime:.3f} print_time={print_time:.3f} etime={etime:.3f}")
        ##############################################################################################
        self.trapq_append(self.trapq, print_time, accel_t, cruise_t, accel_t,
                          0., 0., 0., axis_r, axis_r1, 0., 0., cruise_v, accel)
        # move = Move(toolhead, (0, 0, 0, 0), (*dists, 0, 0), speed)
        # move.set_junction(0, speed**2, 0)
        # self.gcode.respond_info(f"multi_move: start_pos={move.start_pos} "
        #                         f"end_pos={move.end_pos} accel_t={move.accel_t} "
        #                         f"cruise_t={move.cruise_t} decel_t={move.decel_t}")
        # self.trapq_append(
        #     self.trapq, print_time, move.accel_t, move.cruise_t,
        #     move.decel_t, move.start_pos[0], move.start_pos[1],
        #     move.start_pos[2], move.axes_r[0], move.axes_r[1],
        #     move.axes_r[2], move.start_v, move.cruise_v, move.accel)
        # move_time = move.accel_t + move.cruise_t + move.decel_t
        move_time = accel_t + cruise_t + accel_t
        print_time = print_time + move_time
        self.generate_steps(print_time)
        self.trapq_finalize_moves(self.trapq, print_time + 99999.9,
                                  print_time + 99999.9)
        self.set_trapq(toolhead.get_trapq())
        self.set_orig_kinematics()
        toolhead.note_mcu_movequeue_activity(print_time)
        toolhead.dwell(move_time)
        toolhead.flush_step_generation()

    # def _orig_manual_move(self, dist, speed=10., accel=5000.):
    #     toolhead = self.toolhead
    #     stepper = self.rails[0].get_steppers()[0]
    #     toolhead.flush_step_generation()
    #     prev_sk = stepper.set_stepper_kinematics(self.corexy_kins['extra'][stepper.get_name()])
    #     prev_trapq = stepper.set_trapq(self.trapq)
    #     stepper.set_position((0., 0., 0.))
    #     axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(dist, speed, accel)
    #     print_time = toolhead.get_last_move_time()
    #     self.trapq_append(self.trapq, print_time, accel_t, cruise_t, accel_t,
    #                       0., 0., 0., axis_r, 0., 0., 0., cruise_v, accel)
    #     print_time = print_time + accel_t + cruise_t + accel_t
    #     stepper.generate_steps(print_time)
    #     self.trapq_finalize_moves(self.trapq, print_time + 99999.9,
    #                               print_time + 99999.9)
    #     stepper.set_trapq(prev_trapq)
    #     stepper.set_stepper_kinematics(prev_sk)
    #     toolhead.note_mcu_movequeue_activity(print_time)
    #     toolhead.dwell(accel_t + cruise_t + accel_t)
    #     toolhead.flush_step_generation()


class PositionRestorer:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        self.force_move = printer.load_object(config, 'force_move')
        printer.register_event_handler("klippy:connect",
                                       self._handle_connect)
        self.trackers = []
        self.req_count = 0

    def _handle_connect(self):
        self.corexy_move_chelper = CorexyMoveChelper(self.printer)

    def add_tracker(self, tr):
        self.trackers.append(tr)

    def position_filer(self, tracker, dist):
        dir = 1 if dist >= 0 else -1
        rd, _ = tracker.mcu_stepper.get_rotation_distance()
        single_d = rd / tracker.stepper_fspr * 4
        return math.ceil(abs(dist / single_d)) * single_d * dir

    def single_move(self, tracker):
        # mcu_poss = tracker.mcu_stepper.get_mcu_position()
        # self.gcode.respond_info(f"start mcu_pos={mcu_poss}")
        dist, *_ = tracker.calc_position_diff()
        aligned_dist = self.position_filer(tracker, dist)
        self.gcode.respond_info(f"{tracker.sensor_name} move "
                                f"dist={aligned_dist:.3f}")
        self.force_move.manual_move(tracker.mcu_stepper,
                                    aligned_dist, 100, 5000)
        # mcu_poss = tracker.mcu_stepper.get_mcu_position()
        # self.gcode.respond_info(f"finish mcu_pos={mcu_poss}")
        tracker.reset_runout()
        tracker.resume_tracker()

    @staticmethod
    def corexy_to_cartesian(dx, dy):
        return 0.5*(dx+dy), 0.5*(dx-dy)

    # def tool_move(self, newpos, speed):
    #     toolhead = self.printer.lookup_object('toolhead')
    #     move = Move(toolhead, toolhead.commanded_pos, newpos, speed)
    #     if move.is_kinematic_move:
    #         toolhead.kin.check_move(move)
    #     toolhead.lookahead.add_move(move)
    #     if toolhead.print_time > toolhead.need_check_pause:
    #         toolhead._check_pause()

    # def trapq_move(self):
    #     accel = 10000
    #     axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(relpos[0], 200, accel)
    #     axis_r1, *_ = force_move.calc_move_time(relpos[0], 200, accel)
    #     print_time = toolhead.get_last_move_time()
    #     toolhead.trapq_append(toolhead.trapq, print_time, accel_t, cruise_t, accel_t, 0., 0., 0., axis_r, axis_r1, 0., 0., cruise_v, accel)
    #     print_time = print_time + accel_t + cruise_t + accel_t
    #     for st in trackers:
    #         st.mcu_stepper.generate_steps(print_time)
    #     toolhead.trapq_finalize_moves(toolhead.trapq, print_time + 99999.9,
    #                                   print_time + 99999.9)
    #     toolhead.note_mcu_movequeue_activity(print_time)
    #     toolhead.dwell(accel_t + cruise_t + accel_t)
    #     toolhead.flush_step_generation()
    #     mcu_poss = [tr.mcu_stepper.get_mcu_position() for tr in trackers]
    #     self.gcode.respond_info(f"mcu_pos={mcu_poss}")
    #     toolhead.wait_moves()
    # for i, tr in enumerate(trackers):
    #     # tr.mcu_offset += diagpos[i]
    #     # tr.mcu_stepper._set_mcu_position(mcu_poss[i])
    #     tr.reset_runout()
    #     tr.resume_tracker()

    def corexy_move(self):
        trackers = [tr for tr in self.trackers]
        if trackers[0].get_axis() != 'x':
            trackers.reverse()
        diagpos = [tr.calc_position_diff()[0] for tr in trackers]
        relpos = self.corexy_to_cartesian(*diagpos)
        relpos = [self.position_filer(tr, p) for p, tr in zip(relpos, trackers)]
        toolhead = self.printer.lookup_object('toolhead')
        self.gcode.respond_info(f"corexy move rel_x={relpos[0]:.3f} rel_y={relpos[1]:.3f}")
        # mcu_poss = [tr.mcu_stepper.get_mcu_position() for tr in trackers]
        # self.gcode.respond_info(f"start mcu_pos={mcu_poss}")
        self.corexy_move_chelper.multi_move(relpos)
        toolhead.wait_moves()
        # mcu_poss = [tr.mcu_stepper.get_mcu_position() for tr in trackers]
        # self.gcode.respond_info(f"finish mcu_pos={mcu_poss}")
        for i, tr in enumerate(trackers):
            tr.reset_runout()
            tr.resume_tracker()

    def restore_position(self, tracker):
        # tracker.pause_tracker()
        if not all(tr.is_runouted for tr in self.trackers):
            if self.req_count < 5:
                self.req_count += 1
                return
            self.req_count = 0
            self.single_move(tracker)
            return
        self.req_count = 0
        self.corexy_move()


class PositionTracker:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.gcode = gcode = printer.lookup_object('gcode')
        self.restore_chelper = printer.lookup_object('position_restorer')
        self.restore_chelper.add_tracker(self)
        # Register event handlers
        printer.register_event_handler("klippy:connect",
                                       self._handle_connect)
        printer.register_event_handler("klippy:shutdown",
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
        self.sensor_offset = 0.
        self.mcu_offset = 0.
        self.last_batch = []
        self.stepper = None
        self.axis = None
        self.is_running = False
        self.is_paused = False
        self.is_runouted = False
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

    def get_axis(self):
        return self.axis

    def reset_runout(self):
        self.is_runouted = False

    def normalize_encoder_pos(self, pos):
        rd, _ = self.mcu_stepper.get_rotation_distance()
        return rd / ((1 << 16) / (pos - self.sensor_offset + 1.))

    def _wait_moves(self, time):
        self.is_paused = True
        while time > 0:
            now = self.reactor.monotonic()
            self.reactor.pause(now + 0.010)
            time -= 0.010
        self.is_paused = False

    def set_sensor_offset(self):
        self.sensor_offset = self.last_batch[1]

    def set_stepper_offset(self):
        now = self.reactor.monotonic()
        etime = self.toolhead.mcu.estimated_print_time(now)
        mcu_pos = self.mcu_stepper.get_past_mcu_position(etime)
        self.mcu_offset = self.mcu_stepper.mcu_to_commanded_position(mcu_pos)
        # self.mcu_offset = self.mcu_stepper.get_past_mcu_position(self.last_batch[0])

    def calc_position_diff(self):
        sens_ptime, last_position = self.last_batch
        sens_pos = self.normalize_encoder_pos(last_position)
        mcu_past_pos = self.mcu_stepper.get_past_mcu_position(sens_ptime) # + self.mcu_offset
        req_pos = self.mcu_stepper.mcu_to_commanded_position(mcu_past_pos) - self.mcu_offset
        self.resp_clock += 1
        if self.resp_clock == 10:
            self.resp_clock = 0
            mcu_pos = self.mcu_stepper.get_mcu_position()
            mcu_pos_offset = self.mcu_stepper._mcu_position_offset
            mcu_com_pos = self.mcu_stepper.get_commanded_position()
            self.gcode.respond_info(
                f"{self.sensor_name}: "
                f"delta_pos={req_pos-sens_pos:.3f}, "
                f"req_pos={req_pos:.3f}, "
                f"sens_pos={sens_pos:.3f}, "
                # f"sens_ptime={sens_ptime:.3f} "
                f"mcu_past_pos={mcu_past_pos:.3f} "
                f"mcu_pos={mcu_pos:.3f} "
                f"mcu_pos_offset={mcu_pos_offset:.3f} "
                f"mcu_com_pos={mcu_com_pos:.3f}")
        return req_pos - sens_pos, req_pos, sens_pos, sens_ptime

    def _batch_handle(self, samples):
        self.last_batch = samples['data'][-1]
        if not self.is_running:
            return False
        if self.is_paused:
            return True
        diff_d, req_pos, sens_pos, sens_ptime = self.calc_position_diff()
        if abs(diff_d) < self.max_difference:
            return True
        reactor = self.printer.get_reactor()
        now = reactor.monotonic()
        est_print_time = self.toolhead.mcu.estimated_print_time(now)
        # msg = (f'Skipped steps detected on {self.sensor_name}: '
        #        f'required_pos={req_pos:.3f} sensor_pos={enc_pos:.3f} '
        #        f'etime={est_print_time:.3f} sensor_time={enc_ptime:.3f}')
        # return True
        msg = (f'{self.sensor_name}: '
               f'delta_pos={diff_d:.3f} '
               f'req_pos={req_pos:.3f} '
               f'sens_pos={sens_pos:.3f} '
               f'event_ptime={est_print_time:.3f} '
               f'sens_ptime={sens_ptime:.3f}')
        if self.runout_gcode:
            self.gcode.respond_info(msg)
            self.is_runouted = True
            self.restore_chelper.restore_position(self)
            # self.gcode.run_script(self.runout_gcode.render() + "\nM400")
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
        self.set_sensor_offset()
        self.set_stepper_offset()
        self.gcode.respond_info(f"Offset {self.sensor_name}: "
                                f"enc_offset={self.sensor_offset} "
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
        # self.restore_chelper.corexy_move_chelper.debug_set_kin()
        self.gcode.respond_info(f"Position tracker {self.sensor_name}: "
                                f"{self.get_status(None)['status']}")


def load_config_prefix(config):
    printer = config.get_printer()
    tr_res = printer.lookup_object('position_restorer', None)
    if tr_res is None:
        printer.add_object('position_restorer', PositionRestorer(config))
    return PositionTracker(config)
