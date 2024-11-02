# Copyright (C)
# 2016-2024  Kevin O'Connor <kevin@koconnor.net>
# 2024 i8086m "IEOX" <i8086m@gmail.com>
# 2024 Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import time, math, re
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib import ticker
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# Enum
ARC_PLANE_X_Y = 0
ARC_PLANE_X_Z = 1
ARC_PLANE_Y_Z = 2

# Enum
X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2
E_AXIS = 3

class ArcParser:
    def __init__(self):
        # self.printer = config.get_printer()
        self.mm_per_arc_segment = 0.1
        self.plane = ARC_PLANE_X_Y

    def G2(self, i=0.0, j=-15.0, x=0, y=0, old_x=0, old_y=0, clockwise=True):
        # Parse parameters
        asTarget = [x, y, 0]
        I = i
        J = j
        asPlanar = (I, J)
        axes = (X_AXIS, Y_AXIS, Z_AXIS)
        # Build linear coordinates to move
        return self.planArc((old_x, old_y, 0), asTarget, asPlanar, clockwise, *axes)

    def planArc(self, currentPos, targetPos, offset, clockwise, alpha_axis, beta_axis, helical_axis):
        # todo: sometimes produces full circles
        # Radius vector from center to current location
        r_P = -offset[0]
        r_Q = -offset[1]
        # Determine angular travel
        center_P = currentPos[alpha_axis] - r_P
        center_Q = currentPos[beta_axis] - r_Q
        rt_Alpha = targetPos[alpha_axis] - center_P
        rt_Beta = targetPos[beta_axis] - center_Q
        angular_travel = math.atan2(r_P * rt_Beta - r_Q * rt_Alpha,
                                    r_P * rt_Alpha + r_Q * rt_Beta)
        if angular_travel < 0.:
            angular_travel += 2. * math.pi
        if clockwise:
            angular_travel -= 2. * math.pi
        if (angular_travel == 0.
                and currentPos[alpha_axis] == targetPos[alpha_axis]
                and currentPos[beta_axis] == targetPos[beta_axis]):
            # Make a circle if the angular rotation is 0 and the
            # target is current position
            angular_travel = 2. * math.pi
        # Determine number of segments
        linear_travel = targetPos[helical_axis] - currentPos[helical_axis]
        radius = math.hypot(r_P, r_Q)
        flat_mm = radius * angular_travel
        if linear_travel:
            mm_of_travel = math.hypot(flat_mm, linear_travel)
        else:
            mm_of_travel = math.fabs(flat_mm)
        segments = max(1., math.floor(mm_of_travel / self.mm_per_arc_segment))
        # Generate coordinates
        theta_per_segment = angular_travel / segments
        linear_per_segment = linear_travel / segments
        asE = None
        asF = None
        e_per_move = e_base = 0.
        if asE is not None:
            e_per_move = (asE - e_base) / segments
        out = []
        for i in range(1, int(segments) + 1):
            dist_Helical = i * linear_per_segment
            c_theta = i * theta_per_segment
            cos_Ti = math.cos(c_theta)
            sin_Ti = math.sin(c_theta)
            r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti
            r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti

            c = [None, None, None]
            c[alpha_axis] = center_P + r_P
            c[beta_axis] = center_Q + r_Q
            c[helical_axis] = currentPos[helical_axis] + dist_Helical
            if i == segments:
                c = targetPos
            # Convert coords into G1 commands
            g1_params = {'X': c[0], 'Y': c[1], 'Z': c[2]}
            if e_per_move:
                g1_params['E'] = e_base + e_per_move
            if asF is not None:
                g1_params['F'] = asF
            # g1_gcmd = self.gcode.create_gcode_command("G1", "G1", g1_params)
            out.append((g1_params["X"], g1_params["Y"]))
        return out


class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        velocity = min(speed, toolhead.max_velocity)
        self.axes_d = [end_pos[i] - start_pos[i] for i in (0, 1)]
        self.move_d = math.sqrt(sum(d * d for d in self.axes_d))
        if self.move_d < 1e-9:
            return
        inv_move_d = 1. / self.move_d
        self.axes_r = [d * inv_move_d for d in self.axes_d]
        self.accel = toolhead.max_accel
        self.junction_deviation = toolhead.junction_deviation
        self.max_start_v2 = 0.
        self.max_cruise_v2 = velocity ** 2
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.max_smoothed_v2 = 0.
        self.smooth_delta_v2 = 2.0 * self.move_d * toolhead.max_accel_to_decel

    def calc_junction(self, prev_move):
        axes_r, prev_axes_r = self.axes_r, prev_move.axes_r
        cos_theta = max(-(axes_r[0] * prev_axes_r[0] + axes_r[1] * prev_axes_r[1]), -0.999999)
        if math.acos(cos_theta) > math.radians(90):
            sin_theta_d2 = min(max(math.sqrt(0.5 * (1. - round(
                cos_theta * self.toolhead.scv_coeff, 5))), 0.000001), 0.999999)
        else:
            sin_theta_d2 = min(max(math.sqrt(0.5 * (1. - round(cos_theta, 5))), 0.000001), 0.999999)
        R_jd = sin_theta_d2 / (1. - sin_theta_d2)
        tan_theta_d2 = sin_theta_d2 / math.sqrt(0.5 * (1.0 + cos_theta))
        move_centripetal_v2 = .5 * self.move_d * tan_theta_d2 * self.accel
        prev_move_centripetal_v2 = .5 * prev_move.move_d * tan_theta_d2 * prev_move.accel
        self.max_start_v2 = min(
            R_jd * self.junction_deviation * self.accel,
            R_jd * prev_move.junction_deviation * prev_move.accel,
            move_centripetal_v2, prev_move_centripetal_v2,
            self.max_cruise_v2, prev_move.max_cruise_v2,
            prev_move.max_start_v2 + prev_move.delta_v2)
        self.max_smoothed_v2 = min(self.max_start_v2, prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2)

    def set_junction(self, start_v2, cruise_v2, end_v2):
        global avgspeed
        def split_move(start, end, full_time, start_time, end_time):
            from_x = start[0] + (start_time / full_time) * (end[0] - start[0])
            from_y = start[1] + (start_time / full_time) * (end[1] - start[1])
            to_x = start[0] + (end_time / full_time) * (end[0] - start[0])
            to_y = start[1] + (end_time / full_time) * (end[1] - start[1])
            return (from_x, from_y), (to_x, to_y)
        # Determine accel, cruise, and decel portions of the move distance
        half_inv_accel = .5 / self.accel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_accel
        cruise_d = self.move_d - accel_d - decel_d
        # Determine move velocities
        start_v = math.sqrt(start_v2)
        cruise_v = math.sqrt(cruise_v2)
        end_v = math.sqrt(end_v2)
        # Determine time spent in each portion of move (time is the
        # distance divided by average velocity)
        accel_t = accel_d / ((start_v + cruise_v) * 0.5)
        cruise_t = cruise_d / cruise_v
        decel_t = decel_d / ((end_v + cruise_v) * 0.5)
        full_t = accel_t + cruise_t + decel_t
        avgspeed.append(self.move_d / full_t)
        accel_move = split_move(self.start_pos, self.end_pos, full_t, 0, full_t * accel_d / self.move_d)
        cruise_move = split_move(self.start_pos, self.end_pos, full_t, full_t * accel_d / self.move_d, full_t * (accel_d + cruise_d) / self.move_d)
        decel_move = split_move(self.start_pos, self.end_pos, full_t, full_t * (accel_d + cruise_d) / self.move_d, full_t)
        return [(accel_move, (start_v, cruise_v)), (cruise_move, (cruise_v, cruise_v)), (decel_move, (cruise_v, end_v))]


class LookAheadQueue:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.queue = []
        self.output = []

    def flush(self, lazy=False):
        update_flush_count = lazy
        queue = self.queue
        flush_count = len(queue)
        # Traverse queue from last to first move and determine maximum
        # junction speed assuming the robot comes to a complete stop
        # after the last move.
        delayed = []
        next_end_v2 = next_smoothed_v2 = peak_cruise_v2 = 0.
        for i in range(flush_count - 1, -1, -1):
            move = queue[i]
            reachable_start_v2 = next_end_v2 + move.delta_v2
            start_v2 = min(move.max_start_v2, reachable_start_v2)
            reachable_smoothed_v2 = next_smoothed_v2 + move.smooth_delta_v2
            smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)
            if smoothed_v2 < reachable_smoothed_v2:
                # It's possible for this move to accelerate
                if (smoothed_v2 + move.smooth_delta_v2 > next_smoothed_v2
                        or delayed):
                    # This move can decelerate or this is a full accel
                    # move after a full decel move
                    if update_flush_count and peak_cruise_v2:
                        flush_count = i
                        update_flush_count = False
                    peak_cruise_v2 = min(move.max_cruise_v2, (
                            smoothed_v2 + reachable_smoothed_v2) * .5)
                    if delayed:
                        # Propagate peak_cruise_v2 to any delayed moves
                        if not update_flush_count and i < flush_count:
                            mc_v2 = peak_cruise_v2
                            for m, ms_v2, me_v2 in reversed(delayed):
                                mc_v2 = min(mc_v2, ms_v2)
                                res = m.set_junction(min(ms_v2, mc_v2), mc_v2, min(me_v2, mc_v2))
                                self.output.append(res)
                        del delayed[:]
                if not update_flush_count and i < flush_count:
                    cruise_v2 = min((start_v2 + reachable_start_v2) * .5
                                    , move.max_cruise_v2, peak_cruise_v2)
                    res = move.set_junction(min(start_v2, cruise_v2), cruise_v2, min(next_end_v2, cruise_v2))
                    self.output.append(res)
            else:
                # Delay calculating this move until peak_cruise_v2 is known
                delayed.append((move, start_v2, next_end_v2))
            next_end_v2 = start_v2
            next_smoothed_v2 = smoothed_v2
        if update_flush_count or not flush_count:
            return
        # Remove processed moves from the queue
        del queue[:flush_count]

    def add_move(self, move):
        self.queue.append(move)
        if len(self.queue) > 1:
            move.calc_junction(self.queue[-2])


class ToolHead:
    def __init__(self, max_velocity=500, max_accel=5000, mcr=0.5, scv=5.0, scv_coeff=1.0):
        self.lookahead = LookAheadQueue(self)
        self.commanded_pos = [0.0, 0.0]
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.min_cruise_ratio = mcr
        self.square_corner_velocity = scv
        self.junction_deviation = self.max_accel_to_decel = 0.0
        self._calc_junction_deviation()
        self.scv_coeff = scv_coeff

    def flush_lookahead(self):
        self.lookahead.flush()

    def set_position(self, new_pos):
        self.flush_lookahead()
        self.commanded_pos[:] = new_pos

    def move(self, new_pos, speed):
        move = Move(self, self.commanded_pos, new_pos, speed)
        if move.move_d:
            self.commanded_pos[:] = move.end_pos
            self.lookahead.add_move(move)

    def _calc_junction_deviation(self):
        scv2 = self.square_corner_velocity ** 2
        self.junction_deviation = scv2 * (math.sqrt(2.) - 1.) / self.max_accel
        self.max_accel_to_decel = self.max_accel * (1. - self.min_cruise_ratio)


def circle(polygons=100, radius=15, x_center=0, y_center=0):
    thetas = np.linspace(0, 2 * np.pi, polygons + 1, endpoint=True)
    x_coords = radius * np.cos(thetas) + x_center
    y_coords = radius * np.sin(thetas) + y_center
    return np.column_stack((x_coords, y_coords))

def parse_gcode(gcode, stop=0.):
    z = 0.
    old_z = 0.
    coords = {}
    coords[z] = []
    arcs = []
    z_pattern = re.compile(r'^G1.*\sZ([-+]?\d*\.?\d+)')
    g1_pattern = re.compile(r'^G1\s+(?:F[-+]?\d*\.?\d+\s+)'
                            r'?X([-+]?\d*\.?\d+)\s+Y([-+]?\d*\.?\d+)')
    arc_pattern = re.compile(r'(G[23])\s+X([-+]?\d*\.?\d+)\s+Y([-+]?\d*\.?\d+)'
                             r'(?:\s+I([-+]?\d*\.?\d+))?(?:\s+J([-+]?\d*\.?\d+))?')
    for line in tqdm(gcode, desc='\033[97mParsing GCode'):
        z_match = re.search(z_pattern, line)
        if z_match:
            old_z = z
            z = float(z_match.group(1))
            if stop and z > stop:
                break
            if z not in coords:
                coords[z] = []
        g1_match = re.search(g1_pattern, line)
        if g1_match:
            coords[z].append([float(g1_match.group(1)),
                              float(g1_match.group(2))])
            continue
        arc_match = re.search(arc_pattern, line)
        if arc_match:
            command = arc_match.group(1)
            try:
                x, y = coords[z][-1]
            except:
                x, y = coords[old_z][-1]
            x1 = float(arc_match.group(2))
            y1 = float(arc_match.group(3))
            i = float(arc_match.group(4)) if arc_match.group(4) else 0.
            j = float(arc_match.group(5)) if arc_match.group(5) else 0.
            arcs.append([i, j])
            clockwise = command == 'G2'
            arc_pos = ArcParser().G2(i=i, j=j, x=x1, y=y1, old_x=x, old_y=y, clockwise=clockwise)
            coords[z].extend(arc_pos[:-1])
    return coords, np.array(arcs)

def main(path='./', max_velocity=250, max_accel=15000, mcr=0, scv=5., scv_coeff=1.):
    global avgspeed
    if not path:
        path = input('Enter the path to the gcode file: ')
    avgspeed = []
    toolhead = ToolHead(max_velocity, max_accel, mcr, scv, scv_coeff)
    start_tm = time.perf_counter()
    with open(path, 'r') as file:
        lines = file.readlines()
    moves, arcs = parse_gcode(lines, stop=0)
    print(f'GCode file: {path.rsplit("/", 1)[1]}')
    print(f'Total arcs: {arcs.shape[0]}')
    print(f'Total polygons: {len([pos for mas in moves.values() for pos in mas])}')
    flushed_moves = {}
    for layer in tqdm(moves, leave=False, desc=f'Processing movements'):
        if moves[layer]:
            toolhead.lookahead.output = []
            toolhead.set_position(moves[layer][0])
            for _, coord in enumerate(moves[layer]):
                toolhead.move(coord, 500)
            toolhead.flush_lookahead()
            flushed_moves[layer] = toolhead.lookahead.output[::-1]
    avg_spd = np.array(avgspeed).mean()
    print(f"Total time: {time.perf_counter() - start_tm:.3f}")
    msg = (f'Max Speed: {max_velocity} mm/s\n'
           f'Acceleration: {max_accel} mm/s²\n'
           f'Min Cruise Ratio: {mcr}\n'
           f'Square Corner Velocity: {scv} mm/s\n'
           f'Average Speed: {avg_spd:.2f} mm/s')
    print(msg + '\n')
    moves_max_velocity = np.floor(
        max(spd[1][1][1] for layer in flushed_moves.values() for spd in layer))
    plot_data = prepare_plot_data(flushed_moves, moves_max_velocity)
    # plot(plot_data, max_velocity, max_accel, mcr, scv, avg_spd, moves_max_velocity)
    plot_3d(plot_data, max_velocity, max_accel, mcr, scv, avg_spd, moves_max_velocity)

def prepare_plot_data(flushed_moves, moves_max_velocity, max_points=5):
    segments = []
    cmap = plt.get_cmap('plasma')
    for _, (z_value, moves) in enumerate(flushed_moves.items(), start=1):
        for move in tqdm(moves, leave=False, desc=f'Processing layer {_} of {len(flushed_moves)}'):
            for part_move in move:
                color_start, color_end = part_move[1] / moves_max_velocity
                [x1, y1], [x2, y2] = part_move[0]
                if color_start != color_end:
                    x = np.linspace(x1, x2, max_points)
                    y = np.linspace(y1, y2, max_points)
                    gradient_colors = np.linspace(color_start, color_end, max_points)
                    colors = cmap(gradient_colors)[:, :3]
                    for i in range(max_points - 1):
                        segments.append([[x[i], x[i+1]], [y[i], y[i+1]], z_value, colors[i]])
                else:
                    gradient_colors = np.linspace(color_start, color_end, 1)
                    colors = cmap(gradient_colors)[:, :3]
                    segments.append([[x1, x2], [y1, y2], z_value, colors])
    return np.array(segments, dtype=object)

def plot(data, max_velocity, max_accel, min_cruise_ratio,
         square_corner_velocity, avg_spd, moves_max_velocity, feedback=True):
    fig, ax = plt.subplots(figsize=(14, 14), facecolor='#f4f4f4')
    ax.set_xlabel('X, mm', fontsize=14, labelpad=10, color='#333333')
    ax.set_ylabel('Y, mm', fontsize=14, labelpad=10, color='#333333')
    norm = Normalize(vmin=0, vmax=moves_max_velocity)
    cmap = plt.get_cmap('plasma')
    if feedback:
        print('Graph generation')
    # ax.scatter(data[:, 0], data[:, 1], color=data[:, 3], lw=5)
    for moves in data:
        ax.plot([moves[0][0], moves[0][1]], [moves[1][0], moves[1][1]], 0, color=moves[3], lw=5)
    sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    cbar = plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.02, aspect=30, shrink=0.8)
    cbar.locator = ticker.MaxNLocator(nbins=10)
    cbar.set_label('Speed, mm/s', fontsize=12, color='#333333')
    cbar.ax.yaxis.set_tick_params(color='#333333')
    plt.figtext(0.5, 0.04,
                f'Max Speed: {max_velocity} mm/s | '
                f'Acceleration: {max_accel} mm/s² | '
                f'Min Cruise Ratio: {min_cruise_ratio} | '
                f'Square Corner Velocity: {square_corner_velocity} mm/s | '
                f'Average Speed: {avg_spd:.2f} mm/s',
                # f'Cruise Speed: {cruise_spd:.2f} mm/s',
                ha='center', fontsize=12,
                bbox={'facecolor': 'white', 'alpha': 0.8, 'pad': 10}, color='#000000')
    # ax.autoscale()
    xd, yd = np.array(list(zip(*[[a[0][0], a[1][0]] for a in data])))
    ax.set_xlim(xd.min()-1, xd.max()+1)
    ax.set_ylim(yd.min()-1, yd.max()+1)
    # ax.set_zlim(data[:, 4].min(), data[:, 4].max())
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()
    if feedback:
        print('Plot2d was created')

def plot_3d(data, max_velocity, max_accel, min_cruise_ratio,
         square_corner_velocity, avg_spd, moves_max_velocity, feedback=True):
    fig = plt.figure(figsize=(14, 14))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X, mm', fontsize=14, labelpad=10, color='#333333')
    ax.set_ylabel('Y, mm', fontsize=14, labelpad=10, color='#333333')
    ax.set_zlabel('Z, mm', fontsize=14, labelpad=10, color='#333333')
    norm = Normalize(vmin=0, vmax=moves_max_velocity)
    cmap = plt.get_cmap('plasma')
    if feedback:
        print('Graph generation')
    # ax.scatter(data[:, 0], data[:, 1], data[:, 2], color=data[:, 3:6], lw=5)
    for moves in data:
        ax.plot([moves[0][0], moves[0][1]], [moves[1][0], moves[1][1]], moves[2], color=moves[3], lw=5)
    sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    cbar = plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.02, aspect=30, shrink=0.8)
    cbar.locator = ticker.MaxNLocator(nbins=10)
    cbar.set_label('Speed, mm/s', fontsize=12, color='#333333')
    cbar.ax.yaxis.set_tick_params(color='#333333')
    plt.figtext(0.5, 0.04,
                f'Max Speed: {max_velocity} mm/s | '
                f'Acceleration: {max_accel} mm/s² | '
                f'Min Cruise Ratio: {min_cruise_ratio} | '
                f'Square Corner Velocity: {square_corner_velocity} mm/s | '
                f'Average Speed: {avg_spd:.2f} mm/s',
                ha='center', fontsize=12,
                bbox={'facecolor': 'white', 'alpha': 0.8, 'pad': 10}, color='#000000')
    ax.autoscale()
    # ax.set_zlim(0, max(flushed_moves.keys()) * 1)
    plt.tight_layout()
    plt.show()
    if feedback:
        print('Plot3d was created')

def scv_viewer(need_plot=False):
    global avgspeed
    avgspeed = []
    max_velocity = 100
    max_accel = 10000
    mcr = 0.
    scv = 5.
    scv_coeff = 1.
    start = (100, 0)
    radius = 100
    angles = range(0, 181)
    coords = []
    for angle in angles:
        angle = math.radians(angle)
        new_x = start[0] + radius * math.cos(angle)
        new_y = start[1] + radius * math.sin(angle)
        coords.append((new_x, new_y))
    toolhead = ToolHead(max_velocity, max_accel, mcr, scv, scv_coeff)
    real_scvs = []
    for angle, (x, y) in zip(angles, coords):
        toolhead.lookahead.output = []
        toolhead.set_position([0, 0])
        toolhead.move(start, 500)
        toolhead.move([x, y], 500)
        toolhead.flush_lookahead()
        flushed_moves = toolhead.lookahead.output[::-1]
        real_scv = flushed_moves[0][2][1][1]
        real_scvs.append(real_scv)
        rad = math.radians(angle)
        x_сoeff = abs(math.cos(rad))
        y_сoeff = abs(math.sin(rad))
        print(f"Angle: {angle}°, coords: {start[:2]} --> ({x:.2f}, {y:.2f}),"
              f" speed: {real_scv:.2f}, x_speed: {(x_сoeff * real_scv):.2f},"
              f" y_speed: {(y_сoeff * real_scv):.2f} of 250 mm/s")
        if need_plot:
            moves_max_velocity = np.float64(flushed_moves[0][1][1][0])
            plot_data = prepare_plot_data(
                {'0': flushed_moves}, moves_max_velocity, max_points=15)
            plot(plot_data, max_velocity, max_accel, mcr, scv,
                 np.array(avgspeed).mean(), moves_max_velocity, False)
        avgspeed.clear()
    angles_aprox = np.linspace(min(angles), max(angles), num=9999)
    scv_aprox = np.interp(angles_aprox, angles, real_scvs)
    points = np.array([angles_aprox, scv_aprox]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = Normalize(vmin=min(scv_aprox), vmax=max_velocity)
    lc = LineCollection(segments, cmap=plt.get_cmap('plasma'), norm=norm)
    lc.set_array(scv_aprox)
    lc.set_linewidth(2)
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.add_collection(lc)
    ax.set_title('SCV speed on angle')
    ax.set_xlabel('Angle')
    ax.set_ylabel('Speed, mm/s')
    ax.set_xlim(min(angles), max(angles))
    ax.set_ylim(0, max_velocity)
    ax.xaxis.set_major_locator(plt.MultipleLocator(10))
    ax.xaxis.set_minor_locator(plt.MultipleLocator(5))
    ax.yaxis.set_major_locator(plt.MultipleLocator(5))
    ax.yaxis.set_minor_locator(plt.MultipleLocator(2.5))
    ax.grid(which='major', color='gray', linestyle='-', linewidth=0.7)
    ax.grid(which='minor', color='gray', linestyle=':', linewidth=0.5)
    # fig.colorbar(lc, ax=ax, label="Speed, mm/s")
    light_angles = [45, 90, 135]
    light_scvs = np.interp(light_angles, angles_aprox, scv_aprox)
    ax.plot(light_angles, light_scvs, 'o', color='red', markersize=5)
    plt.figtext(
        0.5, 0.025,
        f'Max Speed: {max_velocity:.2f} mm/s | '
        f'Acceleration: {max_accel} mm/s² | '
        f'Min Cruise Ratio: {mcr} | '
        f'Square Corner Velocity: {scv} mm/s | '
        f'Average Speed: {np.mean(scv_aprox):.2f} mm/s',
        ha='center', fontsize=9,
        bbox={'facecolor': 'white', 'alpha': 0.8, 'pad': 10},
        color='#000000')
    plt.show()

if __name__ == '__main__':
    # scv_viewer()
    # exit()
    main()
