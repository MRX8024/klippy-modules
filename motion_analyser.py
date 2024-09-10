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

        sin_theta_d2 = min(max(math.sqrt(0.5 * (1.0 - round(cos_theta, 5))), 0.000001), 0.999999)
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
        avgspeed = np.append(avgspeed, self.move_d / full_t)
        accel_move = split_move(self.start_pos, self.end_pos, full_t, 0, full_t * accel_d / self.move_d)
        cruise_move = split_move(self.start_pos, self.end_pos, full_t, full_t * accel_d / self.move_d, full_t * (accel_d + cruise_d) / self.move_d)
        decel_move = split_move(self.start_pos, self.end_pos, full_t, full_t * (accel_d + cruise_d) / self.move_d, full_t)
        return [(accel_move, (start_v, cruise_v)), (cruise_move, (cruise_v, cruise_v)), (decel_move, (cruise_v, end_v))]


class LookAheadQueue:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.queue = []
        self.output = []

    def add_to_output(self, res):
        self.output = res + self.output

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
                                self.add_to_output(res)
                        del delayed[:]
                if not update_flush_count and i < flush_count:
                    cruise_v2 = min((start_v2 + reachable_start_v2) * .5
                                    , move.max_cruise_v2, peak_cruise_v2)
                    res = move.set_junction(min(start_v2, cruise_v2), cruise_v2, min(next_end_v2, cruise_v2))
                    self.add_to_output(res)
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
    def __init__(self, max_velocity=500, max_accel=5000, mcr=0.5, scv=5.0):
        self.lookahead = LookAheadQueue(self)
        self.commanded_pos = [0.0, 0.0]
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.min_cruise_ratio = mcr
        self.square_corner_velocity = scv
        self.junction_deviation = self.max_accel_to_decel = 0.0
        self._calc_junction_deviation()

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

def parse_gcode(gcode, arc_convert):
    z = 0.
    coords = {}
    arcs = []
    z_pattern = re.compile(r'^G1\s+Z([-+]?\d*\.?\d+)')
    g1_pattern = re.compile(r'^G1\s+(?:F[-+]?\d*\.?\d+\s+)'
                            r'?X([-+]?\d*\.?\d+)\s+Y([-+]?\d*\.?\d+)')
    arc_pattern = re.compile(r'(G[23])\s+X([-+]?\d*\.?\d+)\s+Y([-+]?\d*\.?\d+)'
                             r'(?:\s+I([-+]?\d*\.?\d+))?(?:\s+J([-+]?\d*\.?\d+))?')
    for line in tqdm(gcode, desc='\033[97mParsing GCode'):
        z_match = re.search(z_pattern, line)
        if z_match:
            z = float(z_match.group(1))
            continue
        g1_match = re.search(g1_pattern, line)
        if g1_match:
            if z not in coords:
                coords[z] = []
            coords[z].append([float(g1_match.group(1)),
                              float(g1_match.group(2))])
            continue
        arc_match = re.search(arc_pattern, line)
        if arc_match:
            command = arc_match.group(1)
            x, y = coords[z][-1]
            x1 = float(arc_match.group(2))
            y1 = float(arc_match.group(3))
            i = float(arc_match.group(4)) if arc_match.group(4) else 0.
            j = float(arc_match.group(5)) if arc_match.group(5) else 0.
            arcs.append([i, j])
            clockwise = command == 'G2'
            arc_pos = arc_convert.G2(i=i, j=j, x=x1, y=y1, old_x=x, old_y=y, clockwise=clockwise)
            coords[z].extend(arc_pos[:-1])
    return coords, np.array(arcs)

def main(path='C:/Users/nikit/Downloads/Telegram Desktop/SC_ArcLowres.gcode', max_velocity=500, max_accel=15000, mcr=0.5, scv=5.0):
    global avgspeed
    if not path:
        path = input('Enter the path to the gcode file: ')
    avgspeed = np.array([])
    toolhead = ToolHead(max_velocity, max_accel, mcr, scv)
    arc = ArcParser()
    start_tm = time.perf_counter()
    # toolhead.set_position((15, 0))
    # for _, coord in enumerate(circle_coords):
    #     toolhead.move(coord, 500)
    #     if _ % 100 == 1:
    #         print(f"Progress: {_ / (polygons / 100)}%")
    with open(path, 'r') as file:
        lines = file.readlines()
    moves, arcs = parse_gcode(lines, arc)
    # lenn = round(moves.shape[0] / 2.05)
    # moves = moves[lenn:-lenn]
    print(f'Total arcs: {arcs.shape[0]}')
    print(f'Total polygons: {len([pos for mas in moves.values() for pos in mas])}')
    flushed_moves = {}
    for layer in moves:
        toolhead.lookahead.output = []
        toolhead.set_position(moves[layer][0])
        for _, coord in enumerate(moves[layer]):
            toolhead.move(coord, 500)
            # if _ % 10000 == 1:
            #     print(f"Progress: {_ / (polygons / 100):.2f}%")
        toolhead.flush_lookahead()
        # flushed_moves.append(toolhead.lookahead.output)
        flushed_moves[layer] = toolhead.lookahead.output
    # cruise_spd = (sum(avgspeed[len(avgspeed) // 5:-len(avgspeed) // 5])
    #               / len(avgspeed[len(avgspeed) // 5:-len(avgspeed) // 5]))
    avg_spd = avgspeed.mean()
    print(f"Total time: {time.perf_counter() - start_tm:.3f}")
    msg = (f'Max Speed: {max_velocity} mm/s\n'
           f'Acceleration: {max_accel} mm/s²\n'
           f'Min Cruise Ratio: {mcr}\n'
           f'Square Corner Velocity: {scv} mm/s\n'
           f'Average Speed: {avg_spd:.2f} mm/s')
    print(msg + '\n')
    # create = input('Create a plot? ').lower()
    # if create in ['y', 'yes']:
    # plot(flushed_moves, max_velocity, max_accel, mcr, scv, avg_spd)
    plot_3d(flushed_moves, max_velocity, max_accel, mcr, scv, avg_spd)

def plot(flushed_moves, max_velocity, max_accel, min_cruise_ratio,
         square_corner_velocity, avg_spd):
    fig, ax = plt.subplots(figsize=(14, 14), facecolor='#f4f4f4')
    ax.set_xlabel('X, mm', fontsize=14, labelpad=10, color='#333333')
    ax.set_ylabel('Y, mm', fontsize=14, labelpad=10, color='#333333')
    moves_max_velocity = np.floor(max(max(spd[1][0], spd[1][1]) for spd in flushed_moves))
    sm = ScalarMappable(cmap='plasma', norm=Normalize(vmin=0, vmax=moves_max_velocity))
    sm.set_array([])
    print('Graph generation, wait')
    max_points = 100
    for move in flushed_moves:
        color_start, color_end = move[1] / moves_max_velocity
        # Plot_gradient_line
        [x1, y1], [x2, y2] = move[0]
        x = np.linspace(x1, x2, max_points)
        y = np.linspace(y1, y2, max_points)
        gradient_colors = np.linspace(color_start, color_end, max_points)
        colormap = plt.get_cmap('plasma')
        colors = colormap(gradient_colors)[:, :3]
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, colors=colors, linewidth=5)
        ax.add_collection(lc)
    plt.figtext(0.5, 0.04,
                f'Max Speed: {max_velocity} mm/s | '
                f'Acceleration: {max_accel} mm/s² | '
                f'Min Cruise Ratio: {min_cruise_ratio} | '
                f'Square Corner Velocity: {square_corner_velocity} mm/s | '
                f'Average Speed: {avg_spd:.2f} mm/s',
                # f'Cruise Speed: {cruise_spd:.2f} mm/s',
                ha='center', fontsize=12,
                bbox={'facecolor': 'white', 'alpha': 0.8, 'pad': 10}, color='#000000')
    cbar = plt.colorbar(sm, ax=ax, orientation='vertical', pad=0.02, aspect=30, shrink=0.8)
    cbar.locator = ticker.MaxNLocator(nbins=10)
    cbar.set_label('Speed, mm/s', fontsize=12, color='#333333')
    cbar.ax.yaxis.set_tick_params(color='#333333')
    ax.autoscale()
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()
    print('Plot was created')

def plot_3d(flushed_moves, max_velocity, max_accel, min_cruise_ratio,
         square_corner_velocity, avg_spd, max_points=2):
    fig = plt.figure(figsize=(14, 14))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X, mm', fontsize=14, labelpad=10, color='#333333')
    ax.set_ylabel('Y, mm', fontsize=14, labelpad=10, color='#333333')
    ax.set_zlabel('Z, mm', fontsize=14, labelpad=10, color='#333333')
    moves_max_velocity = np.floor(
        max(max(spd[1][0], spd[1][1]) for layer in flushed_moves.values() for spd in layer))
    norm = Normalize(vmin=0, vmax=moves_max_velocity)
    cmap = plt.get_cmap('plasma')
    print('Graph generation')
    for _, (z_value, moves) in enumerate(flushed_moves.items(), start=1):
        for move in tqdm(moves, leave=False, desc=f'Processing layer {_} of {len(flushed_moves)}'):
            color_start, color_end = move[1] / moves_max_velocity
            [x1, y1], [x2, y2] = move[0]
            x = np.linspace(x1, x2, max_points)
            y = np.linspace(y1, y2, max_points)
            gradient_colors = np.linspace(color_start, color_end, max_points)
            colors = cmap(gradient_colors)[:, :3]
            for i in range(len(x) - 1):
                ax.plot([x[i], x[i + 1]], [y[i], y[i + 1]], [z_value, z_value], color=colors[i], lw=5)
    print('Wait')
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
    ax.set_zlim(0, max(flushed_moves.keys()) * 1)
    plt.tight_layout()
    plt.show()
    print('Plot was created')

if __name__ == '__main__':
    main()