#!/usr/bin/env python3
import os, sys, csv, argparse
import numpy as np
import matplotlib.pyplot as plt, matplotlib.ticker as ticker

RESULTS_FOLDER = os.path.expanduser('~/printer_data/config/adxl_results/coolstep/')

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Generate stallguard graph on CSV data')
    parser.add_argument("csv_path",
                        help="filename of output csv file")
    parser.add_argument("-o", '--output', dest="output",
                        help="filename of output graph")
    parser.add_argument('--over-time', dest="over_time",
                        action='store_true',
                        help='Create overtime graph instead of cummulative')
    parser.add_argument(
        "-s",
        "--min-speed",
        dest="min_speed",
        default=0.0,
        type=float,
        help="minimum speed to plot",
    )
    parser.add_argument(
        "-e",
        "--max-speed",
        dest="max_speed",
        default=4000.0,
        type=float,
        help="maximum speed to plot",
    )
    args = parser.parse_args()
    if len(sys.argv)==1 or args.csv_path is None:
        parser.print_help()
        exit(1)

    file = args.csv_path

    data = {}
    raw_data = []
    source_file_name = os.path.basename(file)
    out_path = args.output if args.output else os.path.join(
        RESULTS_FOLDER, f"{source_file_name.split('.')[0]}.png")

    with open(file, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        header = next(csv_reader)  # Skip the header row
        if header != ['#time', 'velocity', 'sg_result', 'cs_actual']:
            print("Header is not match with expected")
            exit(1)

        for row in csv_reader:
            eventtime = float(row[0])
            velocity = float(row[1])
            sg_result = int(row[2])
            cs_actual = int(row[3])
            if velocity < args.min_speed or velocity > args.max_speed:
                continue
            raw_data.append([eventtime, velocity, sg_result, cs_actual])
            if velocity not in data:
                data[velocity] = {
                    'sg_min': sg_result,
                    'sg_list': [sg_result],
                    'sg_max': sg_result,
                    'cs_min': cs_actual,
                    'cs_list': [cs_actual],
                    'cs_max': cs_actual
                }
                continue

            data[velocity]["sg_min"] = min(data[velocity]["sg_min"], sg_result)
            data[velocity]["sg_max"] = max(data[velocity]["sg_max"], sg_result)
            data[velocity]["sg_list"].append(sg_result)

            data[velocity]["cs_min"] = min(data[velocity]["cs_min"], cs_actual)
            data[velocity]["cs_max"] = max(data[velocity]["cs_max"], cs_actual)
            data[velocity]["cs_list"].append(cs_actual)

    for velocity in data:
        data[velocity]['sg_mean'] = np.mean(data[velocity]['sg_list'])
        data[velocity]['cs_mean'] = np.mean(data[velocity]['cs_list'])

    velocities = list(data.keys())
    velocities.sort()

    fig, ax = plt.subplots(figsize=(12,8))

    plt.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.9)
    if not args.over_time:
        sg_mins, sg_maxs, sg_means, cs_mins, cs_maxs, cs_means = zip(*[[data[velocity][key]
            for key in ['sg_min', 'sg_max', 'sg_mean', 'cs_min', 'cs_max', 'cs_mean']] 
              for velocity in velocities])
        sg_min, sg_max, sg_mean = min(sg_mins), max(sg_maxs), np.mean(sg_means)
        cs_min, cs_max, cs_mean = min(cs_mins), max(cs_maxs), np.mean(cs_means)

        mean_filt = {
            'sg_mean': sg_means,
            'cs_mean': cs_means,
            'sg_mean_filt': [],
            'cs_mean_filt': [],
        }
        half_window = 1
        for param in mean_filt:
            if len(mean_filt[param]) == 0:
                name = '_'.join(param.split('_')[:2])
                mean_filt[param] = np.array(np.mean(
                    [mean_filt[name][i - half_window:i + half_window + 1]
                     for i in range(half_window, len(mean_filt[name]) - half_window)], axis=1))
                for i in range(half_window):
                    mean_filt[param] = np.insert(mean_filt[param], i, mean_filt[name][i])
                    mean_filt[param] = np.insert(mean_filt[param], -i, mean_filt[name][-i])


        ax.plot(velocities, sg_mins, label=f'Min SG: {sg_min:.2f}', linestyle='--', linewidth=1)
        ax.plot(velocities, sg_maxs, label=f'Max SG: {sg_max:.2f}', linestyle='--', linewidth=1)
        ax.plot(velocities, sg_means, label=f'Mean SG: {sg_mean:.2f}', linestyle='-.', linewidth=1.25)
        ax.plot(velocities, mean_filt["sg_mean_filt"], label=f'Smooth mean SG', linestyle='-', linewidth=2)

        ax2 = ax.twinx()
        ax2.plot(velocities, cs_mins, label=f'Min CS: {cs_min:.2f}', linestyle='--', linewidth=1)
        ax2.plot(velocities, cs_maxs, label=f'Max CS: {cs_max:.2f}', linestyle='--', linewidth=1)
        ax2.plot(velocities, cs_means, label=f'Mean CS: {cs_mean:.2f}', linestyle='-.', linewidth=1.25)
        ax2.plot(velocities, mean_filt["cs_mean_filt"], label=f'Smooth mean CS', linestyle='-', linewidth=2)

        # Add labels and title
        ax.set_xlabel('Velocity')
        ax.set_ylabel('Result')
        ax.set_title('sg_result/cs_actual by velocity\n' + source_file_name)
    else:
        eventtimes = [event[0] for event in raw_data]
        velocities = [event[1] for event in raw_data]
        sg_results = [event[2] for event in raw_data]
        cs_actual = [event[3] for event in raw_data]

        ax.plot(eventtimes, velocities, label="Velocity mm/s")
        ax.plot(eventtimes, sg_results, label="SG")
        ax.plot(eventtimes, cs_actual, label="CS")
        ax.set_xlabel('Time')
        ax.set_ylabel('Values')
        ax.set_title('Velocity/sg_result/cs_actual by time\n' + source_file_name)

    ax.set_xlim(args.min_speed, args.max_speed)
    ax.set_ylim(-31, 125)
    ax2.set_ylim(6, 133)
    ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(which='major', color='grey')
    ax.grid(which='minor', color='lightgrey')
    ax.legend(loc='upper right')
    ax2.legend(loc='lower right')
    # plt.savefig(out_path, dpi=500)
    # Show the plot
    plt.show()

if __name__ == '__main__':
    main()
