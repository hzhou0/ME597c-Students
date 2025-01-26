# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import math
import matplotlib.pyplot as plt
from utilities import FileReader


def plot_errors(filename):
    headers, values = FileReader(filename).read_file()
    time_list = []
    first_stamp = values[0][-1]

    for val in values:
        time_list.append(val[-1] - first_stamp)

    if 'ranges' in headers:
        x_coordinates = []
        y_coordinates = []
        angle = 0
        angle_increment = values[0][1]
        for val in values[0][0]:
            if (val == math.inf):
                angle += angle_increment
                continue
            x_coordinates += [val * math.cos(angle)]
            y_coordinates += [val * math.sin(angle)]
            angle += angle_increment
        plt.scatter(x_coordinates, y_coordinates)  # , label= headers[i]+ " linear")
    else:
        for i in range(0, len(headers) - 1):
            plt.plot(time_list, [lin[i] for lin in values], label=headers[i] + " linear")

    # plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.legend()
    plt.grid()
    plt.show()


import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')

    args = parser.parse_args()

    print("plotting the files", args.files)

    filenames = args.files
    for filename in filenames:
        plot_errors(filename)
