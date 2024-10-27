import csv
import argparse
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot Output Data")
    parser.add_argument('-f', '--file', required=True)
    args = parser.parse_args()

    t = []
    measurement = []
    output = []
    with open(args.file) as csv_file:
        csv_data = csv.reader(csv_file)
        for row in csv_data:
            t.append(float(row[0]))
            measurement.append(float(row[1]))
            output.append(float(row[2]))

    plt.plot(t, measurement, color='b', label="measurement")
    plt.plot(t, output, color='r', label="output")

    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.title("Filter Output Plot")
    plt.legend(loc="upper right")
    plt.savefig("FilterOutputPlot.png")
    plt.show()
