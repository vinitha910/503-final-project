import csv
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    files = ["run-data-bug5cma-3872495.csv"]

    plt.style.use('seaborn-darkgrid')
    # create a color palette
    palette = plt.get_cmap('Set1')

    for idx, n in enumerate(files):
        data = []
        with open(n, "r") as f:
            reader = csv.reader(f, delimiter=",")
            for i, line in enumerate(reader):
                if line[2] == 'True':
                    data.append(int(line[4]))
        plt.plot(range(len(data)), data, marker=' ', color=palette(idx), label=n)

    plt.legend()
    plt.title("Learning Curve", loc='left', fontsize=12, fontweight=0, color='orange')
    plt.xlabel("Iteration")
    plt.ylabel("Number of Expansions")
    plt.show(block=True)