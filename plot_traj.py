from matplotlib import pyplot as plt
from tools import *


if __name__ == '__main__':
    """
    Scatter trajectories
    """

    for name, coord in coordinates.items():
        if name is not 'plage':
            xs, ys = coord_to_pos(coord).flatten()
            plt.scatter(xs, ys, c='black')

    x = []
    y = []
    f = open("traj_files/traj_first.txt", 'r')
    for line in f.readlines()[20:]:
        xs, ys = line.split(';')
        x.append(float(xs))
        y.append(float(ys))

    plt.scatter(x, y)
    plt.show()
