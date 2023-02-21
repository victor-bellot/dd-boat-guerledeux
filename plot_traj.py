from scipy.interpolate import interp1d
from matplotlib import pyplot as plt
from tools import *

haxby = np.array([[37, 57, 175],
                  [40, 127, 251],
                  [50, 190, 255],
                  [106, 235, 255],
                  [138, 236, 174],
                  [205, 255, 162],
                  [240, 236, 121],
                  [255, 189, 87],
                  [255, 161, 68],
                  [255, 186, 133],
                  [255, 255, 255]]) / 255

ks = np.linspace(0, 1, 11)
interpolation = interp1d(ks, haxby, axis=0)
# -> map [0; 1[ to rgb color in [0; 1[


def color_map(n): return [interpolation(k/n) for k in range(n)]


if __name__ == '__main__':
    """
    Scatter trajectories
    """
    mission_name = 'second'

    for name, coord in coordinates.items():
        if name != 'plage':
            xs, ys = coord_to_pos(coord).flatten()
            plt.scatter(xs, ys, c='black')

    x = []
    y = []
    f = open("traj_files/traj_%s.txt" % mission_name, 'r')
    for line in f.readlines()[20:]:
        xs, ys = line.split(';')
        x.append(float(xs))
        y.append(float(ys))

    plt.scatter(x, y, c=color_map(len(x)))
    plt.show()
