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
    mission_name = 'test'

    # plot français_to_english("bouées")
    for name, coord in coordinates.items():
        if name != 'plage':
            xs, ys = coord_to_pos(coord).flatten()
            plt.scatter(xs, ys, c='black')

    x = []
    y = []
    vx, vy = [], []
    vxbar, vybar = [], []
    PSI,PSI_BAR = [], []
    f = open("traj_%s.txt" % mission_name, 'r')
    for line in f.readlines()[1:]:
        measures = line.split(';')
        xs, ys, psi, psi_bar = measures if len(measures) == 4 else (measures[0], measures[1], 500, 500)
        psi, psi_bar = float(psi), float(psi_bar)*np.pi/180
        PSI.append(psi)
        PSI_BAR.append(psi_bar)
        x.append(float(xs))
        y.append(float(ys))

        fx, fy = (-np.sin(psi), np.cos(psi)) if psi != 500 else (0, 0)
        vx.append(float(fx))
        vy.append(float(fy))

        fxbar, fybar = (-np.sin(psi_bar),np.cos(psi_bar)) if psi_bar != 500 else (0, 0)
        vxbar.append(fxbar)
        vybar.append(fybar)

    plt.title('Trajectoire avec caps')
    plt.scatter(x, y, c=color_map(len(x)))
    plt.quiver(x, y, vx, vy, color='black', scale=50, width=0.002, label='cap reel')
    plt.quiver(x, y, vxbar, vybar, color='red', scale=50, width=0.002, label='consigne')
    plt.legend()
    plt.show()

    plt.title('Erreur de cap')
    plt.plot(np.array(PSI_BAR)-np.array(PSI),label='erreur de cap')
    plt.legend()
    plt.show()

    plt.title('Comparaison consigne_cap/cap_reel')
    plt.plot(np.array(PSI_BAR), label='consigne')
    plt.plot(np.array(PSI), label='cap reel')
    plt.legend()
    plt.show()
