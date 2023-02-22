from roblib import *


def draw_point3D(ax, x, y, z, col='black'):
    ax.scatter(x, y, z, color=col)


if __name__ == '__main__':
    ax = Axes3D(figure())

    clean3D(ax, size=2)
    draw_axis3D(ax, zoom=1)

    with open('test/mag_measurements.txt', 'r') as f:
        while True:
            try:
                x, y, z = f.readline().split()
            except:
                break
            else:
                draw_point3D(ax, float(x), float(y), float(z))

    plt.pause(20)
