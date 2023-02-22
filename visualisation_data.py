from matplotlib import pyplot as plt
import numpy as np


if __name__ == '__main__':
    data = np.load('data/test_rpm.npy')
    fig, (ax1, ax2) = plt.subplots(2, 1)

    ax1.set_title('Left motor')
    ax1.plot(data[0, :], label='cmd')
    ax1.plot(data[2, :], label='rpm')
    ax1.legend()

    ax2.set_title('Right motor')
    ax2.plot(data[0, :], label='cmd')
    ax2.plot(data[2, :], label='rpm')
    ax2.legend()

    fig.suptitle('RPM tests')
    plt.show()
