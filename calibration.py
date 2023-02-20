import sys
import time
import numpy as np
from imu9_driver_v3 import Imu9IO

labels = ['xn', 'xs', 'xw', 'xu']


def get_time():
    lt = time.localtime()
    return "%i-%i-%i-%i" % (lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec)


if __name__ == "__main__":
    try:
        file_name = sys.argv[1] + '.npy'
    except:
        file_name = 'calibration.npy'

    try:
        n = int(sys.argv[2])
    except:
        n = -1

    imu = Imu9IO()

    if n < 0:
        imu.load_calibration(file_name)
        while True:
            print(imu.get_euler_angles() * (180 / np.pi))
            time.sleep(0.2)
    else:
        with open(file_name, 'wb') as f:
            res = np.empty((4, 3, 1))
            for i in range(4):
                label = labels[i]
                input('Press ENTER to measure ' + label)
                measurements = np.zeros((3, n))
                for k in range(n):
                    measurements[:, k] = imu.read_mag_raw().flatten()
                    time.sleep(0.01)
                res[i] = np.median(measurements, axis=1, keepdims=True)
                print("%s = (%f, %f, %f)" % (label, *list(res[i].flatten())))
            np.save(file_name, res)
