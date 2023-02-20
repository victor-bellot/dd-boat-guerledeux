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
        n = int(sys.argv[1])
    except:
        n = -1

    imu = Imu9IO()

    if n < 0:
        while True:
            print(imu.get_euler_angles())
            time.sleep(0.1)
    else:
        file_name = 'calibration.npy'
        with open(file_name, 'wb') as f:
            res = np.empty((4, 3, 1))
            for i in range(4):
                label = labels[i]
                input('Press ENTER to measure ' + label)
                measurements = np.zeros((3, n))
                for k in range(n):
                    measurements[:, k] = np.array(imu.read_mag_raw())
                    time.sleep(0.01)
                res[i] = np.median(measurements, axis=1, keepdims=True)
                print("%s = (%f, %f, %f)" % (label,) + tuple(res[i].flatten()))
            np.save(file_name, res)
