import sys
import time
import numpy as np
from imu9_driver_v3 import Imu9IO

labels = ['NWU', 'SWD', 'WSU', 'UEN', 'NUE']


def get_time():
    lt = time.localtime()
    return "%i-%i-%i-%i" % (lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec)


if __name__ == "__main__":
    try:
        file_name = sys.argv[1] + '.npy'
    except:
        file_name = 'data/calibration.npy'

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
            save = np.empty((2, 5, 3, 1))
            for i in range(5):
                label = labels[i]
                input('Press ENTER to measure %s (XYZ notation)' % label)
                mag_measures = np.zeros((3, n))
                acc_measures = np.zeros((3, n))
                for k in range(n):
                    mag_measures[:, k] = imu.read_mag_raw().flatten()
                    acc_measures[:, k] = imu.read_accel_raw().flatten()
                    time.sleep(0.01)
                save[0, i] = np.median(mag_measures, axis=1, keepdims=True)  # mag
                save[1, i] = np.median(acc_measures, axis=1, keepdims=True)  # acc
                print("mag : %s = (%f, %f, %f)" % (label, *list(save[0, i].flatten())))
                print("acc : %s = (%f, %f, %f)" % (label, *list(save[1, i].flatten())))
            np.save(file_name, save)
