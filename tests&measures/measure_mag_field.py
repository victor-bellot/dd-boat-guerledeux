from imu9_driver_v3 import Imu9IO
import time


if __name__ == '__main__':
    f = open('tests&measures/data/mag_measurements.txt', 'w')
    imu = Imu9IO()
    imu.load_calibration('calibration.npy')

    try:
        while True:
            # x, y, z = imu.read_mag_raw().flatten()
            x, y, z = imu.correction_mag().flatten()
            f.write('%f %f %f\n' % (x, y, z))
            time.sleep(0.2)

    except:
        f.close()
