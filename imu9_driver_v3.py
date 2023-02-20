import time
from tools import *
import numpy as np
import i2creal as i2c  # currently only real I2C on ddboats (no simulated I2C)

# LIS3DML 0x1e  (mag sensor)
# LSM6    0x6b  (accelero - gyro)


class Imu9IO:
    def __init__(self):
        self.A_1, self.b = np.eye(3), np.zeros(shape=(3, 1))

        self.__bus_nb = 1  # 1 on DDBoat, 2 on DartV2
        self.__addr_mg = 0x1e  # mag sensor
        self.__addr_ag = 0x6b  # accelero - gyro

        self.__dev_i2c_mg = i2c.i2c(self.__addr_mg, self.__bus_nb)
        self.__dev_i2c_ag = i2c.i2c(self.__addr_ag, self.__bus_nb)

        self.__mag_raw = [0.0, 0.0, 0.0]
        self.__accel_raw = [0.0, 0.0, 0.0]
        self.__gyro_raw = [0.0, 0.0, 0.0]

        # configure mag sensor
        # CTRL_REG1 (0x20) = 0b01110000
        # OM = 11 (ultra-high-performance mode for X and Y);
        # DO = 100 (10 Hz ODR)
        self.__dev_i2c_mg.write(0x20, [0x70])
        # CTRL_REG2 (0x21) = 0b00000000
        # FS = 00 (+/- 4 gauss full scale)
        self.__dev_i2c_mg.write(0x21, [0x00])
        # CTRL_REG3 (0x22) = 0b00000000
        # MD = 00 (continuous-conversion mode)
        self.__dev_i2c_mg.write(0x22, [0x00])
        # CTRL_REG4 (0x23) = 0b00001100
        # OMZ = 11 (ultra-high-performance mode for Z)
        self.__dev_i2c_mg.write(0x23, [0x0C])

        # configure accelero + gyro
        # LSM6DS33 gyro
        # CTRL2_G (0x11) = 0b10001100
        # ODR = 1000 (1.66 kHz (high performance))
        # FS_G = 11 (2000 dps)
        self.__dev_i2c_ag.write(0x11, [0x8C])

        # CTRL7_G (0x16) = 0b00000000
        # defaults
        self.__dev_i2c_ag.write(0x16, [0x00])

        # LSM6DS33 accelerometer
        # CTRL1_XL (0x10) = 0b10001100
        # ODR = 1000 (1.66 kHz (high performance))
        # FS_XL = 11 (8 g full scale)
        # BW_XL = 00 (400 Hz filter bandwidth)
        # self.__dev_i2c_ag.write(0x10,[0x8C])
        # more filtering BW_XL = 11 (50 Hz filter bandwidth)
        self.__dev_i2c_ag.write(0x13, [0x00])
        self.__dev_i2c_ag.write(0x10, [0x8C])

        # common
        # CTRL3_C (0x12) 0b00000100
        # IF_INC = 1 (automatically increment address register)
        self.__dev_i2c_ag.write(0x12, [0x04])

    def load_calibration(self, calibration_file_name='calibration.npy'):
        measurements = np.load(calibration_file_name)
        xn, xs, xw, xu = (measurements[k] for k in range(4))
        self.b = (-1 / 2) * (xn + xs)

        yn = np.array([[np.cos(I)], [0], [-np.sin(I)]])
        yw = np.array([[0], [-np.cos(I)], [-np.sin(I)]])
        yu = np.array([[-np.sin(I)], [0], [np.cos(I)]])
        Y = np.hstack([yn, yw, yu])

        X = np.hstack((xn + self.b, xw + self.b, xu + self.b))
        A = X @ np.linalg.inv(Y)
        self.A_1 = np.linalg.inv(A)

    def setup_accel_filter(self, mode):
        if mode == 0:
            self.__dev_i2c_ag.write(0x17, [0x00])
            self.__dev_i2c_ag.write(0x13, [0x00])
            self.__dev_i2c_ag.write(0x10, [0x8C])
        elif mode == 1:
            self.__dev_i2c_ag.write(0x17, [0x00])
            self.__dev_i2c_ag.write(0x13, [0x80])
            self.__dev_i2c_ag.write(0x10, [0x8F])
        elif mode == 2:
            self.__dev_i2c_ag.write(0x17, [0x80])
            self.__dev_i2c_ag.write(0x13, [0x80])
            self.__dev_i2c_ag.write(0x10, [0x8F])

    def read_mag_raw(self):
        v = self.__dev_i2c_mg.read(0x28, 6)
        ix = self.cmpl2(v[0], v[1])
        iy = self.cmpl2(v[2], v[3])
        iz = self.cmpl2(v[4], v[5])
        self.__mag_raw = [ix, iy, iz]
        return np.array(self.__mag_raw).reshape(3, 1)

    def read_gyro_raw(self):
        # OUTX_L_G (0x22)
        v = self.__dev_i2c_ag.read(0x22, 6)
        ix = self.cmpl2(v[0], v[1])
        iy = self.cmpl2(v[2], v[3])
        iz = self.cmpl2(v[4], v[5])
        self.__gyro_raw = [ix, iy, iz]
        return np.array(self.__gyro_raw).reshape(3, 1)

    def read_accel_raw(self):
        # OUTX_L_XL (0x28)
        v = self.__dev_i2c_ag.read(0x28, 6)
        ix = self.cmpl2(v[0], v[1])
        iy = self.cmpl2(v[2], v[3])
        iz = self.cmpl2(v[4], v[5])
        self.__accel_raw = [ix, iy, iz]
        return np.array(self.__accel_raw).reshape(3, 1)

    def cmpl2(self, lsByte, msByte):
        i = lsByte + (msByte << 8)
        if i >= (1 << 15):
            i = i - (1 << 16)
        return i

    def correction_mag(self):
        x = self.read_mag_raw()
        y = self.A_1 @ (x + self.b)
        return y

    def get_euler_angles(self):
        a1 = normalize(self.read_accel_raw())
        y1 = normalize(self.correction_mag())

        phi = np.arcsin(a1[1, 0])
        theta = np.arcsin(a1[0, 0])

        Rh = rot_uv(a1, np.array([[0], [0], [-1]]))
        yh = Rh @ y1
        psi = -np.arctan2(yh[1, 0], yh[0, 0])

        return np.array([phi, theta, psi]).T

    def cap(self):
        return self.get_euler_angles()[2]


if __name__ == "__main__":
    imu = Imu9IO()

    for i in range(200):
        print(imu.read_accel_raw().flatten())
        time.sleep(0.1)
