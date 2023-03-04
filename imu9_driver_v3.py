import time
from tools import *
import numpy as np
import i2creal as i2c  # currently only real I2C on ddboats (no simulated I2C)

# LIS3DML 0x1e  (mag sensor)
# LSM6    0x6b  (accelero - gyro)


class Imu9IO:
    def __init__(self):
        # Current calibration transformation of this IMU object
        self.trans_mag = {'A_1': np.eye(3), 'b': np.zeros(shape=(3, 1))}
        self.trans_acc = {'A_1': np.eye(3), 'b': np.zeros(shape=(3, 1))}

        # Current corrected & normalized magnetic field & acceleration
        self.mag_cor_norm = None
        self.acc_cor_norm = None

        # Current boat euler angles
        self.euler_angles = None

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
        """
        Update trans_mag & trans_acc calibration transformation
        dictionary from a calibration file
        """

        measurements = np.load(calibration_file_name)

        # COMPASS
        mag = measurements[0]
        xn, xs, xw, xu, _ = (mag[k] for k in range(5))
        b = (-1 / 2) * (xn + xs)

        yn = np.array([[np.cos(I)], [0], [-np.sin(I)]])
        yw = np.array([[0], [-np.cos(I)], [-np.sin(I)]])
        yu = np.array([[-np.sin(I)], [0], [np.cos(I)]])
        Y = np.hstack([yn, yw, yu])

        X = np.hstack((xn + b, xw + b, xu + b))
        A = X @ np.linalg.inv(Y)
        self.trans_mag['A_1'] = np.linalg.inv(A)
        self.trans_mag['b'] = b

        # ACCELEROMETER
        acc = measurements[1]
        xz, x_z, _, xx, xy = (acc[k] for k in range(5))
        b = (-1 / 2) * (xz + x_z)

        yz = np.array([[0], [0], [-1]])
        yx = np.array([[-1], [0], [0]])
        yy = np.array([[0], [-1], [0]])
        Y = np.hstack([yz, yx, yy])

        X = np.hstack((xz + b, xx + b, xy + b))
        A = X @ np.linalg.inv(Y)
        self.trans_acc['A_1'] = np.linalg.inv(A)
        self.trans_acc['b'] = b

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

    def compute_mag(self):
        # Update current magnetic field
        x = self.read_mag_raw()
        y = self.trans_mag['A_1'] @ (x + self.trans_mag['b'])
        self.mag_cor_norm = normalize(y)

    def compute_acc(self):
        # Update current acceleration -> median filter on 32 measures
        lst_acc = []
        for _ in range(32):
            lst_acc.append(self.read_accel_raw())
        x = np.median(np.array(lst_acc), axis=0)
        
        y = self.trans_acc['A_1'] @ (x + self.trans_acc['b'])
        self.acc_cor_norm = normalize(y)

    def update(self):
        # Update measures & estimated euler angles
        self.compute_mag()
        self.compute_acc()
        self.compute_euler_angles()

    def compute_euler_angles(self):
        """
        From corrected & normalized measures of magnetic field & acceleration
        compute the current orientation of this IMU expressed with euler angles
        """
        grav = np.array([[0], [0], [-1]])
        y1 = self.mag_cor_norm
        a1 = self.acc_cor_norm

        phi = np.arcsin(a1[1, 0])
        theta = np.arcsin(a1[0, 0])

        Rh = rot_uv(a1, grav)

        mhx, mhy, mhz = (Rh @ y1).flatten()
        psi = -np.arctan2(mhy, mhx)

        self.euler_angles = np.array([phi, theta, psi]).T

    def cap(self):
        # Return the third euler angle -> heading
        return self.euler_angles[2]


if __name__ == "__main__":
    imu = Imu9IO()
    imu.load_calibration()

    t0 = time.time()

    for i in range(200):
        print(imu.read_accel_raw().flatten())
        time.sleep(1e-3)

    print(time.time() - t0)
