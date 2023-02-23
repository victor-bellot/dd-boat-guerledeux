import time
from tools import *
from imu9_driver_v3 import Imu9IO
from tc74_driver_v2 import TempTC74IO
from arduino_driver_v2 import ArduinoIO
from encoders_driver_v2 import EncoderIO


class Control:
    def __init__(self, mission_name, dt=0.5):
        self.mission_name = mission_name

        self.ard = ArduinoIO()
        self.enc = EncoderIO()
        self.imu = Imu9IO()
        self.tpr = TempTC74IO()
        self.gpsm = GpsManager()
        self.lgm = LogManager(mission_name)

        self.imu.load_calibration()  # load calibration.npy

        self.dt = dt
        # set delay between old and new measures : HERE=dt
        self.enc.set_older_value_delay_v2(int(dt * 10))
        self.tpr.set_config(0x0, 0x0)
        self.tpr.set_mode(standby=True, side="both")

        self.cst = {'left': {'kpi': 4e-2}, 'right': {'kpi': 3e-2},
                    'psi': {'kp': (3 / 2) / np.pi, 'ki': 3e-2 / np.pi, 'ci': (3 / 2) * 1e-1},  # OK: 3/4 & 3e-2 & 1e-1
                    'line': {'kd': 32, 'kn': 1},
                    }

        self.u_step = 32
        self.u_max = 128

        self.rpm_step = 100
        self.rpm_max = 4000

        self.exit_attempt_count = 3  # number of attempt before exiting
        self.distance_to_buoy = 5  # distance in meters from buoy to stop

        self.ei_psi = 0
        self.cmd_left, self.cmd_right = 50, 50

    def reset(self):
        self.ei_psi = 0
        self.cmd_left, self.cmd_right = 50, 50

    def close(self):
        self.ard.send_arduino_cmd_motor(0, 0)
        self.lgm.close()

    def change_timing(self, dt):
        self.dt = dt
        self.enc.set_older_value_delay_v2(int(dt * 10))

    def line_to_psi_bar(self, line):
        pos_boat = self.gpsm.get_position()

        if pos_boat is not None:
            kd, kn = self.cst['line']['kd'], self.cst['line']['kn']
            force = get_force(line, pos_boat, kd, kn)

            fx, fy = force.flatten()
            return np.arctan2(-fx, fy)

    def get_rpm(self):
        # 1 : new ; 0 : old
        st1, st0 = self.enc.get_last_and_older_values_v2()
        data_encoders0 = np.array(st0.split(",")).astype(np.float)
        data_encoders1 = np.array(st1.split(",")).astype(np.float)

        odo_left0 = data_encoders0[4]
        odo_right0 = data_encoders0[3]

        odo_left1 = data_encoders1[4]
        odo_right1 = data_encoders1[3]

        rpm_left = (60. / 8.) * delta_odo(odo_left1, odo_left0) / self.dt
        rpm_right = (60. / 8.) * delta_odo(odo_right1, odo_right0) / self.dt

        return rpm_left, rpm_right

    def regulation_rpm(self, rpm_left_bar, rpm_right_bar):
        rpm_left, rpm_right = self.get_rpm()

        # proportional to error
        step_left = self.cst['left']['kpi'] * (rpm_left_bar - rpm_left)
        step_right = self.cst['right']['kpi'] * (rpm_right_bar - (-rpm_right))

        # ceil tension variation
        step_left = ceil(step_left, self.u_step)
        step_right = ceil(step_right, self.u_step)

        # bound between 0 and u_max
        self.cmd_left = max(min(self.u_max, self.cmd_left + step_left), 0)
        self.cmd_right = max(min(self.u_max, self.cmd_right + step_right), 0)

        self.ard.send_arduino_cmd_motor(self.cmd_left, self.cmd_right)

        # print('MEASURED RPM:', rpm_left, rpm_right)
        return rpm_left, rpm_right

    def psi_bar_to_rpm_bar(self, delta_psi, rpm_max):
        self.ei_psi += delta_psi * self.dt

        kpc = self.cst['psi']['kp'] * delta_psi
        kic = self.cst['psi']['ki'] * self.ei_psi

        # Ceil kic by ci constant
        kic = ceil(kic, self.cst['psi']['ci'])

        e_psi = kpc + kic
        print('kpc: %f ; kip: %f' % (kpc, kic))

        if e_psi >= 0:
            rpm_left_bar = rpm_max - e_psi * rpm_max
            rpm_right_bar = rpm_max
        else:
            rpm_right_bar = rpm_max + e_psi * rpm_max
            rpm_left_bar = rpm_max

        # print('RPM BAR:', rpm_left_bar, rpm_right_bar)
        return rpm_left_bar, rpm_right_bar

    def follow_psi(self, duration, psi_bar, speed_rpm):  # psi_bar is given in degrees!
        mission = 'Follow PSI - ' + 'duration: %i ; psi_bar: %s ; spd: %i\n' % (duration, psi_bar, speed_rpm)
        log_labels = ['time', 'd_PSI', 'rpmL', 'rpmR', 'rpmL_bar', 'rpmR_bar', 'thL', 'thR']
        self.lgm.new_mission(mission, log_labels)

        self.reset()
        t0 = time.time()
        while (time.time() - t0) < duration:
            t0loop = time.time()

            # Update IMU & read current heading
            self.imu.update()
            psi = self.imu.cap()

            delta_psi = sawtooth(psi_bar * (np.pi / 180) - psi)
            print("CURRENT PSI: ", int(psi * (180 / np.pi)))

            rpm_left_bar, rpm_right_bar = self.psi_bar_to_rpm_bar(delta_psi, speed_rpm)
            rpm_left, rpm_right = self.regulation_rpm(rpm_left_bar, rpm_right_bar)

            temp_left, temp_right = self.tpr.read_temp()
            data = [(t0loop - t0) * 1000, delta_psi * (180 / np.pi), rpm_left, -rpm_right,
                    rpm_left_bar, rpm_right_bar, temp_left, temp_right]
            self.lgm.new_measures(data)

            pos_boat = self.gpsm.get_position()
            if pos_boat is not None:  # psi_bar in degrees!
                self.lgm.new_gps_measure(pos_boat, psi, psi_bar * (np.pi / 180))

            # print("Time left: ", self.dt - (time.time() - t0loop))
            while time.time() - t0loop < self.dt:
                self.gpsm.update_coord()
                time.sleep(1e-3)

        self.ard.send_arduino_cmd_motor(0, 0)

    def follow_line(self, duration_max, line, speed_rpm):
        mission = 'Follow LINE from %s to %s - duration_max: %i ; spd: %i\n' % \
                  (line.name0, line.name1, duration_max, speed_rpm)
        log_labels = ['time', 'mx', 'my', 'mz', 'ax', 'ay', 'az', 'd_PSI',
                      'rpmL', 'rpmR', 'rpmL_bar', 'rpmR_bar', 'thL', 'thR']
        self.lgm.new_mission(mission, log_labels)

        self.reset()
        psi_bar = line.get_psi()
        t0 = time.time()
        exit_cnt = 0

        while (time.time() - t0) < duration_max:  # TO ADD : ending conditions
            t0loop = time.time()

            # ENDING conditions:
            # - distance to buoy
            # - reach semi-plan
            coord_boat = self.gpsm.coord
            pos_boat = coord_to_pos(coord_boat)

            dist = np.linalg.norm(line.pos1 - pos_boat)
            proj = dot(line.pos1 - line.pos0, line.pos1 - pos_boat)
            if dist <= self.distance_to_buoy or proj < 0:
                exit_cnt += 1
                if exit_cnt >= self.exit_attempt_count:
                    if dist <= self.distance_to_buoy:
                        print('STOP: distance to buoy less than %im.' %
                              self.distance_to_buoy)
                    else:
                        print('STOP: reach semi-plan.')
                    break
            else:
                exit_cnt = 0

            temp = self.line_to_psi_bar(line)
            psi_bar = psi_bar if temp is None else temp
            # print("PSI BAR: ", psi_bar * (180 / np.pi))

            # Update IMU & read current heading
            self.imu.update()
            psi = self.imu.cap()

            delta_psi = sawtooth(psi_bar - psi)
            # print("CURRENT PSI: ", int(psi * (180 / np.pi)))

            rpm_left_bar, rpm_right_bar = self.psi_bar_to_rpm_bar(delta_psi, speed_rpm)
            rpm_left, rpm_right = self.regulation_rpm(rpm_left_bar, rpm_right_bar)

            temp_left, temp_right = self.tpr.read_temp()
            mx, my, mz = self.imu.mag_cor_norm.flatten() * 1e3
            ax, ay, az = self.imu.acc_cor_norm.flatten() * 1e3

            data = [(t0loop - t0) * 1000, mx, my, mz, ax, ay, az, delta_psi * (180 / np.pi),
                    rpm_left, -rpm_right, rpm_left_bar, rpm_right_bar, temp_left, temp_right]
            self.lgm.new_measures(data)

            pos_boat = self.gpsm.get_position()
            if pos_boat is not None:  # psi_bar in radians!
                self.lgm.new_gps_measure(pos_boat, psi, psi_bar)

            # print("Time left: ", self.dt - (time.time() - t0loop))
            while time.time() - t0loop < self.dt:
                self.gpsm.update_coord()
                time.sleep(1e-3)

        self.ard.send_arduino_cmd_motor(0, 0)

    def follow_point(self, duration_max, yd, d_yd, dd_yd):
        # Initialize logs
        mission = 'Follow POINT - duration_max: %i\n' % (duration_max,)
        log_labels = ['time', 'psi', 'rpmL', 'rpmR', 'rpmL_bar', 'rpmR_bar', 'thL', 'thR']
        self.lgm.new_mission(mission, log_labels)

        print('Waiting for GPS...')
        while not self.gpsm.update_coord():
            time.sleep(1e-3)
        pos_boat = self.gpsm.get_position()
        print('Localized!')

        rpm_left_bar, rpm_right_bar = self.get_rpm()

        # Kalman filter
        # x0 Γ0 u y Γα Γβ A C

        x0 = np.vstack((pos_boat, np.array([[1]])))
        Γ0 = np.eye(3) * 100

        Γα = np.array([[1, 0, 0],
                       [0, 1, 0],
                       [0, 0, 10]])
        Γβ = np.array([[25, 0],
                       [0, 25]])

        Ck = np.array([[1, 0, 0],
                       [0, 1, 0]])

        self.reset()
        t0 = time.time()
        t = 0

        while (time.time() - t0) < duration_max:
            t0loop = time.time()

            # Measure psi
            self.imu.update()
            psi = self.imu.cap()

            # Kalman filter -> estimate x y v
            ak = 0
            yk = pos_boat
            Ak = np.array([[1, 0, self.dt * np.cos(psi)],
                           [0, 1, self.dt * np.sin(psi)],
                           [0, 0, 1]])
            uk = np.array([[0], [0], [self.dt * ak]])

            x0, Γ0 = kalman(x0, Γ0, uk, yk, Γα, Γβ, Ak, Ck)
            x_hat, y_hat, v_hat = x0.flatten()

            # print(x_hat, y_hat, v_hat)

            # Controller -> compute u from x y v psi
            A = np.array([[np.cos(psi), -v_hat * np.sin(psi)],
                          [np.sin(psi), v_hat * np.cos(psi)]])

            y = np.array([[x_hat], [y_hat]])
            d_y = np.array([[v_hat * np.cos(psi)], [v_hat * np.sin(psi)]])

            alpha = beta = 1  # play on that as well
            error = alpha * (yd(t) - y) + 2 * beta * (d_yd(t) - d_y) + dd_yd(t)
            u_dv, u_dp = (np.linalg.inv(A) @ error).flatten()

            # ADD scale before ceiling
            u_dv = ceil(u_dv, self.rpm_step)
            u_dp = ceil(u_dp, self.rpm_step)
            # -> play on rpm_step
            # carry negative speed will (wheel!)

            print('error:', error)
            print('INTENTION - spd: %f ; head: %f' % (u_dv, u_dp))

            rpm_left_bar = max(0, min(rpm_left_bar + (u_dv + u_dp), self.rpm_max))
            rpm_right_bar = max(0, min(rpm_right_bar + (u_dv - u_dp), self.rpm_max))

            rpm_left, rpm_right = self.regulation_rpm(rpm_left_bar, rpm_right_bar)

            # Write some log
            temp_left, temp_right = self.tpr.read_temp()
            data = [(t0loop - t0) * 1000, psi * (180/np.pi), rpm_left, -rpm_right,
                    rpm_left_bar, rpm_right_bar, temp_left, temp_right]
            self.lgm.new_measures(data)

            pos_boat = self.gpsm.get_position()
            yd_x, yd_y = yd(t).flatten()
            self.lgm.new_gps_measure(pos_boat, yd_x, yd_y)

            t += self.dt
            # print("Time left: ", self.dt - (time.time() - t0loop))
            while time.time() - t0loop < self.dt:
                self.gpsm.update_coord()
                time.sleep(1e-3)

        self.ard.send_arduino_cmd_motor(0, 0)


if __name__ == '__main__':
    print("--- Control program ---\n")

    mn = input("Mission name: ")
    ctr = Control(mn)

    try:
        mt = input("Mission type (psi, circle, square, line, test, triangle): ")

        if mt == 'line':
            a = input("Starting point: ")
            b = input("Ending point: ")
            my_line = Line(a, b)

            d_input = input("Mission max duration [s]: ")
            d = infinity if d_input == '' else int(d_input)

            s_input = input("Boat RPM speed: ")
            s = 3000 if s_input == '' else int(s_input)

            ctr.follow_line(d, my_line, speed_rpm=s)

        elif mt == 'triangle':
            d = infinity  # no time limit
            s = 3000  # RPM speed

            line1 = Line('est', 'ouest')
            line2 = Line('ouest', 'nord')
            line3 = Line('nord', 'est')

            ctr.follow_line(d, line1, speed_rpm=s)
            ctr.follow_line(d, line2, speed_rpm=s)
            ctr.follow_line(d, line3, speed_rpm=s)

        elif mt == 'square':
            d_input = input("Side duration [s]: ")
            d = infinity if d_input == '' else int(d_input)

            s_input = input("Boat RPM speed: ")
            s = 3000 if s_input == '' else int(s_input)

            ctr.follow_psi(d, speed_rpm=s, psi_bar=cap_to_psi('N'))
            ctr.follow_psi(d, speed_rpm=s, psi_bar=cap_to_psi('W'))
            ctr.follow_psi(d, speed_rpm=s, psi_bar=cap_to_psi('S'))
            ctr.follow_psi(d, speed_rpm=s, psi_bar=cap_to_psi('N'))

        elif mt == 'test':
            d_input = input("Element duration [s]: ")
            d = infinity if d_input == '' else int(d_input)

            s_input = input("Boat RPM speed: ")
            s = 3000 if s_input == '' else int(s_input)

            ctr.follow_psi(d, speed_rpm=s, psi_bar=cap_to_psi('W'))
            ctr.follow_psi(d, speed_rpm=s, psi_bar=cap_to_psi('N'))

        elif mt == 'circle':
            d_input = input("Mission duration [s]: ")
            d = infinity if d_input == '' else int(d_input)

            # cercle de center x0, y0 de rayon r, de pulsation w de phase phi
            c_input = input("Circle center (x0 y0): ")
            if c_input == '':
                c = np.array([[10], [10]])
            else:
                x0 = int(c_input.split()[0])
                y0 = int(c_input.split()[1])
                c = np.array([[x0], [y0]])

            r_input = input("Radius [m]: ")
            r = 10 if d_input == '' else int(d_input)

            w_input = input("Pulsation [°/s]: ")
            w = 0.1 if d_input == '' else float(d_input) * (np.pi/180)

            phi_input = input("Phase [°]: ")
            phi = 0 if d_input == '' else float(d_input) * (np.pi/180)

            def circle(t): return c + r * np.array([[np.cos(w * t + phi)], [np.sin(w * t - phi)]])
            def d_circle(t): return r * w * np.array([[-np.sin(w * t + phi)], [+np.cos(w * t - phi)]])
            def dd_circle(t): return -r * w ** 2 * np.array([[np.cos(w * t + phi)], [np.sin(w * t - phi)]])

            ctr.follow_point(d, circle, d_circle, dd_circle)

        else:  # FOLLOW-PSI ici
            d_input = input("Mission duration [s]: ")
            d = infinity if d_input == '' else int(d_input)

            p_input = input("Psi bar [°]: ")
            p = 0.0 if p_input == '' else int(p_input)

            s_input = input("Boat RPM speed: ")
            s = 3000 if s_input == '' else int(s_input)

            ctr.follow_psi(d, speed_rpm=s, psi_bar=p)

    except KeyboardInterrupt:
        ctr.close()
