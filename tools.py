import numpy as np
#from gps_driver_v2 import GpsIO

data_keys = ['time', 'd_psi', 'rpm_l', 'rpm_r',
             'rpm_lb', 'rpm_rb', 'th_l', 'th_r']

coordinates = {'ponton': [48.198943, -3.014750],
               'nord': [48.199508, -3.015295],
               'ouest': [48.199184, -3.015283],
               'est': [48.199202, -3.015000],
               'plage': [48.199807, -3.014803]}

infinity = int(1e6)  # maximum mission duration in seconds
rho = 110e3  # 6366376  # to check
I = 63.7 / 180 * np.pi  # earth magnetic field angle


def cap_to_psi(cap):
    if cap == 'S':
        return 180
    elif cap == 'W':
        return +90
    elif cap == 'E':
        return -90
    else:
        return 0


def delta_odo(odo1, odo0):
    dodo = odo1 - odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


def normalize(u):
    n = np.linalg.norm(u)
    return u / n if n > 0 else 0


def dot(a, b):
    return np.sum(a * b)


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def adj(w):
    wx, wy, wz = w.flatten()
    return np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])


def expm(M, order=6):
    acc = np.eye(3)
    Mk = np.eye(3)
    scl = 1

    for k in range(1, order+1):
        Mk = Mk @ M
        scl = (1/k) * scl
        acc = acc + scl * Mk

    return acc


def expw(w): return expm(adj(w))


def rot_uv(u, v):  # returns rotation with minimal angle  such that  v=R*u
    u = normalize(u)
    v = normalize(v)

    c = dot(u, v)
    A = v @ u.T - u @ v.T

    return np.eye(3, 3) + A + (1 / (1 + c)) * A @ A


def cvt_gll_ddmm_2_dd(st):
    ilat = st[0]
    ilon = st[2]
    olat = float(int(ilat / 100))
    olon = float(int(ilon / 100))
    olat_mm = (ilat % 100) / 60
    olon_mm = (ilon % 100) / 60
    olat += olat_mm
    olon += olon_mm
    if st[3] == "W":
        olon = -olon
    return olat, olon


# coordinate are given in (ly, lx) format
def convert(data):
    ly_raw = data[0]
    lx_raw = data[2]
    ly = ly_raw // 100
    lx = lx_raw // 100
    ly += (ly_raw % 100) / 60
    lx += (lx_raw % 100) / 60
    if data[3] == 'W':
        lx = -lx
    return ly, lx


# coord : spherical (angles in degrees) ; pos : cartesian
def coord_to_pos(coords, origin='ponton'):
    ly, lx = coords
    lyo, lxo = coordinates[origin]
    x = rho * np.cos(ly * (np.pi / 180)) * (lx - lxo)
    y = rho * (ly - lyo)
    return np.array([[x], [y]])


def get_force(line, pos, kd, kn):
    delta_p = pos - line.pos0
    normal = line.get_normal_toward(pos)
    force = kd * line.get_direction() - kn * 2 * (normal.T @ delta_p) * normal
    return force


class GpsManager:
    def __init__(self):
        self.gps = GpsIO()

        self.coord = coordinates['ponton']
        self.ready = False

    def update_coord(self):
        msg, data = self.gps.read_gll_non_blocking()
        if msg and abs(data[0]) > 1e-3:
            self.coord = convert(data)
            self.ready = True


class Line:
    def __init__(self, name0, name1, coord0=None, coord1=None):
        self.name0 = name0
        self.name1 = name1

        if name0 in coordinates.keys():
            self.coord0 = coordinates[name0]
        else:
            self.coord0 = coord0

        if name1 in coordinates.keys():
            self.coord1 = coordinates[name1]
        else:
            self.coord1 = coord1

        self.pos0 = coord_to_pos(self.coord0)
        self.pos1 = coord_to_pos(self.coord1)

    def get_direction(self):
        delta_pos = self.pos1 - self.pos0
        return delta_pos / np.linalg.norm(delta_pos)

    def get_normal_toward(self, pos):
        d = self.get_direction()
        m = pos - self.pos0
        a = np.sum(m * d) * d
        n = m - a
        return n / np.linalg.norm(n)

    def get_psi(self):
        fx, fy = self.get_direction().flatten()
        return np.arctan2(-fx, fy)


class LogManager:
    def __init__(self, mission_name):
        self.log_file = open("log_files/log_%s.txt" % mission_name, 'a+')
        self.traj_file = open("traj_files/traj_%s.txt" % mission_name, 'a+')

    def new_mission(self, mission, data_names):
        self.log_file.write("\nNew mission : " + mission)
        for data_name in data_names:
            self.log_file.write(data_name + ' ')
        self.log_file.write('\n')

    def new_measures(self, measures,):
        for measure in measures:
            self.log_file.write(str(measure) + ' ')
        self.log_file.write('\n')

    def new_GPS_measure(self, pos_boat, psi=None, psi_bar=None):
        x, y = pos_boat.flatten()
        if psi is None:
            self.traj_file.write("%f;%f;%f;%f;\n" % (x, y))
        else:
            self.traj_file.write("%f;%f;%f;%f\n" % (x, y, psi, psi_bar))

    def close(self):
        self.log_file.close()
        self.traj_file.close()


if __name__ == '__main__':
    pass
