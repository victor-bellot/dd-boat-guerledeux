import numpy as np
import matplotlib.pyplot as plt
from tools import coordinates, rho, coord_to_pos, get_force, Line, normalize

control_cst = {'left': {'kp': 0.01, 'ki': 0.01},
               'right': {'kp': 0.01, 'ki': 0.01},
               'phi': {'kp': (3/4) / np.pi, 'ki': 1e-2 / np.pi},
               'line': {'kd': 150, 'kn': 1},
               }

# Fonction qui renvoie le cap à suivre pour un point donné
"""
def line_to_psi_bar(self, line):
    coord_boat = self.gpsm.coord

    if self.gpsm.updated:
        pos_boat = coord_to_pos(coord_boat)
        x, y = pos_boat.flatten()

        kd, kn = self.cst['line']['kd'], self.cst['line']['kn']
        force = get_force(line, pos_boat, kd, kn)

        fx, fy = force.flatten()
        return np.arctan2(-fx, fy)"""

def line_to_psi_bar(pos_boat, line):
    kd, kn = control_cst['line']['kd'], control_cst['line']['kn']
    force = get_force(line, pos_boat, kd, kn)

    fx, fy = force.flatten()
    return np.arctan2(-fx, fy)

def line_to_force(pos_boat, line):
    kd, kn = control_cst['line']['kd'], control_cst['line']['kn']
    force = get_force(line, pos_boat, kd, kn)
    return force


# What is my ligne
a = "ponton" #(ponton, nord, ouest, est, plage) = ")
b = "ouest"
my_line = Line(a, b)

x, y = my_line.pos0, my_line.pos1
l=25
x_min, x_max, y_min, y_max = min(x[0, 0], y[0, 0])-l, max(
    x[0, 0], y[0, 0])+l, min(x[1, 0], y[1, 0])-l, max(x[1, 0], y[1, 0])+l
#print(x_min, x_max, y_min, y_max)

# Définition de la grille pour tracer le champ de vecteurs
X, Y = np.meshgrid(np.linspace(x_min, x_max, int(abs(x_max-x_min)/3)),
                   np.linspace(y_min, y_max, int(abs(y_max-y_min)/3)))

# Initialisation des tableaux pour stocker les composantes x et y des vecteurs
u = np.zeros_like(X)
v = np.zeros_like(Y)

ubis = np.zeros_like(X)
vbis = np.zeros_like(Y)

# Calcul du champ de vecteurs pour chaque point de la grille
for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        point = np.array([X[i, j], Y[i, j]]).reshape((2, 1))
        force = line_to_force(point, my_line)
        force= normalize(force)
        ubis[i, j] = force[0,0]
        vbis[i, j] = force[1,0]
        
        cap = line_to_psi_bar(point, my_line)
        u[i, j] = -np.sin(cap)
        v[i, j] = np.cos(cap)


x0, x1 = x.flatten()
y0, y1 = y.flatten()
plt.figure()# Tracer le champ de vecteurs
plt.title('sincos')
plt.quiver(X, Y, u, v)
plt.plot(x0, x1, '*r', lw=25)
plt.plot(y0, y1, '*b', lw=25)
plt.plot(np.array([x0, y0]), np.array([x1, y1]), 'orange')

#plt.show()

plt.figure()
plt.title("force")
plt.quiver(X, Y, ubis, vbis)
plt.plot(x0, x1, '*r', lw=25)
plt.plot(y0, y1, '*b', lw=25)
plt.plot(np.array([x0, y0]), np.array([x1, y1]), 'orange')

plt.show()

