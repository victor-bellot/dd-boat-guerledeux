import numpy as np


class KalmanFilter:

    def __init__(self, x0, G0):
        self.x = x0
        self.m = self.x.shape[0]
        self.G = G0
        self.lst_x = [self.x]  # list of x
        self.lst_G = [self.G]  # list of Gamma

        self.y = 0
        self.C = np.zeros((1, self.m))
        self.Gbeta = np.zeros((1, 1))
        self.u = np.zeros((self.m, 1))
        self.A = np.eye(self.m)
        self.B = np.eye(self.m)
        self.Galpha = np.zeros((self.m, self.m))

    def corr(self):
        C = self.C
        y = self.y
        Gbeta = self.Gbeta

        ytilde = y - C @ self.x
        S = C @ self.G @ C.T + Gbeta
        K = self.G @ C.T @ np.linalg.inv(S)
        # results
        xk_k = self.x + K @ ytilde
        Gk_k = (np.eye(self.m) - K @ C) @ self.G
        return xk_k, Gk_k

    def pred(self, xk_k, Gk_k):
        x = self.A @ xk_k + self.B @ self.u
        G = self.A @ Gk_k @ self.A.T + self.Galpha
        return x, G

    def instant_state(self, corr=True, pred=True):
        if not (corr or pred):
            print("boolean arguments all false, at least one True is needed")
        elif corr and pred:
            xk_k, Gk_k = self.corr()
            self.x, self.G = self.pred(xk_k, Gk_k)
            return self.x, self.G
        elif not pred:
            xk_k, Gk_k = self.corr()
            self.x, self.G = xk_k, Gk_k
            return xk_k, Gk_k
        else:
            self.x, self.G = self.pred(self.x, self.G)
            return self.x, self.G
