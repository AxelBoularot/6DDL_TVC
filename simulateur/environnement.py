"""
environnement.py -- atmosphere (ISA) et vent (moyenne + rafales Ornstein-Uhlenbeck).
"""
import math
import numpy as np
import config as C


def densite_air(alt):
    if not C.USE_ISA:
        return C.rho0
    T0, Lr, R, g0 = 288.15, 0.0065, 287.05, 9.80665
    T = T0 - Lr * max(alt, 0.0)
    return C.rho0 * (T / T0) ** (g0 / (Lr * R) - 1.0)


class Vent:
    """Rafales horizontales (X et Y independants), processus d'Ornstein-Uhlenbeck."""
    def __init__(self):
        self.moyen = np.array(C.wind_mean, dtype=float)
        self.w = self.moyen.copy()

    def pas(self, dt):
        for k in (0, 1):
            self.w[k] += (-(self.w[k] - self.moyen[k]) / C.wind_tau) * dt \
                + C.wind_gust_sigma * math.sqrt(2 * dt / C.wind_tau) * np.random.randn()
        return self.w


def bruit_couple(dt):
    """Couple parasite transverse, independant de dt."""
    sig = C.torque_noise_sigma * math.sqrt(C.dt_ref_noise / dt)
    return np.array([np.random.randn() * sig, np.random.randn() * sig, 0.0])
