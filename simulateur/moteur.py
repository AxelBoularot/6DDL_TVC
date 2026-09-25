"""
moteur.py -- poussee, impulsion et masse de propergol.
"""
import numpy as np
import config as C


def discretiser(points, dt):
    """Echantillonne la courbe de poussee au pas dt (interpolation lineaire)."""
    out = []
    for i in range(len(points) - 1):
        tp, fp = points[i]
        tn, fn = points[i + 1]
        d = tn - tp
        n = int(round(d / dt))        # round() : int() perdait un pas (0.0499999/0.001)
        slope = (fn - fp) / d if d > 0 else 0
        for j in range(n):
            tt = tp + j * dt
            out.append((round(tt, 6), round(fp + slope * (tt - tp), 3)))
    out.append((points[-1][0], points[-1][1]))
    return out


class Moteur:
    def __init__(self, points=None, dt=None, m_prop0=None):
        # valeurs lues dans config a la creation (pas a l'import)
        points = C.courbe_poussee if points is None else points
        dt = C.dt if dt is None else dt
        m_prop0 = C.m_prop0 if m_prop0 is None else m_prop0
        courbe = discretiser(points, dt)
        self.t = np.array([p[0] for p in courbe])
        self.F = np.array([p[1] for p in courbe])
        self.cum = np.concatenate(([0.0], np.cumsum(0.5 * (self.F[1:] + self.F[:-1])
                                                    * np.diff(self.t))))
        self.I_total = float(self.cum[-1])
        self.m_prop0 = m_prop0

    def poussee(self, t):
        """Poussee a l'instant t, interpolee sur le TEMPS."""
        if t <= self.t[0] or t >= self.t[-1]:
            return 0.0
        return float(np.interp(t, self.t, self.F))

    def impulsion(self, t):
        if t <= self.t[0]:
            return 0.0
        if t >= self.t[-1]:
            return self.I_total
        return float(np.interp(t, self.t, self.cum))

    def masse_propergol(self, t):
        """m_prop(t) = m_prop0 * (1 - I(t)/I_total)."""
        if self.I_total <= 0:
            return 0.0
        return max(self.m_prop0 * (1.0 - self.impulsion(t) / self.I_total), 0.0)
