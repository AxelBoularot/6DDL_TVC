"""
dynamique.py -- etat du corps rigide et integration d'un pas de temps 6 DDL.
    translation : m dV/dt = F (repere monde)
    rotation    : I dw/dt = tau - w x (I w) (repere corps, terme gyroscopique)
    attitude    : dq/dt = 1/2 q (x) (0, w), puis renormalisation
"""
import math
from dataclasses import dataclass, field
import numpy as np
import config as C
from .quaternions import q_mult, q_from_axis_angle, integrer


@dataclass
class Etat:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))    # monde (m)
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))    # monde (m/s)
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))  # corps (rad/s)


def etat_initial():
    """Fusee au sol, penchee de inclinaison_init_x / _y (config)."""
    q = q_mult(q_from_axis_angle([0, 1, 0], math.radians(C.inclinaison_init_x)),
               q_from_axis_angle([1, 0, 0], math.radians(C.inclinaison_init_y)))
    return Etat(q=q)


def integrer_pas(e, F_world, tau, m, I_body, dt):
    """Euler semi-implicite : vitesses d'abord, puis positions avec les nouvelles vitesses."""
    e.vel = e.vel + F_world / m * dt
    e.pos = e.pos + e.vel * dt
    e.omega = e.omega + (tau - np.cross(e.omega, I_body * e.omega)) / I_body * dt
    e.q = integrer(e.q, e.omega, dt)
    return e
