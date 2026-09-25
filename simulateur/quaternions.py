"""
quaternions.py -- algebre des quaternions (Hamilton, q = corps -> monde).
"""
import math
import numpy as np


def q_mult(a, b):
    """Produit de Hamilton a (x) b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_rotate(q, v):
    """Vecteur du repere CORPS vers le repere MONDE."""
    p = np.array([0.0, v[0], v[1], v[2]])
    return q_mult(q_mult(q, p), q_conj(q))[1:]


def q_rotate_inv(q, v):
    """Vecteur du repere MONDE vers le repere CORPS."""
    p = np.array([0.0, v[0], v[1], v[2]])
    return q_mult(q_mult(q_conj(q), p), q)[1:]


def q_from_axis_angle(axis, angle_rad):
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    s = math.sin(angle_rad / 2.0)
    return np.array([math.cos(angle_rad / 2.0), axis[0]*s, axis[1]*s, axis[2]*s])


def integrer(q, omega, dt):
    """q_dot = 1/2 q (x) (0, omega), Euler explicite, puis renormalisation."""
    q = q + 0.5 * q_mult(q, np.array([0.0, omega[0], omega[1], omega[2]])) * dt
    return q / np.linalg.norm(q)


def erreurs_attitude(q):
    """Erreurs d'attitude (deg) dans le repere CORPS : la verticale vue du corps.
       Penche vers +x_corps -> err_p > 0 ; vers +y_corps -> err_y > 0.
       Reste juste si la fusee tourne en roulis."""
    up_b = q_rotate_inv(q, [0.0, 0.0, 1.0])
    return (math.degrees(math.atan2(-up_b[0], up_b[2])),
            math.degrees(math.atan2(-up_b[1], up_b[2])))


def inclinaison(q):
    """Axe de la fusee dans le monde et ecart a la verticale (deg)."""
    bz = q_rotate(q, [0.0, 0.0, 1.0])
    return bz, math.degrees(math.acos(clamp(bz[2], -1.0, 1.0)))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))
