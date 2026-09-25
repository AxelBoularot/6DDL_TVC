"""
aero.py -- efforts aerodynamiques (Barrowman etendue) : force normale, amortissement
en tangage, trainee (corps ou parachute).
"""
import math
import numpy as np
import config as C
from .quaternions import q_rotate, q_rotate_inv


def efforts_aero(q, v_air, omega, rho, x_cg, parachute):
    """Renvoie (F_monde, tau_corps, incidence_deg, x_cp)."""
    F_world = np.zeros(3)
    tau = np.zeros(3)

    v_body = q_rotate_inv(q, v_air)
    V = np.linalg.norm(v_body)
    axial = v_body[2]
    lat = np.array([v_body[0], v_body[1], 0.0])
    lat_mag = np.linalg.norm(lat)

    aoa_deg = 0.0
    cp_now = C.x_cp0
    # ---- force normale : nez (potentiel) + corps (crossflow sin^2) ----
    if V > 1e-6 and lat_mag > 1e-9:
        alpha = math.atan2(lat_mag, axial)
        aoa_deg = math.degrees(alpha)
        sa = math.sin(alpha)
        CN_nose = C.Cna_fuselage * sa
        CN_body = C.K_cross * (C.Ap / C.S) * sa * abs(sa)
        Cn = CN_nose + CN_body
        cp_now = (CN_nose * C.x_cp0 + CN_body * C.x_fuse) / Cn if abs(Cn) > 1e-9 else C.x_cp0
        lever = x_cg - cp_now

        q_dyn = 0.5 * rho * V**2
        N_body = -q_dyn * C.S * Cn * (lat / lat_mag)       # s'oppose a la vitesse laterale
        F_world += q_rotate(q, N_body)
        tau += np.cross(np.array([0.0, 0.0, lever]), N_body)

    # ---- amortissement en tangage : des que V > 0, station fixe du nez ----
    if V > 1e-6:
        c_damp = 0.5 * rho * V * C.S * C.Cna_fuselage * (C.x_cp0 - x_cg)**2
        tau += -c_damp * np.array([omega[0], omega[1], 0.0])

    # ---- trainee (corps ou parachute), le long du vent relatif ----
    Vw = np.linalg.norm(v_air)
    if Vw > 1e-6:
        Cd_eff, A_eff = (C.Cd_chute, C.A_chute) if parachute else (C.Cx, C.S)
        F_world += -0.5 * rho * A_eff * Vw**2 * Cd_eff * (v_air / Vw)

    return F_world, tau, aoa_deg, cp_now
