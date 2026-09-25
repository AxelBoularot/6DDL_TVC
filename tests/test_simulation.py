"""Tests : python -m pytest
Les tests fixent explicitement les interrupteurs d'arret : ils passent quels que
soient les reglages de config.py."""
import math
import numpy as np
import pytest

import config as C
from simulateur.quaternions import q_from_axis_angle, q_rotate, q_rotate_inv, q_mult, erreurs_attitude
from simulateur.moteur import Moteur
from simulateur.masse import proprietes_massiques
from simulateur.tvc import ChaineServo, PID
from simulateur.simulation import simuler


# ---------------------------------------------------------------- quaternions
def test_rotation_aller_retour():
    q = q_mult(q_from_axis_angle([0, 1, 0], 0.3), q_from_axis_angle([1, 0, 0], -0.2))
    v = np.array([0.3, -1.2, 2.0])
    assert np.allclose(q_rotate_inv(q, q_rotate(q, v)), v)
    assert math.isclose(np.linalg.norm(q_rotate(q, v)), np.linalg.norm(v))


def test_signe_erreurs_attitude():
    # penche de 2 deg vers +X (rotation autour de +Y) -> err_p = +2, err_y = 0
    ep, ey = erreurs_attitude(q_from_axis_angle([0, 1, 0], math.radians(2)))
    assert ep == pytest.approx(2.0) and ey == pytest.approx(0.0, abs=1e-12)
    # penche vers +Y (rotation de -2 deg autour de +X) -> err_y = +2
    ep, ey = erreurs_attitude(q_from_axis_angle([1, 0, 0], math.radians(-2)))
    assert ey == pytest.approx(2.0) and ep == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------- moteur / masse
def test_impulsion_et_poussee():
    m = Moteur()
    assert m.I_total == pytest.approx(19.87, abs=0.01)
    assert m.poussee(0.2) == pytest.approx(25.0)          # point exact de la courbe
    assert m.poussee(0.075) == pytest.approx(6.25)        # interpolation lineaire
    assert m.poussee(3.0) == 0.0


def test_masses():
    m = Moteur()
    m0 = proprietes_massiques(0.0, m, True)[0]
    assert m0 == pytest.approx(C.m_struct + C.m_carter + C.m_prop0)
    assert proprietes_massiques(C.burn_time_end, m, True)[0] == pytest.approx(C.m_struct + C.m_carter)
    assert proprietes_massiques(C.burn_time_end, m, False)[0] == pytest.approx(C.m_struct)


# ---------------------------------------------------------------- chaine TVC
def test_jeu_mecanique():
    s = ChaineServo()
    assert s.jeu(0.2) == 0.0                              # dans le jeu : la tuyere ne bouge pas
    assert s.jeu(1.0) == pytest.approx(1.0 - C.tvc_play / 2)   # suit avec jeu/2 d'ecart
    assert s.jeu(1.0 - C.tvc_play + 0.01) == pytest.approx(1.0 - C.tvc_play / 2)  # retour < jeu
    assert s.jeu(0.0) == pytest.approx(C.tvc_play / 2)


def test_butee_et_ratio():
    assert ChaineServo.vers_servo(50.0) == pytest.approx(C.angle_max_tvc * C.TVC_RATIO)
    assert ChaineServo.vers_servo(1.0) == pytest.approx(C.TVC_RATIO)


def test_retard_servo():
    s = ChaineServo()
    n = int(round(C.servoDelay / C.dt))
    sorties = [s.pas(5.0) for _ in range(n + 1)]
    assert all(v == 0.0 for v in sorties[:n])             # rien pendant le retard
    assert sorties[n] == pytest.approx(5.0 - C.tvc_play / 2)


def test_pid_pas_de_coup_de_derivee():
    pid = PID(e_init=2.0)
    assert pid.calcul(2.0, C.dt) == pytest.approx(C.P * 2.0 + C.I * 2.0 * C.dt)


# ---------------------------------------------------------------- simulation complete
def test_arret_a_l_apogee():
    hist, evts, _ = simuler(seed=0, verbeux=False, stop_apogee=True, stop_combustion=False)
    z = hist['z']
    assert z[-1] == pytest.approx(z.max(), abs=0.01)     # dernier point = apogee
    assert 20.0 < z.max() < 80.0
    assert hist['tilt'][hist['t'] < C.burn_time_end].max() < 15.0   # TVC stable


def test_reproductible():
    h1, _, _ = simuler(seed=3, verbeux=False)
    h2, _, _ = simuler(seed=3, verbeux=False)
    assert np.array_equal(h1['z'], h2['z'])


def test_arret_fin_combustion():
    hist, _, _ = simuler(seed=0, verbeux=False, stop_combustion=True)
    assert hist['t'][-1] == pytest.approx(C.burn_time_end, abs=2 * C.dt)


def test_tvc_coupe_fin_combustion():
    hist, _, _ = simuler(seed=0, verbeux=False, couper_tvc=True,
                         stop_apogee=False, stop_combustion=False)
    apres = hist['t'] > C.burn_time_end + C.servoDelay + 0.01
    # la tuyere revient au centre, a jeu/2 pres
    assert np.abs(hist['dp'][apres]).max() <= C.tvc_play / 2 + 1e-9
    assert np.abs(hist['dy'][apres]).max() <= C.tvc_play / 2 + 1e-9
