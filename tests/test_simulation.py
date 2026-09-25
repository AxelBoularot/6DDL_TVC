"""Tests : python -m pytest
Les tests sont independants des reglages de config.py : les valeurs attendues sont
calculees depuis config, les interrupteurs d'arret sont fixes explicitement, et le
test de stabilite utilise un retard servo de reference (fixture `reference`)."""
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
    pts = np.array(C.courbe_poussee)
    I_attendu = float(np.sum(0.5 * (pts[1:, 1] + pts[:-1, 1]) * np.diff(pts[:, 0])))
    assert m.I_total == pytest.approx(I_attendu, rel=1e-3)
    (t0, f0), (t1, f1) = C.courbe_poussee[1], C.courbe_poussee[2]
    assert m.poussee(t1) == pytest.approx(f1, abs=1e-3)                  # point de la courbe
    assert m.poussee((t0 + t1) / 2) == pytest.approx((f0 + f1) / 2, abs=1e-3)   # interpolation
    assert m.poussee(pts[-1, 0] + 1.0) == 0.0


def test_moteur_lit_config_a_la_creation(monkeypatch):
    """Modifier config.courbe_poussee depuis un script doit changer la poussee."""
    moitie = [(t, F / 2) for t, F in C.courbe_poussee]
    I_ref = Moteur().I_total
    monkeypatch.setattr(C, "courbe_poussee", moitie)
    assert Moteur().I_total == pytest.approx(I_ref / 2, rel=1e-6)


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
    assert pid.P == C.P
    assert pid.calcul(2.0, C.dt) == pytest.approx(C.P * 2.0 + C.I * 2.0 * C.dt)


# ---------------------------------------------------------------- simulation complete
@pytest.fixture
def reference(monkeypatch):
    """Retard servo de reference (30 ms) pour le test de stabilite."""
    monkeypatch.setattr(C, "servoDelay", 0.03)


def test_arret_a_l_apogee(reference):
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


def test_angles_fusee():
    from simulateur.quaternions import angles_fusee
    q = q_from_axis_angle([0, 1, 0], math.radians(5))      # penche de 5 deg vers +X
    ax, ay = angles_fusee(q_rotate(q, [0, 0, 1]))
    assert ax == pytest.approx(5.0) and ay == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------- vols reels
def test_lecture_des_vols():
    import json, os
    from simulateur.vols_reels import lire_vol, DOSSIER_VOLS
    metas = json.load(open(os.path.join(DOSSIER_VOLS, "vols.json"), encoding="utf-8"))["vols"]
    assert len(metas) >= 1
    for meta in metas:
        vol = lire_vol(meta)
        assert vol["alt"].max() > 5.0                             # la fusee a monte
        assert abs(vol["alt"][0]) < 1.0                           # altitude ~0 au debut
        assert len(vol["t_log"]) == len(vol["ang_1"]) == len(vol["tvc_2"])


def test_calage_rotation_libre():
    from simulateur.vols_reels import caler_decollage
    t = np.arange(0, 3, 0.005)
    ang = np.where(t > 1.2, 30.0 * (t - 1.2), 0.0)               # posee, puis tourne a 30 deg/s a 1.2 s
    vol = {"t_log": t, "ang_1": ang, "ang_2": np.zeros_like(t),
           "pression": np.full_like(t, 1013.0), "P0": 1013.0, "t_activite": 0.5}
    t0, methode = caler_decollage(vol, {"t": t, "alt": t}, {})
    assert methode.startswith("rotation libre")
    assert t0 == pytest.approx(1.2, abs=0.03)
    assert caler_decollage(vol, {"t": t, "alt": t}, {"t_decollage": 1.0})[0] == 1.0


def test_debut_activite():
    from simulateur.vols_reels import debut_activite
    t = np.arange(0, 3, 0.005)
    a = np.where(t > 1.5, 2.0, 0.0)                               # repos puis mouvement a 1.5 s
    z = np.zeros_like(t)
    assert debut_activite(t, a, z, z, z) == pytest.approx(1.5, abs=0.01)
    assert debut_activite(t, a + 1.0, z, z, z) == 0.0             # log declenche par la carte


def test_enveloppe_des_tirages():
    from simulateur.vols_reels import enveloppe, CLES_SIM
    t = np.linspace(0, 1, 11)
    sims = [dict({c: np.full(11, float(k)) for c in CLES_SIM}, t=t, t_fin_prop=0.9)
            for k in range(3)]
    env = enveloppe(sims)
    assert env["n"] == 3 and np.allclose(env["ang_1"], 1.0)
    assert np.allclose(env["ang_1_min"], 0.0) and np.allclose(env["ang_1_max"], 2.0)
