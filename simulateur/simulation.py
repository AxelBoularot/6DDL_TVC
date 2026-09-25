"""
simulation.py -- boucle principale : assemble les modules a chaque pas de temps.
    masse -> commande TVC -> efforts (poussee, aero, bruit, ejection, gravite)
    -> evenements -> integration 6 DDL -> historique -> conditions d'arret
"""
import numpy as np
import config as C
from .quaternions import q_rotate, inclinaison, angles_fusee
from .moteur import Moteur
from .masse import proprietes_massiques
from .environnement import densite_air, Vent, bruit_couple
from .aero import efforts_aero
from .tvc import ControleurTVC, poussee_tvc
from .evenements import Evenements
from .dynamique import etat_initial, integrer_pas
from .historique import Historique


def simuler(dt=None, duree=None, seed=None, verbeux=True, stop_apogee=None,
            stop_combustion=None, couper_tvc=None):
    """Lance une simulation. Renvoie (hist, evts, moteur).
       stop_apogee, stop_combustion, couper_tvc : None -> valeur de config.py
       (STOP_A_APOGEE, STOP_FIN_COMBUSTION, COUPER_TVC_FIN_COMBUSTION) ; True/False pour forcer."""
    def choix(v, defaut):
        return defaut if v is None else v

    dt = C.dt if dt is None else dt
    duree = C.duree if duree is None else duree
    seed = C.SEED if seed is None else seed
    if seed is not None:
        np.random.seed(seed)
    moteur = Moteur(dt=dt)
    if verbeux:
        print(f"Impulsion totale : {moteur.I_total:.2f} N.s")

    e = etat_initial()
    vent = Vent()
    tvc = ControleurTVC(e.q, dt, couper=choix(couper_tvc, C.COUPER_TVC_FIN_COMBUSTION))
    evts = Evenements(dt, verbeux,
                      stop_apogee=choix(stop_apogee, C.STOP_A_APOGEE),
                      stop_combustion=choix(stop_combustion, C.STOP_FIN_COMBUSTION))
    hist = Historique()

    for i in range(int(duree / dt)):
        t = i * dt

        # ---- masse, poussee, air ----
        mass, x_cg, Ir, Ia = proprietes_massiques(t, moteur, evts.moteur_present)
        T_force = moteur.poussee(t) if evts.moteur_present else 0.0
        rho = densite_air(e.pos[2])

        # ---- commande TVC ----
        bz_world, tilt = inclinaison(e.q)
        ang_x, ang_y = angles_fusee(bz_world)
        dp, dy = tvc.pas(t, e.q, dt)

        # ---- efforts ----
        F_world, tau = np.zeros(3), np.zeros(3)
        if T_force > 0.0:
            F_b, tau_T = poussee_tvc(T_force, dp, dy, C.x_nozzle - x_cg)
            F_world += q_rotate(e.q, F_b)
            tau += tau_T
        v_air = e.vel - vent.pas(dt)
        F_a, tau_a, aoa_deg, cp_now = efforts_aero(e.q, v_air, e.omega, rho, x_cg, evts.parachute)
        F_world += F_a
        tau += tau_a + bruit_couple(dt)
        F_ej, tau_ej = evts.ejection(t, e.pos, e.q, x_cg,
                                     proprietes_massiques(t, moteur, False)[1])
        F_world += F_ej
        tau += tau_ej
        evts.verifier_apogee(t, e.pos, e.vel)
        F_world += np.array([0.0, 0.0, -mass * C.g])

        # ---- integration (au sol tant que poussee < poids) ----
        if evts.verifier_decollage(t, F_world, T_force, mass * C.g):
            integrer_pas(e, F_world, tau, mass, np.array([Ir, Ir, Ia]), dt)

        hist.ajouter(t=t, x=e.pos[0], y=e.pos[1], z=e.pos[2], tilt=tilt,
                     ang_x=ang_x, ang_y=ang_y, dp=dp, dy=dy,
                     aoa=aoa_deg, cp=cp_now, mass=mass, xcg=x_cg, Ir=Ir,
                     bzx=bz_world[0], bzy=bz_world[1], bzz=bz_world[2])

        motif = evts.arret(t, e.pos, e.vel, tilt)
        if motif:
            if verbeux:
                print(motif)
            break

    hist = hist.tableaux()
    if verbeux:
        print(f"Apogee: {hist['z'].max():.2f}m | tilt max: {hist['tilt'].max():.1f}deg "
              f"| duree: {hist['t'][-1]:.2f}s")
    return hist, evts, moteur
