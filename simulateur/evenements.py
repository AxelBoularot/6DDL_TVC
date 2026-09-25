"""
evenements.py -- decollage, ejection du moteur, parachute a l'apogee.
"""
import numpy as np
import config as C
from .quaternions import q_rotate


class Evenements:
    def __init__(self, dt=None, verbeux=True, stop_apogee=None, stop_combustion=None):
        dt = C.dt if dt is None else dt
        self.verbeux = verbeux
        self.stop_apogee = C.STOP_A_APOGEE if stop_apogee is None else stop_apogee
        self.stop_combustion = C.STOP_FIN_COMBUSTION if stop_combustion is None else stop_combustion
        self.liftoff = False
        self.moteur_present = True
        self.moteur_ejecte = False
        self.t_eject = None
        self.pos_eject = None
        self.n_ejection = int(round(C.dt_ejection / dt))
        self.pas_ejection = 0
        self.parachute = False
        self.t_chute = None
        self.apogee = False
        self.z_prev = 0.0

    def _log(self, msg):
        if self.verbeux:
            print(msg)

    def ejection(self, t, pos, q, x_cg, cg_apres):
        """Largage du moteur en fin de combustion + impulsion de la charge.
           Renvoie (F_monde, tau_corps) a ajouter."""
        F, tau = np.zeros(3), np.zeros(3)
        if C.EJECT_MOTOR and self.moteur_present and t >= C.burn_time_end \
                and not self.moteur_ejecte:
            self.moteur_ejecte = True
            self.t_eject = t
            self.pos_eject = pos.copy()
            self._log(f" >>> EJECTION MOTEUR a t={t:.3f}s | alt={pos[2]:.1f}m "
                  f"| CG saute {x_cg:.3f} -> {cg_apres:.3f} m")
            self.moteur_present = False       # carter retire des le pas suivant
        # impulsion de recul (pousse la fusee vers +body_z), n_ejection pas exacts
        #   F*dt_ejection = 0.54 N.s -> fusee +1.4 m/s, carter ~13.6 m/s vers l'arriere
        if self.moteur_ejecte and self.pas_ejection < self.n_ejection:
            self.pas_ejection += 1
            F = q_rotate(q, np.array([0.0, 0.0, C.F_ejection_max]))
            tau = np.array([C.M_ejection_max, 0.0, 0.0])
        return F, tau

    def verifier_apogee(self, t, pos, vel):
        if (not self.apogee) and t > C.burn_time_end and vel[2] < 0 and pos[2] < self.z_prev:
            self.apogee = True
            if C.DEPLOY_PARACHUTE:
                self.parachute = True
                self.t_chute = t
                self._log(f" >>> PARACHUTE deploye a t={t:.3f}s | apogee={pos[2]:.1f}m")
        self.z_prev = pos[2]

    def verifier_decollage(self, t, F_world, T_force, poids):
        """Pas de rampe : la fusee reste posee tant que la poussee < poids."""
        if not self.liftoff and F_world[2] > 0.0:
            self.liftoff = True
            self._log(f" >>> DECOLLAGE a t={t:.3f}s (poussee {T_force:.2f} N > poids {poids:.2f} N)")
        return self.liftoff

    def arret(self, t, pos, vel, tilt):
        """Conditions d'arret. Renvoie le motif (str) ou None pour continuer."""
        if self.stop_combustion and t >= C.burn_time_end:
            return f"Fin de combustion a t={t:.3f}s -> fin de la simulation"
        if self.stop_apogee and self.liftoff and vel[2] < 0.0:
            return f"Apogee atteinte a t={t:.3f}s -> fin de la simulation"
        if pos[2] < 0.0 and t > 0.5:
            return f"Retour au sol a t={t:.2f}s"
        # perte de controle fatale seulement en vol propulse (apres ejection, la fusee
        # sans ailerons culbute : c'est normal, le parachute gere la descente)
        if tilt > 80 and self.moteur_present:
            return f"Perte de controle en vol propulse (tilt>80) a t={t:.2f}s"
        return None
