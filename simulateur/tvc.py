"""
tvc.py -- controle TVC : PID (anti-windup), chaine servo (ratio, butee, retard,
jeu mecanique) et poussee orientee.
"""
import math
import numpy as np
import config as C
from .quaternions import erreurs_attitude


class PID:
    """PID sur l'erreur d'attitude (deg). Sortie = angle TUYERE voulu (deg).
       Anti-windup : on n'integre que si la sortie n'est pas en butee
       (ou si l'erreur la fait sortir de la butee)."""
    def __init__(self, e_init=0.0, P=None, I=None, D=None, limite=None):
        # valeurs lues dans config AU MOMENT de la creation (pas a l'import) :
        # un script peut modifier config.P etc. avant de lancer simuler()
        self.P = C.P if P is None else P
        self.I = C.I if I is None else I
        self.D = C.D if D is None else D
        self.limite = C.angle_max_tvc if limite is None else limite
        self.integ = 0.0
        self.e_prev = e_init          # pas de "coup de derivee" au 1er pas

    def calcul(self, e, dt):
        d = (e - self.e_prev) / dt
        self.e_prev = e
        u = self.P * e + self.I * (self.integ + e * dt) + self.D * d
        if abs(u) < self.limite or u * e < 0:
            self.integ += e * dt
        return self.P * e + self.I * self.integ + self.D * d


class ChaineServo:
    """Angle tuyere voulu -> x TVC_RATIO -> angle servo (butee +/- max*ratio)
       -> retard servoDelay -> / TVC_RATIO -> jeu mecanique -> angle tuyere reel."""
    def __init__(self, dt=None):
        dt = C.dt if dt is None else dt
        self.n_retard = int(round(C.servoDelay / dt))
        self.histo = [0.0] * self.n_retard   # angles SERVO en attente
        self.i = 0
        self.tuyere = 0.0                     # angle tuyere reel (deg)

    @staticmethod
    def vers_servo(alpha_cible):
        """Angle TUYERE voulu -> angle SERVO, borne a la butee servo."""
        alpha_k = alpha_cible * C.TVC_RATIO
        lim = C.angle_max_tvc * C.TVC_RATIO
        if alpha_k > lim:
            alpha_k = lim
        elif alpha_k < -lim:
            alpha_k = -lim
        return alpha_k

    def jeu(self, cible):
        """Jeu mecanique (backlash) de largeur totale tvc_play : la tuyere ne bouge
           que si la commande sort de [reel - jeu/2, reel + jeu/2], puis suit avec un
           ecart de jeu/2. Hypothese : dans le jeu, la tuyere reste ou elle a ete poussee."""
        diff = cible - self.tuyere
        if diff > C.tvc_play / 2:
            self.tuyere = cible - C.tvc_play / 2
        elif diff < -C.tvc_play / 2:
            self.tuyere = cible + C.tvc_play / 2
        return self.tuyere

    def pas(self, alpha_cible):
        self.histo.append(self.vers_servo(alpha_cible))
        servo_retarde = self.histo[self.i]
        self.i += 1
        return self.jeu(servo_retarde / C.TVC_RATIO)


def poussee_tvc(T, dp_deg, dy_deg, L):
    """Poussee orientee (repere corps) et couple au CG. Tuyere en (0,0,-L).
       Cardan : rotation dy (autour de x_b) puis dp (autour de y_b) -> vecteur unitaire."""
    dp, dy = math.radians(dp_deg), math.radians(dy_deg)
    F = T * np.array([math.sin(dp) * math.cos(dy),
                      math.sin(dy),
                      math.cos(dp) * math.cos(dy)])
    tau = np.cross(np.array([0.0, 0.0, -L]), F)
    return F, tau


class ControleurTVC:
    """Deux axes (pitch, yaw) : erreur d'attitude -> PID -> chaine servo -> tuyere.
       Si couper=True, a la fin de la combustion le PID est coupe et la consigne
       tuyere passe a 0 (retour au centre a travers le retard et le jeu)."""
    def __init__(self, q0, dt=None, couper=None):
        e0_p, e0_y = erreurs_attitude(q0)
        self.pid_p, self.pid_y = PID(e0_p), PID(e0_y)
        self.servo_p, self.servo_y = ChaineServo(dt), ChaineServo(dt)
        self.couper = C.COUPER_TVC_FIN_COMBUSTION if couper is None else couper

    def pas(self, t, q, dt):
        """Renvoie les angles tuyere reels (dp, dy) en degres."""
        if self.couper and t >= C.burn_time_end:
            cmd_p = cmd_y = 0.0
        else:
            err_p, err_y = erreurs_attitude(q)
            cmd_p, cmd_y = self.pid_p.calcul(err_p, dt), self.pid_y.calcul(err_y, dt)
        return self.servo_p.pas(cmd_p), self.servo_y.pas(cmd_y)
