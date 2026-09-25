"""
config.py -- TOUS les parametres de la simulation (a modifier ici uniquement).
Unites SI sauf mention (angles en degres pour la TVC).
"""
import math

# ============================================================================
# INTERRUPTEURS
# ============================================================================
EJECT_MOTOR      = True    # ejecter le moteur en fin de combustion ?
DEPLOY_PARACHUTE = True    # deploiement parachute a l'apogee
USE_ISA          = True    # densite de l'air variable avec l'altitude
SEED             = None    # graine aleatoire (None = tirage different a chaque lancement)
STOP_A_APOGEE    = True    # arreter la simulation a l'apogee (sinon : jusqu'au sol)
STOP_FIN_COMBUSTION = True # arreter la simulation a la fin de la combustion (prioritaire)
COUPER_TVC_FIN_COMBUSTION = True  # fin de combustion : PID coupe, tuyere ramenee au centre

# ============================================================================
# SIMULATION
# ============================================================================
dt     = 0.001             # pas de temps (s)
duree  = 16.0              # duree max (s)

# ============================================================================
# PID (un par axe : pitch et yaw). Sortie = angle TUYERE voulu (deg)
# ============================================================================
P = 0.6
I = 0.05
D = 0.113

# ============================================================================
# GEOMETRIE / MASSES
# ============================================================================
l = 0.42                   # longueur du corps (m)
r = 0.0375                 # rayon du corps (m)
S = math.pi * r**2         # section de reference (m2)

m_prop0  = 0.060           # propergol plein (kg)
m_carter = 0.040           # carter moteur vide, LARGABLE (kg)
m_struct = 0.391           # structure fixe (kg)

# positions le long de l'axe, depuis le NEZ (m)
x_struct = 0.200           # CG structure
x_motor  = 0.400           # CG carter moteur
x_prop   = 0.400           # CG propergol
x_nozzle = l               # tuyere = arriere du moteur -> A REMPLACER par la valeur mesuree
r_motor  = 0.012           # rayon du moteur (D9 : 24 mm)

I_struct_own = 0.00248     # inertie propre de la structure (kg.m2)

# ============================================================================
# AERO (Barrowman etendue)
# ============================================================================
Cx           = 0.5
Cna_fuselage = 2.0         # terme potentiel du nez (par rad)
x_cp0        = 0.04        # CP du terme potentiel (m depuis le nez)
x_fuse       = 0.25        # centroide de l'aire en plan (CP du terme visqueux)
K_cross      = 1.1
Ap           = l * (2 * r) # aire en plan du corps

# ============================================================================
# TVC : PID -> x TVC_RATIO -> servo (butee) -> retard -> / TVC_RATIO -> jeu -> tuyere
# ============================================================================
angle_max_tvc = 10         # butee TUYERE par axe (deg)
TVC_RATIO     = 2.2        # angle servo / angle tuyere
servoDelay    = 0.07       # retard servo (s)
tvc_play      = 0.5        # jeu mecanique TOTAL mesure a la TUYERE (deg)
                           # (mesure au servo : tvc_play = jeu_servo / TVC_RATIO)

# ============================================================================
# ENVIRONNEMENT
# ============================================================================
g    = 9.81
rho0 = 1.225

# vent : moyenne + rafales (Ornstein-Uhlenbeck), repere monde
wind_mean       = (0.0, 0.0, 0.0)
wind_gust_sigma = 2.0      # ecart-type des rafales (m/s)
wind_tau        = 0.3      # temps de correlation (s)

# bruit de couple parasite (N.m), ecart-type DEFINI pour dt_ref_noise
torque_noise_sigma = 2e-3
dt_ref_noise       = 0.001

# ============================================================================
# MOTEUR : courbe de poussee (t [s], F [N])
# ============================================================================
courbe_poussee = [
    (0.000,  0.0), (0.050,  2.5), (0.100, 10.0), (0.150, 15.0),
    (0.200, 25.0), (0.250, 22.5), (0.350, 12.5), (0.500,  9.0),
    (0.750,  9.0), (1.000,  9.0), (1.250,  9.0), (1.500,  9.0),
    (1.750,  9.0), (1.900,  8.0), (2.000,  4.0), (2.100,  0.0)
]
burn_time_end = 2.1        # fin de combustion (s)

# ============================================================================
# EJECTION / PARACHUTE
# ============================================================================
P_ejection  = 1.5e5        # pression de la charge (Pa)
dt_ejection = 0.008        # duree de la poussee de la charge (s)
offset_lat  = 0.004        # desalignement de la charge (m) -> couple
A_motor        = math.pi * r_motor**2
F_ejection_max = P_ejection * A_motor          # ~68 N sur la section du MOTEUR
M_ejection_max = F_ejection_max * offset_lat

Cd_chute = 1.5
A_chute  = 0.20            # m2

# ============================================================================
# ETAT INITIAL
# ============================================================================
inclinaison_init_x = 2.0   # deg, penche vers +X (rotation autour de +Y)
inclinaison_init_y = -2.0  # deg, rotation autour de +X (-2 deg -> penche vers +Y)
