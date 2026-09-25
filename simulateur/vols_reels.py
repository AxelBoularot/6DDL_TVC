"""
vols_reels.py -- comparaison entre les vols reels (logs de la carte de vol) et le simulateur.

Chaque vol de vols/vols.json est :
  1. lu (CSV de la carte de vol), le decollage est detecte, l'altitude est tiree de la pression ;
  2. rejoue dans le simulateur avec SON retard servo et ses angles initiaux ;
  3. trace (angles, tuyeres, altitude : reel contre simulation) dans docs/ ;
  4. resume dans un tableau (angle max, oscillation, saturation tuyere, altitude)
     reecrit dans le README entre les balises COMPARAISON:DEBUT et COMPARAISON:FIN.

Calage du decollage : les tuyeres bougent avant le decollage (servos armes), mais
l'attitude, elle, ne change pas tant que la fusee est posee. Le decollage est donc cale
sur le debut de la rotation libre (voir caler_decollage), ou impose par "t_decollage"
dans vols.json. Le barometre sert de controle.

    python main.py --comparer
"""
import json
import os
import numpy as np
import config as C

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOSSIER_VOLS = os.path.join(RACINE, "vols")
DOSSIER_DOCS = os.path.join(RACINE, "docs")
README = os.path.join(RACINE, "README.md")
DEBUT, FIN = "<!-- COMPARAISON:DEBUT -->", "<!-- COMPARAISON:FIN -->"
SEUIL_SAT = 9.5          # |tuyere| >= 9.5 deg : consideree en butee
SEUIL_MONTEE_HPA = 0.2   # chute de pression qui signe la montee (~1.7 m)
RESOLUTION_HPA = 0.1     # pas du barometre de la carte


# ============================================================================
# LECTURE D'UN LOG
# ============================================================================
def altitude_pression(P, P0):
    """Altitude barometrique (atmosphere standard), m."""
    return 44330.0 * (1.0 - (P / P0) ** 0.1903)


def debut_activite(t, a1, a2, s1, s2, seuil=0.3, calme=0.5):
    """Premier mouvement des angles ou des tuyeres (s). Si le log commence par une phase
       au repos (>= `calme` s), c'est le premier echantillon qui bouge ; sinon le log a ete
       declenche par la carte et c'est son debut. Sert seulement a estimer la pression
       au sol : le decollage lui-meme est cale sur la pression (voir caler_decollage)."""
    bouge = (np.abs(a1) > seuil) | (np.abs(a2) > seuil) | (np.abs(s1) > seuil) | (np.abs(s2) > seuil)
    i = int(np.argmax(bouge)) if bouge.any() else 0
    return t[i] if t[i] - t[0] >= calme else t[0]


def lire_vol(meta):
    """Lit un log. Temps en secondes depuis le debut du log (le calage sur le decollage
       est fait ensuite par caler_decollage, a partir de la montee barometrique)."""
    d = np.genfromtxt(os.path.join(DOSSIER_VOLS, meta["fichier"]), delimiter=";")
    d = d[1:]                                   # 1re ligne = initialisation de la carte
    t = (d[:, 0] - d[0, 0]) / 1000.0
    a1, a2, s1, s2 = d[:, 3], d[:, 4], d[:, 5], d[:, 6]
    t_act = debut_activite(t, a1, a2, s1, s2)
    au_sol = t <= t_act
    P0 = float(np.median(d[au_sol, 2])) if au_sol.sum() >= 5 else float(np.median(d[:5, 2]))
    return {"t_log": t, "ang_1": a1, "ang_2": a2, "tvc_1": s1, "tvc_2": s2,
            "alt": altitude_pression(d[:, 2], P0), "pression": d[:, 2], "P0": P0,
            "temp": d[:, 1], "t_activite": t_act}


def debut_rotation(t, a1, a2, seuil=10.0, duree=0.05, repos=3.0):
    """Debut de la rotation libre (s) : tant que la fusee est posee, son attitude ne
       bouge pas meme si les tuyeres bougent. On cherche la vitesse angulaire (lissee)
       au-dessus de `seuil` deg/s pendant `duree` s, puis on remonte au dernier instant
       ou elle etait sous `repos` deg/s. Renvoie None si rien n'est trouve."""
    w = np.hypot(np.gradient(a1, t), np.gradient(a2, t))
    w = np.convolve(w, np.ones(5) / 5, mode="same")
    n = max(int(round(duree / max(np.median(np.diff(t)), 1e-3))), 1)
    fort = np.convolve((w > seuil).astype(int), np.ones(n, int), mode="full")[:len(w)] >= n
    if not fort.any():
        return None
    i = int(np.argmax(fort)) - n + 1
    calme = np.where(w[:i] < repos)[0]
    return float(t[calme[-1]]) if len(calme) else float(t[0])


def t_montee_barometre(vol, sim, dP=SEUIL_MONTEE_HPA):
    """Estimation par le barometre (controle) : premier echantillon a P0 - dP, moins le
       temps de montee simule. Biaisee vers le tard (pas de 0.1 hPa ~ 0.85 m, montee reelle
       plus lente que la simulee) : sert seulement de verification."""
    monte = vol["pression"] <= vol["P0"] - dP + 1e-9
    if not monte.any():
        return None
    t_reel = vol["t_log"][int(np.argmax(monte))]
    h = altitude_pression(vol["P0"] - (dP - RESOLUTION_HPA / 2), vol["P0"])
    return float(t_reel - sim["t"][int(np.argmax(sim["alt"] >= h))])


def caler_decollage(vol, sim, meta):
    """Instant du decollage dans le log (s) et methode utilisee.
       1. "t_decollage" dans vols.json s'il est donne ;
       2. sinon le debut de la rotation libre (la fusee posee ne tourne pas) ;
       3. a defaut, la montee barometrique."""
    if meta.get("t_decollage") is not None:
        return float(meta["t_decollage"]), "impose (vols.json)"
    t_rot = debut_rotation(vol["t_log"], vol["ang_1"], vol["ang_2"])
    t_baro = t_montee_barometre(vol, sim)
    ctrl = f", barometre {t_baro:.2f}s" if t_baro is not None else ""
    if t_rot is not None:
        return t_rot, f"rotation libre{ctrl}"
    if t_baro is not None:
        return t_baro, "barometre"
    return vol["t_activite"], "premier mouvement"


def recaler(vol, t0, avant=0.3):
    """Temps remis a 0 au decollage ; on garde `avant` s de log avant le decollage."""
    k = vol["t_log"] >= t0 - avant
    out = {c: vol[c][k] for c in ("ang_1", "ang_2", "tvc_1", "tvc_2", "alt", "temp")}
    out["t"] = vol["t_log"][k] - t0
    out["t0"] = t0
    return out


# ============================================================================
# SIMULATION DU MEME VOL
# ============================================================================
def simuler_vol(meta, vol, seed=0):
    """Rejoue le vol : retard servo du vol, angles initiaux mesures, repere de la carte."""
    from .simulation import simuler
    sauve = (C.servoDelay, C.inclinaison_init_x, C.inclinaison_init_y)
    signe = meta.get("signe", -1)
    try:
        C.servoDelay = meta["servoDelay"]
        # angles initiaux mesures, ramenes dans le repere du simulateur
        i = int(np.argmax(vol["t_log"] >= vol["t_activite"]))
        C.inclinaison_init_x = signe * float(vol["ang_1"][i])
        C.inclinaison_init_y = -signe * float(vol["ang_2"][i])
        # TVC non coupe a la fin de combustion : la carte de vol continue de piloter
        hist, evts, _ = simuler(seed=seed, verbeux=False, stop_apogee=True,
                                stop_combustion=False, couper_tvc=False)
    finally:
        C.servoDelay, C.inclinaison_init_x, C.inclinaison_init_y = sauve
    i0 = int(np.argmax(np.abs(hist['z']) > 1e-9))       # decollage simule
    t_decollage = hist['t'][max(i0 - 1, 0)]
    t = hist['t'] - t_decollage
    return {"t_fin_prop": C.burn_time_end - t_decollage,   # fin de combustion, depuis le decollage
            "t": t, "ang_1": signe * hist['ang_x'], "ang_2": signe * hist['ang_y'],
            "tvc_1": signe * hist['dp'], "tvc_2": signe * hist['dy'], "alt": hist['z']}


# ============================================================================
# PLUSIEURS TIRAGES (vent en rafales + bruit de couple sont aleatoires)
# ============================================================================
CLES_SIM = ("ang_1", "ang_2", "tvc_1", "tvc_2", "alt")


def enveloppe(sims):
    """Regroupe plusieurs simulations du meme vol : mediane, min et max a chaque instant
       (toutes les simulations partagent la meme base de temps jusqu'a la plus courte)."""
    n = min(len(x["t"]) for x in sims)
    env = {"t": sims[0]["t"][:n], "t_fin_prop": sims[0]["t_fin_prop"], "n": len(sims)}
    for c in CLES_SIM:
        pile = np.vstack([x[c][:n] for x in sims])
        env[c], env[c + "_min"], env[c + "_max"] = (np.median(pile, 0), pile.min(0), pile.max(0))
        env[c + "_sims"] = pile
    return env


# ============================================================================
# INDICATEURS
# ============================================================================
def indicateurs(s, t_fin):
    """Pendant la propulsion, sur [0, t_fin] : angle max, oscillation (valeur efficace
       de la vitesse angulaire), part du temps tuyere en butee, altitude atteinte."""
    k = (s["t"] >= 0) & (s["t"] <= t_fin)
    t = s["t"][k]
    ang = np.hypot(s["ang_1"][k], s["ang_2"][k])
    tvc = np.maximum(np.abs(s["tvc_1"][k]), np.abs(s["tvc_2"][k]))
    rate = np.hypot(np.gradient(s["ang_1"][k], t), np.gradient(s["ang_2"][k], t))
    return {"ang_max": float(ang.max()), "osc": float(np.sqrt(np.mean(rate**2))),
            "sat": 100.0 * float(np.mean(tvc >= SEUIL_SAT)), "alt": float(s["alt"][k].max())}


# ============================================================================
# FIGURES (textes en anglais : elles sont publiees dans le README)
# ============================================================================
# une couleur par vol, dans l'ordre de vols.json (palette validee daltonisme) ;
# la simulation est toujours en gris : couleur = mesure, gris = simulation
COULEURS_VOLS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
GRIS_BANDE, GRIS_MEDIANE, ENCRE = "#c8c8c8", "#4a4a4a", "#222222"
AVANT = 0.2   # s montrees avant le decollage


def _style(ax):
    ax.grid(alpha=0.25, lw=0.6)
    for cote in ("top", "right"):
        ax.spines[cote].set_visible(False)
    ax.axvline(0, color=ENCRE, lw=0.8, ls=":")


def _legende_commune(n):
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    return [Patch(color=GRIS_BANDE, label=f"Simulation: range of {n} runs"),
            Line2D([], [], color=GRIS_MEDIANE, lw=1.4, ls="--", label="Simulation: median")]


def _etiquettes_fin(ax, items, ymin, ymax):
    """Etiquettes directes en bout de courbe, ecartees pour ne pas se chevaucher."""
    ecart = 0.06 * (ymax - ymin)
    items = sorted(items, key=lambda it: it[1])
    ys = []
    for _, y, _ in items:
        y = max(y, ys[-1] + ecart) if ys else y
        ys.append(min(y, ymax - ecart * 0.5))
    for i in range(len(ys) - 2, -1, -1):          # repasse vers le bas si on a tape le haut
        ys[i] = min(ys[i], ys[i + 1] - ecart)
    for (x, _, texte), y in zip(items, ys):
        ax.annotate(texte, (x, y), xytext=(6, 0), textcoords="offset points",
                    va="center", fontsize=9, color=ENCRE, annotation_clip=False)


def tracer_vol(meta, vol, base, fichier, seed=0):
    """Un vol : angles et tuyeres, axe 1 et axe 2, reel contre UNE simulation faite avec
       les parametres de config.py (retard servo et angles initiaux du vol, graine `seed`)."""
    import matplotlib.pyplot as plt
    t_prop = base["t_fin_prop"]
    # simulation montree sur la meme duree que le log du vol
    k = (base["t"] >= vol["t"][0]) & (base["t"] <= vol["t"][-1])
    fig, axs = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    for ax, cle, titre in [(axs[0, 0], "ang_1", "Rocket angle, axis 1 (deg)"),
                           (axs[0, 1], "ang_2", "Rocket angle, axis 2 (deg)"),
                           (axs[1, 0], "tvc_1", "Nozzle angle, axis 1 (deg)"),
                           (axs[1, 1], "tvc_2", "Nozzle angle, axis 2 (deg)")]:
        ax.plot(base["t"][k], base[cle][k], color="#d9534f", lw=1.4,
                label=f"simulation (config.py parameters, seed {seed})")
        ax.plot(vol["t"], vol[cle], color="#1f4e9c", lw=1.6, label="measured flight")
        ax.axhline(0, color="k", lw=0.6, ls=":")
        ax.axvline(t_prop, color="gray", lw=0.8, ls="--")
        ax.set_title(titre); ax.grid(alpha=0.3)
        if cle.startswith("ang"):
            # echelle : vol reel + simulation pendant la propulsion (apres, elle culbute)
            kp = (base["t"] >= 0) & (base["t"] <= t_prop)
            m = max(np.abs(vol[cle]).max(), np.abs(base[cle][kp]).max(), 10.0) * 1.15
            ax.set_ylim(-m, m)
        else:
            ax.set_ylim(-C.angle_max_tvc * 1.15, C.angle_max_tvc * 1.15)
    axs[0, 0].text(t_prop, 0, " end of\n burn", fontsize=8, color="gray",
                   va="bottom")
    for ax in axs[1]:
        ax.set_xlabel("time since lift-off (s)")
        ax.set_xlim(vol["t"][0], vol["t"][-1])
        ax.axhline(C.angle_max_tvc, color="gray", lw=0.6); ax.axhline(-C.angle_max_tvc, color="gray", lw=0.6)
    axs[0, 0].legend(loc="upper left")
    fig.suptitle(f"{meta['nom']} — servo delay {meta['servoDelay']*1000:.0f} ms: measured vs simulated",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(fichier, dpi=100)
    plt.close(fig)


def tracer_synthese(resultats, fichier):
    """Un panneau par retard servo : l'angle total mesure de chaque vol, par-dessus
       l'enveloppe des simulations faites avec ce retard (les vols au meme retard ont la
       meme simulation : seules leurs conditions reelles, non mesurees, les distinguent)."""
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    groupes = {}
    for i, (meta, vol, sim, _, _) in enumerate(resultats):
        groupes.setdefault(meta["servoDelay"], []).append((i, meta, vol, sim))
    retards = sorted(groupes, reverse=True)
    t_prop = resultats[0][2]["t_fin_prop"]
    # chaque panneau couvre la meme duree que les logs de ses vols
    t_fin = {r: max(v["t"][-1] for _, _, v, _ in groupes[r]) for r in retards}
    # echelle commune : on compare les retards entre eux
    ymax = 0.0
    for meta, vol, sim, _, _ in resultats:
        ks = (sim["t"] >= -AVANT) & (sim["t"] <= t_fin[meta["servoDelay"]])
        ymax = max(ymax, np.hypot(vol["ang_1"], vol["ang_2"])[vol["t"] >= -AVANT].max(),
                   np.median(np.hypot(sim["ang_1_sims"], sim["ang_2_sims"])[:, ks], 0).max())
    ymax = min(ymax * 1.08, 100.0)
    fig, axs = plt.subplots(1, len(retards), figsize=(6.8 * len(retards), 5.6),
                            sharey=True, squeeze=False)
    for ax, retard in zip(axs[0], retards):
        vols = groupes[retard]
        sim = vols[0][3]
        ks = (sim["t"] >= -AVANT) & (sim["t"] <= t_fin[retard])
        ax.axvspan(t_prop, t_fin[retard], color="#f0f0f0", lw=0, zorder=0)
        ax.text(t_prop, ymax, " after burnout", fontsize=8, color=ENCRE, va="top")
        tot = np.hypot(sim["ang_1_sims"], sim["ang_2_sims"])[:, ks]
        ax.fill_between(sim["t"][ks], tot.min(0), tot.max(0), color=GRIS_BANDE, lw=0)
        ax.plot(sim["t"][ks], np.median(tot, 0), color=GRIS_MEDIANE, lw=1.4, ls="--")
        poignees, etiquettes = [], []
        for i, meta, vol, _ in vols:
            c = COULEURS_VOLS[i % len(COULEURS_VOLS)]
            kv = vol["t"] >= -AVANT
            tv, av = vol["t"][kv], np.hypot(vol["ang_1"], vol["ang_2"])[kv]
            ax.plot(tv, av, color=c, lw=2)
            ax.plot(tv[-1], av[-1], "o", color=c, ms=5)
            etiquettes.append((tv[-1], av[-1], meta["nom"]))
            poignees.append(Line2D([], [], color=c, lw=2, label=f"{meta['nom']}: measured"))
        _style(ax)
        ax.set_xlim(-AVANT, t_fin[retard])
        ax.set_ylim(0, ymax)
        _etiquettes_fin(ax, etiquettes, 0, ymax)
        noms = ", ".join(m["nom"].split()[-1] for _, m, _, _ in vols)
        ax.set_title(f"Servo delay {retard*1000:.0f} ms  ·  "
                     f"{'Flight' if len(vols) == 1 else 'Flights'} {noms}",
                     loc="left", fontsize=12)
        ax.set_xlabel("Time since lift-off (s)")
        ax.legend(handles=poignees + _legende_commune(sim["n"]), loc="upper left",
                  frameon=False, fontsize=9)
    axs[0, 0].set_ylabel("Tilt from vertical (deg)")
    fig.suptitle("Rocket tilt over the whole flight log: measured flights (colour) "
                 "vs simulation with the same servo delay (grey)", fontsize=13)
    fig.tight_layout()
    fig.subplots_adjust(right=0.95)
    fig.savefig(fichier, dpi=100)
    plt.close(fig)


# ============================================================================
# README
# ============================================================================
def tableau_markdown(resultats):
    t_prop = resultats[0][2]["t_fin_prop"]
    lignes = [f"From lift-off to burnout ({t_prop:.2f} s), **measured** / simulated:", "",
              "| Flight | Servo delay | Max tilt | Oscillation (RMS rate, °/s) "
              "| Nozzle at end stop | Altitude at burnout |",
              "|---|---|---|---|---|---|"]
    def f(isim, cle, fmt):
        _, lo, hi, b = isim[cle]
        return f"{fmt.format(b)} ({fmt.format(lo)}–{fmt.format(hi)})"
    for meta, vol, sim, ir, isim in resultats:
        lignes.append(f"| {meta['nom']} | {meta['servoDelay']*1000:.0f} ms "
                      f"| **{ir['ang_max']:.1f}°** / {f(isim, 'ang_max', '{:.1f}')}° "
                      f"| **{ir['osc']:.0f}** / {f(isim, 'osc', '{:.0f}')} "
                      f"| **{ir['sat']:.0f} %** / {f(isim, 'sat', '{:.0f}')} % "
                      f"| **{ir['alt']:.1f} m** / {isim['alt'][3]:.1f} m |")
    lignes += ["", "Simulated values: the run with the `config.py` parameters (the flight's servo "
               "delay and initial attitude, seed 0), shown in the per-flight figures; in brackets, "
               f"the min–max over {resultats[0][2]['n']} runs with random wind gusts and "
               "disturbance torque (grey band of the summary figure)."]
    return "\n".join(lignes)


def mettre_a_jour_readme(resultats):
    bloc = [DEBUT, "", tableau_markdown(resultats), "",
            "![Measured vs simulated tilt](docs/comparaison_vols.png)", ""]
    for meta, *_ in resultats:
        bloc += [f"<details><summary>{meta['nom']} — {meta.get('note', '')}</summary>", "",
                 f"![{meta['nom']}](docs/comparaison_{os.path.splitext(meta['fichier'])[0]}.png)",
                 "", "</details>", ""]
    bloc.append(FIN)
    texte = open(README, encoding="utf-8").read()
    if DEBUT in texte and FIN in texte:
        a, b = texte.index(DEBUT), texte.index(FIN) + len(FIN)
        texte = texte[:a] + "\n".join(bloc) + texte[b:]
    else:
        texte = texte.rstrip() + "\n\n## Real flights vs simulation\n\n" + "\n".join(bloc) + "\n"
    open(README, "w", encoding="utf-8").write(texte)


# ============================================================================
# POINT D'ENTREE
# ============================================================================
def comparer(seed=0, readme=True, verbeux=True, n_tirages=10):
    """Compare tous les vols de vols/vols.json, avec n_tirages simulations par vol
       (graines seed, seed+1, ...). Renvoie la liste des resultats."""
    meta_vols = json.load(open(os.path.join(DOSSIER_VOLS, "vols.json"), encoding="utf-8"))["vols"]
    os.makedirs(DOSSIER_DOCS, exist_ok=True)
    resultats = []
    for meta in meta_vols:
        brut = lire_vol(meta)
        sims = [simuler_vol(meta, brut, seed=seed + k) for k in range(n_tirages)]
        sim = enveloppe(sims)
        t0, methode = caler_decollage(brut, sims[0], meta)
        vol = recaler(brut, t0)
        vol["methode"] = methode
        t_fin = min(float(vol["t"][-1]), sim["t_fin_prop"])
        ir = indicateurs(vol, t_fin)
        par_tirage = [indicateurs(x, t_fin) for x in sims]
        # (mediane, min, max, simulation de base = premier tirage, graine `seed`)
        isim = {c: (float(np.median([d[c] for d in par_tirage])),
                    float(min(d[c] for d in par_tirage)), float(max(d[c] for d in par_tirage)),
                    par_tirage[0][c])
                for c in par_tirage[0]}
        tracer_vol(meta, vol, sims[0], os.path.join(
            DOSSIER_DOCS, f"comparaison_{os.path.splitext(meta['fichier'])[0]}.png"), seed=seed)
        resultats.append((meta, vol, sim, ir, isim))
        if verbeux:
            print(f"{meta['nom']:6s} ({meta['servoDelay']*1000:.0f} ms) decollage a "
                  f"t={vol['t0']:.2f}s du log [{methode}] | angle max {ir['ang_max']:.1f} / "
                  f"{isim['ang_max'][3]:.1f} ({isim['ang_max'][1]:.1f}-{isim['ang_max'][2]:.1f}) deg"
                  f" | butee {ir['sat']:.0f} / {isim['sat'][3]:.0f} %")
    tracer_synthese(resultats, os.path.join(DOSSIER_DOCS, "comparaison_vols.png"))
    if readme:
        mettre_a_jour_readme(resultats)
    return resultats
