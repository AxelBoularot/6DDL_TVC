"""
main.py -- point d'entree.

    python main.py                 # simulation jusqu'a l'apogee + graphiques + animation 3D
    python main.py --sol           # jusqu'au sol (ejection, parachute)
    python main.py --combustion    # arret a la fin de la combustion
    python main.py --seed 0        # tirage reproductible (vent, bruit)
    python main.py --no-anim       # graphiques seulement
    python main.py --save figures  # enregistre les graphiques en PNG (sans fenetre)
    python main.py --comparer      # vols reels (vols/) vs simulation -> docs/ + README

Parametres physiques : config.py
"""
import argparse
import os
import config as C


def main():
    ap = argparse.ArgumentParser(description="Simulateur 6 DDL de fusee TVC (quaternions)")
    ap.add_argument("--sol", action="store_true", help="simuler jusqu'au sol au lieu de l'apogee")
    ap.add_argument("--combustion", action="store_true",
                    help="arreter la simulation a la fin de la combustion")
    ap.add_argument("--seed", type=int, default=C.SEED, help="graine aleatoire (reproductible)")
    ap.add_argument("--no-anim", action="store_true", help="pas d'animation 3D")
    ap.add_argument("--save", metavar="DOSSIER", help="enregistrer les graphiques en PNG, sans affichage")
    ap.add_argument("--comparer", action="store_true",
                    help="comparer les vols reels de vols/ a la simulation (figures + README)")
    args = ap.parse_args()

    if args.comparer:
        import matplotlib
        matplotlib.use("Agg")
        from simulateur.vols_reels import comparer
        comparer(seed=0 if args.seed is None else args.seed)
        print("Figures -> docs/comparaison_*.png | tableau mis a jour dans README.md")
        return

    if args.save:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from simulateur.simulation import simuler
    from simulateur.affichage import graphiques, animation_3d

    hist, evts, _ = simuler(seed=args.seed, stop_apogee=not args.sol,
                            stop_combustion=True if args.combustion else None)
    fig = graphiques(hist, evts)

    if args.save:
        os.makedirs(args.save, exist_ok=True)
        chemin = os.path.join(args.save, "resultats.png")
        fig.savefig(chemin, dpi=110)
        print(f"Graphiques -> {chemin}")
        return

    if args.no_anim:
        plt.show()
        return
    plt.show(block=False)
    _, ani = animation_3d(hist, evts)   # garder la reference, sinon l'animation s'arrete
    plt.show()


if __name__ == "__main__":
    main()
