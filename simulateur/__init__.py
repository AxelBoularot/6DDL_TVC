"""
simulateur -- simulateur 6 DDL de fusee TVC (quaternions).
Les parametres sont dans config.py (a la racine du depot).

    from simulateur import simuler
    hist, evts, moteur = simuler(seed=0, verbeux=False)
"""
from .simulation import simuler

__all__ = ["simuler"]
