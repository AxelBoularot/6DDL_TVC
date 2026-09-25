"""
historique.py -- enregistrement des grandeurs a chaque pas.
"""
import numpy as np

CLES = ['t', 'x', 'y', 'z', 'tilt', 'dp', 'dy', 'aoa', 'cp', 'mass', 'xcg', 'Ir',
        'bzx', 'bzy', 'bzz']


class Historique:
    def __init__(self):
        self.d = {k: [] for k in CLES}

    def ajouter(self, **valeurs):
        for k in CLES:
            self.d[k].append(valeurs[k])

    def tableaux(self):
        """dict {cle: np.array}"""
        return {k: np.array(v) for k, v in self.d.items()}
