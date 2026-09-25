"""
masse.py -- masse, CG et inertie variables (structure + carter + propergol).
"""
import config as C


def proprietes_massiques(t, moteur, moteur_present):
    """Renvoie (m, x_cg, Ir, Ia) a l'instant t.
       moteur_present=False -> carter + propergol restant largues."""
    comps = [(C.m_struct, C.x_struct)]
    m_p = moteur.masse_propergol(t)
    if moteur_present:
        comps.append((C.m_carter, C.x_motor))
        comps.append((m_p, C.x_prop))

    m = sum(c[0] for c in comps)
    x_cg = sum(c[0] * c[1] for c in comps) / m

    # inertie transverse (Huygens) : structure calibree + masses moteur
    Ir = C.I_struct_own + C.m_struct * (C.x_struct - x_cg)**2
    if moteur_present:
        Ir += C.m_carter * (C.x_motor - x_cg)**2
        Ir += m_p * (C.x_prop - x_cg)**2
    # inertie de roulis (axe long), quasi cylindre
    Ia = 0.5 * m * C.r**2
    return m, x_cg, Ir, Ia
