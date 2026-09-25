# 6DDL_TVC — 6-DOF simulator for a thrust-vector-controlled model rocket

[![tests](https://github.com/AxelBoularot/6DDL_TVC/actions/workflows/tests.yml/badge.svg)](https://github.com/AxelBoularot/6DDL_TVC/actions/workflows/tests.yml)
![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)

A 6-degree-of-freedom flight simulator for a small rocket stabilised by a gimballed
motor nozzle (TVC), with quaternion attitude, variable mass, a realistic servo chain
(gear ratio, end stops, delay, mechanical backlash), gusty wind, motor ejection and
parachute. Pure Python (NumPy + Matplotlib), easy to read and to modify.

![Results](docs/resultats.png)

## Quick start

```bash
git clone https://github.com/AxelBoularot/6DDL_TVC.git
cd 6DDL_TVC
pip install -r requirements.txt
python main.py                 # simulate to apogee, plots + 3D animation
```

| Option | Effect |
|---|---|
| `--sol` | simulate down to the ground (motor ejection, parachute) instead of stopping at apogee |
| `--combustion` | stop the simulation at the end of the burn |
| `--seed N` | reproducible wind gusts and torque noise |
| `--no-anim` | plots only |
| `--save DIR` | save the plots as PNG, no window |

Switches at the top of `config.py`: `STOP_A_APOGEE`, `STOP_FIN_COMBUSTION` (stop at burnout),
`COUPER_TVC_FIN_COMBUSTION` (at burnout the PID is switched off and the nozzle returns to centre),
`EJECT_MOTOR`, `DEPLOY_PARACHUTE`, `USE_ISA`, `SEED`.

Every physical parameter is in **`config.py`** (PID gains, geometry, masses,
aerodynamics, TVC chain, wind, thrust curve, ejection, parachute).

## What is modelled

- **Rigid body, 6 DOF.** Translation in the world frame, rotation in the body frame
  (Euler's equations with the gyroscopic term), attitude as a unit quaternion
  (Hamilton, body → world), renormalised every step.
- **Variable mass.** Propellant mass follows the burnt impulse; mass, centre of
  gravity and transverse inertia are updated every step; the empty motor casing can
  be ejected at burnout.
- **Thrust vector control.** Two independent PID loops (pitch, yaw) on the attitude
  error measured in the body frame, with anti-windup. The command goes through the
  real mechanical chain:
  `PID → desired nozzle angle → × gear ratio → servo (end stop) → delay → ÷ ratio → backlash → nozzle`.
- **Aerodynamics.** Extended Barrowman body model (potential nose term + viscous
  cross-flow term), pitch damping, drag (or parachute drag after deployment).
- **Environment.** ISA air density, horizontal gusts (Ornstein–Uhlenbeck process),
  random disturbance torque independent of the time step.
- **Events.** Lift-off only when thrust exceeds weight (no launch rail), motor ejection
  impulse on the motor cross-section, parachute at apogee.

## Code layout

```
6DDL_TVC/
├── main.py              command-line entry point
├── config.py            all parameters and switches
├── simulateur/          the simulator package
│   ├── simulation.py    main loop (assembles the modules); simuler() returns the time history
│   ├── dynamique.py     rigid-body state and one 6-DOF integration step
│   ├── quaternions.py   quaternion algebra, attitude integration, attitude errors, tilt
│   ├── tvc.py           PID, ChaineServo (ratio, end stop, delay, backlash), ControleurTVC, vectored thrust
│   ├── aero.py          normal force, pitch damping, drag / parachute
│   ├── moteur.py        thrust curve (time-interpolated), impulse, propellant mass
│   ├── masse.py         variable mass, CG and inertia
│   ├── environnement.py air density, wind gusts, disturbance torque
│   ├── evenements.py    lift-off, motor ejection, parachute, stop conditions
│   ├── historique.py    time-history recording
│   └── affichage.py     2D plots and 3D animation
├── tests/               unit and integration tests (python -m pytest)
└── docs/                figure used in this README
```

Use it from your own script (run from the repository root), e.g. for gain tuning or
Monte Carlo runs:

```python
from simulateur import simuler
hist, evts, motor = simuler(seed=0, verbeux=False)
print(hist['z'].max(), hist['tilt'].max())
```

## Conventions

- World frame: Z up (altitude), gravity (0, 0, −g).
- Body frame: the nose points along +z_b; stations along the body are measured from the nose.
- Angles in `config.py` and in the plots are in degrees; everything else is SI.

## Limitations

- Semi-implicit Euler integration with a fixed step (1 ms by default).
- No fins in the aerodynamic model: without thrust the rocket is not aerodynamically stable.
- No launch rail, no roll control, no İω or jet-damping terms.
- The backlash model assumes the nozzle stays where it was last pushed.
- Parameters are those of one specific rocket: measure your own (mass, CG, inertia,
  nozzle position, servo delay, backlash) before trusting the results.

## Tests

```bash
pip install pytest
python -m pytest
```

## Licence

MIT — see [LICENSE](LICENSE).

---

## 🇫🇷 En bref

Simulateur 6 DDL d'une fusée stabilisée par tuyère orientable (TVC) : attitude par
quaternions, masse/CG/inertie variables, chaîne servo réaliste (rapport, butée,
retard, jeu), vent en rafales, éjection du moteur et parachute. Tous les réglages
sont dans `config.py`. Lancer : `python main.py` (arrêt à l'apogée) ou
`python main.py --sol` (jusqu'au sol).
