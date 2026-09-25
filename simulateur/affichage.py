"""
affichage.py -- graphiques 2D et animation 3D.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import config as C


def graphiques(hist, evts):
    tL = hist['t']
    fig, axs = plt.subplots(2, 3, figsize=(15, 9))
    plt.subplots_adjust(hspace=0.35, wspace=0.3)

    axs[0, 0].plot(tL, hist['z'], 'b-', lw=2)
    axs[0, 0].axvline(C.burn_time_end, color='r', ls='--', label='Fin moteur')
    if evts.t_chute:
        axs[0, 0].axvline(evts.t_chute, color='c', ls=':', label='Parachute')
    axs[0, 0].set_title('Altitude'); axs[0, 0].set_ylabel('m'); axs[0, 0].legend()

    axs[0, 1].plot(tL, hist['tilt'], 'g-', lw=2)
    axs[0, 1].axvline(C.burn_time_end, color='r', ls='--')
    axs[0, 1].set_title('Ecart / vertical'); axs[0, 1].set_ylabel('deg')

    axs[0, 2].plot(tL, hist['dp'], color='purple', lw=1.3, label='TVC pitch')
    axs[0, 2].plot(tL, hist['dy'], color='teal', lw=1.3, label='TVC yaw')
    axs[0, 2].axvline(C.burn_time_end, color='r', ls='--')
    axs[0, 2].set_title('Angle tuyere reel (retard + jeu)'); axs[0, 2].set_ylabel('deg')
    axs[0, 2].legend()

    axs[1, 0].plot(tL, hist['mass'], 'k-', lw=2)
    axs[1, 0].axvline(C.burn_time_end, color='r', ls='--')
    axs[1, 0].set_title('Masse totale'); axs[1, 0].set_ylabel('kg')

    axs[1, 1].plot(tL, hist['xcg'], color='orange', lw=2, label='CG')
    axs[1, 1].plot(tL, hist['cp'], color='brown', lw=2, label='CP')
    axs[1, 1].axvline(C.burn_time_end, color='r', ls='--')
    axs[1, 1].set_title('CG & CP depuis le nez'); axs[1, 1].set_ylabel('m')
    axs[1, 1].invert_yaxis(); axs[1, 1].legend()

    axs[1, 2].plot(tL, hist['Ir'], 'purple', lw=2)
    axs[1, 2].axvline(C.burn_time_end, color='r', ls='--')
    axs[1, 2].set_title('Inertie transverse'); axs[1, 2].set_ylabel('kg.m2')

    for a in axs.ravel():
        a.grid(alpha=0.3)
    for a in axs[1]:
        a.set_xlabel('t (s)')
    plt.suptitle('3D Quaternion — masse/CG/inertie variables + ejection moteur', fontsize=14)
    plt.tight_layout()
    return fig


def animation_3d(hist, evts, skip=30):
    xa, ya, za = hist['x'][::skip], hist['y'][::skip], hist['z'][::skip]
    bzx, bzy, bzz = hist['bzx'][::skip], hist['bzy'][::skip], hist['bzz'][::skip]
    ta = hist['t'][::skip]
    max_val = max(np.max(np.abs(hist['x'])), np.max(np.abs(hist['y'])), np.max(hist['z']), 1.0)

    fig3d = plt.figure(figsize=(10, 8))
    ax = fig3d.add_subplot(111, projection='3d')
    ax.set_title("Trajectoire 3D (attitude quaternion)", fontsize=13, fontweight='bold')
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Altitude Z (m)")
    ax.set_xlim(-max_val/2, max_val/2); ax.set_ylim(-max_val/2, max_val/2); ax.set_zlim(0, max_val)

    gx = np.linspace(-max_val/2, max_val/2, 10)
    XX, YY = np.meshgrid(gx, gx)
    ax.plot_surface(XX, YY, np.zeros_like(XX), alpha=0.1, color='green')

    line_traj,   = ax.plot([], [], [], lw=1, color='gray', ls='--')
    rocket_line, = ax.plot([], [], [], lw=3, color='#ff6600')
    shadow_dot,  = ax.plot([], [], [], marker='o', color='black', alpha=0.3, ls='None')
    eject_dot,   = ax.plot([], [], [], marker='^', markerfacecolor='magenta',
                           markeredgecolor='black', markersize=12, ls='None',
                           label='Ejection moteur')
    time_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes, fontsize=11)
    ax.legend(loc='upper left')
    vis_len = max_val / 15

    def init():
        for a in [line_traj, rocket_line, shadow_dot, eject_dot]:
            a.set_data(np.array([]), np.array([])); a.set_3d_properties(np.array([]))
        time_text.set_text("")
        return line_traj, rocket_line, shadow_dot, eject_dot, time_text

    def update(i):
        line_traj.set_data(xa[:i], ya[:i]); line_traj.set_3d_properties(za[:i])
        xc, yc, zc = xa[i], ya[i], za[i]
        dx, dy_, dz = vis_len * bzx[i], vis_len * bzy[i], vis_len * bzz[i]
        rocket_line.set_data([xc - dx/2, xc + dx/2], [yc - dy_/2, yc + dy_/2])
        rocket_line.set_3d_properties([zc - dz/2, zc + dz/2])
        rocket_line.set_color('#ff6600' if ta[i] < C.burn_time_end else 'black')
        shadow_dot.set_data(np.array([xc]), np.array([yc]))
        shadow_dot.set_3d_properties(np.array([0.0]))
        if evts.pos_eject is not None and ta[i] >= evts.t_eject:
            eject_dot.set_data(np.array([evts.pos_eject[0]]), np.array([evts.pos_eject[1]]))
            eject_dot.set_3d_properties(np.array([evts.pos_eject[2]]))
        time_text.set_text(f"T: {ta[i]:.2f}s | Alt: {zc:.1f}m")
        return line_traj, rocket_line, shadow_dot, eject_dot, time_text

    ani = FuncAnimation(fig3d, update, frames=len(ta), init_func=init, interval=20, blit=False)
    plt.tight_layout()
    return fig3d, ani
