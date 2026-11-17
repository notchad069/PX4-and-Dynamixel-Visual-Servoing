#!/usr/bin/env python3
"""
trajectory_test.py
==================
Standalone script to visualise and verify the quintic/cubic polynomial
trajectories BEFORE running the full ROS simulation.

Run with:  python3 trajectory_test.py
Requires:  numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── copy the polynomial classes here so file is self-contained ─────────────

class QuinticPolynomial:
    def __init__(self, q0, qf, v0=0.0, vf=0.0, a0=0.0, af=0.0, T=5.0):
        self.T = T
        T2,T3,T4,T5 = T**2,T**3,T**4,T**5
        A = np.array([
            [1, 0,   0,    0,     0,      0    ],
            [0, 1,   0,    0,     0,      0    ],
            [0, 0,   2,    0,     0,      0    ],
            [1, T,   T2,   T3,    T4,     T5   ],
            [0, 1,  2*T,  3*T2,  4*T3,  5*T4  ],
            [0, 0,   2,   6*T,  12*T2, 20*T3  ],
        ])
        b = np.array([q0, v0, a0, qf, vf, af])
        self.c = np.linalg.solve(A, b)

    def eval(self, t):
        t = np.clip(t, 0.0, self.T)
        t2,t3,t4,t5 = t**2,t**3,t**4,t**5
        c = self.c
        pos = c[0]+c[1]*t+c[2]*t2+c[3]*t3+c[4]*t4+c[5]*t5
        vel = c[1]+2*c[2]*t+3*c[3]*t2+4*c[4]*t3+5*c[5]*t4
        acc = 2*c[2]+6*c[3]*t+12*c[4]*t2+20*c[5]*t3
        return pos, vel, acc


class CubicPolynomial:
    def __init__(self, q0, qf, v0=0.0, vf=0.0, T=5.0):
        self.T = T
        T2,T3 = T**2,T**3
        A = np.array([
            [1, 0,   0,    0  ],
            [0, 1,   0,    0  ],
            [1, T,   T2,   T3 ],
            [0, 1,  2*T,  3*T2],
        ])
        b = np.array([q0, v0, qf, vf])
        self.c = np.linalg.solve(A, b)

    def eval(self, t):
        t = np.clip(t, 0.0, self.T)
        t2,t3 = t**2,t**3
        c = self.c
        pos = c[0]+c[1]*t+c[2]*t2+c[3]*t3
        vel = c[1]+2*c[2]*t+3*c[3]*t2
        acc = 2*c[2]+6*c[3]*t
        return pos, vel, acc


def simulate_closed_loop(traj_polys, T, Kp, Kd, dt=0.01, noise_std=0.0):
    """
    Simple discrete-time simulation of the PD control law:
        e      = q - q_r
        e_dot  = v - v_r
        a_cmd  = a_r - Kd*e_dot - Kp*e
    Integrates forward with Euler for visualisation.
    """
    t_arr = np.arange(0, T+dt, dt)
    n = len(t_arr)
    q_hist = np.zeros((n, 3))
    v_hist = np.zeros((n, 3))
    e_hist = np.zeros((n, 3))
    q_r_hist = np.zeros((n, 3))
    a_r_hist = np.zeros((n, 3))

    # initial conditions (at rest at start of trajectory)
    q = np.array([p.eval(0)[0] for p in traj_polys])
    v = np.zeros(3)

    for i, t in enumerate(t_arr):
        q_r = np.array([p.eval(t)[0] for p in traj_polys])
        v_r = np.array([p.eval(t)[1] for p in traj_polys])
        a_r = np.array([p.eval(t)[2] for p in traj_polys])

        e     = q - q_r
        e_dot = v - v_r

        # PD control law from del Prete eq.(2):  a_cmd = a_r - Kd*e_dot - Kp*e
        a_cmd = a_r - Kd * e_dot - Kp * e

        q_hist[i]   = q
        v_hist[i]   = v
        e_hist[i]   = e
        q_r_hist[i] = q_r
        a_r_hist[i] = a_r

        # Euler integration (plant: a = a_cmd + noise)
        noise = np.random.randn(3) * noise_std
        a_true = a_cmd + noise
        v = v + a_true * dt
        q = q + v * dt

    return t_arr, q_hist, v_hist, e_hist, q_r_hist


def main():
    # ── Scenario: takeoff (0→1.5m z) then fly to marker (5, 0, 2.5) ────────
    T1, T2_seg = 4.0, 8.0
    Kp = np.array([2.0, 2.0, 3.0])
    Kd = np.array([2.8, 2.8, 3.5])

    # Takeoff trajectory (quintic, axis by axis)
    start1 = np.array([0.0, 0.0, 0.0])
    goal1  = np.array([0.0, 0.0, 1.5])
    polys_takeoff = [QuinticPolynomial(start1[i], goal1[i], T=T1) for i in range(3)]

    # Main trajectory to marker (quintic, with zero vel/acc at endpoints)
    start2 = goal1
    goal2  = np.array([4.5, 0.0, 2.5])
    polys_main = [QuinticPolynomial(start2[i], goal2[i], T=T2_seg) for i in range(3)]

    # Simulate closed-loop (with small noise to test robustness)
    t1, q1, v1, e1, qr1 = simulate_closed_loop(polys_takeoff, T1, Kp, Kd, noise_std=0.05)
    t2, q2, v2, e2, qr2 = simulate_closed_loop(polys_main, T2_seg, Kp, Kd, noise_std=0.05)
    t2_shifted = t2 + T1

    # ── Figure 1: Trajectory profiles ───────────────────────────────────────
    ts   = np.linspace(0, T2_seg, 500)
    axes_labels = ['X [m]', 'Y [m]', 'Z [m]']
    fig, axs = plt.subplots(3, 3, figsize=(15, 9))
    fig.suptitle('Quintic Polynomial Trajectory — Main Segment (Hover→Marker)', fontsize=13, fontweight='bold')

    for i, (lbl, p) in enumerate(zip(axes_labels, polys_main)):
        pos_arr = [p.eval(t)[0] for t in ts]
        vel_arr = [p.eval(t)[1] for t in ts]
        acc_arr = [p.eval(t)[2] for t in ts]

        axs[i][0].plot(ts, pos_arr, 'b-', lw=2)
        axs[i][0].set_ylabel(lbl); axs[i][0].set_title('Position' if i==0 else '')
        axs[i][0].grid(True, alpha=0.3)

        axs[i][1].plot(ts, vel_arr, 'g-', lw=2)
        axs[i][1].set_ylabel(lbl.replace('m','m/s')); axs[i][1].set_title('Velocity' if i==0 else '')
        axs[i][1].grid(True, alpha=0.3)

        axs[i][2].plot(ts, acc_arr, 'r-', lw=2)
        axs[i][2].set_ylabel(lbl.replace('m','m/s²')); axs[i][2].set_title('Acceleration' if i==0 else '')
        axs[i][2].grid(True, alpha=0.3)

    for ax in axs[-1]: ax.set_xlabel('Time [s]')
    plt.tight_layout()
    plt.savefig('trajectory_profiles.png', dpi=150, bbox_inches='tight')
    print('Saved: trajectory_profiles.png')

    # ── Figure 2: Closed-loop simulation ────────────────────────────────────
    fig2, axs2 = plt.subplots(2, 2, figsize=(13, 9))
    fig2.suptitle('Closed-Loop PD Control Simulation (with disturbance noise)', fontsize=13, fontweight='bold')

    # 3D path
    ax3d = fig2.add_subplot(221, projection='3d') if False else None
    ax = axs2[0][0]
    ax.plot(t1, q1[:, 2], 'b-',  lw=2, label='actual Z')
    ax.plot(t1, qr1[:, 2], 'b--', lw=1, label='ref Z')
    ax.plot(t2_shifted, q2[:, 0], 'g-',  lw=2, label='actual X')
    ax.plot(t2_shifted, qr2[:, 0], 'g--', lw=1, label='ref X')
    ax.axvline(T1, color='gray', ls=':', lw=1, label='phase switch')
    ax.set_xlabel('Time [s]'); ax.set_ylabel('Position [m]')
    ax.set_title('Position Tracking (X, Z)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Y axis tracking
    ax = axs2[0][1]
    ax.plot(t2_shifted, q2[:, 1], 'purple', lw=2, label='actual Y')
    ax.plot(t2_shifted, qr2[:, 1], 'purple', ls='--', lw=1, label='ref Y')
    ax.set_xlabel('Time [s]'); ax.set_ylabel('Y Position [m]')
    ax.set_title('Y Position Tracking')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # Position error norm
    e_norm1 = np.linalg.norm(e1, axis=1)
    e_norm2 = np.linalg.norm(e2, axis=1)
    ax = axs2[1][0]
    ax.plot(t1, e_norm1, 'r-', lw=2, label='takeoff phase')
    ax.plot(t2_shifted, e_norm2, 'darkorange', lw=2, label='goto marker phase')
    ax.axhline(0.15, color='k', ls=':', lw=1, label='tolerance 0.15m')
    ax.axvline(T1, color='gray', ls=':', lw=1)
    ax.set_xlabel('Time [s]'); ax.set_ylabel('||e|| [m]')
    ax.set_title('Position Error Norm')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # 3D trajectory
    ax = axs2[1][1]
    ax.plot(q2[:, 0], q2[:, 2], 'b-',  lw=2, label='actual')
    ax.plot(qr2[:, 0], qr2[:, 2], 'r--', lw=1, label='reference')
    ax.scatter([start2[0]], [start2[2]], c='g', s=80, zorder=5, label='start')
    ax.scatter([goal2[0]], [goal2[2]], c='r', s=80, marker='*', zorder=5, label='marker')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Z [m]')
    ax.set_title('X-Z Plane Trajectory (hover→marker)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('closed_loop_simulation.png', dpi=150, bbox_inches='tight')
    print('Saved: closed_loop_simulation.png')

    # ── Quintic vs Cubic comparison ──────────────────────────────────────────
    fig3, axs3 = plt.subplots(1, 3, figsize=(14, 4))
    fig3.suptitle('Quintic vs Cubic Polynomial — Z-axis Takeoff (0→1.5m)', fontsize=12, fontweight='bold')

    ts_t = np.linspace(0, T1, 300)
    qp = QuinticPolynomial(0.0, 1.5, T=T1)
    cp = CubicPolynomial(0.0, 1.5, T=T1)

    for j, (lbl, idx) in enumerate([('Position [m]', 0), ('Velocity [m/s]', 1), ('Acceleration [m/s²]', 2)]):
        q_vals = [qp.eval(t)[idx] for t in ts_t]
        c_vals = [cp.eval(t)[idx] for t in ts_t]
        axs3[j].plot(ts_t, q_vals, 'b-',  lw=2, label='Quintic (5th)')
        axs3[j].plot(ts_t, c_vals, 'r--', lw=2, label='Cubic (3rd)')
        axs3[j].set_xlabel('Time [s]'); axs3[j].set_ylabel(lbl)
        axs3[j].legend(); axs3[j].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('quintic_vs_cubic.png', dpi=150, bbox_inches='tight')
    print('Saved: quintic_vs_cubic.png')

    # Print boundary condition verification
    print('\n── Quintic BC verification (main segment) ──')
    for i, (ax, poly) in enumerate(zip(['X','Y','Z'], polys_main)):
        p0,v0,a0 = poly.eval(0.0)
        pT,vT,aT = poly.eval(T2_seg)
        print(f'  {ax}: pos {start2[i]:.2f}→{p0:.4f}({goal2[i]:.2f}→{pT:.4f})  '
              f'vel 0→{v0:.4f}(0→{vT:.4f})  acc 0→{a0:.4f}(0→{aT:.4f})')

    plt.show()


if __name__ == '__main__':
    main()
