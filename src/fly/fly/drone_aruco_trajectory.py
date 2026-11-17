#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


# ---------------------------------------------------------------------------
# Polynomial Trajectory Generator
# ---------------------------------------------------------------------------

class QuinticPolynomial:
    """
    Quintic (5th-order) polynomial trajectory for a single DoF.

    Boundary conditions (6 equations → 6 coefficients):
        q(0)     = q0,   q(T)     = qf
        qdot(0)  = v0,   qdot(T)  = vf
        qddot(0) = a0,   qddot(T) = af

    Standard form:  q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
    """

    def __init__(self, q0, qf, v0=0.0, vf=0.0, a0=0.0, af=0.0, T=5.0):
        self.T = T
        T2, T3, T4, T5 = T**2, T**3, T**4, T**5
        A = np.array([
            [1, 0,   0,    0,     0,       0     ],
            [0, 1,   0,    0,     0,       0     ],
            [0, 0,   2,    0,     0,       0     ],
            [1, T,   T2,   T3,    T4,      T5    ],
            [0, 1,   2*T,  3*T2,  4*T3,    5*T4  ],
            [0, 0,   2,    6*T,   12*T2,   20*T3 ],
        ])
        b = np.array([q0, v0, a0, qf, vf, af])
        self.c = np.linalg.solve(A, b)

    def eval(self, t):
        """Returns (position, velocity, acceleration) at time t."""
        t = np.clip(t, 0.0, self.T)
        t2, t3, t4, t5 = t**2, t**3, t**4, t**5
        c = self.c
        pos = c[0] + c[1]*t   + c[2]*t2   + c[3]*t3    + c[4]*t4    + c[5]*t5
        vel = c[1] + 2*c[2]*t + 3*c[3]*t2  + 4*c[4]*t3  + 5*c[5]*t4
        acc = 2*c[2] + 6*c[3]*t + 12*c[4]*t2 + 20*c[5]*t3
        return pos, vel, acc


class CubicPolynomial:
    """
    Cubic (3rd-order) polynomial trajectory for a single DoF.
    BCs: q(0)=q0, q(T)=qf, qdot(0)=v0, qdot(T)=vf
    """

    def __init__(self, q0, qf, v0=0.0, vf=0.0, T=5.0):
        self.T = T
        T2, T3 = T**2, T**3
        A = np.array([
            [1, 0,   0,    0   ],
            [0, 1,   0,    0   ],
            [1, T,   T2,   T3  ],
            [0, 1,   2*T,  3*T2],
        ])
        b = np.array([q0, v0, qf, vf])
        self.c = np.linalg.solve(A, b)

    def eval(self, t):
        t = np.clip(t, 0.0, self.T)
        t2, t3 = t**2, t**3
        c = self.c
        pos = c[0] + c[1]*t   + c[2]*t2   + c[3]*t3
        vel = c[1] + 2*c[2]*t + 3*c[3]*t2
        acc = 2*c[2] + 6*c[3]*t
        return pos, vel, acc


class Trajectory3D:
    """
    3-axis trajectory composed of independent 1D polynomials.
    Uses quintic by default (smooth accel) or cubic.

    NOTE: PX4 NED convention — z is DOWN (negative = up).
          The trajectory operates in NED space; the caller must pass
          z values already in NED (i.e. hover at z = -HOVER_ALT).
    """

    def __init__(self, start, goal,
                 v_start=None, v_goal=None,
                 a_start=None, a_goal=None,
                 T=5.0, poly_order=5):
        start = np.array(start, dtype=float)
        goal  = np.array(goal,  dtype=float)
        v0 = np.zeros(3) if v_start is None else np.array(v_start, dtype=float)
        vf = np.zeros(3) if v_goal  is None else np.array(v_goal,  dtype=float)
        a0 = np.zeros(3) if a_start is None else np.array(a_start, dtype=float)
        af = np.zeros(3) if a_goal  is None else np.array(a_goal,  dtype=float)
        self.T = T
        self.polys = []
        for i in range(3):
            if poly_order == 5:
                self.polys.append(
                    QuinticPolynomial(start[i], goal[i],
                                      v0[i], vf[i], a0[i], af[i], T))
            else:
                self.polys.append(
                    CubicPolynomial(start[i], goal[i], v0[i], vf[i], T))

    def eval(self, t):
        """Returns (pos, vel, acc) each as np.ndarray shape (3,)."""
        results = [p.eval(t) for p in self.polys]
        pos = np.array([r[0] for r in results])
        vel = np.array([r[1] for r in results])
        acc = np.array([r[2] for r in results])
        return pos, vel, acc

    @property
    def duration(self):
        return self.T


# ---------------------------------------------------------------------------
# Joint-Space PD Controller  (Del Prete control law, drone adaptation)
# ---------------------------------------------------------------------------

class JointSpacePDController:

    def __init__(self, Kp, Kd, mass=1.5, g=9.81):
        self.Kp   = np.diag(Kp) if not np.isscalar(Kp) else np.eye(3) * Kp
        self.Kd   = np.diag(Kd) if not np.isscalar(Kd) else np.eye(3) * Kd
        self.mass = mass
        self.g    = g

    def compute(self, q, v, q_r, v_r, a_r):
        
     
        e     = q - q_r      # position error
        e_dot = v - v_r      # velocity error  (ė in the doc)

        # Core control law — eq. (2):
        #   v̇_d = v̇_r − Kd·ė − Kp·e
        a_cmd = a_r - self.Kd @ e_dot - self.Kp @ e

        return a_cmd, e, e_dot


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class DroneControllerNode(Node):

    # ── Configuration ───────────────────────────────────────────────────────
    POLY_ORDER = 5           # 5=quintic (zero vel+acc BCs), 3=cubic
    HOVER_ALT  = 1.5         # [m] AGL hover altitude (positive = up, ENU sense)
    GOAL_TOL   = 0.15        # [m] position tolerance to declare "reached"
    TRAJ_T     = 8.0         # [s] trajectory duration (hover → ArUco)
    CTRL_HZ    = 50.0        # [Hz] control loop rate

    # PD gains — Kd ≥ 2√(Kp) for critical damping per axis
    KP = [2.0, 2.0, 3.0]    # [x, y, z(NED)]
    KD = [2.8, 2.8, 3.5]    # [x, y, z(NED)]

    # ArUco marker world position from the SDF (ENU):  (5, 0, 2.5)
    # We target 0.5 m in front of the marker face → x=4.5
    # Converted to NED: x_NED=x_ENU, y_NED=y_ENU, z_NED=-z_ENU
    ARUCO_NED = np.array([4.5, 0.0, -2.5])

    def __init__(self):
        super().__init__('drone_pd_controller')

        # ── QoS (matches PX4 uXRCE-DDS bridge requirements) ─────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_cb, qos)
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self._vehicle_status_cb, qos)

        # ── State ────────────────────────────────────────────────────────
        self.local_pos    = None   # VehicleLocalPosition msg
        self.vehicle_status = None # VehicleStatus msg
        self.traj         = None   # active Trajectory3D
        self.traj_start   = None   # rclpy.Time when traj started
        self.phase        = 'INIT' # state machine phase
        self.goal_ned     = self.ARUCO_NED.copy()

        # Pre-arm setpoint counter (PX4 needs N setpoints before OFFBOARD)
        self.setpoint_counter = 0
        self.PREARM_COUNT     = 10  # ~1 s at 10 Hz pre-arm; actual ctrl at 50 Hz

        # ── Controller ───────────────────────────────────────────────────
        self.controller = JointSpacePDController(
            Kp=self.KP, Kd=self.KD, mass=1.5, g=9.81)

        # ── Control timer @ CTRL_HZ ──────────────────────────────────────
        self.create_timer(1.0 / self.CTRL_HZ, self._control_loop)

        self.get_logger().info('[DroneCtrl] Node initialised. Waiting for telemetry...')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self.local_pos = msg

    def _vehicle_status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg

    # ── State accessors ────────────────────────────────────────────────────

    def _current_pos_ned(self) -> np.ndarray:
        """Returns current position in NED [m]."""
        if self.local_pos is None:
            return np.zeros(3)
        return np.array([self.local_pos.x,
                         self.local_pos.y,
                         self.local_pos.z])

    def _current_vel_ned(self) -> np.ndarray:
        """Returns current velocity in NED [m/s]."""
        if self.local_pos is None:
            return np.zeros(3)
        return np.array([self.local_pos.vx,
                         self.local_pos.vy,
                         self.local_pos.vz])

    # ── PX4 command helpers ────────────────────────────────────────────────

    def _publish_offboard_mode(self, *, position=False, velocity=False,
                                acceleration=False):
        """
        Heartbeat — must be published at >2 Hz while in OFFBOARD.
        We publish it every control tick (50 Hz).
        """
        msg = OffboardControlMode()
        msg.position     = position
        msg.velocity     = velocity
        msg.acceleration = acceleration
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def _publish_accel_setpoint(self, a_cmd: np.ndarray, yaw: float = 0.0):
        """
        Publish TrajectorySetpoint using acceleration control.
        PX4 uses NaN to signal 'ignore this field'.
        With acceleration mode: position & velocity = NaN, acceleration = a_cmd.

        NOTE: PX4 NED → z_NED = -z_ENU (up is negative).
              a_cmd is already in NED when coming from the controller.
        """
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        msg.position    = [nan, nan, nan]
        msg.velocity    = [nan, nan, nan]
        msg.acceleration = [float(a_cmd[0]),
                            float(a_cmd[1]),
                            float(a_cmd[2])]
        msg.yaw         = yaw
        self.trajectory_pub.publish(msg)

    def _publish_position_setpoint(self, pos_ned: np.ndarray, yaw: float = 0.0):
        """
        Publish TrajectorySetpoint using position control (used during pre-arm
        streaming and as a fallback).
        """
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        msg.position     = [float(pos_ned[0]),
                             float(pos_ned[1]),
                             float(pos_ned[2])]
        msg.velocity     = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.yaw          = yaw
        self.trajectory_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, **params):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(params.get('param1', 0.0))
        msg.param2           = float(params.get('param2', 0.0))
        msg.param3           = float(params.get('param3', 0.0))
        msg.param4           = float(params.get('param4', 0.0))
        msg.param5           = float(params.get('param5', 0.0))
        msg.param6           = float(params.get('param6', 0.0))
        msg.param7           = float(params.get('param7', 0.0))
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(msg)

    def _engage_offboard(self):
        """Switch to OFFBOARD mode."""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0, param2=6.0)
        self.get_logger().info('[DroneCtrl] OFFBOARD mode command sent.')

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('[DroneCtrl] Arm command sent.')

    # ── Trajectory planning ────────────────────────────────────────────────

    def _plan_trajectory(self, start_ned, goal_ned,
                          v_start=None, v_goal=None,
                          T=None, poly_order=None) -> Trajectory3D:
        T          = T          or self.TRAJ_T
        poly_order = poly_order or self.POLY_ORDER
        traj = Trajectory3D(
            start=start_ned, goal=goal_ned,
            v_start=v_start, v_goal=v_goal,
            T=T, poly_order=poly_order)
        self.get_logger().info(
            f'[DroneCtrl] Trajectory planned: {start_ned} → {goal_ned} '
            f'in {T:.1f}s (order={poly_order})')
        return traj

    # ── Main control loop ──────────────────────────────────────────────────

    def _control_loop(self):
        """
        Timer callback at CTRL_HZ.

        State machine:
            INIT         — wait for telemetry, stream setpoints, arm & OFFBOARD
            TAKEOFF      — quintic trajectory from ground → hover altitude
            GOTO_MARKER  — quintic trajectory from hover → ArUco NED goal
            HOVER_MARKER — pure PD hold at goal
        """
        now = self.get_clock().now()

        # ── INIT: wait for telemetry then arm ───────────────────────────
        if self.phase == 'INIT':
            if self.local_pos is None:
                return  # still waiting

            # Pre-stream position setpoints so PX4 accepts OFFBOARD
            # NED hover: z_NED = -HOVER_ALT (up is negative in NED)
            hover_ned = np.array([0.0, 0.0, -self.HOVER_ALT])
            self._publish_offboard_mode(position=True)
            self._publish_position_setpoint(hover_ned)

            self.setpoint_counter += 1
            if self.setpoint_counter >= self.PREARM_COUNT:
                self._engage_offboard()
                self._arm()

                # Plan takeoff trajectory
                start_ned = self._current_pos_ned()
                self.traj       = self._plan_trajectory(
                    start_ned=start_ned,
                    goal_ned=hover_ned,
                    T=4.0, poly_order=self.POLY_ORDER)
                self.traj_start = now
                self.phase      = 'TAKEOFF'
                self.get_logger().info(
                    f'[DroneCtrl] Phase: TAKEOFF → hover at NED {hover_ned}')
            return

        # ── Shared setup for active flight phases ────────────────────────
        q = self._current_pos_ned()
        v = self._current_vel_ned()

        # ── TAKEOFF ──────────────────────────────────────────────────────
        if self.phase == 'TAKEOFF':
            hover_ned   = np.array([0.0, 0.0, -self.HOVER_ALT])
            t_elapsed   = (now - self.traj_start).nanoseconds * 1e-9
            q_r, v_r, a_r = self.traj.eval(t_elapsed)

            a_cmd, e, e_dot = self.controller.compute(q, v, q_r, v_r, a_r)

            self._publish_offboard_mode(acceleration=True)
            self._publish_accel_setpoint(a_cmd)

            # Transition when trajectory done AND close enough
            if (t_elapsed >= self.traj.duration and
                    np.linalg.norm(q - hover_ned) < self.GOAL_TOL):
                self.get_logger().info(
                    '[DroneCtrl] Hover reached. Planning trajectory to ArUco...')
                self.traj = self._plan_trajectory(
                    start_ned=hover_ned,
                    goal_ned=self.goal_ned,
                    v_start=self._current_vel_ned(),
                    v_goal=np.zeros(3),
                    T=self.TRAJ_T, poly_order=self.POLY_ORDER)
                self.traj_start = now
                self.phase      = 'GOTO_MARKER'
                self.get_logger().info(
                    f'[DroneCtrl] Phase: GOTO_MARKER → NED {self.goal_ned}')

        # ── GOTO_MARKER ───────────────────────────────────────────────────
        elif self.phase == 'GOTO_MARKER':
            t_elapsed      = (now - self.traj_start).nanoseconds * 1e-9
            q_r, v_r, a_r = self.traj.eval(t_elapsed)

            a_cmd, e, e_dot = self.controller.compute(q, v, q_r, v_r, a_r)

            self._publish_offboard_mode(acceleration=True)
            self._publish_accel_setpoint(a_cmd)

            dist = np.linalg.norm(q - self.goal_ned)
            if t_elapsed >= self.traj.duration and dist < self.GOAL_TOL:
                self.get_logger().info(
                    f'[DroneCtrl] ArUco marker reached! Final error: {dist:.3f} m')
                self.phase = 'HOVER_MARKER'
            else:
                # Periodic progress log
                if int(t_elapsed * 2) % 4 == 0:  # ~every 2 s
                    self.get_logger().info(
                        f'[DroneCtrl] GOTO_MARKER | t={t_elapsed:.1f}/{self.traj.duration:.1f}s '
                        f'dist={dist:.2f}m pos_err={np.linalg.norm(e):.3f}m')

        # ── HOVER_MARKER ──────────────────────────────────────────────────
        elif self.phase == 'HOVER_MARKER':
            # Pure PD hold — zero reference vel & accel
            a_cmd, e, e_dot = self.controller.compute(
                q, v, self.goal_ned, np.zeros(3), np.zeros(3))

            self._publish_offboard_mode(acceleration=True)
            self._publish_accel_setpoint(a_cmd)

            # Periodic status log
            t_hold = (now - self.traj_start).nanoseconds * 1e-9
            if int(t_hold) % 5 == 0 and int(t_hold * 10) % 10 == 0:
                self.get_logger().info(
                    f'[DroneCtrl] HOVERING at marker | '
                    f'pos_err={np.linalg.norm(e):.3f} m '
                    f'vel_err={np.linalg.norm(e_dot):.3f} m/s')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    print('[DroneCtrl] Starting drone PD controller node...')
    rclpy.init(args=args)
    node = DroneControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()