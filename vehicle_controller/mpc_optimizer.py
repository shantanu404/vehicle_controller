from typing import Tuple

import cvxpy
import numpy as np

# ---------------------------------------------------------------------
# Tunable constants and physical limits
# ---------------------------------------------------------------------
VEHICLE_LENGTH = 2.5  # [m]
ACC_MAX, ACC_MIN = 3.0, -3.0  # [m/s²]
STEER_MAX = 0.50  # [rad]  (≈ 28.6°)
VEL_MAX, VEL_MIN = 15.0, 0.0  # [m/s]
V_DES = 2.5  # desired cruise speed

# Weights for the quadratic cost
Q_LATERAL = 20.0  # lateral (y) tracking
Q_HEADING = 10.0  # heading tracking
Q_SPEED = 10.0  # speed tracking
R_ACCEL = 0.1  # minimise |accel|
R_STEER = 0.1  # minimise |steer|
R_DACCEL = 10.0  # smooth accel (Δa)
R_DSTEER = 10.0  # smooth steer (Δδ)


def do_mpc(
    reference_polynomial: np.ndarray,
    current_state: np.ndarray,
    horizon: int,
    delta_time: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Model-predictive controller that follows a 2-nd order polynomial
    centre-line written in the *vehicle frame*.

    Parameters
    ----------
    reference_polynomial : np.ndarray
        [a0, a1, a2, a3] so that y_ref(x) = a0 x^3 + a1 x^2 + a2 x + a3
    current_state : np.ndarray
        [x, y, theta, v]   (x should be 0 in a vehicle-fixed frame)
    horizon : int
        Prediction horizon (number of steps).
    delta_time : float
        Sample time [s].

    Returns
    -------
    np.ndarray
        [acceleration, steering_angle] for the next  Δt.
    """
    # ------------------------------------------------------------------
    # Pre-compute reference trajectory for the horizon (open-loop, using
    # the *current* speed as a constant forward velocity).
    # ------------------------------------------------------------------
    v_bar = float(current_state[3])  # freeze velocity
    step_distance = v_bar * delta_time  # ≈ Δx per step

    x_ref = np.arange(1, horizon + 1) * step_distance

    y_ref = (
        reference_polynomial[0] * x_ref**3
        + reference_polynomial[1] * x_ref**2
        + reference_polynomial[2] * x_ref
        + reference_polynomial[3]
    )

    dy_dx = (
        reference_polynomial[0] * 3 * x_ref**2
        + reference_polynomial[1] * 2 * x_ref
        + reference_polynomial[2]
    )

    theta_ref = np.arctan(dy_dx)  # small-angle ok

    # Include the starting point so arrays line up with state variables
    y_ref = np.insert(y_ref, 0, current_state[1])
    theta_ref = np.insert(theta_ref, 0, current_state[2])

    # ------------------------------------------------------------------
    # Decision variables
    # ------------------------------------------------------------------
    a = cvxpy.Variable(horizon)  # longitudinal accel
    d = cvxpy.Variable(horizon)  # steering angle  (δ)
    y = cvxpy.Variable(horizon + 1)  # lateral offset
    th = cvxpy.Variable(horizon + 1)  # heading
    v = cvxpy.Variable(horizon + 1)  # speed

    # ------------------------------------------------------------------
    # Constraints
    # ------------------------------------------------------------------
    cons = [
        y[0] == current_state[1],
        th[0] == current_state[2],
        v[0] == current_state[3],
    ]

    for k in range(horizon):
        # Linearised kinematics (small-angle, speed frozen in coupling):
        cons += [
            y[k + 1] == y[k] + v_bar * th[k] * delta_time,
            th[k + 1] == th[k] + v_bar * d[k] / VEHICLE_LENGTH * delta_time,
            v[k + 1] == v[k] + a[k] * delta_time,
            # Actuator & state boxes
            a[k] >= ACC_MIN,
            a[k] <= ACC_MAX,
            d[k] >= -STEER_MAX,
            d[k] <= STEER_MAX,
        ]

    cons += [v >= VEL_MIN, v <= VEL_MAX]  # element-wise

    # ------------------------------------------------------------------
    # Objective
    # ------------------------------------------------------------------
    objective = 0
    for k in range(horizon):
        # tracking costs (k+1 is the *next* predicted state)
        objective += Q_LATERAL * cvxpy.square(y[k + 1] - y_ref[k + 1])
        objective += Q_HEADING * cvxpy.square(th[k + 1] - theta_ref[k + 1])
        objective += Q_SPEED * cvxpy.square(v[k + 1] - V_DES)

        # effort
        objective += R_ACCEL * cvxpy.square(a[k])
        objective += R_STEER * cvxpy.square(d[k])

        # smoothness (first difference, zero for k=0)
        if k > 0:
            objective += R_DACCEL * cvxpy.square(a[k] - a[k - 1])
            objective += R_DSTEER * cvxpy.square(d[k] - d[k - 1])

    prob = cvxpy.Problem(cvxpy.Minimize(objective), cons)
    prob.solve(solver=cvxpy.OSQP, warm_start=True)

    # Infeasible → coast straight ahead
    if prob.status not in ("optimal", "optimal_inaccurate"):
        return np.array([0.0, 0.0], dtype=float), np.array([])

    # Calculate the trajectory of optimal control inputs
    trajectory = []
    for k in range(horizon):
        # Linearised kinematics (small-angle, speed frozen in coupling):
        current_state[0] += current_state[3] * delta_time
        current_state[1] += current_state[3] * current_state[2] * delta_time
        current_state[2] += current_state[3] * d[k].value / VEHICLE_LENGTH * delta_time
        current_state[3] += a[k].value * delta_time
        trajectory.append(current_state[:2].copy())

    trajectory = np.vstack(trajectory)
    # ------------------------------------------------------------------
    # Return the first move (or a safe fallback)
    # ------------------------------------------------------------------
    return np.array([a.value[0], d.value[0]], dtype=float), trajectory
