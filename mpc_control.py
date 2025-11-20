from carModel import *
import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline

def cost_function(u, state0, x_ref, y_ref, L, v, dt, horizon):
    """
    u: array of steering rates/angles over the horizon (omega_s)
    state0: [x, y, theta, phi] at current time
    x_ref, y_ref: reference trajectory arrays of same length as u
    """
    x, y, theta, phi = state0
    # convert speed+dt into per-step displacement used by the kinematics
    V_step = v * dt
    cost = 0.0

    # weights
    w_pos = 8     # tracking
    w_phi = 5     # phi to zero
    w_steer = 0.1  # penalty on steering magnitude

    for i in range(horizon):
        omega_s = u[i]*dt

        # propagate state one step ahead
        x, y, theta, phi = car_kinematics(
            [x, y, theta, phi],
            L,
            V_step,
            omega_s,
        )

        # tracking error to reference at this step
        ex = x - x_ref[i]
        ey = y - y_ref[i]
        dist_sq = ex*ex + ey*ey

        # add to cost
        cost += w_pos * dist_sq + w_phi * (phi**2)
        if i > 0:
            cost += w_steer * (u[i] - u[i-1])**2

    return cost

def generate_trajectory(x_ref, y_ref, x_car, y_car, v, dt):
    distance = np.sqrt((x_ref - x_car)**2 + (y_ref - y_car)**2)

    # Protect against zero or extremely small speeds / dt
    waypoints_distance = v * dt
    if waypoints_distance <= 0:
        waypoints_distance = 1e-6

    # Compute number of waypoints (at least 2 required for CubicSpline)
    num_waypoints = int(np.floor(distance / waypoints_distance))
    if num_waypoints < 2:
        # ensure at least two points so CubicSpline doesn't raise
        num_waypoints = 2

    # Generate waypoints (start and end points)
    waypoints_x = np.linspace(x_car, x_ref, num_waypoints)
    waypoints_y = np.linspace(y_car, y_ref, num_waypoints)

    # Create cubic spline interpolation for smooth trajectory
    t_knots = np.linspace(0, 1, num_waypoints)
    spline_x = CubicSpline(t_knots, waypoints_x)
    spline_y = CubicSpline(t_knots, waypoints_y)

    # Generate smooth trajectory based on spline interpolation
    trajectory_x = spline_x(t_knots)
    trajectory_y = spline_y(t_knots)
    return trajectory_x, trajectory_y



def solve_mpc(state, x_ref, y_ref, L, v, dt, horizon=10):
    # Initial guess for the control inputs (steering angles)
    u0 = np.zeros(horizon)  # Zero steering input initially

    # Constraints: Steering limit ([-max_steering, max_steering])
    max_steering = PHI_RATE
    bounds = [(-max_steering, max_steering)] * horizon
    
    # Optimize the control inputs using the cost function
    # limit optimizer iterations and use a bounded quasi-Newton method for speed
    result = minimize(
        cost_function,
        u0,
        args=(state, x_ref, y_ref, L, v, dt, horizon),
        bounds=bounds,
        method='L-BFGS-B',
        options={'maxiter': 50}
    )
    
    if not result.success:
        print("MPC failed:", result.message)
        print("Returning 0 steering as fallback.")
        return 0.0
    
    # Return the optimal steering input
    return result.x[0]  # Apply only the first steering angle from the optimal sequence

