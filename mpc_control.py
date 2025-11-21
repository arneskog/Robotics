from carModel import *
import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline

def cost_function(u, state0, x_ref, y_ref, L, v, dt, horizon):
    x, y, theta, phi = state0

    V_step = v * dt
    cost = 0.0

    w_pos = 30   
    w_phi = 1   
    w_steer = 0.1

    for i in range(horizon):
        omega_s = u[i]

        x, y, theta, phi = car_kinematics(
            [x, y, theta, phi],
            L,
            V_step,
            omega_s,
        )

        ex = x - x_ref[i]
        ey = y - y_ref[i]
        dist_sq = ex*ex + ey*ey

        cost += w_pos * dist_sq + w_phi * (phi**2)
        if i > 0:
            cost += w_steer * (u[i] - u[i-1])**2
        
        if abs(phi) > PHI_MAX:
            cost += 1e3 * (abs(phi) - PHI_MAX)**2

    return cost

def generate_trajectory(x_ref, y_ref, x_car, y_car, v, near_left_lane, near_right_lane, dt, curvature_factor=8.0):
    distance = np.sqrt((x_ref - x_car)**2 + (y_ref - y_car)**2)

    waypoints_distance = v * dt
    if waypoints_distance <= 0:
        waypoints_distance = 1e-6

    num_waypoints = int(np.floor(distance / waypoints_distance))
    if num_waypoints < 2:
        num_waypoints = 2

    waypoints_x = np.linspace(x_car, x_ref, num_waypoints)
    waypoints_y = np.linspace(y_car, y_ref, num_waypoints)

    t_knots = np.linspace(0, 1, num_waypoints)

    spline_x = CubicSpline(t_knots, waypoints_x)
    spline_y = CubicSpline(t_knots, waypoints_y)

    trajectory_x = spline_x(t_knots)
    trajectory_y = spline_y(t_knots)

    if near_left_lane:
        trajectory_x = trajectory_x + curvature_factor * np.cos(np.linspace(0, np.pi, num_waypoints)) * 0.05
        trajectory_y = trajectory_y + -curvature_factor * np.sin(np.linspace(0, np.pi, num_waypoints)) * 0.05
    if near_right_lane:
        trajectory_x = trajectory_x + curvature_factor * np.cos(np.linspace(0, np.pi, num_waypoints)) * 0.05
        trajectory_y = trajectory_y + curvature_factor * np.sin(np.linspace(0, np.pi, num_waypoints)) * 0.05

    return trajectory_x, trajectory_y



def solve_mpc(state, x_ref, y_ref, L, v, dt, horizon=10):
    u0 = np.zeros(horizon) 

    max_steering = PHI_RATE
    bounds = [(-max_steering, max_steering)] * horizon

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

    return result.x[0]

