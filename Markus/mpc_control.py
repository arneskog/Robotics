import car

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
    cost = 0.0

    # weights
    w_pos = 1.0     # tracking
    w_steer = 0.1   # penalty on steering magnitude

    for i in range(horizon):
        omega_s = u[i]

        # propagate state one step ahead
        x, y, theta, phi = car.car_kinematics(
            [x, y, theta, phi],
            L=L,
            v=v,
            w_s=omega_s,
            dt=dt
        )

        # tracking error to reference at this step
        ex = x - x_ref[i]
        ey = y - y_ref[i]
        dist_sq = ex*ex + ey*ey

        # add to cost
        cost += w_pos * dist_sq + w_steer * (omega_s**2)

    return cost

def generate_trajectory(x_ref, y_ref, x_car, y_car, v, dt):
    distance = np.sqrt((x_ref - x_car)**2 + (y_ref - y_car)**2)

    # Calculate the total time to travel this distance at the constant speed
    time_to_travel = distance / v # Time = Distance / Speed

    # Calculate the number of waypoints to generate based on the car speed and dt
    waypoints_distance = v * dt  # How far the car moves per time step
    num_waypoints = int(np.floor(distance / waypoints_distance))  # Number of waypoints

    # Generate waypoints (start and end points)
    waypoints_x = np.linspace(x_car, x_ref, num_waypoints)
    waypoints_y = np.linspace(y_car, y_ref, num_waypoints)

    # Create cubic spline interpolation for smooth trajectory
    spline_x = CubicSpline(np.linspace(0, 1, num_waypoints), waypoints_x)
    spline_y = CubicSpline(np.linspace(0, 1, num_waypoints), waypoints_y)

    # Generate smooth trajectory based on spline interpolation
    trajectory_x = spline_x(np.linspace(0, 1, num_waypoints))
    trajectory_y = spline_y(np.linspace(0, 1, num_waypoints))

    return trajectory_x, trajectory_y



def solve_mpc(state, x_ref, y_ref, L, v, dt, horizon=10):
    # Initial guess for the control inputs (steering angles)
    u0 = np.zeros(horizon)  # Zero steering input initially

    # Constraints: Steering limit ([-max_steering, max_steering])
    max_steering = np.radians(35)
    bounds = [(-max_steering, max_steering)] * horizon
    
    # Optimize the control inputs using the cost function
    result = minimize(cost_function, u0, args=(state, x_ref, y_ref, L, v, dt, horizon),
                      bounds=bounds, method='SLSQP')
    
    if not result.success:
        print("MPC failed:", result.message)
        print("Returning 0 steering as fallback.")
        return 0.0
    
    # Return the optimal steering input
    return result.x[0]  # Apply only the first steering angle from the optimal sequence

