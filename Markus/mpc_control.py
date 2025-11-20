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

# def generate_trajectory(x_ref, y_ref, x_car, y_car, v, dt):
#     distance = np.sqrt((x_ref - x_car)**2 + (y_ref - y_car)**2)

#     # Calculate the total time to travel this distance at the constant speed
#     time_to_travel = distance / v # Time = Distance / Speed

#     # Calculate the number of waypoints to generate based on the car speed and dt
#     waypoints_distance = v * dt  # How far the car moves per time step
#     num_waypoints = int(np.floor(distance / waypoints_distance))  # Number of waypoints

#     # Generate waypoints (start and end points)
#     waypoints_x = np.linspace(x_car, x_ref, num_waypoints)
#     waypoints_y = np.linspace(y_car, y_ref, num_waypoints)

#     # Create cubic spline interpolation for smooth trajectory
#     spline_x = CubicSpline(np.linspace(0, 1, num_waypoints), waypoints_x)
#     spline_y = CubicSpline(np.linspace(0, 1, num_waypoints), waypoints_y)

#     # Generate smooth trajectory based on spline interpolation
#     trajectory_x = spline_x(np.linspace(0, 1, num_waypoints))
#     trajectory_y = spline_y(np.linspace(0, 1, num_waypoints))

#     return trajectory_x, trajectory_y


def generate_trajectory(
    x_car, y_car, theta,
    measured_x_lane_1, measured_y_lane_1,
    measured_x_lane_2, measured_y_lane_2,
    v, dt, horizon
):
    """
    Build an optimal local centerline trajectory from lane sensor points.

    Returns:
        x_ref_array: (horizon,) array of centerline x positions
        y_ref_array: (horizon,) array of centerline y positions
        theta_ref_array: (horizon,) array of centerline headings
                         (can be ignored if not used in cost)
    """

    def to_body(x_lane, y_lane):
        dx = x_lane - x_car
        dy = y_lane - y_car
        bx =  np.cos(theta) * dx + np.sin(theta) * dy   # forward
        by = -np.sin(theta) * dx + np.cos(theta) * dy   # left
        return bx, by

    # --- 1) transform into car frame ---
    bx1, by1 = to_body(measured_x_lane_1, measured_y_lane_1)
    bx2, by2 = to_body(measured_x_lane_2, measured_y_lane_2)

    # --- 2) try to keep only points in front; if none, fall back to all ---
    front_mask_1 = bx1 > 0.0
    front_mask_2 = bx2 > 0.0

    if np.any(front_mask_1):
        x1_f = measured_x_lane_1[front_mask_1]
        y1_f = measured_y_lane_1[front_mask_1]
        bx1_f = bx1[front_mask_1]
    else:
        # fallback: use all visible points from lane 1
        x1_f = measured_x_lane_1
        y1_f = measured_y_lane_1
        bx1_f = bx1

    if np.any(front_mask_2):
        x2_f = measured_x_lane_2[front_mask_2]
        y2_f = measured_y_lane_2[front_mask_2]
        bx2_f = bx2[front_mask_2]
    else:
        # fallback: use all visible points from lane 2
        x2_f = measured_x_lane_2
        y2_f = measured_y_lane_2
        bx2_f = bx2

    # If still nothing on either lane, really no info.
    if len(x1_f) == 0 or len(x2_f) == 0:
        return None, None, None

    # --- 3) sort each lane by forward distance ---
    order1 = np.argsort(bx1_f)
    order2 = np.argsort(bx2_f)

    x1_sorted = x1_f[order1]
    y1_sorted = y1_f[order1]
    bx1_sorted = bx1_f[order1]

    x2_sorted = x2_f[order2]
    y2_sorted = y2_f[order2]
    bx2_sorted = bx2_f[order2]

    # --- 4) pair points index-wise and compute midpoints ---
    M = min(len(x1_sorted), len(x2_sorted))
    if M == 0:
        return None, None, None

    x1_used = x1_sorted[:M]
    y1_used = y1_sorted[:M]
    x2_used = x2_sorted[:M]
    y2_used = y2_sorted[:M]

    x_mid = 0.5 * (x1_used + x2_used)
    y_mid = 0.5 * (y1_used + y2_used)

    # --- 5) sort midpoints along forward direction ---
    bx_mid, _ = to_body(x_mid, y_mid)
    order_mid = np.argsort(bx_mid)
    x_mid = x_mid[order_mid]
    y_mid = y_mid[order_mid]
    bx_mid = bx_mid[order_mid]

    # if only one midpoint, duplicate it so we can form headings
    if len(x_mid) == 1:
        x_mid = np.array([x_mid[0], x_mid[0]])
        y_mid = np.array([y_mid[0], y_mid[0]])
        bx_mid = np.array([bx_mid[0], bx_mid[0]])

    # --- 6) choose 'horizon' reference points spaced ~ v*dt ---
    step_dist = v * dt
    x_ref = []
    y_ref = []

    for k in range(horizon):
        target_bx = (k + 1) * step_dist
        idx = np.argmin(np.abs(bx_mid - target_bx))
        x_ref.append(x_mid[idx])
        y_ref.append(y_mid[idx])

    x_ref_array = np.array(x_ref)
    y_ref_array = np.array(y_ref)

    # --- 7) compute heading along the reference ---
    theta_ref = []
    for i in range(len(x_ref_array) - 1):
        dx = x_ref_array[i+1] - x_ref_array[i]
        dy = y_ref_array[i+1] - y_ref_array[i]
        theta_ref.append(np.arctan2(dy, dx))

    if len(theta_ref) == 0:
        theta_ref.append(theta)
    else:
        theta_ref.append(theta_ref[-1])

    theta_ref_array = np.array(theta_ref)

    return x_ref_array, y_ref_array, theta_ref_array




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

