import numpy as np

def car_kinematics(state, L, v, w_s, dt):
    # Unpack the state
    x, y, theta, phi = state

    # Calculate the derivatives (rates of change)
    dxdt = np.cos(theta) * np.cos(phi) * v
    dydt = np.sin(theta) * np.cos(phi) * v
    dthetadt = np.sin(phi) / L * v
    dphidt = w_s

    # Update the state by integrating the derivatives over the time step (dt)
    x_new = x + dxdt * dt
    y_new = y + dydt * dt
    theta_new = theta + dthetadt * dt
    phi_new = phi + dphidt * dt

    # Return the updated state
    return [x_new, y_new, theta_new, phi_new]
