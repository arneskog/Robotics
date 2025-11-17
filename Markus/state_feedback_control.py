import numpy as np


def LTA_control_law(state, x_ref, y_ref, theta_ref, K_s, K_l):
    x, y, theta, phi = state

    # Position error in world frame
    x_e = x_ref - x
    y_e = y_ref - y

    # Heading error (wrap to [-pi, pi] ideally)
    theta_e = theta_ref - theta
    theta_e = (theta_e + np.pi) % (2*np.pi) - np.pi

    # Transform to body frame
    b_e_x = np.cos(theta) * x_e + np.sin(theta) * y_e
    b_e_y = -np.sin(theta) * x_e + np.cos(theta) * y_e
    b_e_theta = theta_e

    # Lateral control: state-feedback on heading + lateral error
    w_s = K_s * b_e_theta + K_l * b_e_y

    return w_s
