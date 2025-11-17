import numpy as np





def build_road(X_max, N, A, k, half_width):
    """
    Build a static road in space:
      - centerline: x_center, y_center
      - heading at each point: theta_center
      - left/right lanes: x_lane_1, y_lane_1, x_lane_2, y_lane_2
    """
    x_center = np.linspace(0.0, X_max, N)
    y_center = A * np.sin(k * x_center)

    dy_dx = A * k * np.cos(k * x_center)
    theta_center = np.arctan2(dy_dx, np.ones_like(dy_dx))  # atan(dy/dx)

    # lane offsets (pure geometry)
    x_lane_1 = x_center - half_width * np.sin(theta_center)
    y_lane_1 = y_center + half_width * np.cos(theta_center)

    x_lane_2 = x_center + half_width * np.sin(theta_center)
    y_lane_2 = y_center - half_width * np.cos(theta_center)

    return x_center, y_center, theta_center, x_lane_1, y_lane_1, x_lane_2, y_lane_2
