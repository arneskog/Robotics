from config import *
def camera_sensor(car_x, car_y, x_lane_1, y_lane_1, x_lane_2, y_lane_2):

    lane_1_measurement =measure_lane(car_x, car_y, x_lane_1, y_lane_1)
    lane_2_measurement =measure_lane(car_x, car_y, x_lane_2, y_lane_2)

    if lane_1_measurement == None or lane_2_measurement == None:
        return None
    else:
        measured_x_lane_1, measured_y_lane_1, distances_to_car_lane_1 = lane_1_measurement
        measured_x_lane_2, measured_y_lane_2, distances_to_car_lane_2 = lane_2_measurement

        return measured_x_lane_1, measured_y_lane_1, distances_to_car_lane_1, measured_x_lane_2, measured_y_lane_2, distances_to_car_lane_2
    
def measure_lane(car_x, car_y, x_lane, y_lane):
        dx = x_lane - car_x
        dy = y_lane - car_y
        dist = np.sqrt(dx**2 + dy**2)
        mask = dist <= MAX_RANGE

        x_detected = x_lane[mask]
        y_detected = y_lane[mask]
        dist_detected = dist[mask]

        if ENABLE_NOISE:
            sigmas = BASE_SIGMA + SIGMA_SLOPE * dist_detected
            x_noisy = x_detected + np.random.normal(0.0, sigmas)
            y_noisy = y_detected + np.random.normal(0.0, sigmas)
        else:
            x_noisy = x_detected
            y_noisy = y_detected

        return x_noisy, y_noisy, dist_detected
