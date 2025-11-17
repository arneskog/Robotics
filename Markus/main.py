import road
import car
import sensors
import state_feedback_control as state_feedback_control
import mpc_control


import matplotlib.pyplot as plt
import numpy as np


CAR_SPEED = 2
CAR_LENGTH = 1

K_S = 0.1
K_L = 0.05

CRITICAL_DISTANCE = 0.2

LANE_WIDTH = 2
X_MAX = 100
N = 1000



def main():
    x_center, y_center, theta_center, x_lane_1, y_lane_1, x_lane_2, y_lane_2 = road.build_road(X_max=X_MAX, N=N, A=2, k=0.1, half_width=LANE_WIDTH/2)

    time_array = np.linspace(0, 30, 1000)

    initial_state = [0, 0, 0, 0]  # [x(0), y(0), theta(0), phi(0)]
    states = np.zeros((len(time_array), 4))
    states[0] = initial_state
    dt=time_array[1]-time_array[0]

    ref = np.zeros((len(time_array), 2))

    for i in range(len(time_array)-1):
        x_car, y_car, theta, phi = states[i]

        
        camera_measurement = sensors.camera_sensor(x_car, y_car, x_lane_1, y_lane_1, x_lane_2, y_lane_2)
      
        if camera_measurement == None:
            break
        else:
            measured_x_lane_1, measured_y_lane_1, distances_to_car_lane_1, measured_x_lane_2, measured_y_lane_2, distances_to_car_lane_2 = camera_measurement
        
        if (np.min(distances_to_car_lane_1) <= CRITICAL_DISTANCE) or (np.min(distances_to_car_lane_2) <= CRITICAL_DISTANCE):
            
            if (np.sum(measured_x_lane_1 > x_car) and np.sum(measured_x_lane_2 > x_car)) > int(8*N/X_MAX): # Ma distance for the lookahead to 8m
                indices_ahead_1 = np.where(measured_x_lane_1 > x_car)[0]
                indices_ahead_2 = np.where(measured_x_lane_2 > x_car)[0]

                idx_1 = indices_ahead_1[int(8*N/X_MAX)-1]
                idx_2 = indices_ahead_2[int(8*N/X_MAX)-1]
                

            else:
                if len(measured_x_lane_1) <= len(measured_x_lane_2):
                    idx_1 = -1
                    idx_2 = len(measured_x_lane_1) - 1
                else:
                    idx_1 = len(measured_x_lane_2) - 1
                    idx_2 = -1

            x_ref = 0.5 * (measured_x_lane_1[idx_1] + measured_x_lane_2[idx_2])
            y_ref = 0.5 * (measured_y_lane_1[idx_1] + measured_y_lane_2[idx_2])

            #print(x_car, y_car)

            x_ref_array, y_ref_array = mpc_control.generate_trajectory(x_ref, y_ref, x_car, y_car, v=CAR_SPEED, dt=dt)


            w_s = mpc_control.solve_mpc(states[i], x_ref_array, y_ref_array, L=CAR_LENGTH, v=CAR_SPEED, dt=dt, horizon=len(x_ref_array))

            # theta_ref = np.arctan2(y_ref - y_car, x_ref - x_car)
            # w_s = state_feedback_control.LTA_control_law(states[i], x_ref, y_ref, theta_ref, K_s = K_S, K_l = K_L)
            # ref[i] = (x_ref, y_ref, theta_ref)
            # if x_ref or y_ref == 0:
            #     print("WARNING")
        else:
            w_s = 0
            ref[i] = (0, 0)

        states[i+1] = car.car_kinematics(states[i], L = CAR_LENGTH, v = CAR_SPEED, w_s=w_s, dt=dt)

    plt.figure(figsize=(12, 6))

    # Plot lanes
    plt.plot(x_lane_1, y_lane_1, '-', color='gray', label="Lane 1 (left)")
    plt.plot(x_lane_2, y_lane_2, '-', color='gray', label="Lane 2 (right)")

    #plt.plot(measured_x_lane_1, measured_y_lane_1, '--', color='black', label="Measured lane 1 (left)")
    #plt.plot(measured_x_lane_2, measured_y_lane_2, '--', color='black', label="Measured lane 2 (right)")

    plt.plot(states[:, 0], states[:, 1], label="Car Trajectory", color="red")
    plt.plot(ref[:, 0], ref[:, 1], 'x', label="Ref", color='blue')
    
    plt.axis('equal')
    plt.grid(True)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Road Visualization")
    plt.legend()

    plt.show()




if __name__ == "__main__":
    main()