from config import *
from enviorment import *
import numpy as np
from joystick import *
from carModel import *
from sensor_old import *
from plot import *
from sensors import *
from mpc_control import *

def main():
    state = INIT_STATE   
    omega_s = INIT_OMEGA_S
    running = True
    omega_s_array_human = []
    omega_s_array_human.append(omega_s)
    omega_s_array_mpc = []
    omega_s_array_mpc.append(omega_s)
    state_array = []
    state_array.append((state[0], state[1]))
    time_array = []
    time_array.append(0.0)
    current_time = 0.0
    ref = []
    # throttle MPC to run only every N frames to reduce CPU load
    frame_counter = 0
    MPC_INTERVAL = 4  # run MPC every 4 frames (approx every 4/FPS seconds)


    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        screen.fill((30, 30, 30))
        x_lane_1, y_lane_1, x_lane_2, y_lane_2 = draw_sin_lanes(screen, state[0], state[1])
        # convert to numpy arrays for vectorized operations in sensors
        x_lane_1 = np.array(x_lane_1, dtype=float)
        y_lane_1 = np.array(y_lane_1, dtype=float)
        x_lane_2 = np.array(x_lane_2, dtype=float)
        y_lane_2 = np.array(y_lane_2, dtype=float)
        draw_car(screen, state)

        camera_measurement = camera_sensor(state[0], state[1], x_lane_1, y_lane_1, x_lane_2, y_lane_2)

        omega_s_human = joystick_input(omega_s)

        if camera_measurement == None:
            continue
        else:
            measured_x_lane_1, measured_y_lane_1, distances_to_car_lane_1, measured_x_lane_2, measured_y_lane_2, distances_to_car_lane_2 = camera_measurement
            draw_camera_points(screen, state[0], state[1], measured_x_lane_1, measured_y_lane_1, measured_x_lane_2, measured_y_lane_2)
        if (np.min(distances_to_car_lane_1) <= CRITICAL_DISTANCE) or (np.min(distances_to_car_lane_2) <= CRITICAL_DISTANCE):
            
            if (np.sum(measured_x_lane_1 > state[0]) and np.sum(measured_x_lane_2 > state[0])) > int(32): # Ma distance for the lookahead to 8m
                indices_ahead_1 = np.where(measured_x_lane_1 > state[0])[0]
                indices_ahead_2 = np.where(measured_x_lane_2 > state[0])[0]

                idx_1 = indices_ahead_1[int(32)-1]
                idx_2 = indices_ahead_2[int(32)-1]
                

            else:
                if len(measured_x_lane_1) <= len(measured_x_lane_2):
                    idx_1 = -1
                    idx_2 = len(measured_x_lane_1) - 1
                else:
                    idx_1 = len(measured_x_lane_2) - 1
                    idx_2 = -1

            x_ref = 0.5 * (measured_x_lane_1[idx_1] + measured_x_lane_2[idx_2])
            y_ref = 0.5 * (measured_y_lane_1[idx_1] + measured_y_lane_2[idx_2])

            x_ref_array, y_ref_array = generate_trajectory(x_ref, y_ref, state[0], state[1], V, dt=dt)

            horizon = min(len(x_ref_array), 32)
            w_s = solve_mpc(state, x_ref_array, y_ref_array, L, V, dt=dt, horizon=horizon)
            # draw the MPC reference trajectory on the screen
            try:
                draw_mpc_trajectory(screen, state[0], state[1], x_ref_array, y_ref_array)
            except Exception:
                pass
            omega_s * DAMPED_OMEGA

        else:
            w_s = 0
            ref.append((0, 0))

        
        omega_s_array_human.append(omega_s_human)
        omega_s_array_mpc.append(w_s)

        current_time += dt
        time_array.append(current_time)

        V_step = V * dt  
        state = car_kinematics(state, L, V_step, omega_s_human+w_s)


        state_array.append((state[0], state[1]))


        if state[3] > PHI_MAX:
            state[3]  = PHI_MAX
        elif state[3]  < -PHI_MAX:
            state[3]  = -PHI_MAX  


        pygame.display.flip()

        frame_counter += 1

    pygame.quit()

    plot_lanes_and_position_car(state_array)
    plot_omega_s(omega_s_array_human, time_array)
    plot_omega_s(omega_s_array_mpc, time_array)
    sys.exit()

if __name__ == "__main__":
    main()
