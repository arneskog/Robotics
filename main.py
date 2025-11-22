from config import *
from enviorment import *
import numpy as np
from joystick import *
from carModel import *
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
    
    sensor_snapshots = []          
    snapshot_interval = 5.0        
    next_snapshot_time = 0.0



    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        screen.fill((30, 30, 30))
        x_lane_1, y_lane_1, x_lane_2, y_lane_2 = draw_sin_lanes(screen, state[0], state[1])
        draw_car(screen, state)

        camera_measurement = camera_sensor(state[0], state[1], x_lane_1, y_lane_1, x_lane_2, y_lane_2)

        omega_s_human = joystick_input(omega_s)

        if camera_measurement == None:
            continue
        else:
            measured_x_lane_1, measured_y_lane_1, distances_to_car_lane_1, measured_x_lane_2, measured_y_lane_2, distances_to_car_lane_2 = camera_measurement
            draw_camera_points(screen, state[0], state[1], measured_x_lane_1, measured_y_lane_1, measured_x_lane_2, measured_y_lane_2)
        if CONTROLLER_ACTIVE and ((np.min(distances_to_car_lane_1) <= CRITICAL_DISTANCE) or (np.min(distances_to_car_lane_2) <= CRITICAL_DISTANCE)):

            if (np.min(distances_to_car_lane_1) <= CRITICAL_DISTANCE):
                near_left_lane = True
            if (np.min(distances_to_car_lane_2) <= CRITICAL_DISTANCE):
                near_right_lane = True
            
            if (np.sum(measured_x_lane_1 > state[0]) and np.sum(measured_x_lane_2 > state[0])) > int(LOOKAHEAD_DISTANCE/SAMPLING_INTERVAL):
                indices_ahead_1 = np.where(measured_x_lane_1 > state[0])[0]
                indices_ahead_2 = np.where(measured_x_lane_2 > state[0])[0]

                idx_1 = indices_ahead_1[int(LOOKAHEAD_DISTANCE/SAMPLING_INTERVAL)-1]
                idx_2 = indices_ahead_2[int(LOOKAHEAD_DISTANCE/SAMPLING_INTERVAL)-1]
                

            else:
                if len(measured_x_lane_1) <= len(measured_x_lane_2):
                    idx_1 = -1
                    idx_2 = len(measured_x_lane_1) - 1
                else:
                    idx_1 = len(measured_x_lane_2) - 1
                    idx_2 = -1

            x_ref = 0.5 * (measured_x_lane_1[idx_1] + measured_x_lane_2[idx_2])
            y_ref = 0.5 * (measured_y_lane_1[idx_1] + measured_y_lane_2[idx_2])

            x_ref_array, y_ref_array = generate_trajectory(x_ref, y_ref, state[0], state[1], V, near_left_lane, near_right_lane, dt)

            horizon = min(len(x_ref_array), LOOKAHEAD_DISTANCE)
            w_s = solve_mpc(state, x_ref_array, y_ref_array, L, V, dt=dt, horizon=horizon)
            draw_mpc_trajectory(screen, state[0], state[1], x_ref_array, y_ref_array)

        else:
            w_s = 0
            ref.append((0, 0))
            near_right_lane = False
            near_left_lane = False


        
        omega_s_array_human.append(omega_s_human)
        omega_s_array_mpc.append(w_s)

        current_time += dt
        time_array.append(current_time)

        if current_time >= next_snapshot_time:
            sensor_snapshots.append([
                measured_x_lane_1.copy(),
                measured_y_lane_1.copy(),
                measured_x_lane_2.copy(),
                measured_y_lane_2.copy()
            ])
            next_snapshot_time += snapshot_interval

        V_step = V*dt  
        state = car_kinematics(state, L, V_step, omega_s_human+w_s)


        state_array.append((state[0], state[1]))


        if state[3] > PHI_MAX:
            state[3]  = PHI_MAX
        elif state[3]  < -PHI_MAX:
            state[3]  = -PHI_MAX  


        pygame.display.flip()

    pygame.quit()
    lanes_and_position_path   = os.path.join('Results', f'lanes_and_position.png') 
    human_path = os.path.join('Results', f'omega_human.png')
    mpc_path   = os.path.join('Results', f'omega_mpc.png')
    sensor_snapshots_path = os.path.join('Results', 'sensor_snapshots.png')


    plot_lanes_and_position_car(state_array, save_path=lanes_and_position_path)
    plot_omega_s_human(omega_s_array_human, time_array=time_array, save_path=human_path)
    plot_omega_s_mpc(omega_s_array_mpc, time_array=time_array, save_path=mpc_path)
    plot_sensor_snapshots(sensor_snapshots, state_array=state_array, save_path=sensor_snapshots_path)
 
    sys.exit()

if __name__ == "__main__":
    main()
