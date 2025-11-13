from config import *
from enviorment import *
from joystick import *
from carModel import *
from sensor import *
from plot import *

def main():
    state = INIT_STATE   
    omega_s = INIT_OMEGA_S
    running = True
    omega_s_array = []
    omega_s_array.append(omega_s)
    state_array = []
    state_array.append((state[0], state[1]))
    time_array = []
    time_array.append(0.0)
    current_time = 0.0


    while running:
        dt = clock.tick(FPS) / 1000.0  

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False


        omega_s_human = joystick_input(omega_s)
        omega_s_array.append(omega_s_human)

        current_time += dt
        time_array.append(current_time)

        V_step = V * dt  
        state = car_kinematics(state, L, V_step, omega_s_human)
        state_array.append((state[0], state[1]))


        if state[3] > PHI_MAX:
            state[3]  = PHI_MAX
        elif state[3]  < -PHI_MAX:
            state[3]  = -PHI_MAX

        screen.fill((30, 30, 30))

        points_left_lane, points_right_lane = draw_sin_lanes(screen, state[0], state[1])

        draw_car(screen, state)

        detect_and_draw_sensor_points(screen, state, points_left_lane, points_right_lane, steps=None, max_lateral=3.0)
        

        pygame.display.flip()

    pygame.quit()

    plot_lanes_and_position_car(state_array)
    plot_omega_s(omega_s_array, time_array)


    sys.exit()

if __name__ == "__main__":
    main()
