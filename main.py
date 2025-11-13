from config import *
from enviorment import *
from joystick import *
from carModel import *

def main():
    state = INIT_STATE   
    omega_s = INIT_OMEGA_S
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0  

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False


        omega_s = joystick_input(omega_s)

        V_step = V * dt  
        state = car_kinematics(state, L, V_step, omega_s)


        if state[3] > PHI_MAX:
            state[3]  = PHI_MAX
        elif state[3]  < -PHI_MAX:
            state[3]  = -PHI_MAX

        screen.fill((30, 30, 30))

        points_left_lane, points_right_lane = draw_sin_lanes(screen, state[0], state[1])

        points_car_corners = draw_car(screen, state)
        

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
