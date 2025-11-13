import pygame
import math
import sys


pygame.init()
WIDTH, HEIGHT = 900, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()


L = 2.5                
V = 10.0               
SCALE = 25.0           
FPS = 60


PHI_MAX = math.radians(35)
PHI_RATE = math.radians(3)


LANE_WIDTH = 6       
AMPLITUDE = 4.0              
FREGUENCY = 0.05             


#Task 1, joystick function to control angel on front wheel 
def joystick_input(omega_s):
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        omega_s = +PHI_RATE
    elif keys[pygame.K_RIGHT]:
        omega_s = -PHI_RATE
    else:
        omega_s = 0.0
    return omega_s


#Task 2a, model of the car
def car_kinematics(state, L, V_step, omega_s):
    x, y, theta, phi = state

    dx = math.cos(theta) * math.cos(phi) * V_step
    dy = math.sin(theta) * math.cos(phi) * V_step
    dtheta = math.sin(phi) / L * V_step
    dphi = omega_s

    x += dx
    y += dy
    theta += dtheta
    phi += dphi

    return [x, y, theta, phi]

#Task 2b, model for sensor onboard


#Task 3, world_to_screen, draw_sin_lanes and draw_car to create a simulation enviornment
def world_to_screen(wx, wy, car_x, car_y):
    sx = WIDTH / 2 + (wx - car_x) * SCALE
    sy = HEIGHT / 2 - (wy - car_y) * SCALE
    return int(sx), int(sy)

def draw_sin_lanes(screen, car_x, car_y):
    points_left = []
    points_right = []

    x_min = car_x - 30
    x_max = car_x + 50
    step = 0.5 

    x = x_min
    while x <= x_max:
        y_center = AMPLITUDE * math.sin(FREGUENCY * x)
        y_left = y_center - LANE_WIDTH / 2.0
        y_right = y_center + LANE_WIDTH / 2.0

        sx_left, sy_left = world_to_screen(x, y_left, car_x, car_y)
        sx_right, sy_right = world_to_screen(x, y_right, car_x, car_y)

        points_left.append((sx_left, sy_left))
        points_right.append((sx_right, sy_right))

        x += step

    pygame.draw.lines(screen, (200, 200, 200), False, points_left, 3)
    pygame.draw.lines(screen, (200, 200, 200), False, points_right, 3)

    

def draw_car(screen, state):
    x, y, theta, phi = state

    car_length = 4.0
    car_width = 2.0

    half_L = car_length / 2.0
    half_W = car_width / 2.0

    corners = [
        (+half_L, +half_W),
        (+half_L, -half_W),
        (-half_L, -half_W),
        (-half_L, +half_W),
    ]

    world_corners = []
    for cx, cy in corners:
        wx = x + cx * math.cos(theta) - cy * math.sin(theta)
        wy = y + cx * math.sin(theta) + cy * math.cos(theta)
        world_corners.append((wx, wy))

    screen_corners = [
        world_to_screen(wx, wy, x, y) for (wx, wy) in world_corners
    ]

    pygame.draw.polygon(screen, (0, 100, 255), screen_corners)


    front_world = (
        x + half_L * math.cos(theta),
        y + half_L * math.sin(theta),
    )
    front_screen = world_to_screen(front_world[0], front_world[1], x, y)

    #line to show front wheel direction
    wheel_len = 2.0
    wx_wheel = front_world[0] + wheel_len * math.cos(theta + phi)
    wy_wheel = front_world[1] + wheel_len * math.sin(theta + phi)
    wheel_screen = world_to_screen(wx_wheel, wy_wheel, x, y)
    pygame.draw.line(screen, (255, 0, 0), front_screen, wheel_screen, 2)


#Task 4, plot detection of the lane's left and right white lines

#Task 5, controller for LTA, show effectivness with empiric strategies 

#Task 6, mathematical arguments show effectivness

def main():
    state = [0.0, 0.0, 0.0, 0.0]   #[x, y, theta, phi]
    omega_s = 0
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

        draw_sin_lanes(screen, state[0], state[1])

        draw_car(screen, state)
        

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
