import pygame
import math
import sys

# ---------- Car kinematics (your function, slightly adapted to use math) ----------

# ---------- Pygame setup ----------
pygame.init()
WIDTH, HEIGHT = 900, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()


L = 2.5                 # distance between back and front wheel
V = 10.0                # constant forward speed (m/s)
SCALE = 25.0            # pixels per meter
FPS = 60

# Steering parameters
OMEGA_MAX = 1.5             # max steering rate (rad/s)
PHI_MAX = math.radians(35)  # max steering angle


LANE_WIDTH = 6       # distance between left and right lane (meters)
A = 4.0              # amplitude of sin in meters
K = 0.05             # spatial frequency of sin




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


def joystick_input(omega_s):
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        omega_s = +0.04
    elif keys[pygame.K_RIGHT]:
        omega_s = -0.04
    else:
        omega_s = 0.0
    return omega_s

def world_to_screen(wx, wy, car_x, car_y):
    """
    Transform world coordinates (wx, wy) into screen coordinates.
    The camera is centered on the car.
    """
    sx = WIDTH / 2 + (wx - car_x) * SCALE
    sy = HEIGHT / 2 - (wy - car_y) * SCALE
    return int(sx), int(sy)

def draw_sin_lanes(screen, car_x, car_y):
    """
    Draw two lanes defined as sinusoidal curves in world coordinates,
    centered around y = A * sin(K * x).
    """
    points_left = []
    points_right = []

    # Draw for an x-range around the car
    x_min = car_x - 30
    x_max = car_x + 50
    step = 0.5  # meters

    x = x_min
    while x <= x_max:
        y_center = A * math.sin(K * x)
        y_left = y_center - LANE_WIDTH / 2.0
        y_right = y_center + LANE_WIDTH / 2.0

        sx_left, sy_left = world_to_screen(x, y_left, car_x, car_y)
        sx_right, sy_right = world_to_screen(x, y_right, car_x, car_y)

        points_left.append((sx_left, sy_left))
        points_right.append((sx_right, sy_right))

        x += step

    if len(points_left) > 1:
        pygame.draw.lines(screen, (200, 200, 200), False, points_left, 3)
    if len(points_right) > 1:
        pygame.draw.lines(screen, (200, 200, 200), False, points_right, 3)

def draw_car(screen, state):
    x, y, theta, phi = state

    # Car size in meters
    car_length = 4.0
    car_width = 2.0

    # Define rectangle in car local frame (centered at (0,0))
    half_L = car_length / 2.0
    half_W = car_width / 2.0

    corners = [
        (+half_L, +half_W),
        (+half_L, -half_W),
        (-half_L, -half_W),
        (-half_L, +half_W),
    ]

    # Rotate and translate into world coordinates
    world_corners = []
    for cx, cy in corners:
        wx = x + cx * math.cos(theta) - cy * math.sin(theta)
        wy = y + cx * math.sin(theta) + cy * math.cos(theta)
        world_corners.append((wx, wy))

    # Convert to screen coordinates (camera centered on car)
    screen_corners = [
        world_to_screen(wx, wy, x, y) for (wx, wy) in world_corners
    ]

    # Draw car body
    pygame.draw.polygon(screen, (0, 100, 255), screen_corners)

    # Draw a small line at the front to show heading
    front_world = (
        x + half_L * math.cos(theta),
        y + half_L * math.sin(theta),
    )
    front_screen = world_to_screen(front_world[0], front_world[1], x, y)
    center_screen = world_to_screen(x, y, x, y)
    pygame.draw.line(screen, (255, 255, 255), center_screen, front_screen, 3)

    # Optional: draw front wheel direction (just a short line)
    wheel_len = 2.0
    wx_wheel = front_world[0] + wheel_len * math.cos(theta + phi)
    wy_wheel = front_world[1] + wheel_len * math.sin(theta + phi)
    wheel_screen = world_to_screen(wx_wheel, wy_wheel, x, y)
    pygame.draw.line(screen, (255, 0, 0), front_screen, wheel_screen, 2)



def main():
    state = [0.0, 0.0, 0.0, 0.0]   #[x, y, theta, phi]
    omega_s = 0
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0  

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Keyboard control
        omega_s = joystick_input(omega_s)

        # Integrate car kinematics
        V_step = V * dt  # distance this frame
        state = car_kinematics(state, L, V_step, omega_s)


        if state[3] > PHI_MAX:
            state[3]  = PHI_MAX
        elif state[3]  < -PHI_MAX:
            state[3]  = -PHI_MAX

        # ---- Drawing ----
        screen.fill((30, 30, 30))

        # Draw road lanes
        draw_sin_lanes(screen, state[0], state[1])

        # Draw car
        draw_car(screen, state)
        

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
