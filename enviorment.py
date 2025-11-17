from config import *

def world_to_screen(wx, wy, car_x, car_y):
    sx = WIDTH / 2 + (wx - car_x) * SCALE
    sy = HEIGHT / 2 - (wy - car_y) * SCALE
    return int(sx), int(sy)

def draw_sin_lanes(screen, car_x, car_y):
    points_left_screen = []
    points_right_screen = []

    points_left_world = []
    points_right_world = []

    x_min = car_x - 30
    x_max = car_x + 50
    step = 0.5

    x = x_min
    while x <= x_max:
        y_center = AMPLITUDE * math.sin(FREGUENCY * x)
        y_left = y_center - LANE_WIDTH / 2.0
        y_right = y_center + LANE_WIDTH / 2.0

        points_left_world.append((x, y_left))
        points_right_world.append((x, y_right))

        sx_left, sy_left = world_to_screen(x, y_left, car_x, car_y)
        sx_right, sy_right = world_to_screen(x, y_right, car_x, car_y)

        points_left_screen.append((sx_left, sy_left))
        points_right_screen.append((sx_right, sy_right))

        x += step

    pygame.draw.lines(screen, (200, 200, 200), False, points_left_screen, 3)
    pygame.draw.lines(screen, (200, 200, 200), False, points_right_screen, 3)

    return points_left_world, points_right_world
    

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