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
    step = 1.0

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
    
    x_lane_1 = np.array([p[0] for p in points_left_world], dtype=float)
    y_lane_1 = np.array([p[1] for p in points_left_world], dtype=float)
    x_lane_2 = np.array([p[0] for p in points_right_world], dtype=float)
    y_lane_2 = np.array([p[1] for p in points_right_world], dtype=float)

    return x_lane_1, y_lane_1, x_lane_2, y_lane_2


def draw_camera_points(screen, car_x, car_y,
                       measured_x_lane_1, measured_y_lane_1,
                       measured_x_lane_2=None, measured_y_lane_2=None,
                       color_left=(0, 255, 0), color_right=(255, 255, 0), radius=4):
    
    # draw left-lane detections
    if measured_x_lane_1 is not None and measured_y_lane_1 is not None:
            for mx, my in zip(measured_x_lane_1, measured_y_lane_1):
                sx, sy = world_to_screen(float(mx), float(my), car_x, car_y)
                pygame.draw.circle(screen, color_left, (sx, sy), radius)

    # draw right-lane detections
    if measured_x_lane_2 is not None and measured_y_lane_2 is not None:
            for mx, my in zip(measured_x_lane_2, measured_y_lane_2):
                sx, sy = world_to_screen(float(mx), float(my), car_x, car_y)
                pygame.draw.circle(screen, color_right, (sx, sy), radius)
    

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

    wheel_len = 2.0
    wx_wheel = front_world[0] + wheel_len * math.cos(theta + phi)
    wy_wheel = front_world[1] + wheel_len * math.sin(theta + phi)
    wheel_screen = world_to_screen(wx_wheel, wy_wheel, x, y)
    pygame.draw.line(screen, (255, 0, 0), front_screen, wheel_screen, 2)


def draw_mpc_trajectory(screen, car_x, car_y, x_ref_array, y_ref_array, color=(255, 0, 255), width=2):
    if x_ref_array is not None or y_ref_array is not None:
        pts = []
        for xw, yw in zip(x_ref_array, y_ref_array):
            sx, sy = world_to_screen(float(xw), float(yw), car_x, car_y)
            pts.append((sx, sy))

        pygame.draw.lines(screen, color, False, pts, width)