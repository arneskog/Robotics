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
    # coarser sampling to reduce drawing/work per frame
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

    # split world points into separate x/y lists so callers can unpack easily
    x_left = [p[0] for p in points_left_world]
    y_left = [p[1] for p in points_left_world]
    x_right = [p[0] for p in points_right_world]
    y_right = [p[1] for p in points_right_world]

    return x_left, y_left, x_right, y_right


def draw_camera_points(screen, car_x, car_y,
                       measured_x_lane_1, measured_y_lane_1,
                       measured_x_lane_2=None, measured_y_lane_2=None,
                       color_left=(0, 255, 0), color_right=(255, 255, 0), radius=4):
    """Draw points detected by the camera sensor.

    - `measured_x_lane_*`, `measured_y_lane_*` can be lists or numpy arrays.
    - If lane 2 is not provided, only lane 1 points are drawn.
    """
    # draw left-lane detections
    if measured_x_lane_1 is not None and measured_y_lane_1 is not None:
        try:
            for mx, my in zip(measured_x_lane_1, measured_y_lane_1):
                if mx is None or my is None:
                    continue
                sx, sy = world_to_screen(float(mx), float(my), car_x, car_y)
                pygame.draw.circle(screen, color_left, (sx, sy), radius)
        except Exception:
            # if inputs can't be iterated, skip drawing
            pass

    # draw right-lane detections
    if measured_x_lane_2 is not None and measured_y_lane_2 is not None:
        try:
            for mx, my in zip(measured_x_lane_2, measured_y_lane_2):
                if mx is None or my is None:
                    continue
                sx, sy = world_to_screen(float(mx), float(my), car_x, car_y)
                pygame.draw.circle(screen, color_right, (sx, sy), radius)
        except Exception:
            pass
    

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


def draw_mpc_trajectory(screen, car_x, car_y, x_ref_array, y_ref_array, color=(255, 0, 255), width=2, marker_radius=3):
    """Draw MPC-generated trajectory (world coordinates) on the screen.

    - `x_ref_array`, `y_ref_array` can be lists or numpy arrays of same length.
    - Returns the list of screen-space points drawn.
    """
    if x_ref_array is None or y_ref_array is None:
        return []

    pts = []
    try:
        for xw, yw in zip(x_ref_array, y_ref_array):
            sx, sy = world_to_screen(float(xw), float(yw), car_x, car_y)
            pts.append((sx, sy))

        if not pts:
            return []

        # draw polyline and small markers at each waypoint
        pygame.draw.lines(screen, color, False, pts, width)
        for p in pts:
            pygame.draw.circle(screen, color, p, marker_radius)

        return pts
    except Exception:
        return []