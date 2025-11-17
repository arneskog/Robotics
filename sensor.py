from config import *
from enviorment import *

def detect_and_draw_sensor_points(screen, state, points_left_world, points_right_world, steps=None, max_lateral=3.0):
    if steps is None:
        steps = [5.0, 10.0, 15.0, 25.0]

    car_x, car_y, theta, _ = state
    dir_x = math.cos(theta)
    dir_y = math.sin(theta)

    detected_left = []
    detected_right = []

    for d in steps:
        sx = car_x + d * dir_x
        sy = car_y + d * dir_y

        # find nearest point on left lane ahead of the car
        best_l = None
        best_ld = float('inf')
        for px, py in points_left_world:
            dx = px - sx
            dy = py - sy
            dist = math.hypot(dx, dy)
            proj = ((px - car_x) * dir_x + (py - car_y) * dir_y)
            if proj >= 0 and dist < best_ld:
                best_ld = dist
                best_l = (px, py)

        if best_l is not None and best_ld <= max_lateral:
            detected_left.append(best_l)
            ps = world_to_screen(best_l[0], best_l[1], car_x, car_y)
            pygame.draw.circle(screen, (0, 255, 0), ps, 5)
        else:
            detected_left.append(None)

        # find nearest point on right lane ahead of the car
        best_r = None
        best_rd = float('inf')
        for px, py in points_right_world:
            dx = px - sx
            dy = py - sy
            dist = math.hypot(dx, dy)
            proj = ((px - car_x) * dir_x + (py - car_y) * dir_y)
            if proj >= 0 and dist < best_rd:
                best_rd = dist
                best_r = (px, py)

        if best_r is not None and best_rd <= max_lateral:
            detected_right.append(best_r)
            ps = world_to_screen(best_r[0], best_r[1], car_x, car_y)
            pygame.draw.circle(screen, (255, 255, 0), ps, 5)
        else:
            detected_right.append(None)

        # draw sensor sample origin (small red dot)
        s_screen = world_to_screen(sx, sy, car_x, car_y)
        pygame.draw.circle(screen, (255, 0, 0), s_screen, 3)

    return detected_left, detected_right
