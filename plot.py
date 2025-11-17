import matplotlib.pyplot as plt
import numpy as np
from config import FPS


def plot_lanes_and_position_car(state_array, points_left_world, points_right_world, save_path=None):
    """Plot lane lines (from provided lane point lists) and car XY trajectory.

    - `state_array`: list of (x, y) tuples
    - `points_left_world`: list of (x, y) tuples defining the left lane
    - `points_right_world`: list of (x, y) tuples defining the right lane
    - `save_path`: optional path to save PNG; if None the plot window is shown
    """
    if not state_array:
        return

    xs_car = np.array([p[0] for p in state_array])
    ys_car = np.array([p[1] for p in state_array])

    plt.figure(figsize=(10, 6))

    # plot left lane if available
    if points_left_world and len(points_left_world) > 0:
        xl, yl = zip(*points_left_world)
        plt.plot(xl, yl, color=(0.8, 0.8, 0.8), linewidth=2, label='left lane')

    # plot right lane if available
    if points_right_world and len(points_right_world) > 0:
        xr, yr = zip(*points_right_world)
        plt.plot(xr, yr, color=(0.8, 0.8, 0.8), linewidth=2, label='right lane')

    plt.scatter(xs_car, ys_car, s=8, c='blue', label='car path')
    plt.plot(xs_car, ys_car, color='blue', linewidth=1, alpha=0.6)

    plt.xlabel('x (world)')
    plt.ylabel('y (world)')
    plt.title('Car trajectory and lanes')
    plt.axis('equal')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend()

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        plt.close()
    else:
        plt.show()


def plot_omega_s(omega_s_array, time_array=None, save_path=None):
    """Plot steering input over time.

    - `omega_s_array`: list of omega_s samples
    - `time_array`: optional list of timestamps (same length). If None, FPS from config is used.
    """
    import numpy as _np

    ys = _np.array(omega_s_array)
    if time_array is None:
        ts = _np.arange(len(ys)) / float(FPS)
    else:
        ts = _np.array(time_array)

    plt.figure(figsize=(8, 3.5))
    plt.plot(ts, ys, color='tab:orange')
    plt.xlabel('time (s)')
    plt.ylabel('omega_s (rad/s)')
    plt.title('Human steering input over time')
    plt.grid(True, linestyle=':', alpha=0.6)

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        plt.close()
    else:
        plt.show()
from config import *



def plot_lanes_and_position_car(state_array, x_margin=10.0, save_path=None):
	"""Plot lane lines and car XY trajectory collected during runtime.

	- `state_array`: list of (x, y) tuples
	- `x_margin`: how far beyond recorded x-range to render lanes
	- `save_path`: optional path to save PNG; if None the plot window is shown
	"""
	if len(state_array) == 0:
		return

	xs_car = np.array([p[0] for p in state_array])
	ys_car = np.array([p[1] for p in state_array])

	x_min = float(np.min(xs_car)) - x_margin
	x_max = float(np.max(xs_car)) + x_margin

	xs = np.linspace(x_min, x_max, 2000)
	y_center = AMPLITUDE * np.sin(FREGUENCY * xs)
	y_left = y_center - LANE_WIDTH / 2.0
	y_right = y_center + LANE_WIDTH / 2.0

	plt.figure(figsize=(10, 6))
	plt.plot(xs, y_left, color=(0.8, 0.8, 0.8), linewidth=2, label='left lane')
	plt.plot(xs, y_right, color=(0.8, 0.8, 0.8), linewidth=2, label='right lane')
	plt.plot(xs, y_center, color=(0.6, 0.6, 0.6), linestyle='--', linewidth=1, label='center')

	plt.scatter(xs_car, ys_car, s=8, c='blue', label='car path')
	plt.plot(xs_car, ys_car, color='blue', linewidth=1, alpha=0.6)

	plt.xlabel('x (world)')
	plt.ylabel('y (world)')
	plt.title('Car trajectory and lanes')
	plt.axis('equal')
	plt.grid(True, linestyle=':', alpha=0.6)
	plt.legend()

	if save_path:
		plt.savefig(save_path, dpi=200, bbox_inches='tight')
		plt.close()
	else:
		plt.show()


def plot_omega_s(omega_s_array, time_array=None, save_path=None):
	"""Plot steering input over time.

	- `omega_s_array`: list of omega_s samples
	- `time_array`: optional list of timestamps (same length). If None, FPS from config is used.
	"""
	import numpy as _np

	ys = _np.array(omega_s_array)
	if time_array is None:
		ts = _np.arange(len(ys)) / float(FPS)
	else:
		ts = _np.array(time_array)

	plt.figure(figsize=(8, 3.5))
	plt.plot(ts, ys, color='tab:orange')
	plt.xlabel('time (s)')
	plt.ylabel('omega_s (rad/s)')
	plt.title('Human steering input over time')
	plt.grid(True, linestyle=':', alpha=0.6)

	if save_path:
		plt.savefig(save_path, dpi=200, bbox_inches='tight')
		plt.close()
	else:
		plt.show()