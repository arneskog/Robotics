from config import *



def plot_lanes_and_position_car(state_array, x_margin=10.0, save_path=None):
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


def plot_omega_s_human(omega_s_array, time_array=None, save_path=None):
    """Plot steering input over time.

    - `omega_s_array`: list of omega_s samples
    - `time_array`: optional list of timestamps (same length). If None, FPS from config is used.
    """
    import numpy as _np
    import matplotlib.pyplot as plt

    ys = _np.array(omega_s_array)
    if time_array is None:
        ts = _np.arange(len(ys)) / float(FPS)
    else:
        ts = _np.array(time_array)

    plt.figure(figsize=(10, 5))  # Adjusted size for better spacing
    plt.plot(ts, ys, color='tab:orange', linewidth=1.5)  # Decreased line width for a smoother look
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel(r'$\omega_s$ (rad/s)', fontsize=12)  # Improved readability with LaTeX
    plt.title('Human steering input over time', fontsize=14)
    plt.grid(True, linestyle=':', alpha=0.6)

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        plt.close()
    else:
        plt.show()


def plot_omega_s_mpc(omega_s_array, time_array=None, save_path=None):
    """Plot MPC steering input over time with improved appearance."""
    import numpy as _np
    import matplotlib.pyplot as plt

    ys = _np.array(omega_s_array)
    if time_array is None:
        ts = _np.arange(len(ys)) / float(FPS)
    else:
        ts = _np.array(time_array)

    plt.figure(figsize=(10, 5))  # Adjusted size for better spacing
    plt.plot(ts, ys, color='tab:blue', linewidth=1.5)  # Decreased line width for a smoother look
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel(r'$\omega_s$ (rad/s)', fontsize=12)  # Improved readability with LaTeX
    plt.title('MPC Steering Input Over Time', fontsize=14)
    plt.grid(True, linestyle=':', alpha=0.6)

    plt.xlim([0, ts[-1]])  # Ensure the x-axis covers the entire time range
    plt.ylim([min(ys) - 0.01, max(ys) + 0.01])  # Slight padding on y-axis

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        plt.close()
    else:
        plt.show()
