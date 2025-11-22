from config import *
def plot_lanes_and_position_car(state_array, x_margin=10.0, save_path=None):
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
    ys = np.array(omega_s_array)
    ts = np.array(time_array)

    plt.figure(figsize=(10, 5))
    plt.plot(ts, ys, color='tab:orange', linewidth=1.5)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel(r'$\omega_s$ (rad/s)', fontsize=12)
    plt.title('Human steering input over time', fontsize=14)
    plt.grid(True, linestyle=':', alpha=0.6)

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        plt.close()
    else:
        plt.show()


def plot_omega_s_mpc(omega_s_array, time_array=None, save_path=None):
    ys = np.array(omega_s_array)
    ts = np.array(time_array)

    plt.figure(figsize=(10, 5))
    plt.plot(ts, ys, color='tab:blue', linewidth=1.5)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel(r'$\omega_s$ (rad/s)', fontsize=12)
    plt.title('MPC Steering Input Over Time', fontsize=14)
    plt.grid(True, linestyle=':', alpha=0.6)

    plt.xlim([0, ts[-1]])
    plt.ylim([min(ys) - 0.01, max(ys) + 0.01])

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        plt.close()
    else:
        plt.show()


def plot_sensor_snapshots(sensor_snapshots, state_array, x_margin = 10.0, save_path=None):
    plt.figure(figsize=(6, 4))
    cmap = plt.get_cmap("tab10")
    num_snapshots = max(1, len(sensor_snapshots))

    for k, snapshot in enumerate(sensor_snapshots):
        if len(snapshot) >= 6:
            x1, y1, x2, y2, car_x, car_y = snapshot[:6]
        else:
            x1, y1, x2, y2 = snapshot
            car_x, car_y = (np.nan, np.nan)

        x1 = np.array(x1, dtype=float) if x1 is not None else np.array([])
        y1 = np.array(y1, dtype=float) if y1 is not None else np.array([])
        x2 = np.array(x2, dtype=float) if x2 is not None else np.array([])
        y2 = np.array(y2, dtype=float) if y2 is not None else np.array([])

        color = cmap(float(k) / float(max(1, num_snapshots - 1)))
        if x1.size > 0:
            plt.scatter(x1, y1, s=4, c=[color], marker='.', alpha=0.9)
        if x2.size > 0:
            plt.scatter(x2, y2, s=4, c=[color], marker='.', alpha=0.9)
        if not np.isnan(car_x) and not np.isnan(car_y):
            plt.scatter([car_x], [car_y], s=40, c=[color], marker='o', edgecolors='k')

    plt.xlabel("x (world)")
    plt.ylabel("y (world)")
    plt.title("Camera lane detections (snapshots)")
    plt.grid(True)
    plt.axis("equal")

    if save_path is not None:
        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        plt.close()
    else:
        plt.show()










