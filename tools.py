import numpy as np


class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.target = 0
        self.prev = 0
        self.sum = 0

    def compute(self, val):
        err = val - self.target
        derr = self.prev - err
        self.sum += err
        self.prev = err
        return self.kp * err + self.ki * self.sum + self.kd * derr

    def reset(self):
        self.prev = 0
        self.sum = 0


def filter_lidar_dir(data, head, aperture=10):
    L = []
    for measure in data:
        dh = 2 * np.arctan(np.tan(np.radians(head - measure[0]) / 2))
        if abs(dh) < np.radians(aperture / 2.):
            L.append(measure)

    return np.array(L)


def get_lidar_distance(data, dir, aperture=10):
    if dir == 'front':
        head = 0
    elif dir == 'right':
        head = 90
    elif dir == 'rear':
        head = 180
    elif dir == 'left':
        head = 270
    else:
        return -1

    filtered_data = filter_lidar_dir(data, head, aperture)
    return min(filtered_data, key=lambda item: item[1])[1]


def hough_transform(x, y, num_rho=30, num_theta=180, threshold=30):
    """
    Perform Hough Transform to detect lines in a set of points.

    Parameters:
    - X and Y: numpy arrays of shape (N, 1) containing point coordinates.
    - num_rho: Number of rho values (bins for rho)
    - num_theta: Number of theta values (bins for theta)
    - threshold: Minimum number of votes to consider a line

    Returns:
    - lines: numpy array of shape (M, 2) containing [rho, theta] for detected lines
    """

    # Compute theta range
    thetas = np.linspace(0, np.pi, num_theta)
    cos_th = np.cos(thetas)
    sin_th = np.sin(thetas)

    # Compute rho range
    max_rho = np.hypot(x, y).max()
    rho_min = -max_rho
    rho_max = max_rho
    rhos = np.linspace(rho_min, rho_max, num_rho)

    # Initialize the accumulator array
    accumulator = np.zeros((num_rho, num_theta), dtype=np.uint64)

    # Vote in the accumulator
    for xi, yi in zip(x, y):
        rho_values = (xi * cos_th + yi * sin_th)
        rho_indices = np.round((rho_values - rho_min) * (num_rho - 1) / (rho_max - rho_min)).astype(int)
        rho_indices = np.clip(rho_indices, 0, num_rho - 1)
        accumulator[rho_indices, np.arange(num_theta)] += 1

    while threshold > 20:
        # Find peaks in the accumulator array
        indices = np.argwhere(accumulator == threshold)
        rho_peaks = rhos[indices[:, 0]]
        theta_peaks = thetas[indices[:, 1]]

        # Combine rho and theta values
        lines = np.stack((rho_peaks, np.degrees(theta_peaks)), axis=1)
        if len(lines):
            return np.mean(lines, axis=0), threshold

        threshold -= 1

    return np.array([]), -1


def get_line(data, dir):
    if dir == 'right':
        head = 90
    elif dir == 'left':
        head = 270
    else:
        return np.array([])

    filtered_data = filter_lidar_dir(data, head, 120)
    X = filtered_data[:, 1] * np.cos(np.radians(filtered_data[:, 0]))
    Y = filtered_data[:, 1] * np.sin(np.radians(filtered_data[:, 0]))

    return hough_transform(X, Y)
