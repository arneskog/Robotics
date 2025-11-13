import pygame
import numpy as np

# Initialize pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Car Simulation with Sine Lanes")

# Parameters
L = 20  # Length of the car
V = 5   # Constant forward speed (units per frame)
omega_s = 0  # Initial rotational speed (degrees per frame)

# Car state [x, y, theta, phi]
state = [400, 300, 0, 0]  # Starting at the center, facing along the x-axis

# Function for car kinematics
def car_kinematics(state, L, V, omega_s):
    x, y, theta, phi = state
    dx = np.cos(theta) * np.cos(phi) * V
    dy = np.sin(theta) * np.cos(phi) * V
    dtheta = np.sin(phi) / L * V
    dphi = omega_s

    x += dx
    y += dy
    theta += dtheta
    phi += dphi
    
    return [x, y, theta, phi]

# Function to generate sine waves for the lanes
def draw_sine_lane(amplitude, frequency, phase_shift, color, offset=0, car_pos_x=0):
    # Shift the lane position to follow the car's x position
    x_vals = np.linspace(car_pos_x - WIDTH//2, car_pos_x + WIDTH//2, 1000)
    y_vals = amplitude * np.sin(frequency * x_vals + phase_shift) + offset
    points = [(int(x), int(y)) for x, y in zip(x_vals, y_vals)]
    pygame.draw.lines(screen, color, False, points, 2)

# Main game loop
running = True
clock = pygame.time.Clock()

while running:
    screen.fill((255, 255, 255))  # Clear scree
