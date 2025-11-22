import pygame
import math
import sys
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline


#Pygame window
pygame.init()
WIDTH, HEIGHT = 900, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
SCALE = 25.0           
FPS = 20

#Sin road
LANE_WIDTH = 3       
AMPLITUDE = 4             
FREGUENCY = 0.05

#Vehicle 
L = 2.5                
V = 4.0
INIT_STATE = [0.0, 0.0, 0.0, 0.0]   #[x, y, theta, phi]
INIT_OMEGA_S = 0
PHI_MAX = math.radians(35)
PHI_RATE = math.radians(3)

#MPC
CONTROLLER_ACTIVE = True
CRITICAL_DISTANCE = 0.2
LOOKAHEAD_DISTANCE = 6
SAMPLING_INTERVAL = 0.5

#Sensor
ENABLE_NOISE = True
MAX_RANGE = 8.0         
BASE_SIGMA = 0.05        
SIGMA_SLOPE = 0.02       
