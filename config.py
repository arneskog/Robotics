import pygame
import math
import sys
import matplotlib.pyplot as plt
import numpy as np

pygame.init()
WIDTH, HEIGHT = 900, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

L = 2.5                
V = 4.0               
SCALE = 25.0           
FPS = 30
INIT_STATE = [0.0, 0.0, 0.0, 0.0]   #[x, y, theta, phi]
INIT_OMEGA_S = 0
DAMPED_OMEGA = 0.1


PHI_MAX = math.radians(35)
PHI_RATE = math.radians(3)


LANE_WIDTH = 3       
AMPLITUDE = 4.0              
FREGUENCY = 0.05


#MPC
K_S = 0.1
K_L = 0.05

CRITICAL_DISTANCE = 0.2

X_MAX = 100
N = 1000

#Sensor
MAX_RANGE = 12.0         
BASE_SIGMA = 0.05        
SIGMA_SLOPE = 0.02       

ENABLE_NOISE = True