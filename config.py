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
V = 10.0               
SCALE = 25.0           
FPS = 60
INIT_STATE = [0.0, 0.0, 0.0, 0.0]   #[x, y, theta, phi]
INIT_OMEGA_S = 0


PHI_MAX = math.radians(35)
PHI_RATE = math.radians(3)


LANE_WIDTH = 6       
AMPLITUDE = 4.0              
FREGUENCY = 0.05