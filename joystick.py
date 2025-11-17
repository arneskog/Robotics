from config import *

def joystick_input(omega_s):
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        omega_s = +PHI_RATE
    elif keys[pygame.K_RIGHT]:
        omega_s = -PHI_RATE
    else:
        omega_s = 0.0
    return omega_s