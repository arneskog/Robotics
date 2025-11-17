from config import *

def car_kinematics(state, L, V_step, omega_s):
    x, y, theta, phi = state

    dx = math.cos(theta) * math.cos(phi) * V_step
    dy = math.sin(theta) * math.cos(phi) * V_step
    dtheta = math.sin(phi) / L * V_step
    dphi = omega_s

    x += dx
    y += dy
    theta += dtheta
    phi += dphi

    return [x, y, theta, phi]