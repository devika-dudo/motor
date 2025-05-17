import pygame
import time

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick name: {joystick.get_name()}")
print(f"Number of axes: {joystick.get_numaxes()}")

while True:
    pygame.event.pump()  # Update joystick events
    
    axes = []
    for i in range(joystick.get_numaxes()):
        axis_val = joystick.get_axis(i)
        axes.append(axis_val)
    
    print("Axes:", ["{:.2f}".format(a) for a in axes])
    time.sleep(0.1)
