"""Test PS4 controller input. Press Ctrl+C to quit."""
import pygame
import os
import time

os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['SDL_AUDIODRIVER'] = 'dummy'

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
if count == 0:
    print("No joystick detected. Is the PS4 controller connected via Bluetooth?")
    print("Check: ls /dev/input/js*")
    exit(1)

joy = pygame.joystick.Joystick(0)
joy.init()
print(f"Connected: {joy.get_name()}")
print(f"Axes: {joy.get_numaxes()}  Buttons: {joy.get_numbuttons()}  Hats: {joy.get_numhats()}")
print("Move sticks or press buttons — values will update below. Ctrl+C to quit.\n")

BUTTON_NAMES = {0:'Cross', 1:'Circle', 2:'Square', 3:'Triangle',
                4:'L1', 5:'R1', 6:'L2', 7:'R2',
                8:'Share', 9:'Options', 10:'L3', 11:'R3', 12:'PS'}

try:
    while True:
        pygame.event.pump()

        axes = [round(joy.get_axis(i), 3) for i in range(joy.get_numaxes())]
        buttons = {BUTTON_NAMES.get(i, f'but{i}'): joy.get_button(i)
                   for i in range(joy.get_numbuttons())}
        hat = joy.get_hat(0) if joy.get_numhats() > 0 else (0, 0)

        pressed = [name for name, val in buttons.items() if val]

        print(f"\rL({axes[0]:+.2f},{axes[1]:+.2f}) R({axes[2]:+.2f},{axes[3]:+.2f}) "
              f"L2={axes[4]:+.2f} R2={axes[5]:+.2f} "
              f"Hat={hat} "
              f"Pressed={pressed}          ", end='', flush=True)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nDone.")
    pygame.quit()
