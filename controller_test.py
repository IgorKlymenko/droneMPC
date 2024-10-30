# Code to test Logitech F310 controller inputs using pygame

import hid

def get_input():
# Open the device using vendor_id and product_id

    VENDOR_ID = 1133
    PRODUCT_ID = 49686
    gamepad = hid.device()
    gamepad.open(VENDOR_ID, PRODUCT_ID)  # Logitech Dual Action's vendor_id and product_id

    epsilon = 7

    print("Connected to Logitech Dual Action")


    normalized_ang_vel = 127
    # Continuously read input reports
    try:
        while True:
            report = gamepad.read(64)  # Read up to 64 bytes

            if report[1] < 127 - epsilon:
                print("ascend")
                return "throttle_up"

            elif report[0] < 127 - epsilon:
                print("yaw left")
                return "yaw_counter_clockwise"

            elif report[1] > 127 + epsilon:
                print("descend")
                return "throttle_down"

            elif report[0] > 127 + epsilon:
                print("yaw right")
                return "yaw_clockwise"

            if report[3] < 127 - epsilon:
                print("move forward")
                return "pitch_forward"

            elif report[2] < 127 - epsilon:
                print("move left")
                return "roll_left"

            elif report[3] > 127 + epsilon:
                print("move backward")
                return "pitch_backward"

            elif report[2] > 127 + epsilon:
                print("move right")
                return "roll_right"


    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        print("done")
        gamepad.close()

"""



import pygame
import sys

# Initialize pygame
pygame.init()

# Set up the game window (optional)
screen = pygame.display.set_mode((640, 480))

# Initialize the joystick
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    pygame.quit()
    sys.exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Joystick name: {}".format(joystick.get_name()))

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        # Handle joystick events
        if event.type == pygame.JOYAXISMOTION:
            axis_x = joystick.get_axis(0)  # Left stick X-axis
            axis_y = joystick.get_axis(1)  # Left stick Y-axis
            print("Axis X: {:.2f}, Axis Y: {:.2f}".format(axis_x, axis_y))

        if event.type == pygame.JOYBUTTONDOWN:
            print("Button {} pressed.".format(event.button))

        if event.type == pygame.JOYBUTTONUP:
            print("Button {} released.".format(event.button))

# Clean up
pygame.quit()
import pygame
import sys

# Initialize pygame
pygame.init()

# Set up the game window (optional)
screen = pygame.display.set_mode((640, 480))

# Initialize the joystick
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    pygame.quit()
    sys.exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Joystick name: {}".format(joystick.get_name()))

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        # Handle joystick events
        if event.type == pygame.JOYAXISMOTION:
            axis_x = joystick.get_axis(0)  # Left stick X-axis
            axis_y = joystick.get_axis(1)  # Left stick Y-axis
            print("Axis X: {:.2f}, Axis Y: {:.2f}".format(axis_x, axis_y))

        if event.type == pygame.JOYBUTTONDOWN:
            print("Button {} pressed.".format(event.button))

        if event.type == pygame.JOYBUTTONUP:
            print("Button {} released.".format(event.button))

# Clean up
pygame.quit()
"""