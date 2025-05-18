import pygame
import serial
import time

# Setup
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"[INFO] Joystick detected: {joystick.get_name()}")

# Try to connect to Arduino
try:
    ser = serial.Serial('/dev/ttyACM0', 9600)  # Change port if needed
    time.sleep(2)  # Allow Arduino to reset
    print("[INFO] Connected to Arduino.")
except serial.SerialException:
    print("[ERROR] Could not connect to Arduino on /dev/ttyACM0.")
    ser = None

# Button indices
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3

# Mapping from button to mode index
button_to_mode = {
    BUTTON_A: 0,
    BUTTON_X: 1,
    BUTTON_Y: 2
}

current_mode = 0

def apply_deadzone(value, threshold=0.1):
    return 0.0 if abs(value) < threshold else value

while True:
    pygame.event.pump()

    # Check which button is pressed to switch modes
    for button, mode in button_to_mode.items():
        if joystick.get_button(button):
            current_mode = mode
            print(f"[INFO] Switched to mode {current_mode} (controlling joints {current_mode*2} and {current_mode*2 + 1})")

    joint_values = [0.0] * 6

    # Get left and right stick Y-axis
    left = apply_deadzone(joystick.get_axis(1))  # Left stick Y
    right = apply_deadzone(joystick.get_axis(4)) # Right stick Y

    # Assign to appropriate joint indices based on mode
    joint_values[current_mode * 2] = left
    joint_values[current_mode * 2 + 1] = right

    # Convert to string and send
    msg = " ".join(f"{val:.2f}" for val in joint_values) + "\n"
    
    # Print feedback
    print(f"[SEND] {msg.strip()}")

    # Send only if serial is connected
    if ser:
        ser.write(msg.encode())

    time.sleep(0.05)

