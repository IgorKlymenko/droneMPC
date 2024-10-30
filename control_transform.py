import hid

VENDOR_ID = 1133
PRODUCT_ID = 49686

# Open the device using vendor_id and product_id
gamepad = hid.device()
gamepad.open(VENDOR_ID, PRODUCT_ID)  # Logitech Dual Action's vendor_id and product_id

epsilon = 7

print("Connected to Logitech Dual Action")

# Continuously read input reports
try:
    while True:
        report = gamepad.read(64)  # Read up to 64 bytes

        # Throttle, yaw, pitch, and roll calculations
        throttle = (report[1] - 127) / 127  # Normalized throttle (-1 to +1)
        yaw_rate = (report[0] - 127) / 127  # Normalized yaw rate (-1 to +1)
        pitch = (report[3] - 127) / 127      # Normalized pitch (-1 to +1)
        roll = (report[2] - 127) / 127       # Normalized roll (-1 to +1)

        # Calculate angular velocities for each rotor
        angular_velocity_1 = throttle + (yaw_rate / 2) + (pitch / 2) - (roll / 2)  # Rotor 1 (Top Right - CW)
        angular_velocity_2 = throttle - (yaw_rate / 2) + (pitch / 2) + (roll / 2)  # Rotor 2 (Top Left - CCW)
        angular_velocity_3 = throttle + (yaw_rate / 2) - (pitch / 2) + (roll / 2)  # Rotor 3 (Bottom Left - CW)
        angular_velocity_4 = throttle - (yaw_rate / 2) - (pitch / 2) - (roll / 2)  # Rotor 4 (Bottom Right - CCW)

        # Display angular velocities starting from Rotor 1 (top right)
        print(f"Angular Velocity for Rotor 1 (Top Right): {angular_velocity_1:.2f}")
        print(f"Angular Velocity for Rotor 2 (Top Left): {angular_velocity_2:.2f}")
        print(f"Angular Velocity for Rotor 3 (Bottom Left): {angular_velocity_3:.2f}")
        print(f"Angular Velocity for Rotor 4 (Bottom Right): {angular_velocity_4:.2f}")
        print("---")

except KeyboardInterrupt:
    print("Exiting...")
finally:
    gamepad.close()
