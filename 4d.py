import pybullet as p
import pybullet_data
import time
import json
import os

import controller_test

# Initialize PyBullet in GUI mode
p.connect(p.GUI)

# Load plane and drone
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
drone_body = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.02])
drone = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=drone_body, basePosition=[0, 0, 1])

# Initial state and orientation
initial_state = [0, 0, 1, 0, 0, 0]  # Set the original position and orientation
state = initial_state.copy()  # Current state
previous_state = state.copy()  # Store the previous state
velocity = [0] * 12  # Initial velocity vector
THRUST_SCALING = 0.1  # Scale down thrust to reduce rapid upward movement

def map_4d_to_12d(input_vector):
    """
    Map 4D input (pitch, roll, yaw, thrust) to a 12D velocity output.
    """
    thrust = input_vector[3] * THRUST_SCALING
    return [
        input_vector[0], input_vector[1], thrust,  # x, y, z velocity (scaled thrust)
        input_vector[0] * 0.1, input_vector[1] * 0.1, input_vector[2] * 0.1,  # Orientation roll, pitch, yaw velocities
        0, 0, 0,  # Placeholder for linear accelerations
        0, 0, 0   # Placeholder for angular accelerations
    ]

def update_drone_state(state, velocity):
    """
    Updates the drone's position and orientation based on input state and velocity vectors.
    """
    global previous_state
    previous_state = state.copy()  # Save the current state as the previous state

    # Update position (x, y, z) based on velocity
    position = [state[i] + velocity[i] for i in range(3)]
    orientation = [state[i + 3] + velocity[i + 3] for i in range(3)]
    quaternion = p.getQuaternionFromEuler(orientation)
    p.resetBasePositionAndOrientation(drone, position, quaternion)

    # Update state to reflect the new position and orientation
    state[:3] = position
    state[3:] = orientation

def load_velocities_from_file():
    filename = os.path.join(os.path.dirname(__file__), "4d_velocity.json")
    try:
        with open(filename, "r") as f:
            data = json.load(f)
        return data["velocities_update"] # was just velocities here
    except (FileNotFoundError, KeyError, json.JSONDecodeError) as e:
        print("Error loading velocities from file:", e)
        return {}

def save_velocity_to_file(velocity_name, velocity_vector):
    filename = os.path.join(os.path.dirname(__file__), "4d_velocity.json")
    try:
        with open(filename, "r") as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        data = {"velocities": {}}
    
    # Add the new velocity and save
    data["velocities"][velocity_name] = velocity_vector
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)
    print(f"Velocity '{velocity_name}' saved successfully.")

def user_menu():
    """
    Display a menu for user to select, add, reset to previous state, or exit.
    """
    while True:
        print("\nMenu:")
        print("1. Select an existing velocity")
        print("2. Add a new velocity")
        print("3. Reset to original position")
        print("4. Exit simulation")
        choice = input("Enter your choice (1/2/3/4): ")

        if choice == "1":
            velocity_options = load_velocities_from_file()
            if not velocity_options:
                print("No velocities available. Add a new velocity first.")
                continue
            
            #print("Available velocities:", list(velocity_options.keys()))
            #selected_velocity_name = input("Enter the name of the velocity to use: ")
            selected_velocity_name = "hover"
            selected_velocity_name = controller_test.get_input()

            print(selected_velocity_name)
            if selected_velocity_name in velocity_options:
                return map_4d_to_12d(velocity_options[selected_velocity_name])
            else:
                print("Invalid selection. Try again.")
        
        elif choice == "2":
            velocity_name = input("Enter a name for the new velocity: ")
            velocity_vector = input("Enter 4 values for the velocity vector (pitch, roll, yaw, thrust), separated by spaces: ")
            velocity_vector = [float(v) for v in velocity_vector.split()]
            if len(velocity_vector) == 4:
                save_velocity_to_file(velocity_name, velocity_vector)
            else:
                print("Invalid input. Please enter exactly 4 values.")
        
        elif choice == "3":
            # Reset to the original position and orientation
            print("Resetting to original position:", initial_state)
            global state
            state = initial_state.copy()  # Reset to the initial position and orientation
            p.resetBasePositionAndOrientation(drone, initial_state[:3], p.getQuaternionFromEuler(initial_state[3:]))
            return [0] * 12  # Set velocity to zero to stop movement after reset

        elif choice == "4":
            print("Exiting simulation.")
            exit(0)  # Exit the program

        else:
            print("Invalid choice. Please enter 1, 2, 3, or 4.")


def controller():
    velocity_options = load_velocities_from_file()
    
    #print("Available velocities:", list(velocity_options.keys()))
    #selected_velocity_name = input("Enter the name of the velocity to use: ")
    selected_velocity_name = "hover"
    selected_velocity_name = controller_test.get_input()

    print(selected_velocity_name)
    if selected_velocity_name in velocity_options:
        return map_4d_to_12d(velocity_options[selected_velocity_name])
    else:
        print("Invalid selection. Try again.")

# Main loop for continuous simulation at 30 FPS
fps = 30
velocity = controller()  # Get the initial velocity from user selection

menu_interval = 0.1  # seconds
last_menu_time = time.time()

print("Starting simulation with velocity:", velocity)  # Confirm velocity selection

while True:
    update_drone_state(state, velocity)
    p.stepSimulation()
    time.sleep(1.0 / fps)

    # Check if it's time to reopen the menu
    if time.time() - last_menu_time > menu_interval:
        velocity = controller()
        last_menu_time = time.time()
