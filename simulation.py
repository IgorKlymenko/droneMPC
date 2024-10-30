import pybullet as p
import pybullet_data
import time
import json
import os
import sys

# Initialize PyBullet


# Initialize PyBullet in DIRECT mode for testing

p.connect(p.GUI)  # Start PyBullet in GUI mode


# Load plane and drone

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")  # Ground plane for reference
drone_body = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.02])
drone = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=drone_body, basePosition=[0, 0, 1])

# Initial state and orientation
state = [0, 0, 1, 0, 0, 0] 
previous_state = state.copy() # [x, y, z, roll, pitch, yaw]
velocity = [0] * 12    # Initial velocity vector

def update_drone_state(state, velocity):
    """
    Updates the drone's position and orientation based on input state and velocity vectors.

    Parameters:
    - state: List of [x, y, z, roll, pitch, yaw] representing the current position and orientation.
    - velocity: 12-element list where the first 6 values update position and orientation velocities.
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
    # Determine the full path to velocity.json based on the script's location
    filename = os.path.join(os.path.dirname(__file__), "velocity.json")
    try:
        #print(f"Attempting to load {filename}...")  # Debug print
        with open(filename, "r") as f:
            data = json.load(f)
        #print("File loaded successfully.")  # Debug print
        return data["velocities"]
    except (FileNotFoundError, KeyError, json.JSONDecodeError) as e:
        #print("Error loading velocities from file:", e)
        return {}

    
def save_velocity_to_file(velocity_name, velocity_vector):
    """
    Save a new velocity configuration to velocity.json.
    """
    filename = os.path.join(os.path.dirname(__file__), "velocity.json")
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
        print("3. Reset to previous state")
        print("4. Exit simulation")
        choice = input("Enter your choice (1/2/3/4): ")

        if choice == "1":
            velocity_options = load_velocities_from_file()
            if not velocity_options:
                print("No velocities available. Add a new velocity first.")
                continue
            
            print("Available velocities:", list(velocity_options.keys()))
            selected_velocity_name = input("Enter the name of the velocity to use: ")
            if selected_velocity_name in velocity_options:
                return velocity_options[selected_velocity_name]
            else:
                print("Invalid selection. Try again.")
        
        elif choice == "2":
            velocity_name = input("Enter a name for the new velocity: ")
            velocity_vector = input("Enter 12 values for the velocity vector, separated by spaces: ")
            velocity_vector = [float(v) for v in velocity_vector.split()]
            if len(velocity_vector) == 12:
                save_velocity_to_file(velocity_name, velocity_vector)
            else:
                print("Invalid input. Please enter exactly 12 values.")
        
        elif choice == "3":
            # Reset to the previous state
            print("Resetting to previous state:", previous_state)
            global state
            state = previous_state.copy()  # Restore the previous state
            return [0] * 12  # Set velocity to zero to stop movement after reset

        elif choice == "4":
            print("Exiting simulation.")
            exit(0)  # Exit the program

        else:
            print("Invalid choice. Please enter 1, 2, 3, or 4.")

# Main loop for continuous simulation at 30 FPS
fps = 30
velocity = user_menu()  # Get the initial velocity from user selection


# Set interval for re-opening the menu (e.g., every 10 seconds)
menu_interval = 4  # seconds
last_menu_time = time.time()

print("Starting simulation with velocity:", velocity)  # Confirm velocity selection

while True:
    # Update the drone's state with the selected velocity vector
    update_drone_state(state, velocity)
    p.stepSimulation()
    time.sleep(1.0 / fps)

    # Check if it's time to reopen the menu
    if time.time() - last_menu_time > menu_interval:
        
        velocity = user_menu()  # Reopen the menu
        last_menu_time = time.time()  # Reset the timer
