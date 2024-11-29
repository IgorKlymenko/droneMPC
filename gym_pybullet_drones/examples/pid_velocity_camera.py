import os
import time
import argparse
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.VelocityAviary import VelocityAviary
from gym_pybullet_drones.envs.drone_fpv_camera import FPVCameraEnv
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

DEFAULT_DRONE = DroneModel("cf2x")
DEFAULT_GUI = True
DEFAULT_RECORD_VIDEO = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 5
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

def run(
        drone=DEFAULT_DRONE,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VIDEO,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB
        ):
    # Initialize the simulation
    INIT_XYZS = np.array([
        [0, 0, .1],
        [.3, 0, .1],
        [.6, 0, .1],
        [0.9, 0, .1]
    ])
    INIT_RPYS = np.array([
        [0, 0, 0],
        [0, 0, np.pi/3],
        [0, 0, np.pi/4],
        [0, 0, np.pi/2]
    ])

    env = VelocityAviary(
        drone_model=drone,
        num_drones=4,
        initial_xyzs=INIT_XYZS,
        initial_rpys=INIT_RPYS,
        physics=Physics.PYB,
        neighbourhood_radius=10,
        pyb_freq=simulation_freq_hz,
        ctrl_freq=control_freq_hz,
        gui=gui,
        record=record_video,
        obstacles=obstacles,
        user_debug_gui=user_debug_gui
    )

    PYB_CLIENT = env.getPyBulletClient()
    DRONE_IDS = env.getDroneIds()

    # Initialize FPV camera for the first drone
    fpv_camera = FPVCameraEnv(drone_id=DRONE_IDS[0])

    # Compute number of control steps in the simulation
    PERIOD = duration_sec
    NUM_WP = control_freq_hz * PERIOD
    wp_counters = np.array([0 for _ in range(4)])

    # Initialize the velocity target
    TARGET_VEL = np.zeros((4, NUM_WP, 4))
    for i in range(NUM_WP):
        TARGET_VEL[0, i, :] = [-0.5, 1, 0, 0.99] if i < (NUM_WP/8) else [0.5, -1, 0, 0.99]
        TARGET_VEL[1, i, :] = [0, 1, 0, 0.99] if i < (NUM_WP/8+NUM_WP/6) else [0, -1, 0, 0.99]
        TARGET_VEL[2, i, :] = [0.2, 1, 0.2, 0.99] if i < (NUM_WP/8+2*NUM_WP/6) else [-0.2, -1, -0.2, 0.99]
        TARGET_VEL[3, i, :] = [0, 1, 0.5, 0.99] if i < (NUM_WP/8+3*NUM_WP/6) else [0, -1, -0.5, 0.99]

    # Initialize the logger
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=4,
                    output_folder=output_folder,
                    colab=colab)

    # Run the simulation
    action = np.zeros((4, 4))
    start_time = time.time()
    for step in range(0, int(duration_sec * env.CTRL_FREQ)):
        obs, reward, terminated, truncated, info = env.step(action)

        # Compute control for the current waypoint
        for j in range(4):
            action[j, :] = TARGET_VEL[j, wp_counters[j], :]

        # Go to the next waypoint and loop
        for j in range(4):
            wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP - 1) else 0

        # Log the simulation
        for j in range(4):
            logger.log(
                drone=j,
                timestamp=step / env.CTRL_FREQ,
                state=obs[j],
                control=np.hstack([TARGET_VEL[j, wp_counters[j], 0:3], np.zeros(9)])
            )

        # Capture and display FPV view for the first drone
        if step % 10 == 0:  # Update every 10 steps
            rgb_image = fpv_camera.capture_fpv()
            plt.imshow(rgb_image)
            plt.axis("off")
            plt.pause(0.001)

        # Render the simulation
        env.render()

        # Sync the simulation
        if gui:
            sync(step, start_time, env.CTRL_TIMESTEP)

    # Close the environment
    env.close()

    # Plot the simulation results
    logger.save_as_csv("vel")  # Optional CSV save
    if plot:
        logger.plot()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Velocity control example using VelocityAviary')
    parser.add_argument('--drone', default=DEFAULT_DRONE, type=DroneModel, help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--gui', default=DEFAULT_GUI, type=str2bool, help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video', default=DEFAULT_RECORD_VIDEO, type=str2bool, help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot', default=DEFAULT_PLOT, type=str2bool, help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui', default=DEFAULT_USER_DEBUG_GUI, type=str2bool, help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--obstacles', default=DEFAULT_OBSTACLES, type=str2bool, help='Whether to add obstacles to the environment (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ, type=int, help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz', default=DEFAULT_CONTROL_FREQ_HZ, type=int, help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec', default=DEFAULT_DURATION_SEC, type=int, help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder', default=DEFAULT_OUTPUT_FOLDER, type=str, help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab', default=DEFAULT_COLAB, type=bool, help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
