import os
import time
import argparse
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.envs.drone_fpv_camera import FPVCameraEnv
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

# Default parameters
DEFAULT_DRONES = DroneModel.CF2X
DEFAULT_NUM_DRONES = 3
DEFAULT_PHYSICS = Physics.PYB
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 12
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

def run(
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
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
    H = .1
    H_STEP = .05
    R = .3
    INIT_XYZS = np.array([[R*np.cos((i/6)*2*np.pi+np.pi/2), R*np.sin((i/6)*2*np.pi+np.pi/2)-R, H+i*H_STEP] for i in range(num_drones)])
    INIT_RPYS = np.array([[0, 0, i * (np.pi/2) / num_drones] for i in range(num_drones)])

    env = CtrlAviary(
        drone_model=drone,
        num_drones=num_drones,
        initial_xyzs=INIT_XYZS,
        initial_rpys=INIT_RPYS,
        physics=physics,
        neighbourhood_radius=10,
        pyb_freq=simulation_freq_hz,
        ctrl_freq=control_freq_hz,
        gui=gui,
        record=record_video,
        obstacles=obstacles,
        user_debug_gui=user_debug_gui
    )
    PYB_CLIENT = env.getPyBulletClient()
    drone_id = env.getDroneIds()[0]  # Use the first drone for FPV camera

    # Initialize FPV camera
    fpv_camera = FPVCameraEnv(drone_id=drone_id)

    # Initialize logger
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab)

    # Initialize PID controllers
    ctrl = [DSLPIDControl(drone_model=drone) for _ in range(num_drones)]

    # Define trajectory
    PERIOD = 10
    NUM_WP = control_freq_hz * PERIOD
    TARGET_POS = np.zeros((NUM_WP, 3))
    for i in range(NUM_WP):
        TARGET_POS[i, :] = R * np.cos((i/NUM_WP)*(2*np.pi)+np.pi/2) + INIT_XYZS[0, 0], \
                           R * np.sin((i/NUM_WP)*(2*np.pi)+np.pi/2) - R + INIT_XYZS[0, 1], 0
    wp_counters = np.array([int((i*NUM_WP/6) % NUM_WP) for i in range(num_drones)])

    # Control variables
    action = np.zeros((num_drones, 4))
    start_time = time.time()

    # Run the simulation
    for step in range(0, int(duration_sec * env.CTRL_FREQ)):
        obs, _, _, _, _ = env.step(action)

        # Compute control for each drone
        for j in range(num_drones):
            action[j, :], _, _ = ctrl[j].computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP,
                state=obs[j],
                target_pos=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2]]),
                target_rpy=INIT_RPYS[j]
            )

        # Increment waypoint counters
        for j in range(num_drones):
            wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP - 1) else 0

        # Log data
        for j in range(num_drones):
            logger.log(drone=j,
                       timestamp=step / env.CTRL_FREQ,
                       state=obs[j],
                       control=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2], INIT_RPYS[j], np.zeros(6)]))

        # Capture and display FPV view for the first drone
        if step % 10 == 0:  # Update every 10 steps
            rgb_image = fpv_camera.capture_fpv()
            plt.imshow(rgb_image)
            plt.axis("off")
            plt.pause(0.001)

        # Render and sync the simulation
        env.render()
        sync(step, start_time, env.CTRL_TIMESTEP)

    # Close environment
    env.close()

    # Save logs
    logger.save_as_csv("pid_with_fpv")  # Optional CSV save
    if plot:
        logger.plot()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='PID control with FPV camera integration')
    parser.add_argument('--drone', default=DEFAULT_DRONES, type=DroneModel, choices=DroneModel, metavar='', help='Drone model (default: CF2X)')
    parser.add_argument('--num_drones', default=DEFAULT_NUM_DRONES, type=int, metavar='', help='Number of drones (default: 3)')
    parser.add_argument('--physics', default=DEFAULT_PHYSICS, type=Physics, choices=Physics, metavar='', help='Physics updates (default: PYB)')
    parser.add_argument('--gui', default=DEFAULT_GUI, type=str2bool, metavar='', help='Whether to use PyBullet GUI (default: True)')
    parser.add_argument('--record_video', default=DEFAULT_RECORD_VISION, type=str2bool, metavar='', help='Whether to record a video (default: False)')
    parser.add_argument('--plot', default=DEFAULT_PLOT, type=str2bool, metavar='', help='Whether to plot the simulation results (default: True)')
    parser.add_argument('--user_debug_gui', default=DEFAULT_USER_DEBUG_GUI, type=str2bool, metavar='', help='Whether to add debug lines to the GUI (default: False)')
    parser.add_argument('--obstacles', default=DEFAULT_OBSTACLES, type=str2bool, metavar='', help='Whether to add obstacles (default: True)')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ, type=int, metavar='', help='Simulation frequency (default: 240 Hz)')
    parser.add_argument('--control_freq_hz', default=DEFAULT_CONTROL_FREQ_HZ, type=int, metavar='', help='Control frequency (default: 48 Hz)')
    parser.add_argument('--duration_sec', default=DEFAULT_DURATION_SEC, type=int, metavar='', help='Duration of the simulation (default: 12 seconds)')
    parser.add_argument('--output_folder', default=DEFAULT_OUTPUT_FOLDER, type=str, metavar='', help='Folder for logs (default: "results")')
    parser.add_argument('--colab', default=DEFAULT_COLAB, type=bool, metavar='', help='Run in Colab (default: False)')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
