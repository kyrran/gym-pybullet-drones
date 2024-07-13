import os
import time
import argparse
from datetime import datetime
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
import pandas as pd
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

# Default parameters
DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 12
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False
TRAJECTORY_FILE = 'trajectory_1.csv'  # Original trajectory file
PROCESSED_TRAJECTORY_FILE = 'processed_trajectory_1.csv'  # Smoothed trajectory file

def load_waypoints(filename):
    """Load waypoints from a CSV file."""
    try:
        data = pd.read_csv(filename)
        waypoints = data[['x', 'y', 'z']].values  # Extract x, y, z columns
        return waypoints
    except Exception as e:
        print(f"Error loading waypoints: {e}")
        return None

def process_trajectory(file_path, processed_file_path):
    # Load the CSV file
    trajectory_data = pd.read_csv(file_path)

    # Extract x, y, z columns for processing
    waypoints = trajectory_data[['x', 'y', 'z']].values

    # Define the velocity and acceleration limits
    vlim = np.array([1, 1, 1])  # velocity limits in each axis
    alim = np.array([0.5, 0.5, 0.5])  # acceleration limits in each axis

    # Create path from waypoints
    path = ta.SplineInterpolator(np.linspace(0, 1, len(waypoints)), waypoints)

    # Create velocity and acceleration constraints
    pc_vel = constraint.JointVelocityConstraint(vlim)
    pc_acc = constraint.JointAccelerationConstraint(alim)

    # Setup the parameterization problem
    instance = algo.TOPPRA([pc_vel, pc_acc], path, solver_wrapper='seidel')

    # Compute the trajectory
    jnt_traj = instance.compute_trajectory(0, 0)

    # Sample the trajectory
    N_samples = 1000
    ss = np.linspace(0, jnt_traj.duration, N_samples)
    qs = jnt_traj(ss)

    # Extract the x, y, z components of the trajectory
    x = qs[:, 0]
    y = qs[:, 1]
    z = qs[:, 2]

    # Save the processed waypoints to a new CSV file
    processed_waypoints = pd.DataFrame({
        'x': x,
        'y': y,
        'z': z
    })
    processed_waypoints.to_csv(processed_file_path, index=False)

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
    #### Process the trajectory ################################
    process_trajectory(TRAJECTORY_FILE, PROCESSED_TRAJECTORY_FILE)
    
    #### Load the waypoints from the trajectory file ###########
    TARGET_POS = load_waypoints(PROCESSED_TRAJECTORY_FILE)
    if TARGET_POS is None:
        print("Failed to load waypoints. Exiting.")
        return

    #### Initialize the simulation #############################
    INIT_XYZS = np.array([[0, 0, 0] for _ in range(num_drones)])  # Start at ground level
    INIT_RPYS = np.array([[0, 0, i * (np.pi/2)/num_drones] for i in range(num_drones)])
    
    NUM_WP = TARGET_POS.shape[0]
    wp_counters = np.array([0 for _ in range(num_drones)])  # Ensure wp_counters is initialized

    #### Create the environment ################################
    env = CtrlAviary(drone_model=drone,
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

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone) for _ in range(num_drones)]

    #### Run the simulation ####################################
    action = np.zeros((num_drones, 4))
    START = time.time()
    for i in range(0, int(duration_sec * env.CTRL_FREQ)):

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        for j in range(num_drones):
            wp_index = min(wp_counters[j], NUM_WP - 1)  # Cap wp_index to prevent overflow
            action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                 state=obs[j],
                                                                 target_pos=TARGET_POS[wp_index, :],
                                                                 target_rpy=INIT_RPYS[j, :]
                                                                 )

        #### Go to the next way point ##############################
        wp_counters += 1

        #### Log the simulation ####################################
        for j in range(num_drones):
            wp_index = min(wp_counters[j], NUM_WP - 1)  # Cap wp_index to prevent overflow
            logger.log(drone=j,
                       timestamp=i / env.CTRL_FREQ,
                       state=obs[j],
                       control=np.hstack([TARGET_POS[wp_index, :], INIT_RPYS[j, :], np.zeros(6)])
                       )

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()
    logger.save_as_csv("pid")  # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Flight script using CtrlAviary and DSLPIDControl')
    parser.add_argument('--drone', default=DEFAULT_DRONES, type=DroneModel, help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones', default=DEFAULT_NUM_DRONES, type=int, help='Number of drones (default: 1)', metavar='')
    parser.add_argument('--physics', default=DEFAULT_PHYSICS, type=Physics, help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--gui', default=DEFAULT_GUI, type=str2bool, help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video', default=DEFAULT_RECORD_VISION, type=str2bool, help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot', default=DEFAULT_PLOT, type=str2bool, help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui', default=DEFAULT_USER_DEBUG_GUI, type=str2bool, help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--obstacles', default=DEFAULT_OBSTACLES, type=str2bool, help='Whether to add obstacles to the environment (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ, type=int, help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz', default=DEFAULT_CONTROL_FREQ_HZ, type=int, help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec', default=DEFAULT_DURATION_SEC, type=int, help='Duration of the simulation in seconds (default: 12)', metavar='')
    parser.add_argument('--output_folder', default=DEFAULT_OUTPUT_FOLDER, type=str, help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab', default=DEFAULT_COLAB, type=bool, help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
