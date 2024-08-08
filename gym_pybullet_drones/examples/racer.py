import os
import time
import argparse
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
import pandas as pd

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool
from process_trajectory import load_waypoints, process_trajectory
from simulation.tether import Tether
from simulation.weight import Weight
from simulation.branch import Branch

import random
from gym_pybullet_drones.control.CTBRControl import CTBRControl

DEFAULT_DRONES = DroneModel("racer")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 30
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False
DEFAULT_HOVER_HEIGHT = 1.1  # Default hover height set to 1 meter
DEFAULT_PROCESSED_TRAJECTORY_FILE = 'processed_trajectory_1.csv'  # Default trajectory file
DEFAULT_TRAJECTORY_FILE = 'trajectory_1.csv'
DEFAULT_TETHER_LENGTH = 1.0
DEFAULT_BRANCH_POSITION = [0,0,2.7]

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
        colab=DEFAULT_COLAB,
        hover_height=DEFAULT_HOVER_HEIGHT,  # Add hover height parameter
        trajectory_file=DEFAULT_TRAJECTORY_FILE,  # Add trajectory file parameter
        processed_trajectory_file=DEFAULT_PROCESSED_TRAJECTORY_FILE  # Add processed trajectory file parameter
        ):
    #### Initialize the simulation #############################
    H = hover_height  # Use the provided hover height
    INIT_XYZS = np.array([[0, 0, 0.0] for _ in range(num_drones)])  # Start at ground level
    INIT_RPYS = np.array([[0, 0, i * (np.pi/2)/num_drones] for i in range(num_drones)])

    #### Load the trajectory from CSV file #####################
    process_trajectory(trajectory_file, processed_trajectory_file, [0, 0, hover_height])
    trajectory_points = load_waypoints(processed_trajectory_file)
    
    #### Initialize the ascent trajectory ######################
    PERIOD = 3
    NUM_WP = control_freq_hz * PERIOD
    ASCENT_DURATION = int(NUM_WP / 4)  # Ascent duration is a quarter of the total period
    TARGET_POS = np.zeros((NUM_WP + len(trajectory_points), 3))

    for i in range(NUM_WP):
        if i < ASCENT_DURATION:
            TARGET_POS[i, :] = [0, 0, (H / ASCENT_DURATION) * i]
        else:
            TARGET_POS[i, :] = [0, 0, H]

    #### Append the trajectory points to the target positions ###
    TARGET_POS[NUM_WP:NUM_WP + len(trajectory_points), :] = trajectory_points

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
        
    if drone in [DroneModel.RACE]:
        ctrl = [DSLPIDControl(drone_model=drone) for _ in range(num_drones)]
        
    #### Add a tree branch to the environment ##################
    branch = Branch()
    branch_position = DEFAULT_BRANCH_POSITION 
    branch.add_tree_branch(position=branch_position)
    
    
    #### Create and attach the tether with payload #############
    tether_length = DEFAULT_TETHER_LENGTH
    tether = Tether(env.DRONE_IDS[0],length=tether_length, physics_client=PYB_CLIENT)
    tether.attach_to_drone(env.DRONE_IDS[0])


    #### Create and attach the weight (payload) ################
    # payload_start_position = tether.get_world_centre_bottom()
    # weight = Weight(INIT_XYZS[0] - tether_length)
    # tether.attach_weight(weight)
    
    
    payload_start_position_top = tether.get_world_centre_bottom()
    print(payload_start_position_top)
    weight = Weight(payload_start_position_top)
    tether.attach_weight(weight)
    print(weight.get_position())
    #### Run the simulation ####################################
    action = np.zeros((num_drones, 4))
    START = time.time()
    for i in range(0, int(duration_sec * env.CTRL_FREQ)):

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        for j in range(num_drones):
            wp_index = min(wp_counters[j], len(TARGET_POS) - 1)  # Ensure wp_index doesn't exceed target positions
            action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                 state=obs[j],
                                                                 target_pos=TARGET_POS[wp_index, :],
                                                                 target_rpy=INIT_RPYS[j, :]
                                                                 )

        #### Go to the next way point ##############################
        wp_counters += 1
        
        if np.all(wp_counters >= len(TARGET_POS)):
            print("reached")
        
        #### Get payload position ##################################
        payload_position = weight.get_position()
        #print(payload_position)
        #### Log the simulation ####################################
        for j in range(num_drones):
            logger.log(drone=j,
                       timestamp=i / env.CTRL_FREQ,
                       state=obs[j],
                       control=np.hstack([TARGET_POS[wp_index, :], INIT_RPYS[j, :], np.zeros(6)]),
                       payload_position=payload_position  # Log the payload position
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
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary and DSLPIDControl')
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
    parser.add_argument('--duration_sec', default=DEFAULT_DURATION_SEC, type=int, help='Duration of the simulation in seconds (default: 30)', metavar='')
    parser.add_argument('--output_folder', default=DEFAULT_OUTPUT_FOLDER, type=str, help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab', default=DEFAULT_COLAB, type=bool, help='Whether example is being run by a notebook (default: "False")', metavar='')
    parser.add_argument('--hover_height', default=DEFAULT_HOVER_HEIGHT, type=float, help='Hover height in meters (default: 1.0)', metavar='')  # Add hover height argument
    parser.add_argument('--trajectory_file', default=DEFAULT_TRAJECTORY_FILE, type=str, help='CSV file with trajectory points (default: "trajectory_1.csv")', metavar='')  # Add trajectory file argument
    parser.add_argument('--processed_trajectory_file', default=DEFAULT_PROCESSED_TRAJECTORY_FILE, type=str, help='CSV file with processed trajectory points (default: "processed_trajectory_1.csv")', metavar='')  # Add processed trajectory file argument
    
    ARGS = parser.parse_args()

    run(**vars(ARGS))
