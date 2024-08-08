import os
import numpy as np
import pybullet as p
from gymnasium import spaces
from collections import deque

from gym_pybullet_drones.utils.enums import DroneModel, Physics, ActionType, ObservationType, ImageType
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl

from gym_pybullet_drones.envs.HoverAviary import HoverAviary

from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync

from gym_pybullet_drones.envs.simulationComponents.tether import Tether
from gym_pybullet_drones.envs.simulationComponents.weight import Weight
from gym_pybullet_drones.envs.simulationComponents.branch import Branch
from gym_pybullet_drones.envs.simulationComponents.room import Room


class DroneSimulation(HoverAviary):
    """Drone simulation class inheriting from BaseRLAviary."""

    def __init__(self, 
                 start_pos=[0, 0, 0],
                 drone=DroneModel.CF2X, 
                 num_drones=1, 
                 physics=Physics.PYB, 
                 gui=True, 
                 record_video=False, 
                 plot=True, 
                 user_debug_gui=False, 
                 obstacles=False, 
                 simulation_freq_hz=240, 
                 control_freq_hz=48, 
                 duration_sec=30, 
                 output_folder='results', 
                 colab=False, 
                 hover_height=2.0,
                 obs=ObservationType.KIN, 
                 act=ActionType.VEL):
        
        self.drone_pos = start_pos
        
        self.init_xyzs = np.array([self.drone_pos for _ in range(num_drones)])
        self.init_rpys = np.array([[0, 0, i * (np.pi / 2) / num_drones] for i in range(num_drones)])
        self.wp_counters = np.array([0 for _ in range(num_drones)])
        
        
        self.drone = drone
        self.num_drones = num_drones
        self.physics = physics
        self.gui = gui
        self.record_video = record_video
        self.plot = plot
        self.user_debug_gui = user_debug_gui
        self.obstacles = obstacles
        self.simulation_freq_hz = simulation_freq_hz
        self.control_freq_hz = control_freq_hz
        self.duration_sec = duration_sec
        self.output_folder = output_folder
        self.colab = colab
        self.hover_height = hover_height
     
        
        
        # Call the parent class constructor with the necessary parameters
        super().__init__(drone_model=drone,
                         initial_xyzs=self.init_xyzs,
                         initial_rpys=self.init_rpys,
                         physics=physics,
                         pyb_freq=simulation_freq_hz,
                         ctrl_freq=control_freq_hz,
                         gui=gui,
                         record=record_video, 
                         obs=obs,
                         act=act)

        self.logger = Logger(logging_freq_hz=control_freq_hz,
                             num_drones=num_drones,
                             output_folder=output_folder,
                             colab=colab)

        self.ctrl = [DSLPIDControl(drone_model=drone) for _ in range(num_drones)]
        
        # Initialize custom simulation components like tether, branch, etc.
        self.initialize_simulation_components()

    def initialize_simulation_components(self):
        self.branch = Branch()
        self.branch.add_tree_branch(position=[0, 0, 2.7])
        self.branch_body_id = self.branch.get_body_id()  # Store the branch body ID
        
        
        self.tether = Tether(self.DRONE_IDS[0], length=1.0, drone_position=self.init_xyzs[0], physics_client=self.CLIENT)
        self.tether.attach_to_drone(self.DRONE_IDS[0])
        payload_start_position_top = self.tether.get_world_centre_bottom()
        self.weight = Weight(payload_start_position_top)
        self.tether.attach_weight(self.weight)

    def step(self, action: np.ndarray = None):
        assert isinstance(action, (np.ndarray, type(None))), "action must be an instance of np.ndarray"

        obs, reward, terminated, truncated, info = super().step(action)

        has_collided = self.check_collisions()
        dist_tether_branch = self._distance(self.tether.get_mid_point(), self.branch.get_tree_branch_midpoint())
        dist_drone_branch = self._distance(self.drone_pos, self.branch.get_tree_branch_midpoint())
        dist_drone_ground = self.drone_pos[2]
        
        num_rotations = self.tether.compute_total_rotation() if has_collided else 0.0

        return obs, reward, terminated, truncated, info, has_collided, dist_tether_branch, dist_drone_branch, dist_drone_ground, num_rotations

    def check_collisions(self):
        for part_id in self.tether.get_segments():
            contacts = p.getContactPoints(bodyA=self.branch_body_id, bodyB=part_id)
            if contacts:
                return True
        return False

    def reset(self, pos=None):
        if pos is not None:
            self.drone_pos = pos
        self.init_xyzs = np.array([self.drone_pos for _ in range(self.NUM_DRONES)])
        self.wp_counters = np.array([0 for _ in range(self.NUM_DRONES)])
        
        super().reset()
        
        self.initialize_simulation_components()

    def close(self):
        if self.CLIENT is not None:
            p.disconnect(self.CLIENT)
            self.CLIENT = None

    def _distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

