# drone_simulation.py
import os
import time
import numpy as np
import pybullet as p
import pandas as pd

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync
from simulation.tether import Tether
from simulation.weight import Weight
from simulation.branch import Branch
from simulation.room import Room

class DroneSimulation:
    def __init__(self, start_pos=[0,0,0],
                 drone=DroneModel("cf2x"), 
                 num_drones=1, 
                 physics=Physics("pyb"), 
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
                 hover_height=2.0):
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
        self.drone_pos = start_pos
        self.init_xyzs = np.array([self.drone_pos for _ in range(self.num_drones)])
        self.init_rpys = np.array([[0, 0, i * (np.pi/2)/self.num_drones] for i in range(self.num_drones)])
        self.wp_counters = np.array([0 for _ in range(self.num_drones)])
        self.env = None
        self.logger = None
        self.ctrl = None
    
        self.client = None
   
        
        # if gui:
        #     self.client = p.connect(p.GUI)
        # else:
        #     self.client = p.connect(p.DIRECT)

    def initialize_environment(self):
        
        self.env = CtrlAviary(drone_model=self.drone,
                              num_drones=self.num_drones,
                              initial_xyzs=self.init_xyzs,
                              initial_rpys=self.init_rpys,
                              physics=self.physics,
                              neighbourhood_radius=10,
                              pyb_freq=self.simulation_freq_hz,
                              ctrl_freq=self.control_freq_hz,
                              gui=self.gui,
                              record=self.record_video,
                              obstacles=self.obstacles,
                              user_debug_gui=self.user_debug_gui,client=self.client)
        
        self.logger = Logger(logging_freq_hz=self.control_freq_hz,
                             num_drones=self.num_drones,
                             output_folder=self.output_folder,
                             colab=self.colab)
        
        if self.drone in [DroneModel.CF2X, DroneModel.CF2P]:
            self.ctrl = [DSLPIDControl(drone_model=self.drone) for _ in range(self.num_drones)]
        # self.room = Room()
        # self.room.create_room()
        self.branch = Branch()
        self.branch.add_tree_branch(position=[0, 0, 2.7])
        
        self.client = self.env.getPyBulletClient()
        
        self.tether = Tether(self.env.DRONE_IDS[0], length=1.0, drone_position=self.init_xyzs[0],physics_client=self.client)
        self.tether.attach_to_drone(self.env.DRONE_IDS[0])
        payload_start_position_top = self.tether.get_world_centre_bottom()
        self.weight = Weight(payload_start_position_top)
        self.tether.attach_weight(self.weight)
        
    def step(self, action: np.ndarray = None) -> None:
        assert isinstance(action, (np.ndarray, type(None))), "action must be an instance of np.ndarray"

        if self.gui:
            time.sleep(0.001)

        if action is not None:
            self.drone_pos += action
            p.resetBasePositionAndOrientation(self.env.DRONE_IDS[0], self.drone_pos, [0, 0, 0, 1])
        
        p.stepSimulation()

        has_collided = self.check_collisions()
        dist_tether_branch = self._distance(self.tether.get_mid_point(), self.branch.get_tree_branch_midpoint())
        dist_drone_branch = self._distance(self.drone_pos, self.branch.get_tree_branch_midpoint())
        dist_drone_ground = self.drone_pos[2]
        
        num_rotations = self.tether.compute_total_rotation() if has_collided else 0.0

        return has_collided, dist_tether_branch, dist_drone_branch, dist_drone_ground, num_rotations

    def check_collisions(self):
        for part_id in self.tether.get_segments():
            contacts = p.getContactPoints(bodyA=self.branch, bodyB=part_id)
            if contacts:
                return True
        return False

    def reset(self, pos: np.ndarray) -> None:
        assert isinstance(pos, np.ndarray), "pos must be an instance of np.ndarray"
        # assert isinstance(pos, np.ndarray), "pos must be an instance of np.ndarray"
        p.resetSimulation()
        p.setGravity(0, 0, -10)
        
        self.drone_pos = pos
        self.init_xyzs = np.array([self.drone_pos for _ in range(self.num_drones)])
        self.wp_counters = np.array([0 for _ in range(self.num_drones)])
        
        self.initialize_environment()

    def close(self):
        if self.client is not None:
            p.disconnect(self.client)
            self.client = None

    def _distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))
