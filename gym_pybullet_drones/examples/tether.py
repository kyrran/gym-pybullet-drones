import pybullet as p
from typing import List, Any
import numpy as np
import random

class Tether:
    RADIUS = 0.005
    MASS = 0.0000001

    def __init__(self, length: float, top_position: np.ndarray, physics_client: int, num_segments: int = 20) -> None:
        assert isinstance(length, float), "length must be an instance of float"
        assert isinstance(top_position, np.ndarray), "top_position must be an instance of np.ndarray"
        assert isinstance(physics_client, int), "physics_client must be an instance of int"
        assert isinstance(num_segments, int), "num_segments must be an instance of int"

        self.physics_client = physics_client
        self.length = length
        self.num_segments = num_segments
        self.segment_length = length / num_segments
        self.top_position = top_position
        self.segment_mass = self.MASS  # Distribute the mass across the segments
        self.segments = []

        self._parent_frame_pos = np.array([0, 0, -0.5 * self.segment_length], dtype=np.float32)
        self._child_frame_pos = np.array([0, 0, 0.5 * self.segment_length], dtype=np.float32)
    
        self._object_len = np.array([0, 0, self.length], dtype=np.float32)

        self.create_tether()

        mid_index = len(self.segments) // 2
        self.mid_indices = [mid_index - 1, mid_index]

        self.weight_prev_angle = None
        self.drone_prev_angle = None
        self.weight_cumulative_angle_change = 0.0
        self.weight_wraps = 0.0
        self.drone_cumulative_angle_change = 0.0
        self.drone_wraps = 0.0
        self.time = 0

        # # Increase the number of solver iterations
        # p.setPhysicsEngineParameter(self.physics_client, numSolverIterations=200)
        # p.setPhysicsEngineParameter(self.physics_client, numSubSteps=4)

    def create_tether(self) -> None:
        # Create each segment
        for i in range(self.num_segments):
            segment_top_position = [
                self.top_position[0],
                self.top_position[1],
                self.top_position[2] - i * self.segment_length
            ]
            segment_base_position = [
                segment_top_position[0],
                segment_top_position[1],
                segment_top_position[2] - 0.5 * self.segment_length
            ]

            # Collision and visual shapes
            collisionShapeId = p.createCollisionShape(p.GEOM_CAPSULE, radius=self.RADIUS, height=self.segment_length)
            visualShapeId = p.createVisualShape(p.GEOM_CAPSULE, radius=self.RADIUS,
                                                length=self.segment_length, rgbaColor=[0, 0, 1, 1])

            # Create the segment
            segment_id = p.createMultiBody(baseMass=self.segment_mass,
                                           baseCollisionShapeIndex=collisionShapeId,
                                           baseVisualShapeIndex=visualShapeId,
                                           basePosition=segment_base_position,
                                           baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
            self.segments.append(segment_id)

            p.changeDynamics(segment_id, -1, lateralFriction=.2, linearDamping=0.0, angularDamping=0.0)

            # Connect this segment to the previous one (if not the first)
            if i > 0:
                self.create_rotational_joint(
                    parent_body_id=self.segments[i - 1],
                    child_body_id=segment_id,
                    parent_frame_pos=self._parent_frame_pos,
                    child_frame_pos=self._child_frame_pos
                )

    def attach_to_drone(self, drone_id: Any, drone_bottom_offset: np.ndarray) -> None:
        assert isinstance(drone_bottom_offset, np.ndarray), "drone_bottom_offset must be an instance of np.ndarray"

        # Convert drone_id to int if it's not already
        drone_id = int(drone_id)

        # Use the create_fixed_joint function to attach the top segment to the drone
        self.create_fixed_joint(
            parent_body_id=drone_id,
            child_body_id=self.segments[0],
            parent_frame_pos=drone_bottom_offset,
            child_frame_pos=[0, 0, self.segment_length / 2]
        )

    def attach_weight(self, weight: Any) -> None:
        # Attach the weight to the bottom segment
        tether_attachment_point = self._parent_frame_pos
        weight_attachment_point = weight.get_body_centre_top()
        self.create_fixed_joint(parent_body_id=self.segments[-1],  # Bottom segment
                                child_body_id=weight.weight_id,
                                parent_frame_pos=tether_attachment_point,
                                child_frame_pos=weight_attachment_point)

    def create_rotational_joint(self, parent_body_id: int, child_body_id: int, parent_frame_pos: np.ndarray,
                                child_frame_pos: np.ndarray) -> None:
        assert isinstance(parent_body_id, int), "parent_body_id must be an instance of int"
        assert isinstance(child_body_id, int), "child_body_id must be an instance of int"
        assert isinstance(parent_frame_pos, np.ndarray), "parent_frame_pos must be an instance of np.ndarray"
        assert isinstance(child_frame_pos, np.ndarray), "child_frame_pos must be an instance of np.ndarray"

        # Use a point-to-point joint to connect the segments
        p.createConstraint(parentBodyUniqueId=parent_body_id,
                           parentLinkIndex=-1,
                           childBodyUniqueId=child_body_id,
                           childLinkIndex=-1,
                           jointType=p.JOINT_POINT2POINT,
                           jointAxis=[0, 0, 0],
                           parentFramePosition=parent_frame_pos.tolist(),
                           childFramePosition=child_frame_pos.tolist())

    def create_fixed_joint(self, parent_body_id: int, child_body_id: int, parent_frame_pos: Any,
                           child_frame_pos: Any) -> None:
        assert isinstance(parent_body_id, int), "parent_body_id must be an instance of int"
        assert isinstance(child_body_id, int), "child_body_id must be an instance of int"
        assert isinstance(parent_frame_pos, (List, np.ndarray)), "wrong type"
        assert isinstance(child_frame_pos, (List, np.ndarray)), "wrong type"

        p.createConstraint(parentBodyUniqueId=parent_body_id,
                           parentLinkIndex=-1,
                           childBodyUniqueId=child_body_id,
                           childLinkIndex=-1,
                           jointType=p.JOINT_FIXED,
                           jointAxis=[0, 0, 0],
                           parentFramePosition=parent_frame_pos.tolist() if isinstance(parent_frame_pos, np.ndarray) else parent_frame_pos,
                           childFramePosition=child_frame_pos.tolist() if isinstance(child_frame_pos, np.ndarray) else child_frame_pos,
                           parentFrameOrientation=[0, 0, 0, 1],
                           childFrameOrientation=[0, 0, 0, 1])

    def compute_total_rotation(self):
        # ANGLE FOR WEIGHT
        (weight_x, _, weight_y), _ = p.getBasePositionAndOrientation(self.segments[-1])
        weight_delta_x = weight_x - 0
        weight_delta_y = 2.7 - weight_y

        # Compute the angle using arctan2, which considers quadrant location
        weight_angle_radians = np.arctan2(weight_delta_x, weight_delta_y)
        weight_angle_degrees = np.degrees(weight_angle_radians)

        if self.weight_prev_angle is not None:
            # Calculate angle change considering the wrap around at 180/-180
            weight_angle_change = weight_angle_degrees - self.weight_prev_angle
            if weight_angle_change > 180:
                weight_angle_change -= 360
            elif weight_angle_change < -180:
                weight_angle_change += 360

            # Update cumulative angle change
            self.weight_cumulative_angle_change += weight_angle_change

            # Update wraps as a float
            self.weight_wraps = self.weight_cumulative_angle_change / 360.0

        # Update the previous angle for the next call
        self.weight_prev_angle = weight_angle_degrees

        # ANGLE FOR DRONE
        (drone_x, _, drone_y), _ = p.getBasePositionAndOrientation(self.segments[0])
        drone_delta_x = drone_x - 0
        drone_delta_y = 2.7 - drone_y

        # Compute the angle using arctan2, which considers quadrant location
        drone_angle_radians = np.arctan2(drone_delta_x, drone_delta_y)
        drone_angle_degrees = np.degrees(drone_angle_radians)

        if self.drone_prev_angle is not None:
            # Calculate angle change considering the wrap around at 180/-180
            drone_angle_change = drone_angle_degrees - self.drone_prev_angle
            if drone_angle_change > 180:
                drone_angle_change -= 360
            elif drone_angle_change < -180:
                drone_angle_change += 360

            # Update cumulative angle change
            self.drone_cumulative_angle_change += drone_angle_change

            # Update wraps as a float
            self.drone_wraps = self.drone_cumulative_angle_change / 360.0

        # Update the previous angle for the next call
        self.drone_prev_angle = drone_angle_degrees

        self.time += 1
        return abs(self.weight_wraps)

    def get_segments(self):
        return self.segments

    def get_mid_point(self):
        positions = [p.getBasePositionAndOrientation(obj_id)[0] for obj_id in
                     (self.segments[i] for i in self.mid_indices)]

        # Calculate the midpoint
        midpoint = [(pos1 + pos2) / 2 for pos1, pos2 in zip(positions[0], positions[1])]
        return midpoint

    def get_world_centre_bottom(self) -> np.ndarray:
    
        # Check the condition and create the weight accordingly
        if np.all(self.top_position - self._object_len) > 0:
            weight_position = np.array(self.top_position - self._object_len)
        else:
            while True:
                    r = random.uniform(0, self.segment_length)
                    theta = random.uniform(0, 2 * np.pi)
                    x = r * np.cos(theta)
                    y = r * np.sin(theta)
                    if x != 0 and y != 0:
                        break
            weight_position = np.array([x, y, 0]) 
        return weight_position