import numpy as np
import pybullet as p

class Tether:
    RADIUS = 0.001
    MASS = 0.0001

    def __init__(self, length: float, physics_client: int, num_segments: int = 20) -> None:
        assert isinstance(length, float), "length must be an instance of float"
        assert isinstance(physics_client, int), "physics_client must be an instance of int"
        assert isinstance(num_segments, int), "num_segments must be an instance of int"

        self.physics_client = physics_client
        self.length = length
        self.num_segments = num_segments
        self.segment_length = length / num_segments
        self.segment_mass = self.MASS  # Distribute the mass across the segments
        self.segments = []

        self._parent_frame_pos = np.array([0, 0, -0.5 * self.segment_length], dtype=np.float32)
        self._child_frame_pos = np.array([0, 0, 0.5 * self.segment_length], dtype=np.float32)

        self.weight_prev_angle = None
        self.drone_prev_angle = None
        self.weight_cumulative_angle_change = 0.0
        self.weight_wraps = 0.0
        self.drone_cumulative_angle_change = 0.0
        self.drone_wraps = 0.0
        self.time = 0

    def create_tether(self, top_position: np.ndarray) -> None:
        # Create each segment
        for i in range(self.num_segments):
            segment_top_position = [
                top_position[0],
                top_position[1],
                top_position[2] - i * self.segment_length
            ]
            segment_base_position = [
                segment_top_position[0],
                segment_top_position[1],
                segment_top_position[2] - 0.5 * self.segment_length
            ]

            # Collision and visual shapes
            collisionShapeId = p.createCollisionShape(p.GEOM_CYLINDER, radius=self.RADIUS, height=self.segment_length)
            visualShapeId = p.createVisualShape(p.GEOM_CYLINDER, radius=self.RADIUS,
                                                length=self.segment_length, rgbaColor=[1, 0, 1, 1])

            # Create the segment
            segment_id = p.createMultiBody(baseMass=self.segment_mass,
                                           baseCollisionShapeIndex=collisionShapeId,
                                           baseVisualShapeIndex=visualShapeId,
                                           basePosition=segment_base_position,
                                           baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
            self.segments.append(segment_id)

            p.changeDynamics(segment_id, -1, lateralFriction=1.2)

            # Connect this segment to the previous one (if not the first)
            if i > 0:
                self.create_rotational_joint(
                    parent_body_id=self.segments[i - 1],
                    child_body_id=segment_id,
                    parent_frame_pos=self._parent_frame_pos,
                    child_frame_pos=self._child_frame_pos
                )

    def create_rotational_joint(self, parent_body_id: int, child_body_id: int, parent_frame_pos: np.ndarray,
                                child_frame_pos: np.ndarray) -> None:
        assert isinstance(parent_body_id, int), "parent_body_id must be an instance of int"
        assert isinstance(child_body_id, int), "child_body_id must be an instance of int"
        assert isinstance(parent_frame_pos, np.ndarray), "parent_frame_pos must be an instance of np.ndarray"
        assert isinstance(child_frame_pos, np.ndarray), "child_frame_pos must be an instance of np.ndarray"

        p.createConstraint(parentBodyUniqueId=parent_body_id,
                           parentLinkIndex=-1,
                           childBodyUniqueId=child_body_id,
                           childLinkIndex=-1,
                           jointType=p.JOINT_POINT2POINT,
                           jointAxis=[0, 0, 0],
                           parentFramePosition=parent_frame_pos,
                           childFramePosition=child_frame_pos,
                           parentFrameOrientation=[0, 0, 0, 1],
                           childFrameOrientation=[0, 0, 0, 1])

    def attach_to_drone(self, drone_id: int, drone_bottom_offset: np.ndarray) -> None:
        # Attach the top segment of the tether to the bottom center of the drone
        p.createConstraint(
            parentBodyUniqueId=drone_id,
            parentLinkIndex=-1,
            childBodyUniqueId=self.segments[0],
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=drone_bottom_offset,
            childFramePosition=[0, 0, self.segment_length / 2]
        )
