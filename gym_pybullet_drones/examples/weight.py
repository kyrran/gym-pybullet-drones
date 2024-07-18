import pybullet as p
from typing import List
import numpy as np


class Weight:
    MASS: float = 0.000001
    RADIUS: float = 0.03
    DRAG_COEF: float = 0.472

    _body_centre_top = np.array([0, 0, RADIUS], dtype=np.float32)

    def __init__(self, top_position: np.ndarray) -> None:
        assert isinstance(top_position, np.ndarray), "top_position must be an instance of np.ndarray"

        top_x, top_y, top_z = top_position
        self.base_position = [top_x, top_y, top_z - self.RADIUS]
        self.create_weight()
        self.cross_area = 3 * self.RADIUS * self.RADIUS

    def create_weight(self) -> None:
        collisionShapeId = p.createCollisionShape(p.GEOM_SPHERE, radius=self.RADIUS)
        visualShapeId = p.createVisualShape(p.GEOM_SPHERE, radius=self.RADIUS, rgbaColor=[1, 0, 0, 1])

        self.weight_id = p.createMultiBody(baseMass=self.MASS,
                                           baseCollisionShapeIndex=collisionShapeId,
                                           baseVisualShapeIndex=visualShapeId,
                                           basePosition=self.base_position,
                                           baseOrientation=[0, 0, 0, 1])

    def get_position(self) -> List[float]:
        position, _ = p.getBasePositionAndOrientation(self.weight_id)
        return position

    def get_body_centre_top(self) -> np.ndarray:
        return self._body_centre_top