import pybullet as p

class Room:
    def __init__(self, length=10.5, width=5.5, height=6.2, wall_thickness=0.1):
        self.length = length
        self.width = width
        self.height = height
        self.wall_thickness = wall_thickness

    def create_room(self):
        # Create the floor
        floor_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[self.length / 2, self.width / 2, 0.05],
                                                    rgbaColor=[1,1,1, 1])
        floor_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                          halfExtents=[self.length / 2, self.width / 2, 0.05])
        p.createMultiBody(baseMass=0,
                          baseCollisionShapeIndex=floor_collision_shape_id,
                          baseVisualShapeIndex=floor_visual_shape_id,
                          basePosition=[0, 0, -0.05])

        # Create the walls - front
        wall_thickness = self.wall_thickness

        # Wall 1
        wall1_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[self.length / 2, wall_thickness / 2, self.height / 2],
                                                    rgbaColor=[1, 1, 1, 0.1])
        wall1_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                          halfExtents=[self.length / 2, wall_thickness / 2, self.height / 2])
        p.createMultiBody(baseMass=0,
                          baseCollisionShapeIndex=wall1_collision_shape_id,
                          baseVisualShapeIndex=wall1_visual_shape_id,
                          basePosition=[0, -self.width / 2, self.height / 2])

        # Wall 2
        wall2_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[self.length / 2, wall_thickness / 2, self.height / 2],
                                                    rgbaColor=[1, 1, 1, 1])
        wall2_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                          halfExtents=[self.length / 2, wall_thickness / 2, self.height / 2])
        p.createMultiBody(baseMass=0,
                          baseCollisionShapeIndex=wall2_collision_shape_id,
                          baseVisualShapeIndex=wall2_visual_shape_id,
                          basePosition=[0, self.width / 2, self.height / 2])

        # Wall 3 - left
        wall3_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[wall_thickness / 2, self.width / 2, self.height / 2],
                                                    rgbaColor=[1, 1, 1, 1])
        wall3_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                          halfExtents=[wall_thickness / 2, self.width / 2, self.height / 2])
        p.createMultiBody(baseMass=0,
                          baseCollisionShapeIndex=wall3_collision_shape_id,
                          baseVisualShapeIndex=wall3_visual_shape_id,
                          basePosition=[-self.length / 2, 0, self.height / 2])

        # Wall 4 - right
        wall4_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[wall_thickness / 2, self.width / 2, self.height / 2],
                                                    rgbaColor=[1, 1, 1, 1])
        wall4_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                          halfExtents=[wall_thickness / 2, self.width / 2, self.height / 2])
        p.createMultiBody(baseMass=0,
                          baseCollisionShapeIndex=wall4_collision_shape_id,
                          baseVisualShapeIndex=wall4_visual_shape_id,
                          basePosition=[self.length / 2, 0, self.height / 2])

        # Create the ceiling
        ceiling_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                      halfExtents=[self.length / 2, self.width / 2, 0.05],
                                                      rgbaColor=[1, 1, 0, 1])
        ceiling_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                            halfExtents=[self.length / 2, self.width / 2, 0.05])
        p.createMultiBody(baseMass=0,
                          baseCollisionShapeIndex=ceiling_collision_shape_id,
                          baseVisualShapeIndex=ceiling_visual_shape_id,
                          basePosition=[0, 0, self.height + 0.05])
