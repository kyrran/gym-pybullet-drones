import pybullet as p
import pybullet_data
import os

class Room:
    def __init__(self, length=10.5, width=5.5, height=6.2, wall_thickness=0.1):
        self.length = length
        self.width = width
        self.height = height
        self.wall_thickness = wall_thickness

    def create_room(self, floor_texture_path=None, wall_texture_path="/home/kangle/Documents/FYP/gym-pybullet-drones/gym_pybullet_drones/examples/simulation/wall-texture-with-white-spots.jpg", ceiling_texture_path="/home/kangle/Documents/FYP/gym-pybullet-drones/gym_pybullet_drones/examples/simulation/grey.png"):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load textures if provided
        floor_texture_id = p.loadTexture(floor_texture_path) if floor_texture_path else None
        wall_texture_id = p.loadTexture(wall_texture_path) if wall_texture_path else None
        ceiling_texture_id = p.loadTexture(ceiling_texture_path) if ceiling_texture_path else None

        # Create the floor
        floor_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[self.length / 2, self.width / 2, 0.05],
                                                    rgbaColor=[1, 1, 1, 1])
        floor_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                          halfExtents=[self.length / 2, self.width / 2, 0.05])
        floor_id = p.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=floor_collision_shape_id,
                                     baseVisualShapeIndex=floor_visual_shape_id,
                                     basePosition=[0, 0, -0.05])
        
        # Apply texture to the floor if provided
        if floor_texture_id:
            p.changeVisualShape(floor_id, -1, textureUniqueId=floor_texture_id)

        # Create the walls
        wall_thickness = self.wall_thickness

        # Wall 1
        wall1_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[self.length / 2, wall_thickness / 2, self.height / 2],
                                                    rgbaColor=[1, 1, 1, 0.1])
        wall1_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                          halfExtents=[self.length / 2, wall_thickness / 2, self.height / 2])
        wall1_id = p.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=wall1_collision_shape_id,
                                     baseVisualShapeIndex=wall1_visual_shape_id,
                                     basePosition=[0, -self.width / 2, self.height / 2])

        # Apply texture to Wall 1 if provided
        if wall_texture_id:
            p.changeVisualShape(wall1_id, -1, textureUniqueId=wall_texture_id)

        # Wall 2
        wall2_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[self.length / 2, wall_thickness / 2, self.height / 2],
                                                    rgbaColor=[1, 1, 1, 0.9])
        wall2_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                          halfExtents=[self.length / 2, wall_thickness / 2, self.height / 2])
        wall2_id = p.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=wall2_collision_shape_id,
                                     baseVisualShapeIndex=wall2_visual_shape_id,
                                     basePosition=[0, self.width / 2, self.height / 2])

        # Apply texture to Wall 2 if provided
        if wall_texture_id:
            p.changeVisualShape(wall2_id, -1, textureUniqueId=wall_texture_id)

        # Wall 3
        wall3_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[wall_thickness / 2, self.width / 2, self.height / 2],
                                                    rgbaColor=[1, 1, 1, 1])
        wall3_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                          halfExtents=[wall_thickness / 2, self.width / 2, self.height / 2])
        wall3_id = p.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=wall3_collision_shape_id,
                                     baseVisualShapeIndex=wall3_visual_shape_id,
                                     basePosition=[-self.length / 2, 0, self.height / 2])

        # Apply texture to Wall 3 if provided
        if wall_texture_id:
            p.changeVisualShape(wall3_id, -1, textureUniqueId=wall_texture_id)

        # Wall 4
        wall4_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                    halfExtents=[wall_thickness / 2, self.width / 2, self.height / 2],
                                                    rgbaColor=[1, 1, 1, 1])
        wall4_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                          halfExtents=[wall_thickness / 2, self.width / 2, self.height / 2])
        wall4_id = p.createMultiBody(baseMass=0,
                                     baseCollisionShapeIndex=wall4_collision_shape_id,
                                     baseVisualShapeIndex=wall4_visual_shape_id,
                                     basePosition=[self.length / 2, 0, self.height / 2])

        # Apply texture to Wall 4 if provided
        if wall_texture_id:
            p.changeVisualShape(wall4_id, -1, textureUniqueId=wall_texture_id)

        # Create the ceiling
        ceiling_visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                                      halfExtents=[self.length / 2, self.width / 2, 0.05],
                                                      rgbaColor=[1, 1, 1, 1])
        ceiling_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                            halfExtents=[self.length / 2, self.width / 2, 0.05])
        ceiling_id = p.createMultiBody(baseMass=0,
                                       baseCollisionShapeIndex=ceiling_collision_shape_id,
                                       baseVisualShapeIndex=ceiling_visual_shape_id,
                                       basePosition=[0, 0, self.height + 0.05])

        # Apply texture to the ceiling if provided
        if ceiling_texture_id:
            p.changeVisualShape(ceiling_id, -1, textureUniqueId=ceiling_texture_id)

        # # Add lighting
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)  # Disable rendering during setup
        # p.addUserDebugLine([0, 0, self.height], [0, 0, self.height + 1], [1, 1, 1], 2)  # Light source visualization
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)  # Enable rendering after setup

