import pybullet as p
import pybullet_data

# Start the physics client
CLIENT = p.connect(p.GUI)  # or p.DIRECT for non-GUI version

# Optionally, set the search path for URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the room URDF
ROOM_ID = p.loadURDF("tether.urdf", physicsClientId=CLIENT)

# Load the plane URDF
PLANE_ID = p.loadURDF("plane.urdf", physicsClientId=CLIENT)

# Set the plane's initial position (inside the room)
initial_plane_position = [5, 2.5, 3]
p.resetBasePositionAndOrientation(PLANE_ID, initial_plane_position, [0, 0, 0, 1])

# Continue with your simulation setup and logic
