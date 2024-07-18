import numpy as np
import pandas as pd
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

def load_waypoints(filename):
    """Load waypoints from a CSV file."""
    try:
        data = pd.read_csv(filename)
        waypoints = data[['x', 'y', 'z']].values  # Extract x, y, z columns
        return waypoints
    except Exception as e:
        print(f"Error loading waypoints: {e}")
        return None

def process_trajectory(file_path, processed_file_path, initial_hover_point):
    
    # Extract x, y, z columns for processing
    waypoints = load_waypoints(file_path)
    
    initial_hover_point = np.array([initial_hover_point])
    waypoints = np.vstack((initial_hover_point, waypoints))

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
    N_samples = 200
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
