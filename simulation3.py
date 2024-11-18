import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import time

# Simulation parameters
initial_spacing = 0.3
bar_length = 4.0
bar_radius = 0.05
pencil_length = 0.4
pencil_radius = 0.02
gravity = -9.8
drop_height = 10
drop_interval = 3  # Time between each pencil drop (in simulation seconds)
check_time = 3  # Time to check each pencil's position after it was dropped (in simulation seconds)
target_pass_rate = 50  # Target pass rate (%)
tolerance = 1  # Tolerance for convergence to the target pass rate
batch_size = 50  # Number of pencils per batch

# Initialize PyBullet
p.connect(p.DIRECT)  # Use DIRECT mode to avoid GUI overhead
p.setGravity(0, 0, gravity)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
time_step = p.getPhysicsEngineParameters()['fixedTimeStep']
print(f"Time step: {time_step}")

# Function to create polarizer bars
def create_polarizer(spacing, num_bars, z_angle, polarizer_height):
    bars = []
    offset = -(num_bars * spacing) / 2
    for i in range(num_bars):
        x = i * spacing + offset
        collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=bar_radius, height=bar_length)
        orientation = p.getQuaternionFromEuler([np.pi / 2, z_angle, 0])
        bar_body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            basePosition=[x, 0, polarizer_height],
            baseOrientation=orientation
        )
        bars.append(bar_body)
    return bars

# Function to create a falling pencil
def create_pencil(x, y, angle):
    collision_shape = p.createCollisionShape(p.GEOM_CAPSULE, radius=pencil_radius, height=pencil_length)
    mass = 1.0
    orientation = p.getQuaternionFromEuler([np.pi / 2, 0, angle])
    body = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape,
        basePosition=[x, y, drop_height],
        baseOrientation=orientation
    )
    p.setCollisionFilterGroupMask(body, -1, 1, 0)
    return body

def has_reached_ground(pencil_id):
    position, _ = p.getBasePositionAndOrientation(pencil_id)
    return position[2] <= 0.1

def run_simulation(spacing, num_pencils):
    successful_passes = 0
    total_pencils = 0
    dropped_pencils = []

    # Create polarizer bars
    p.resetSimulation()
    create_polarizer(spacing, 20, np.pi / 2, 5)

    pencil_count = 0
    last_drop_time = 0
    simulation_steps = 0

    while pencil_count < num_pencils or len(dropped_pencils) > 0:
        current_simulation_time = simulation_steps * time_step

        # Drop a new pencil every interval
        if pencil_count < num_pencils and current_simulation_time - last_drop_time >= drop_interval:
            x = np.random.uniform(-1, 1)
            y = np.random.uniform(-1, 1)
            angle = np.random.uniform(0, np.pi)
            pencil = create_pencil(x, y, angle)
            dropped_pencils.append((pencil, current_simulation_time))
            pencil_count += 1
            last_drop_time = current_simulation_time

        # Step the simulation
        p.stepSimulation()
        simulation_steps += 1

        # Check pencils that were dropped check_time ago
        for pencil, drop_time in dropped_pencils[:]:
            if current_simulation_time - drop_time >= check_time:
                total_pencils += 1
                if has_reached_ground(pencil):
                    successful_passes += 1
                dropped_pencils.remove((pencil, drop_time))
                p.removeBody(pencil)

    pass_rate = (successful_passes / total_pencils) * 100 if total_pencils > 0 else 0
    return pass_rate

def find_optimal_spacing():
    # Step 1: Find the maximum pencil batch size that results in a ~1% pass rate
    spacing = initial_spacing
    while True:
        pass_rate = run_simulation(spacing, batch_size)
        print(f"Testing spacing {spacing:.4f}: Pass rate = {pass_rate:.2f}%")
        if pass_rate <= 1:
            break
        spacing -= 0.01  # Decrease spacing to lower pass rate

    print(f"Found baseline spacing for ~1% pass rate: {spacing:.4f}\n")

    # Step 2: Adjust the spacing to converge to a 50% pass rate
    low = spacing
    high = spacing + 0.1  # Start with a small range
    while True:
        mid_spacing = (low + high) / 2
        pass_rate = run_simulation(mid_spacing, batch_size)
        print(f"Adjusting spacing {mid_spacing:.4f}: Pass rate = {pass_rate:.2f}%")

        if abs(pass_rate - target_pass_rate) <= tolerance:
            print(f"Optimal spacing found: {mid_spacing:.4f} with pass rate {pass_rate:.2f}%")
            break

        if pass_rate > target_pass_rate:
            high = mid_spacing
        else:
            low = mid_spacing

find_optimal_spacing()

# Clean up
p.disconnect()
