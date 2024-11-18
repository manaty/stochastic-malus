import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import time

# Simulation parameters
num_pencils = 100
spacing_between_bars = 0.2
bar_length = 4.0
bar_radius = 0.05
pencil_length = 0.4
pencil_radius = 0.02
gravity = -9.8
drop_height = 10
polarizer_height = 5
drop_interval = 0.1  # Time between each pencil drop (in simulation seconds)
check_time = 3  # Time to check each pencil's position after it was dropped (in simulation seconds)

# Initialize PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, gravity)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(cameraDistance=6, cameraYaw=-45, cameraPitch=-20, cameraTargetPosition=[0, 0, 5])

# Get the time step for each simulation step
time_step = p.getPhysicsEngineParameters()['fixedTimeStep']

# Function to create polarizer bars in the (x, y) plane
def create_polarizer(spacing, num_bars, z_angle):
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

# Function to create a falling pencil with no collisions between pencils
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
    # Disable collisions between pencils
    p.setCollisionFilterGroupMask(body, -1, 1, 0)
    return body

# Function to check if a pencil has reached the z = 0 plane
def has_reached_ground(pencil_id):
    position, _ = p.getBasePositionAndOrientation(pencil_id)
    return position[2] <= 0.1

# Function to draw a graph of the percentage of pencils that passed
def draw_pass_rate_graph(success_rates):
    plt.figure(figsize=(3, 2))
    plt.plot(success_rates, marker='o', color='blue')
    plt.title('Pencils Passing Rate')
    plt.xlabel('Trial')
    plt.ylabel('% Passed')
    plt.ylim(0, 100)
    plt.grid(True)
    plt.savefig('pass_rate.png')
    plt.close()

# Function to update the pass rate display
def update_pass_rate_display(successful_passes, total_pencils, success_rates):
    pass_rate = (successful_passes / total_pencils) * 100 if total_pencils > 0 else 0
    success_rates.append(pass_rate)
    p.addUserDebugText(
        f'Pass Rate: {pass_rate:.2f}%', 
        [2, 2, 3], 
        textSize=2, 
        textColorRGB=[0, 0, 1], 
        replaceItemUniqueId=1
    )
    draw_pass_rate_graph(success_rates)

# Create polarizer bars
num_bars = 20
create_polarizer(spacing_between_bars, num_bars, np.pi / 2)

# Simulation loop variables
successful_passes = 0
total_pencils = 0
success_rates = []
dropped_pencils = []

pencil_count = 0
last_drop_time = 0
simulation_steps = 0

# Run the simulation
while pencil_count < num_pencils or len(dropped_pencils) > 0:
    current_simulation_time = simulation_steps * time_step

    # Drop a new pencil every 0.1 seconds of simulation time
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

    # Check pencils that were dropped 10 seconds ago
    for pencil, drop_time in dropped_pencils[:]:
        if current_simulation_time - drop_time >= check_time:
            total_pencils += 1
            if has_reached_ground(pencil):
                successful_passes += 1
            update_pass_rate_display(successful_passes, total_pencils, success_rates)
            dropped_pencils.remove((pencil, drop_time))
            p.removeBody(pencil)

# Print the final results
print(f"\nTotal pencils that passed through: {successful_passes}/{num_pencils}")

# Clean up
p.disconnect()
