import math
import time
import pybullet as p
import pybullet_data

# -------------------------------------------------------------------
# 1. SIMULATION & PHYSICAL CONSTANTS
# -------------------------------------------------------------------
GRAVITY = -9.81           # m/s^2 along Z-axis
DT = 1.0 / 240.0          # Pybullet default step (240 Hz)

SUBJECT_MASS = 75.0       # kg
INITIAL_HEIGHT = 40.0     # meters (Z-axis)
TRIGGER_ALTITUDE = 15.0   # meters (Z-axis)

# -------------------------------------------------------------------
# 2. INITIALIZE PYBULLET (3D GRAPHICAL WORLD)
# -------------------------------------------------------------------
# Connect with GUI (opens OpenGL interactive window)
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, GRAVITY)

# Load 3D Ground Plane
plane_id = p.loadURDF("plane.urdf")

# Create Falling Subject (Capsule shape to approximate human body)
radius = 0.3
height = 1.8

# Quick swap using GEOM_CYLINDER:
col_shape_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.3, height=1.8)
vis_shape_id = p.createVisualShape(p.GEOM_CYLINDER, radius=0.3, length=1.8, rgbaColor=[0.1, 0.4, 0.8, 1])

# Spawn subject vertically at (X=0, Y=0, Z=INITIAL_HEIGHT)
start_pos = [0, 0, INITIAL_HEIGHT]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
subject_id = p.createMultiBody(
    baseMass=SUBJECT_MASS,
    baseCollisionShapeIndex=col_shape_id,
    baseVisualShapeIndex=vis_shape_id,
    basePosition=start_pos,
    baseOrientation=start_orientation
)

# -------------------------------------------------------------------
# 3. ENVIRONMENT OBJECT: 45-DEGREE REDIRECTION WEDGE
# -------------------------------------------------------------------
# Spawn a physical 3D wedge surface at Z = 10m to physically deflect the body
wedge_col = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[5, 5, 1]
)
wedge_vis = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[5, 5, 1], rgbaColor=[0.8, 0.2, 0.2, 0.8]
)

# Rotate box by 45 degrees to act as a physical ramp
wedge_orientation = p.getQuaternionFromEuler([0, math.radians(45), 0])
wedge_id = p.createMultiBody(
    baseMass=0,  # Static object (fixed in space)
    baseCollisionShapeIndex=wedge_col,
    baseVisualShapeIndex=wedge_vis,
    basePosition=[2, 0, 10],
    baseOrientation=wedge_orientation
)

# Set high-friction surface on the ramp
p.changeDynamics(wedge_id, -1, lateralFriction=0.8)
p.changeDynamics(subject_id, -1, lateralFriction=0.5)

# Camera Setup (Focused on impact zone)
p.resetDebugVisualizerCamera(
    cameraDistance=25,
    cameraYaw=50,
    cameraPitch=-20,
    cameraTargetPosition=[0, 0, 15]
)

# -------------------------------------------------------------------
# 4. EXECUTION LOOP WITH 3D TELEMETRY
# -------------------------------------------------------------------
print("Starting 3D PyBullet Egress Simulation...")
ramp_deployed = False

for step in range(240 * 10):  # Run for 10 seconds of simulated time
    # Get 3D Spatial Telemetry
    pos, orient = p.getBasePositionAndOrientation(subject_id)
    vel_linear, vel_angular = p.getBaseVelocity(subject_id)

    pos_x, pos_y, pos_z = pos
    vx, vy, vz = vel_linear

    # ---------------------------------------------------------------
    # STATE MACHINE & VECTOR TRANSFORMATION
    # ---------------------------------------------------------------
    # Detect altitude threshold on Z-axis
    if pos_z <= TRIGGER_ALTITUDE and not ramp_deployed:
        ramp_deployed = True
        print(f"[TRIGGER] Altitude threshold reached ({pos_z:.2f}m)! Vector thrust & redirection engaging...")

        # Smooth impulse thrust along X-axis to drive body onto the 45-degree ramp
        # Softening the vector transition to keep G-forces under human threshold
        target_vx = abs(vz) * 0.75
        target_vz = vz * 0.25

        p.resetBaseVelocity(subject_id, linearVelocity=[target_vx, vy, target_vz])

    # ---------------------------------------------------------------
    # GEL DRAG PHASE (Horizontal braking along X-axis)
    # ---------------------------------------------------------------
    if ramp_deployed and vx > 0.1:
        # Apply non-Newtonian drag force vector in 3D [-Fx, 0, 0]
        gel_density = 1050.0
        drag_force_x = -0.5 * gel_density * (vx ** 2) * 1.8 * 1.2 * 0.01  # Scaled for 3D step
        p.applyExternalForce(subject_id, -1, [drag_force_x, 0, 0], pos, p.WORLD_FRAME)

    # Advance 3D Physics Integration
    p.stepSimulation()

    # Real-time pacing for viewing (matches 240Hz physics clock)
    time.sleep(DT)

    # Stop simulation when subject lands on the ground
    if pos_z <= 0.8:
        print(f"[LANDING] Target safely settled at X={pos_x:.2f}m, Y={pos_y:.2f}m.")
        break

p.disconnect()
print("3D Simulation Complete.")