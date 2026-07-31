import math
import matplotlib.pyplot as plt
import pymunk

# -------------------------------------------------------------------
# 1. PHYSICAL CONSTANTS & ENVIRONMENT CONFIGURATION
# -------------------------------------------------------------------
GRAVITY_Y = -9.81          # m/s^2 (Downward gravity)
DT = 0.001                 # Integration timestep (1 ms for high fidelity)
TOTAL_SIM_TIME = 5.0       # Total duration to run simulation (seconds)

SUBJECT_MASS = 75.0        # Mass in kg
INITIAL_HEIGHT = 50.0      # Drop height in meters
TRIGGER_ALTITUDE = 15.0    # Height at which the 45-degree ramp deploys (meters)


class EgressSimulation:
    def __init__(self):
        # Initialize Pymunk Space (Physics World)
        self.space = pymunk.Space()
        self.space.gravity = (0.0, GRAVITY_Y)

        # Create Dynamic Body (Falling Subject)
        # Using a circle shape for basic moment of inertia
        radius = 0.5
        inertia = pymunk.moment_for_circle(SUBJECT_MASS, 0, radius)
        self.body = pymunk.Body(SUBJECT_MASS, inertia)
        self.body.position = (0.0, INITIAL_HEIGHT)

        # Shape definition
        shape = pymunk.Circle(self.body, radius)
        shape.friction = 0.5
        self.space.add(self.body, shape)

        # State Machine Flags
        self.ramp_deployed = False

        # Telemetry Recording Buffers
        self.time_history = []
        self.pos_y_history = []
        self.vel_x_history = []
        self.vel_y_history = []
        self.g_force_history = []

    def run(self):
        time = 0.0
        prev_vel_x, prev_vel_y = self.body.velocity

        while time < TOTAL_SIM_TIME and self.body.position.y > 0:
            pos_x, pos_y = self.body.position
            vel_x, vel_y = self.body.velocity

            # ---------------------------------------------------------------
            # STATE MACHINE & VECTOR REDIRECTION LOGIC
            # ---------------------------------------------------------------
            # 1. Trigger 45-degree Wedge Deployment at Alt threshold
            if pos_y <= TRIGGER_ALTITUDE:
                if not self.ramp_deployed:
                    self.ramp_deployed = True
                    # Lock in target redirected speeds based on entry velocity
                    self.target_vel_x = abs(vel_y) * 0.70
                    self.target_vel_y = vel_y * 0.30

                # SMOOTH VECTOR TRANSITION (Spreads redirection over ~0.25 seconds)
                # Instead of instantly snapping velocity, blend it toward the target
                blend_rate = 0.005  # Controls how soft/hard the redirection curve is

                smoothed_vx = curr_vel_x + (self.target_vel_x - curr_vel_x) * blend_rate
                smoothed_vy = curr_vel_y + (self.target_vel_y - curr_vel_y) * blend_rate

                self.body.velocity = (smoothed_vx, smoothed_vy)

            # 2. Gel Drag Phase (Triggers once redirected horizontally)
            if self.ramp_deployed:
                # Apply non-Newtonian quadratic drag force horizontally
                # F_drag = -0.5 * rho * v^2 * Cd * A
                gel_density = 1050.0   # kg/m^3
                drag_coeff = 1.8
                cross_area = 1.2

                curr_vel_x = self.body.velocity.x
                if curr_vel_x > 0.1:
                    drag_force_x = -0.5 * gel_density * (curr_vel_x ** 2) * drag_coeff * cross_area
                    # Apply central impulse/force to body
                    self.body.apply_force_at_local_point((drag_force_x, 0.0), (0.0, 0.0))

            # ---------------------------------------------------------------
            # INTEGRATION STEP
            # ---------------------------------------------------------------
            self.space.step(DT)
            time += DT

            # ---------------------------------------------------------------
            # TELEMETRY LOGGING
            # ---------------------------------------------------------------
            curr_vel_x, curr_vel_y = self.body.velocity

            # Calculate Instantaneous Acceleration (a = dv / dt)
            acc_x = (curr_vel_x - prev_vel_x) / DT
            acc_y = (curr_vel_y - prev_vel_y) / DT
            total_acc = math.sqrt(acc_x**2 + acc_y**2)

            # Convert total acceleration to G-Force units
            g_force = total_acc / 9.81

            # Log data
            self.time_history.append(time)
            self.pos_y_history.append(pos_y)
            self.vel_x_history.append(curr_vel_x)
            self.vel_y_history.append(curr_vel_y)
            self.g_force_history.append(g_force)

            prev_vel_x, prev_vel_y = curr_vel_x, curr_vel_y

    def plot_telemetry(self):
        """Generates real-time analysis graphs of the drop."""
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig.suptitle('Egress Vector Redirection Telemetry Analysis', fontsize=14, fontweight='bold')

        # Altitude
        axs[0].plot(self.time_history, self.pos_y_history, color='blue', label='Altitude (m)')
        axs[0].axhline(y=TRIGGER_ALTITUDE, color='r', linestyle='--', label='Ramp Deploy Altitude')
        axs[0].set_ylabel('Height (m)')
        axs[0].grid(True)
        axs[0].legend()

        # Velocities
        axs[1].plot(self.time_history, self.vel_y_history, color='purple', label='Vertical Vel (m/s)')
        axs[1].plot(self.time_history, self.vel_x_history, color='orange', label='Horizontal Vel (m/s)')
        axs[1].set_ylabel('Velocity (m/s)')
        axs[1].grid(True)
        axs[1].legend()

        # G-Force Load
        axs[2].plot(self.time_history, self.g_force_history, color='red', label='G-Force Felt')
        axs[2].axhline(y=12.0, color='black', linestyle=':', label='Human Tolerance Threshold (12G)')
        axs[2].set_ylabel('Load (Gs)')
        axs[2].set_xlabel('Time (seconds)')
        axs[2].grid(True)
        axs[2].legend()

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    sim = EgressSimulation()
    print("Running Pymunk Egress Simulation...")
    sim.run()
    print("Simulation complete! Plotting G-Force and Velocity Telemetry...")
    sim.plot_telemetry()