import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

class PersonFollowerSimulation:
    def __init__(self):
        self.person_position = [0.0, 5.0]  # Initial position of the person
        self.robot_orientation = 0.0  # Initial orientation of the robot (theta)
        self.previous_offset = 0.0
        self.kp_omega = 1.0
        self.navigation_constant = 1.0
        self.lambda_gain = 0.5
        self.kd_lambda = 0.2
        self.a = 2.0
        self.dt = 0.1  # Time step

    def update_person_position(self, t):
        """ Simulates the person moving left and right in a sinusoidal pattern."""
        self.person_position[1] = 5.0 * np.sin(0.2 * t)

    def calculate_offset(self):
        """Calculate the lateral offset between robot and person."""
        return self.person_position[1]

    def pn(self, offset):
        omega = self.navigation_constant * self.kp_omega * offset
        return omega

    def mpn(self, offset):
        offset_rate = (offset - self.previous_offset) / self.dt
        omega = self.navigation_constant * self.kp_omega * offset + self.lambda_gain * offset_rate
        return omega

    def new_mpn(self, offset):
        offset_rate = (offset - self.previous_offset) / self.dt
        dynamic_kd = self.kd_lambda * (1 - np.exp(-self.a * abs(offset_rate))) / (1 + np.exp(-self.a * abs(offset_rate)))
        omega = self.navigation_constant * self.kp_omega * offset + dynamic_kd * offset_rate
        return omega

    def update_robot_orientation(self, omega):
        self.robot_orientation += omega * self.dt
        self.robot_orientation = np.mod(self.robot_orientation, 2 * np.pi)  # Keep within 0 to 2*pi

    def reset_robot_orientation(self):
        self.robot_orientation = 0.0
        self.previous_offset = 0.0

# Set up the simulation
simulation = PersonFollowerSimulation()
modes = ["pn", "mpn", "new_mpn"]
labels = ["PN", "MPN", "NEW MPN"]
colors = ["red", "blue", "green"]

# Prepare the figure
fig, axes = plt.subplots(1, 3, figsize=(15, 5))
plots = []
trajectories = []
viewcones = []
person_points = []

for ax, label in zip(axes, labels):
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_title(label)
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    robot_plot, = ax.plot([], [], "o", color="red", label="Robot")
    person_plot, = ax.plot([], [], "o", color="blue", label="Person")
    trajectory, = ax.plot([], [], "-", color="red", alpha=0.5)
    viewcone, = ax.plot([], [], "-", color="orange", alpha=0.3)
    ax.legend()
    plots.append((robot_plot, person_plot))
    trajectories.append(trajectory)
    viewcones.append(viewcone)
    person_points.append([])

# Animation update function
def update(frame):
    t = frame * simulation.dt
    simulation.update_person_position(t)
    for i, mode in enumerate(modes):
        simulation.reset_robot_orientation()
        offset = simulation.calculate_offset()

        if mode == "pn":
            omega = simulation.pn(offset)
        elif mode == "mpn":
            omega = simulation.mpn(offset)
        elif mode == "new_mpn":
            omega = simulation.new_mpn(offset)

        simulation.update_robot_orientation(omega)

        robot_x, robot_y = 0, 0  # Robot is stationary
        robot_orientation = simulation.robot_orientation

        # Define the robot's view cone
        cone_angle = np.linspace(robot_orientation - 0.5, robot_orientation + 0.5, 20)
        cone_x = np.cos(cone_angle) * 5
        cone_y = np.sin(cone_angle) * 5

        plots[i][0].set_data(robot_x, robot_y)
        plots[i][1].set_data(0, simulation.person_position[1])
        viewcones[i].set_data(cone_x, cone_y)
        person_points[i].append((robot_x, robot_y))

        if len(person_points[i]) > 1:
            x_vals, y_vals = zip(*person_points[i])
            trajectories[i].set_data(x_vals, y_vals)

# Run the animation
ani = FuncAnimation(fig, update, frames=200, interval=50)

# Save as GIF (optional)
ani.save("human_follow_rotation_simulation.gif", writer="imagemagick")

plt.show()
