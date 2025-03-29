import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class BrownianRobot:
    def __init__(self, arena_size=10, speed=0.1):
        self.arena_size = arena_size
        self.position = np.array([arena_size / 2, arena_size / 2])  # Start at center
        self.speed = speed
        self.angle = 0  # Random initial angle

        self.fig, self.ax = plt.subplots(figsize=(5, 5))
        self.ax.set_xlim(0, arena_size)
        self.ax.set_ylim(0, arena_size)

        self.robot_plot, = self.ax.plot([], [], 'bo', markersize=16)  # Empty at start

    def move(self):
        self.position[0] += self.speed * np.cos(self.angle)  # X position
        self.position[1] += self.speed * np.sin(self.angle)  # Y position

        if self.position[0] <= 0 or self.position[0] >= self.arena_size:
            self.angle = np.random.uniform(0, 2 * np.pi)
            self.position[0] = np.clip(self.position[0], 0, self.arena_size)

        if self.position[1] <= 0 or self.position[1] >= self.arena_size:
            self.angle = np.random.uniform(0, 2 * np.pi)
            self.position[1] = np.clip(self.position[1], 0, self.arena_size)

    def update(self, frame):
        self.move()
        self.robot_plot.set_data([self.position[0]], [self.position[1]])

        return self.robot_plot,

    def animate(self, frames=200):
        """Run the animation."""
        ani = animation.FuncAnimation(self.fig, self.update, frames=frames, interval=50, blit=False)  # FIXED: blit=False
        plt.show()
