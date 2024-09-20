#!/usr/bin/python3

import threading
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
from messages.msg import ConeEstimates, ConeEstimate, Points, Point

class LiveMap:
    def __init__(self):
        """Initialize."""
        rospy.init_node("example_node", anonymous=True)
        self.fig, self.ax = plt.subplots()
        # Create thread lock to prevent multiaccess threading errors
        self.lock = threading.Lock()
        self.cones = []
        self.trajectory = []

        # Subscriber for cone estimates
        self.sub_cones = rospy.Subscriber("/cone_estimates", ConeEstimates, self.cones_callback, queue_size=1)
        # Subscriber for trajectory points
        self.sub_trajectory = rospy.Subscriber("/trajectory", Points, self.trajectory_callback, queue_size=1)

    def cones_callback(self, msg):
        print(f"msg.cones: {msg.cones}")
        with self.lock:
            self.cones = [(cone.x, cone.y) for cone in msg.cones]

    def trajectory_callback(self, msg):
        print(f"msg.points: {msg.points}")
        with self.lock:
            self.trajectory = [(point.x, point.y) for point in msg.points]

    def plt_func(self, _):
        with self.lock:
            self.ax.clear()  # Clear previous points
            # Plot cone points
            if self.cones:
                x_cones, y_cones = zip(*self.cones)
                self.ax.scatter(x_cones, y_cones, color="red", label="Cones")  # Plot cone points
            # Plot trajectory points
            if self.trajectory:
                x_trajectory, y_trajectory = zip(*self.trajectory)
                self.ax.scatter(x_trajectory, y_trajectory, color="blue", label="Trajectory")
            
            self.ax.set_xlim(-100, 100)
            self.ax.set_ylim(0, 500)
            self.ax.set_title("Map")
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Z")
            plt.grid()
            self.ax.legend()  # Show legend
            return self.ax

    def plt_show(self):
        """Start the matplotlib animation."""
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
        plt.show()

def main():
    node = LiveMap()
    thread = threading.Thread(target=rospy.spin, daemon=True)
    thread.start()
    node.plt_show()

if __name__ == "__main__":
    main()

