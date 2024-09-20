#!/usr/bin/python3

import threading
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
from messages.msg import ConeEstimates, ConeEstimate, Points, Point
from std_msgs.msg import Float32


class LiveMap:
    def __init__(self):
        """Initialize."""
        rospy.init_node("example_node", anonymous=True)
        self.fig, self.ax = plt.subplots()
        # Create thread lock to prevent multiaccess threading errors
        self.lock = threading.Lock()
        self.cones = []
        self.trajectory = []
        self.target = []
        self.steering = 0

        self.sub_cones = rospy.Subscriber("/cone_estimates", ConeEstimates, self.cones_callback, queue_size=1)
        self.sub_trajectory = rospy.Subscriber("/trajectory", Points, self.trajectory_callback, queue_size=1)
        self.sub_steering = rospy.Subscriber("/control/steering", Float32, self.steering_callback, queue_size=1)
        self.sub_target = rospy.Subscriber("/control/target", Point, self.target_callback, queue_size=1)

        

    def cones_callback(self, msg):
        with self.lock:
            self.cones = [(cone.x, cone.y) for cone in msg.cones]

    def trajectory_callback(self, msg):
        with self.lock:
            self.trajectory = [(point.x, point.y) for point in msg.points]
    
    def steering_callback(self, msg):
        self.steering = msg.data
        print(f"Steering: {msg.data}")

    def target_callback(self, msg):
        with self.lock:
            self.target = [msg.x, msg.y]


    def plt_func(self, _):
        with self.lock:
            self.ax.clear()  # Clear previous points
            # Plot cone points
            if self.cones:
                x_cones, y_cones = zip(*self.cones)
                self.ax.scatter(x_cones, y_cones, color="orange", label="Cones")  # Plot cone points
            # Plot trajectory points
            if self.trajectory:
                x_trajectory, y_trajectory = zip(*self.trajectory)
                self.ax.scatter(x_trajectory, y_trajectory, color="blue", label="Trajectory", s=20)
            
            if self.target:
                self.ax.scatter(self.target[0], self.target[1], color='red', label="Target")

            if self.steering:
                self.ax.text(
                    0.05, 0.95,                      
                    f"Steering: {self.steering}",
                    transform=self.ax.transAxes,     # Use axes coordinates for positioning
                    fontsize=12,                     # Font size of the text
                    verticalalignment='top',         # Align the text to the top
                    horizontalalignment='left',      # Align the text to the left
                    bbox=dict(facecolor='white', alpha=0.5)  # Optional: Add a background box with transparency
                )
            
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

