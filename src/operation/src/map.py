#!/usr/bin/python3

import threading
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from messages.msg import ConeEstimates, ConeEstimate, Points, Point
from std_msgs.msg import Float32


class LiveMap:
    def __init__(self):
        rospy.init_node("live_map", anonymous=True)
        self.fig, self.ax = plt.subplots()
        # thread lock to prevent multiaccess threading errors (tkinter)
        self.lock = threading.Lock()
        self.cones = []
        self.trajectory = []
        self.target = []
        self.steering = None
        self.steering_angle = 0

        self.sub_cones = rospy.Subscriber("/cone_estimates", ConeEstimates, self.cones_callback, queue_size=1)
        self.sub_trajectory = rospy.Subscriber("/trajectory", Points, self.trajectory_callback, queue_size=1)
        self.sub_steering = rospy.Subscriber("/control/steering", Float32, self.steering_callback, queue_size=1)
        self.sub_steering_angle = rospy.Subscriber("/control/steering_angle", Float32, self.steering_angle_callback, queue_size=1)
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

    def steering_angle_callback(self, msg):
        self.steering_angle = msg.data
        print(f"Steering Angle: {msg.data}")

    def target_callback(self, msg):
        with self.lock:
            self.target = [msg.x, msg.y]


    def plt_func(self, _):
        with self.lock:
            self.ax.clear()  # Clear previous points
            if self.cones:
                x_cones, y_cones = zip(*self.cones)
                self.ax.scatter(x_cones, y_cones, color="orange", label="Cones")  # Plot cone points
                for x, y in zip(x_cones, y_cones):
                    self.ax.text(x, y + 0.3, f'({x:.0f}, {y:.0f})', fontsize=8, ha='center')

            if self.trajectory:
                x_trajectory, y_trajectory = zip(*self.trajectory)
                self.ax.scatter(x_trajectory, y_trajectory, color="blue", label="Trajectory", s=20)
            
            if self.target:
                self.ax.scatter(self.target[0], self.target[1], color='red', label="Target")

            if self.steering != None:
                self.ax.text(
                    0.05, 0.95,                      
                    f"Steering: {self.steering}",
                    transform=self.ax.transAxes,
                    fontsize=12,
                    verticalalignment='top',  
                    horizontalalignment='left',
                    bbox=dict(facecolor='white', alpha=0.5) 
                )
            
            self.ax.set_xlim(-100, 100)
            self.ax.set_ylim(0, 500)
            self.ax.set_title("Map")
            self.ax.set_xlabel("X (cm)")
            self.ax.set_ylabel("Z (cm)")
            plt.grid()
            self.ax.legend()
            return self.ax

    def plt_show(self):
        """Start the matplotlib animation."""
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
        plt.savefig('plot.png', dpi=300, bbox_inches='tight')
        plt.show()

def main():
    node = LiveMap()
    thread = threading.Thread(target=rospy.spin, daemon=True)
    thread.start()
    node.plt_show()

if __name__ == "__main__":
    main()
