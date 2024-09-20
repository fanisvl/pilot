#!/usr/bin/python3

import rospy
from messages.msg import Points, Point
from std_msgs.msg import Float32
import math
import numpy as np

class Control:
    def __init__(self):
        rospy.init_node("control_node")
        self.tajectory_sub =  rospy.Subscriber("/trajectory", Points, self.control)
        self.steering_pub = rospy.Publisher("/control/steering", Float32, queue_size=1)
        self.target_pub = rospy.Publisher("/control/target", Point, queue_size=1)

        self.WHEELBASE_LEN = 15 #cm
        self.PURE_PURSUIT_L = 30 #cm

        self.TURN_THRESHOLDS = [
            (1.0,   0.00),    # TURN_MIN
            (1.14,  0.25),    # TURN_A
            (1.28,  0.50),    # TURN_MID
            (1.42,  0.75),    # TURN_B
            (1.57,  1.00)     # TURN_MAX
        ]
        rospy.loginfo("Control initialized.")


    def pure_pursuit(self, trajectory_points, L=5):
        while L > 0:
            outerpoints = [point for point in trajectory_points if self.euclidean_distance_origin((point.x, point.y)) >= L]
            innerpoints = [point for point in trajectory_points if self.euclidean_distance_origin((point.x, point.y)) < L]
            
            if outerpoints:
                min_outer = min(outerpoints, key=lambda p: self.euclidean_distance_origin((p.x, p.y)))
                max_inner = max(innerpoints, key=lambda p: self.euclidean_distance_origin((p.x, p.y))) if innerpoints else Point(0, 0, 0)
                return self.interpolate_to_circle((0, 0), L, (max_inner.x, max_inner.y), (min_outer.x, min_outer.y))
            
            L *= 0.9
        
        raise ValueError("Pure Pursuit: No suitable points found")

    def steering(self, target):
        """
        INPUT: Target point
        Steering angle output from pure pursuit ranges from (-1.57, 1.57) = (-pi/2, pi/2)
        OUTPUT: Steering angle normalized to (-1, 1)
        """
        tx, ty = target
        arc_curvature = math.degrees((2*tx) / (self.PURE_PURSUIT_L**2))
        steering_angle = math.atan(arc_curvature * self.WHEELBASE_LEN)
        
        turn_heading = 1 if steering_angle >= 0 else -1
        abs_steering_angle = abs(steering_angle)
        
        for threshold, value in self.TURN_THRESHOLDS:
            if abs_steering_angle <= threshold:
                return turn_heading * value
        
        rospy.logwarn(f"Couldn't find valid turn threshold for steering angle: {abs_steering_angle}")
        return 0

    def euclidean_distance_origin(self, point):
        return math.sqrt(point[0]**2 + point[1]**2)

    def interpolate_to_circle(self, center, radius, inner_point, outer_point):
        x_c, y_c = center
        x_inner, y_inner = inner_point
        x_outer, y_outer = outer_point
        vx = x_outer - x_inner
        vy = y_outer - y_inner

        A = vx**2 + vy**2
        B = 2 * (vx * (x_inner - x_c) + vy * (y_inner - y_c))
        C = (x_inner - x_c)**2 + (y_inner - y_c)**2 - radius**2

        discriminant = B**2 - 4 * A * C
        if discriminant < 0:
            raise ValueError("The points do not form a line that intersects the circle.")
        t1 = (-B + np.sqrt(discriminant)) / (2 * A)
        t2 = (-B - np.sqrt(discriminant)) / (2 * A)
        t = max(t1, t2)
        x_interp = x_inner + t * vx
        y_interp = y_inner + t * vy

        return (x_interp, y_interp)

    def control(self, trajectory_msg):
        """
        For speed just use a very low value constant
        INPUT: Trajectory points from planning
        OUTPUT: Steering commands to actuators
        """

        trajectory_points = trajectory_msg.points
        target = self.pure_pursuit(trajectory_points, self.PURE_PURSUIT_L)
        steering = self.steering(target)

        self.steering_pub.publish(Float32(steering))
        self.target_pub.publish(Point(x=target[0], y=target[1]))

if __name__ == "__main__":
    node = Control()
    rospy.spin()



