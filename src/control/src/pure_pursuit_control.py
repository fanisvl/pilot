#!/usr/bin/python3
import rospy
import math
import numpy as np
from std_msgs.msg import Float32

class PurePursuitController:
    def __init__(self):
        self.WHEELBASE_LEN = 15 #cm
        self.PURE_PURSUIT_L = 100 #cm
        self.TURN_THRESHOLDS = [
            (1.0,   0.00),    # TURN_MIN
            (1.14,  0.25),    # TURN_A
            (1.28,  0.50),    # TURN_MID
            (1.42,  0.75),    # TURN_B
            (1.57,  1.00)     # TURN_MAX
        ]

        self.raw_steering_pub = rospy.Publisher('/control/raw_steering', Float32, queue_size=1)
        self.norm_steering_pub = rospy.Publisher('/control/norm_steering', Float32, queue_size=1)
    
    def compute_steering_angle(self, trajectory_points):
        try:
            target = self.find_target(trajectory_points)
            if target is None: return None
            tx, ty = target
            arc_curvature = math.degrees((2*tx) / (self.PURE_PURSUIT_L**2))
            steering_angle = math.atan(arc_curvature * self.WHEELBASE_LEN)
            normalized = self.normalized_steering_angle(steering_angle) # [-1,1]
            self.raw_steering_pub.publish(Float32(data=steering_angle))
            self.norm_steering_pub.publish(Float32(data=normalized))
            return normalized
        except Exception as e:
            rospy.logerr(f"Error in compute_steering_angle: {e}")
            return None

    def find_target(self, trajectory_points, L=100):
        outerpoints = [point for point in trajectory_points 
                        if self.euclidean_distance_origin(point) >= L]
        innerpoints = [point for point in trajectory_points 
                        if self.euclidean_distance_origin(point) < L]
        if not innerpoints or not outerpoints:
            return None
        min_outer = min(outerpoints, key=lambda p: self.euclidean_distance_origin(p))
        max_inner = max(innerpoints, key=lambda p: self.euclidean_distance_origin(p))
        return self.interpolate_to_circle((0, 0), L, max_inner, min_outer)
        
    def normalized_steering_angle(self, steering_angle):
        """
        Normalize steering angle from pure pursuit output (-1.57, +1.57) to (-1, 1)
        based on self.TURN_THRESHOLDS
        """
        turn_heading = 1 if steering_angle >= 0 else -1
        abs_steering_angle = abs(steering_angle)
        
        for threshold, value in self.TURN_THRESHOLDS:
            if abs_steering_angle <= threshold:
                return turn_heading * value
        
        rospy.logwarn(f"Couldn't find valid turn threshold for steering angle: {abs_steering_angle}")
        return None

    def euclidean_distance_origin(self, point):
        return math.sqrt(point.x**2 + point.y**2)

    def interpolate_to_circle(self, center, radius, inner, outer):
        x_c, y_c = center
        vx = outer.x - inner.x
        vy = outer.y - inner.y

        A = vx**2 + vy**2
        B = 2 * (vx * (inner.x - x_c) + vy * (inner.y - y_c))
        C = (inner.x - x_c)**2 + (inner.y - y_c)**2 - radius**2

        discriminant = B**2 - 4 * A * C
        if discriminant < 0:
            rospy.logwarn("The points do not form a line that intersects the circle.")
            return None
        t1 = (-B + np.sqrt(discriminant)) / (2 * A)
        t2 = (-B - np.sqrt(discriminant)) / (2 * A)
        t = max(t1, t2)
        x_interp = inner.x + t * vx
        y_interp = inner.y + t * vy
        return (x_interp, y_interp)