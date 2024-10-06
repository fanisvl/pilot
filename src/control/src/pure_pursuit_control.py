#!/usr/bin/python3
import rospy
import math
import numpy as np

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
    
    def compute_steering_angle(self, trajectory_points):
        try:
            target = self.find_target(trajectory_points)
            if target is None: return None
            tx, ty = target
            arc_curvature = math.degrees((2*tx) / (self.PURE_PURSUIT_L**2))
            steering_angle = math.atan(arc_curvature * self.WHEELBASE_LEN)
            return self.normalized_steering_angle(steering_angle)
        except Exception as e:
            rospy.logerr(f"Error in compute_steering_angle: {e}")
            return None

    def find_target(self, trajectory_points, L=100):
        outerpoints = [point for point in trajectory_points 
                        if self.euclidean_distance_origin((point.x, point.y)) >= L]
        innerpoints = [point for point in trajectory_points 
                        if self.euclidean_distance_origin((point.x, point.y)) < L]
        if not innerpoints or not outerpoints:
            return None
        min_outer = min(outerpoints, key=lambda p: self.euclidean_distance_origin((p.x, p.y)))
        max_inner = max(innerpoints, key=lambda p: self.euclidean_distance_origin((p.x, p.y)))
        return self.interpolate_to_circle((0, 0), L, (max_inner.x, max_inner.y), (min_outer.x, min_outer.y))
        
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
            rospy.logwarn("The points do not form a line that intersects the circle.")
            return None
        t1 = (-B + np.sqrt(discriminant)) / (2 * A)
        t2 = (-B - np.sqrt(discriminant)) / (2 * A)
        t = max(t1, t2)
        x_interp = x_inner + t * vx
        y_interp = y_inner + t * vy
        return (x_interp, y_interp)