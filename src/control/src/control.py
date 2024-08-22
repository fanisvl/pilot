#!/usr/bin/python3

import rospy
from messages.msg import CenterLinePoints, Point
import math
import numpy as np

class Control:
    def __init__(self):
        rospy.init_node("control_node")
        self.centerline_sub =  rospy.Subscriber("centerline_points", CenterLinePoints, self.control)

        self.WHEELBASE_LEN = 0.5
        self.PURE_PURSUIT_L = 3

        self.TURN_MIN = 0.0
        self.TURN_A   = 0.25
        self.TURN_MID = 0.50
        self.TURN_B   = 0.75
        self.TURN_MAX = 1.0


    def pure_pursuit(self, centerline_points):
        L = self.PURE_PURSUIT_L
        while True:
            outerpoints = []
            innerpoints = []
            innerpoints.append((0,0))
            for point in centerline_points:
                point = (point.x, point.y)
                if self.euclidean_distance_origin(point) >= L:
                    outerpoints.append(point)
                else:
                    innerpoints.append(point)

            if len(outerpoints) != 0:
                break

            elif L == 0:
                raise ValueError("Pure Pursuit: L = 0")

            else:
                L *= 0.9

        min_outer = min(outerpoints, key=self.euclidean_distance_origin)
        max_inner = max(innerpoints, key=self.euclidean_distance_origin)
        target_point = self.interpolate_to_circle((0,0), L, max_inner, min_outer)
        tx, ty = target_point[0], target_point[1]

        arc_curvature = math.degrees((2*tx) / math.pow(L, 2.0))
        steering_angle = math.atan(arc_curvature * self.WHEELBASE_LEN)

        return steering_angle

    def normalize_steering_angle(self, steering_angle):
        """
        Helper function for control
        INPUT: Steering angle output from pure pursuit ranges from (-1.57, 1.57) = (-pi/2, pi/2)
        OUTPUT: Normalize to (-1, 1)
        """
        # determine turn heading
        if steering_angle >= 0:
            turn_heading = 1 # right
        else:
            turn_heading = -1 # left

        normalized_steering_angle = turn_heading
        steering_angle = abs(steering_angle)

        # Normalize the steering angle
        if steering_angle < 1:
            normalized_steering_angle *= self.TURN_MIN
        elif 1 <= steering_angle < 1.1425:
            normalized_steering_angle *= self.TURN_A
        elif 1.1425 <= steering_angle < 1.285:
            normalized_steering_angle *= self.TURN_MID
        elif 1.285 <= steering_angle < 1.4275:
            normalized_steering_angle *= self.TURN_B
        elif 1.4275 <= steering_angle <= 1.57:
            normalized_steering_angle *= self.TURN_MAX

        return normalized_steering_angle

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

    def control(self, centerline_points_msg):
        """
        For speed just use a very low value constant
        INPUT: Steering angle from pure pursuit
        OUTPUT: Steering commands to actuators
        """
        centerline_points = centerline_points_msg.centerline
        steering_angle = self.pure_pursuit(centerline_points)
        normalized = self.normalize_steering_angle(steering_angle)
        print(f"Normalized: {normalized}")

if __name__ == "__main__":
    node = Control()
    rospy.spin()



