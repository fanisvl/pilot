#!/usr/bin/python3

import rospy
from messages.msg import Points, Point
from std_msgs.msg import Float32
import math
import numpy as np
from low_level_control import LowLevelController

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
        max_inner = max(innerpoints, key=lambda p: self.euclidean_distance_origin((p.x, p.y))) if innerpoints else Point(0, 0, 0)
        return self.interpolate_to_circle((0, 0), L, (max_inner.x, max_inner.y), (min_outer.x, min_outer.y))
        
    def normalized_steering_angle(self, steering_angle):
        """ 
        INPUT: Target point
        Steering angle output from pure pursuit ranges from (-1.57, 1.57) = (-pi/2, pi/2)
        OUTPUT: Steering angle normalized to (-1, 1)
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
            print("The points do not form a line that intersects the circle.")
            return None
        t1 = (-B + np.sqrt(discriminant)) / (2 * A)
        t2 = (-B - np.sqrt(discriminant)) / (2 * A)
        t = max(t1, t2)
        x_interp = x_inner + t * vx
        y_interp = y_inner + t * vy
        return (x_interp, y_interp)

class Control:
    def __init__(self):
        rospy.init_node("control_node")
        self.tajectory_sub =  rospy.Subscriber("/trajectory", Points, self.trajectory_callback)

        self.low_level_controller = LowLevelController()
        self.pure_pursuit_controller = PurePursuitController()

        self.current_throttle = 0
        self.current_steering = 0
        self.last_trajectory = None
        self.active = True
        self.update_rate = rospy.Rate(10) # hz
        
        rospy.loginfo("Control initialized.")

    def update(self):
            while not rospy.is_shutdown() and self.active:
                if self.last_trajectory:
                    self.control(self.last_trajectory)
                else:
                    self.low_level_controller.set_throttle(0)
                    rospy.loginfo("No trajectory. Stopping vehicle.")
                self.update_rate.rate.sleep()
    
    def trajectory_callback(self, msg):
        self.last_trajectory = msg

    def control(self, trajectory_msg):
        try:
            steering_angle = self.pure_pursuit_controller.compute_steering_angle(trajectory_msg.points)
            if steering_angle is None:
                self.low_level_controller.set_throttle(0)
            else:
                self.low_level_controller.set_steering(steering_angle)
                # throttle is a low constant for now
            self.last_trajectory = None
        except Exception as e:
            rospy.logerr(f"Error in control loop: {e}")
            self.stop_vehicle()
    
    def stop_vehicle(self):
        rospy.loginfo("Stopping vehicle.")
        self.set_throttle(0)
        self.set_steering(0)
    
    def start_vehicle(self):
        rospy.loginfo("Starting vehicle.")
        self.low_level_controller.soft_launch()
    
    def set_throttle(self, value):
        if value < -1 or value > 1:
            print("Value must be between -1 and 1.")
            return
        self.current_throttle = value
        self.low_level_controller.set_throttle(value)

    def set_steering(self, value):
        if value < -1 or value > 1:
            print("Value must be between -1 and 1.")
            return
        if abs(value - self.current_steering) > 0.5:
            rospy.logwarn(f"Large steering change detected: {self.current_steering} to {value}")
        self.current_steering = value
        self.low_level_controller.set_steering(value)


if __name__ == "__main__":
    try:
        node = Control()
        node.update()
    except rospy.ROSInterruptException:
        pass



