#!/usr/bin/python3
import rospy

from low_level_control import LowLevelController
from pure_pursuit_control import PurePursuitController
from messages.msg import Points, Point
import time

class Control:
    def __init__(self, low_level_controller: LowLevelController):
        rospy.init_node("control_node")
        rospy.on_shutdown(self.shutdown)
        self.trajectory_sub =  rospy.Subscriber("/trajectory", Points, self.trajectory_callback)

        self.low_level_controller = low_level_controller
        self.pure_pursuit_controller = PurePursuitController()

        self.current_throttle = 0
        self.current_steering = 0
        self.last_trajectory = None
        self.last_trajectory_time = None
        self.timeout_ms = 250

        self.update_rate = rospy.Rate(10) # hz
        
        rospy.loginfo("Control initialized.")

    def update(self):
            input("Press enter to start")
            while not rospy.is_shutdown():
                self.check_trajectory_timeout(self.timeout_ms)
                if self.last_trajectory:
                    if self.current_throttle == 0:
                        rospy.loginfo("Trajectory found, starting vehicle.")
                        self.start_vehicle()
                    self.control(self.last_trajectory)
                elif self.current_throttle != 0: # no trajectory and moving
                    rospy.logwarn("No trajectory found, stopping vehicle.")
                    self.stop_vehicle()
                print(f"Current Throttle: {self.current_throttle}")
                print(f"Current Steering: {self.current_steering}")
                print()
                self.update_rate.sleep()
    
    def trajectory_callback(self, msg):
        self.last_trajectory = msg
        self.last_trajectory_time = rospy.Time.now()

    def control(self, trajectory_msg):
        try:
            steering_angle = self.pure_pursuit_controller.compute_steering_angle(trajectory_msg.points)
            if steering_angle is None:
                self.set_throttle(0)
            else:
                self.set_steering(steering_angle)
                # throttle is a low constant for now
            
        except Exception as e:
            rospy.logerr(f"Error in control loop: {e}")
            self.stop_vehicle()
    
    def stop_vehicle(self):
        rospy.loginfo("Stopping vehicle.")
        self.set_throttle(0)
    
    def start_vehicle(self):
        rospy.loginfo("Starting vehicle.")
        self.soft_launch()

    def soft_launch(self):
        """
        Sets the throttle to the minimum required to start moving, 
        then to the minimum required to keep moving.
        """
        self.set_throttle(1)
        time.sleep(1)
        self.set_throttle(0.9)

    def shutdown(self):
        self.set_throttle(0)
        self.low_level_controller.shutdown()
    
    def set_throttle(self, value):
        if value < -1 or value > 1:
            rospy.logwarn("Value must be between -1 and 1.")
            return
        self.current_throttle = value
        self.low_level_controller.set_throttle(value)

    def set_steering(self, value):
        if value < -1 or value > 1:
            rospy.logwarn("Value must be between -1 and 1.")
            return
        if abs(value - self.current_steering) > 0.5:
            rospy.logwarn(f"Large steering change detected: {self.current_steering} to {value}")
        self.current_steering = value
        self.low_level_controller.set_steering(value)
    
    def check_trajectory_timeout(self, timeout_ms=250):
        if not self.last_trajectory:
            return
        if (rospy.Time.now() - self.last_trajectory_time).to_sec() > (timeout_ms*0.001):
            self.last_trajectory = None
            self.last_trajectory_time = None
            self.stop_vehicle()
            rospy.loginfo(f"Trajectory timeout after {timeout_ms}ms.")
    
if __name__ == "__main__":
    try:
        low_level = LowLevelController()
        node = Control(low_level)
        node.update()
    except Exception as e:
        rospy.signal_shutdown()