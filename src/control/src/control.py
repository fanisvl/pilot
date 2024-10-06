import rospy

from low_level_control import LowLevelController
from pure_pursuit_control import PurePursuitController
from messages.msg import Points, Point

class Control:
    def __init__(self):
        rospy.init_node("control_node")
        self.tajectory_sub =  rospy.Subscriber("/trajectory", Points, self.trajectory_callback)

        self.low_level_controller = LowLevelController()
        self.pure_pursuit_controller = PurePursuitController()

        self.current_throttle = 0
        self.current_steering = 0
        self.last_trajectory = None
        self.update_rate = rospy.Rate(10) # hz
        
        rospy.loginfo("Control initialized.")

    def update(self):
            while not rospy.is_shutdown():
                if self.last_trajectory:
                    self.control(self.last_trajectory)
                else:
                    self.set_throttle(0)
                    rospy.loginfo("No trajectory. Stopping vehicle.")
                self.update_rate.rate.sleep()
    
    def trajectory_callback(self, msg):
        self.last_trajectory = msg

    def control(self, trajectory_msg):
        try:
            steering_angle = self.pure_pursuit_controller.compute_steering_angle(trajectory_msg.points)
            if steering_angle is None:
                self.set_throttle(0)
            else:
                self.set_steering(steering_angle)
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


if __name__ == "__main__":
    try:
        node = Control()
        node.update()
    except rospy.ROSInterruptException:
        pass