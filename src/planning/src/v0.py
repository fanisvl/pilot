#!/usr/bin/python3

from messages.msg import ConeEstimates, ConeEstimate, Point, Points
import numpy as np
import math

""""
V0

A planner that's as simple as possible.
Only requires a pair of cones. Find the midpoint of the closest pair and generate a linear trajectory.
"""

class Planner:
    def __init__(self):
        rospy.init_node('planning_node')
        self.target_pub = rospy.Publisher('/trajectory', Point, queue_size=1)

        # Initialize Publisher and Subscriber
        self.cone_estimates_sub = rospy.Subscriber('/cone_estimates', ConeEstimates, self.plan)

    def remove_duplicate_estimates(self, estimates, threshold=0.5):
        """
        Sometimes cone estimation returns duplicate estimates.
        Remove cone estimates that are closer than a specified threshold in cm.
        """
        if len(estimates) == 0:
            return estimates
        estimates = np.array(estimates)
        filtered_estimates = []
        for estimate in estimates:
            if not any(np.linalg.norm(estimate - p) < threshold for p in filtered_estimates):
                filtered_estimates.append(estimate)
        return filtered_estimates
    
    def find_closest_cone_pair(self, estimates):
        """
        Find the closest cone estimate pair to (0,0)
        """
        distances = [(np.sqrt(estimates[0]**2 + estimate[1]**2), estimate) for estimate in estimates]
        distances.sort(key=lambda d: d[0])
        closest_pair = [distances[0][1], distances[1][1]]

        return closest_pair
    
    def calculate_midpoint(self, p1, p2):
        x1,y1 = p1
        x2,y2 = p2
        midpoint = ((x1+x2)/2, (y1+y2)/2)
        return midpoint
    
    def generate_trajectory(p1, p2, samples=5):
        """
        Generate points along the linear interpolation of p1 and p2.
        """
        x1, y1 = p1
        x2, y2 = p2
        trajectory = []
        for i in range(samples + 1):
            t = i / samples
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            trajectory.append((x, y))
        return trajectory

    def plan(self, estimates):
        estimates = [(cone.x, cone.y) for cone in estimates.cones]
        estimates = self.remove_duplicate_estimates(estimates, 3)
        # estimates = [(28.47, 112.4), (-17.17, 99.25), (21.38, 160.38)] # demo
            
        path_points = []
        if len(estimates) < 2:
            return 
        else:
            #TODO: Check if they're actually a valid pair based on distance between them
            p1, p2 = self.find_closest_cone_pair(estimates)
            path_points.append(self.calculate_midpoint(p1, p2))

        # TODO
        # With the current logic, path_points only contains one element so this is redundant.
        # Will probably need to add a look ahead of at least two goal points,
        # as we get closer to the goal point.
        closest_path_point = min(path_points, key=lambda p: math.sqrt(p[0]**2 + p[1]**2))

        trajectory_points = self.generate_trajectory((0,0), closest_path_point, samples=10)

        trajectory_msg = Points()
        trajectory_msg.points = trajectory_points
        self.target_pub.publish(trajectory_msg)
    
if __name__ == '__main__':
    node = Planner()
    rospy.spin()