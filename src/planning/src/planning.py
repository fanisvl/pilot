#!/usr/bin/python3

import rospy
from messages.msg import ConeEstimates, ConeEstimate, Point, Points
import numpy as np
import math
from scipy.spatial import Delaunay

class Planner:
    def __init__(self):
        rospy.init_node('planning_node')
        self.target_pub = rospy.Publisher('/trajectory', Points, queue_size=1)
        self.cone_estimates_sub = rospy.Subscriber('/cone_estimates', ConeEstimates, self.plan, queue_size=1)

    def plan(self, estimates):
        estimates = [(cone.x, cone.y) for cone in estimates.cones]
        estimates = self.remove_duplicate_estimates(estimates, 3)
            
        path_points = []
        if len(estimates) < 2:
            return 
        elif len(estimates) < 3:
            #TODO: Check if they're actually a valid pair and not noise
            p1, p2 = self.find_closest_cone_pair(estimates)
            path_points.append(self.calculate_midpoint(p1, p2))
        else:
            points_array = np.array(estimates)
            triangulation = Delaunay(points_array)
            for triangle in triangulation.simplices:
                for i in range(3):
                    v1 = points_array[triangle[i]]
                    v2 = points_array[triangle[(i + 1) % 3]]
                    midpoint = (v1 + v2) / 2
                    # Filter for horizontally aligned edges 
                    #TODO: This will probably fail for more complicated cases
                    if abs(v1[0] - v2[0]) > abs(v1[1] - v2[1]):  # edge is more horizontal than vertical
                        path_points.append(midpoint)
        if not path_points:
            return
        
        # sort based on distance from (0,0)
        path_points.sort(key=lambda p: math.sqrt(p[0]**2 + p[1]**2))
        trajectory_msg = Points()
        trajectory_msg.points = []
        current_pos = (0,0)
        for next_point in path_points:
            trajectory_msg.points.extend(self.generate_trajectory(current_pos, next_point, samples=5))
            current_pos = next_point

        self.target_pub.publish(trajectory_msg)
    
    def remove_duplicate_estimates(self, estimates, threshold=0.5):
        """
        Sometimes cone estimation returns duplicate estimates.
        Remove cone estimates that are closer than a specified threshold in cm.
        """
        if len(estimates) == 0:
            return estimates
        filtered_estimates = []
        estimates = np.array(estimates)
        for estimate in estimates:
            if not any(np.linalg.norm(estimate - p) < threshold for p in filtered_estimates):
                filtered_estimates.append(estimate)
        return filtered_estimates
    
    def find_closest_cone_pair(self, estimates):
        """
        Find the closest cone estimate pair to (0,0)
        """
        distances = [(np.sqrt(estimate[0]**2 + estimate[1]**2), estimate) for estimate in estimates]
        distances.sort(key=lambda d: d[0])
        closest_pair = [distances[0][1], distances[1][1]]
        return closest_pair
    
    def calculate_midpoint(self, p1, p2):
        x1,y1 = p1
        x2,y2 = p2
        midpoint = ((x1+x2)/2, (y1+y2)/2)
        return midpoint
    
    def generate_trajectory(self, p1, p2, samples=5):
        """
        Generate points along the linear interpolation of p1 and p2.
        """
        x1, y1 = p1
        x2, y2 = p2
        trajectory = []
        for i in range(samples + 1):
            t = i / samples # te[0,1] so p1 and p2 are included in the trajectory
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            trajectory.append(self.point_msg(x,y))
        return trajectory

    def point_msg(self, x, y):
        p = Point()
        p.x = x
        p.y = y
        return p

if __name__ == '__main__':
    node = Planner()
    rospy.spin()
