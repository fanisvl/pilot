#!/usr/bin/python3

from messages.msg import ConeEstimates, ConeEstimate, Point
import matplotlib.pyplot as plt
import numpy as np

""""
V0

A planner that's as simple as possible.
Only requires a pair of cones. Find the midpoint of the closest pair.
"""

class Planner:
    def __init__(self):
        rospy.init_node('planning_node')

        # Initialize Publisher and Subscriber
        self.target_pub = rospy.Publisher('/target_point', Point, queue_size=1)
        self.cone_estimates_sub = rospy.Subscriber('/cone_estimates', ConeEstimates, self.plan)

    def plan(self, cone_estimates_msg):
        """
        Finds the closest pair of cones, calculate midpoint,
        return as target for pure pursuit in control.
        """
        distances = []
        for cone in cone_estimates_msg.cones:
            distances.append(np.sqrt((cone.x**2 + cone.y**2), (cone.x, cone.y)))
        
        distances.sort(key=lambda d: d[0])
        closest_pair = [distances[0][1], distances[1][1]]
        
        x1,y1 = closest_pair[0]
        x2,y2 = closest_pair[1]

        midpoint = Point((x1+x2)/2, (y1+y2)/2)

        self.target_pub.publish(midpoint)
    
if __name__ == '__main__':
    node = Planner()
    rospy.spin()