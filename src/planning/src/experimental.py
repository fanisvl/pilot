#!/usr/bin/python3

import rospy
import json
import math
import numpy as np
from scipy.spatial import Delaunay
from sklearn.cluster import DBSCAN
from std_msgs.msg import Float64
from messages.msg import ConeEstimates, CenterLinePoints, Point

class Planning:
    def __init__(self):
        rospy.init_node('planning')

        # Initialize Publisher and Subscriber
        self.centerline_pub = rospy.Publisher('/path_points', CenterLinePoints, queue_size=1)
        self.cone_estimates_sub = rospy.Subscriber('/cone_estimates', ConeEstimates, self.planning)

        self.TRACK_WIDTH = 3

    def planning(self, cone_estimates_msg):
        """
        INPUT: Cone Estimates
        OUTPUT: Centerline Path Points
        """
        cone_estimates = [(cone.x, cone.y) for cone in cone_estimates_msg.cones]

        # 
        if len(cone_estimates) < 2:
            print(f"Not enough ")
            return


        if len(cone_estimates) < 4:
            print(f"Planning: Need at least 3 points (Delaunay), only have {len(cone_estimates)}.")
            return

        path_points = self.find_midpoints(cone_estimates)
        self.centerline_pub.publish(path_points)

    def find_midpoints(self, points):
        centerline_msg = CenterLinePoints()
        path_points = centerline_msg.centerline

        points_array = np.array(points)
        
        triangulation = Delaunay(points_array)

        for simplex in triangulation.simplices:
            p1 = points_array[simplex[0]]
            p2 = points_array[simplex[1]]
            p3 = points_array[simplex[2]]

            edges = [(p1, p2), (p2, p3), (p3, p1)]
            for (v1, v2) in edges:
                midpoint = Point(x=(v1[0] + v2[0]) / 2, y=(v1[1] + v2[1]) / 2)
                path_points.append(midpoint)
        return path_points

    def remove_background_points(self, points):
        """
        INPUT: Cone Estimate points
        OUTPUT: Remove points that belong in the furthest away cluster from (0,0)
        """
        # DBSCAN parameters: eps (maximum distance between two samples for them to be considered as in the same neighborhood)
        # and min_samples (number of samples in a neighborhood for a point to be considered as a core point)
        db = DBSCAN(eps=5, min_samples=1).fit(points)
        labels = db.labels_
        unique_labels = set(labels)

        min_distance = float('inf')
        closest_cluster_points = None
        for label in unique_labels:
            if label == -1:  # -1 is the noise label
                continue
            # get the points in the current cluster
            current_cluster_points = np.array(points)[labels == label]
            # calculate the centroid of the current cluster
            centroid = np.mean(current_cluster_points, axis=0)
            # calculate the distance of the centroid from (0,0)
            distance = np.linalg.norm(centroid)
            # Update the closest cluster if needed
            if distance < min_distance:
                min_distance = distance
                closest_cluster_points = current_cluster_points

        return closest_cluster_points

    def find_virtual_points(self, points, empty):
        """"
        INPUT: Left side or right side only points, side missing
        OUTPUT: Virtual points for the missing side

        1. Calculate two closest points (x1,y1), (x2,y2) to (0,0)
        2. Draw line between them
        3. Virtual points are found on the perpendicular to that line at length TRACK_WIDTH starting from each point.
        """

        virtual_points = []

        # Calculate two closest points
        distances = np.linalg.norm(points, axis=1)
        closest_indices = np.argsort(distances)

        if len(closest_indices) < 2:
            raise ValueError("Need at least two points to calculate virtual points.")


        for i in range(0, len(closest_indices)):
            if i == len(closest_indices)-1:
                break

            x1, y1 = points[closest_indices[i]][0],points[closest_indices[i]][1]
            x2, y2 = points[closest_indices[i+1]][0], points[closest_indices[i+1]][1]

            # Calculate the slope of line AB
            if x2 == x1:
                slope_AB = float('inf')
            else:
                slope_AB = (y2 - y1) / (x2 - x1)

            if slope_AB == 0:
                perp_slope = float('inf')
                raise ValueError("slope_AB = 0 => perp_slope = inf")
            elif slope_AB == float('inf'):
                perp_slope = 0
            else:
                perp_slope = -1 / slope_AB

            # Calculate points to draw lines from A and B
            if empty == "right_empty":
                x1_virtual = x1 + self.TRACK_WIDTH / np.sqrt(1 + perp_slope**2)
                y1_virtual = y1 + perp_slope * (x1_virtual - x1)

                x2_virtual = x2 + self.TRACK_WIDTH / np.sqrt(1 + perp_slope**2)
                y2_virtual = y2 + perp_slope * (x2_virtual - x2)

            elif empty == "left_empty":
                perp_slope = abs(perp_slope)
                x1_virtual = x1 - self.TRACK_WIDTH / np.sqrt(1 + perp_slope**2)
                y1_virtual = y1 + perp_slope * (x1_virtual - x1)

                x2_virtual = x2 - self.TRACK_WIDTH / np.sqrt(1 + perp_slope**2)
                y2_virtual = y2 + perp_slope * (x2_virtual - x2)

            virtual_points.append((x1_virtual, y1_virtual))
            virtual_points.append((x2_virtual, y2_virtual))

        return virtual_points

if __name__ == '__main__':
    node = Planning()
    rospy.spin()

