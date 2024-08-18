#!/usr/bin/python3

import rospy
import json
import math
import numpy as np
from scipy.spatial import Delaunay
from sklearn.cluster import DBSCAN
from std_msgs.msg import Float64
from messages.msg import ConeEstimates, CenterLinePoints, Point

class PathPlanningNode:
    def __init__(self):
        rospy.init_node('path_planning_node')

        # Initialize Publisher and Subscriber
        self.centerline_pub = rospy.Publisher('/centerline_points', CenterLinePoints, queue_size=1)
        self.cone_estimates_sub = rospy.Subscriber('/cone_estimates', ConeEstimates, self.cone_estimates_callback)

        self.TRACK_WIDTH = 3

    def cone_estimates_callback(self, cone_estimates_msg):
        
        cone_estimates = cone_estimates_msg.cones

        centerline_points = self.path_planning(cone_estimates)

        self.centerline_pub.publish(centerline_points)


    def path_planning(self, cone_estimates):
        """
        INPUT: Cone Estimates
        OUTPUT: Centerline Path Points
        """
        points = [(cone.x, cone.y) for cone in cone_estimates]
        points_of_interest = self.remove_background_points(points)

        filtered_cone_estimates = [cone for cone in cone_estimates if (cone.x, cone.y) in points_of_interest]

        left_points = []
        right_points = []
        for cone in filtered_cone_estimates:
            point = (cone.x, cone.y)
            if cone.label == 0:
                left_points.append(point)
            elif cone.label == 1:
                right_points.append(point)

        if len(left_points) < 2 and len(right_points) < 2:
            print("Not left and right points to calculate")
            print(f"left points: {len(left_points)}, right_points: {len(right_points)}")
            return

        if len(left_points) == 0:
            left_points = self.find_virtual_points(right_points, "left_empty")
        elif len(right_points) == 0:
            right_points = self.find_virtual_points(left_points, "right_empty")


        centerline_points = self.find_midpoints(left_points, right_points)
        return centerline_points

    def find_midpoints(self, left_points, right_points):
        
        centerline_msg = CenterLinePoints()
        centerline_points = centerline_msg.centerline

        points = left_points + right_points
        points_array = np.array(points)
        triangulation = Delaunay(points_array)

        left_points_set = set(map(tuple, left_points))
        right_points_set = set(map(tuple, right_points))

        for simplex in triangulation.simplices:
            p1 = points_array[simplex[0]]
            p2 = points_array[simplex[1]]
            p3 = points_array[simplex[2]]

            p1_tuple = tuple(p1)
            p2_tuple = tuple(p2)
            p3_tuple = tuple(p3)

            edges = [(p1_tuple, p2_tuple), (p2_tuple, p3_tuple), (p3_tuple, p1_tuple)]
            for (v1, v2) in edges:
                if (v1 in left_points_set and v2 in right_points_set) or (v1 in right_points_set and v2 in left_points_set):
                    midpoint = Point(x=(v1[0] + v2[0]) / 2, y=(v1[1] + v2[1]) / 2)
                    centerline_points.append(midpoint)

        return centerline_points

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
    node = PathPlanningNode()
    rospy.spin()

