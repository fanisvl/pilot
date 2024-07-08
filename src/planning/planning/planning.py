import json
import matplotlib.pyplot as plt
from CGAL.CGAL_Triangulation_2 import Delaunay_triangulation_2
from CGAL.CGAL_Kernel import Point_2
import math
import numpy as np
from sklearn.cluster import DBSCAN

from messages.msg import ConeEstimates
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

colors = {
        0: '#0000FF',
        1: '#FFFF00',
        2: '#FFA500',
        3: '#FFA500',
    }

class Planning(Node):
    def __init__(self):
        super().__init__("planning")
        self.get_logger().info("Planning node initialized.")

        self.subscription = self.create_subscription(
                ConeEstimates,
                '/cone_estimates',
                self.path_planning,
                10
            )

        self.publisher = self.create_publisher(
                Float64,
                'pure_pursuit_steering_angle',
                10
            )

        self.TRACK_WIDTH = 3

    def path_planning(self, cone_estimates_msg):

        # TODO: Calculate track width as a constant at the start using the orange cones
            # This can be later used to create virtual boundaries if only one side is visible
            # Mostly used during turns so adjust accordingly
        self.TRACK_WIDTH = 3

        cone_estimates = cone_estimates_msg.cones

        # TODO: TWO EDGE CASES:
        # 1) Cones on only one side of the path, but no background cones eg. imgs 15, 16, 17, 18, 22, 38
        # SOLUTION: Add virtual boundary (points) using predetermined path width

        # 2) Cones on only one side of the path, with background cones  eg. imgs 23, 26, 36
        # SOLUTION: Ignore background cones using clustering, then use solution 1

        # TODO: Remove a few unnecessary for loops
        points = [(cone.x, cone.y) for cone in cone_estimates]
        points_of_interest = self.remove_background_points(points)
        filtered_cone_estimates = [cone for cone in cone_estimates if [cone.x, cone.y] in points_of_interest]
        left_points = []
        right_points = []
        for cone in filtered_cone_estimates:
            point = (cone.x, cone.y)
            if cone.label == 0:
                left_points.append(point)
            elif cone.label == 1:
                right_points.append(point)
        
                
        if len(left_points) == 0 and len(right_points) == 0:
            print("(!) No left or right points detected")
            return
        # Check if only one side is visible, if so add virtual boundary
        if len(left_points) == 0:
            left_points.extend(self.find_virtual_points(right_points, "left_empty"))

        elif len(right_points) == 0:
            right_points.extend(self.find_virtual_points(left_points, "right_empty"))

        centerline_points = self.find_midpoints(left_points, right_points)
            
        steering_angle = self.pure_pursuit(centerline_points)
        steering_angle_msg = Float64()
        steering_angle_msg.data = steering_angle
        self.publisher.publish(steering_angle_msg)
        print(steering_angle)

    def find_midpoints(self, left_points, right_points):
        """
        Input: left, right points
        Output: Midpoints between cone pairs
        """
        points = left_points+right_points
        cgal_points = [Point_2(point[0], point[1]) for point in points]
        triangulation = Delaunay_triangulation_2()
        triangulation.insert(cgal_points)

        centerline_points = []
        for edge in triangulation.finite_edges():
            # An edge is represented by a tuple (face_handle, vertex_index)
            # face_handle is the triangle (face) that the edge belongs to, and i is the index of the edge
            face_handle, i = edge
            v1 = face_handle.vertex(i)
            v2 = face_handle.vertex((i + 1) % 3) # (i + 1) % 3 ensures cyclic access to the vertices of the triangle
            p1 = (v1.point().x(), v1.point().y())
            p2 = (v2.point().x(), v2.point().y())

        if ((p1 in left_points and p2 in right_points) or 
                (p1 in right_points and p2 in left_points)):
                midpoint = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
                centerline_points.append(midpoint)
        
        return centerline_points    

    def interpolate_to_circle(self, center, radius, inner_point, outer_point):
        # Unpack the coordinates
        x_c, y_c = center
        x_inner, y_inner = inner_point
        x_outer, y_outer = outer_point
        
        # Calculate the vector components
        vx = x_outer - x_inner
        vy = y_outer - y_inner
        
        # Coefficients of the quadratic equation
        A = vx**2 + vy**2
        B = 2 * (vx * (x_inner - x_c) + vy * (y_inner - y_c))
        C = (x_inner - x_c)**2 + (y_inner - y_c)**2 - radius**2
        
        # Solving the quadratic equation A*t^2 + B*t + C = 0
        discriminant = B**2 - 4 * A * C
        
        if discriminant < 0:
            raise ValueError("The points do not form a line that intersects the circle.")
        t1 = (-B + np.sqrt(discriminant)) / (2 * A)
        t2 = (-B - np.sqrt(discriminant)) / (2 * A)
        
        # Choose the positive t which corresponds to the point on the circle
        t = max(t1, t2)
        
        # Calculate the interpolated point on the circle
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
        if len(points) < 2:
            print("Need at least two points to calculate virtual points.")
            return

        virtual_points = []

        # Calculate two closest points
        distances = np.linalg.norm(points, axis=1)
        closest_indices = np.argsort(distances)


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

    def euclidean_distance_origin(self, point):
        return math.sqrt(point[0]**2 + point[1]**2)

    def pure_pursuit(self, centerline_points, L=3, WHEELBASE_LEN=1):
        # TODO: Pure pursuit returns steering angles that are biased to be >1
        # perfect straight line output: 0.0/1.57
        # real-world almost straight line output (img04): 1.07/1.57
        # Currently patched by normalizing steering angle output to ignore values < 1 in control.py

        """"
        Input: Centerline waypoints
        Output: Select target point, arc_curvature to reach target point

        L: Lookahead distance, distance of target point from car
        WHEELBASE: Distance between front and rear wheels

        Small L leads to more aggresive maneuvering,
        Large L leads to smooth trajectory, but larger tracking errors, infeasible paths
        L can be assigned as a function of vehicle speed
        """

        # Pick target point by finding the closest point outside L 

        # if innerpoints is empty we can interpolate between min outer and (0,0)
        # if outerpoints is empty we lower L and try again
        while True:
            outerpoints = []
            innerpoints = []
            innerpoints.append((0,0))
            for point in centerline_points:
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
        # find target point on L circle by interpolating between inner and outer
        target_point = self.interpolate_to_circle((0,0), L, max_inner, min_outer)
        tx, ty = target_point[0], target_point[1]

        # Steering angle should be proportional to target arc curvature
        # pure_pursuit formula has reversed x and y axes so we use tx instead of ty.
        arc_curvature = math.degrees((2*tx) / math.pow(L, 2.0))
        steering_angle = math.atan(arc_curvature * WHEELBASE_LEN)
        
        return steering_angle


def main(args=None):
    rclpy.init(args=args)

    planning = Planning()
    rclpy.spin(planning) # Node is kept alive until killed with ctrl+c
    planning.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()