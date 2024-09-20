import matplotlib.pyplot as plt
import json
import numpy as np
from scipy.spatial import Delaunay
import math

def find_closest_pair_midpoint(estimates):
    """
    Find the closest cone estimate pair to (0,0)
    Return the midpoint
    """
    # Calculate all distances from (0,0)
    distances = [(np.sqrt(estimates[0]**2 + estimate[1]**2), estimate) for estimate in estimates]
    distances.sort(key=lambda d: d[0])
    closest_pair = [distances[0][1], distances[1][1]]
    x1,y1 = closest_pair[0]
    x2,y2 = closest_pair[1]
    midpoint = ((x1+x2)/2, (y1+y2)/2)
    return midpoint

def remove_duplicate_estimates(estimates, threshold=0.5):
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

def plan(estimates):

    with open(estimates, "r") as file:
        cone_estimates = json.load(file)

    estimates = [(cone['X'], cone['Y']) for cone in cone_estimates.values() if cone['X'] > -5]

    estimates = remove_duplicate_estimates(estimates, 3)
    estimates = [(28.47, 112.4), (-17.17, 99.25), (21.38, 160.38)]
        
    path_points = []
    points_array = np.array(estimates)

    if len(estimates) < 2:
        return 
    
    elif len(estimates) == 2:
        #TODO: Check if they're actually a valid pair based on distance
        path_points.append(find_closest_pair_midpoint(estimates))

    else:
        # TODO: Is this required? Maybe just simplify and find closest pair
        triangulation = Delaunay(points_array)
        for simplex in triangulation.simplices:
            for i in range(3):
                # get the edge points
                v1 = points_array[simplex[i]]
                v2 = points_array[simplex[(i + 1) % 3]]
                midpoint = tuple((v1 + v2) / 2)
                # Filter for horizontally aligned edges
                if abs(v1[0] - v2[0]) > abs(v1[1] - v2[1]) and midpoint not in path_points:  # Edge is more horizontal than vertical
                    print(f"Added midpoint: {midpoint} for v1: {v1}, v2: {v2}")
                    path_points.append(midpoint)

    closest_path_point = min(path_points, key=lambda p: math.sqrt(p[0]**2 + p[1]**2))
    trajectory_points = generate_trajectory((0,0), (closest_path_point), samples=10)
    

    fig, ax = plt.subplots()
    # Plot cone estimates on the first subplot
    ax.set_title(f"{estimates}")
    ax.scatter(0, 0, color='orange', label='Camera')
    ax.annotate("(0,0)", (0, 0))
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Depth-axis")
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')

    for x,y in estimates:
        ax.scatter(x, y, color="gray")

    for x,y in path_points:
        plt.scatter(x,y, color='orange')

    for x, y in trajectory_points:
        ax.scatter(x, y, color='blue', s=5) 

    plt.show()

if __name__ == '__main__':
    plan("/home/fanis/repos/planning/data/json_estimates/run1/12.json")