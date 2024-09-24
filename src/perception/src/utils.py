import matplotlib.pyplot as plt
import cv2
import numpy as np
import yaml
import time
from functools import wraps

def get_cam_info(config_file):
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    left_intr = np.array(config['camera_intrinsics']['left']['intrinsic_matrix'], dtype=np.float32)
    left_dist = np.array(config['camera_intrinsics']['left']['distortion_coefficients'], dtype=np.float32)
    right_intr = np.array(config['camera_intrinsics']['right']['intrinsic_matrix'], dtype=np.float32)
    right_dist = np.array(config['camera_intrinsics']['right']['distortion_coefficients'], dtype=np.float32)
    R = np.array(config['camera_extrinsics']['rotation_matrix'], dtype=np.float32)
    T = np.array(config['camera_extrinsics']['translation_vector'], dtype=np.float32)

    return {
        'LEFT_INTR': left_intr,
        'LEFT_DIST': left_dist,
        'RIGHT_INTR': right_intr,
        'RIGHT_DIST': right_dist,
        'R': R,
        'T': T
    }

def plot_cone_estimates(cone_estimates_msg):
    """Plot cone estimates msg"""
    plt.figure(figsize=(6, 6))
    for cone in cone_estimates_msg.cones:
        plt.scatter(cone.x, cone.y, c='orange', s=80, edgecolor='black')
        plt.text(cone.x+3, cone.y+3, f'{cone.id}: ({cone.x:.2f}, {cone.y:.2f})', fontsize=9, ha='right')

    plt.scatter(0, 0, c='black', marker='x', label='Vehicle')
    plt.xlabel("X (cm)")
    plt.ylabel("Z (cm)")
    plt.title("Stereo | Cone Estimation Map")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def debug_pipeline(left_frame, right_frame, bbox_left, bbox_right, 
                   keypoints_left, keypoints_right, good_matches):
    
    # Draw matches
    matches_img = cv2.drawMatches(left_frame, keypoints_left, right_frame, keypoints_right, 
                                  good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    
    # Get dimensions of the matches image
    h, w = matches_img.shape[:2]
    
    # Draw bounding box on left image
    cv2.rectangle(matches_img, (bbox_left[0], bbox_left[1]), 
                  (bbox_left[0] + bbox_left[2], bbox_left[1] + bbox_left[3]), (0, 255, 0), 2)
    
    # Draw bounding box on right image (offset by the width of the left image)
    cv2.rectangle(matches_img, (w // 2 + bbox_right[0], bbox_right[1]), 
                  (w // 2 + bbox_right[0] + bbox_right[2], bbox_right[1] + bbox_right[3]), (0, 255, 0), 2)

    # Display the image
    cv2.imshow("Matches and Bounding Boxes", matches_img)
    cv2.waitKey(0)
    
    # Save the image
    cv2.imwrite('matches_and_bboxes.png', matches_img)

benchmark_times = {
    'detect': [],
    'propagation': [],
    'sift': [],
    'total': []
}

def benchmark(time_list_name):
    def decorator(func):
        @wraps(func)
        def wrapper(instance, *args, **kwargs):
            start_time = time.time()
            result = func(instance, *args, **kwargs)
            elapsed_time = (time.time() - start_time) * 1000 #ms
            benchmark_times[time_list_name].append(elapsed_time)
            return result
        return wrapper
    return decorator
    
def print_benchmark_info(_):
    def print_avg_time(name, times):
        if times:
            avg_time = np.mean(times)
            print(f"Avg. {name} Time: {avg_time:.0f} ms ({1000/avg_time:.2f} Hz)")
    if benchmark_times['detect']:
        print("\nBenchmark Information:")
    for name, times in benchmark_times.items():
        if times:
            print_avg_time(name.capitalize(), times)

    for times in benchmark_times.values():
        times.clear()
