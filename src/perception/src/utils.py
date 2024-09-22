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
        plt.scatter(cone.x, cone.y, c='orange')
        plt.text(cone.x, cone.y+3, f'{cone.id}: ({cone.x:.2f}, {cone.y:.2f})', fontsize=9, ha='right')

    plt.scatter(0, 0, c='red', marker='x')
    plt.xlabel("X (cm)")
    plt.ylabel("Z (cm)")
    plt.title("Cone Estimation Map")
    plt.legend()
    plt.grid(True)
    plt.show()

def debug_pipeline(left_frame, right_frame, bbox_left, bbox_right, 
                        keypoints_left, keypoints_right, good_matches):
    
    # Visualize bounding boxes
    left_frame_copy = left_frame.copy()
    right_frame_copy = right_frame.copy()
    
    cv2.rectangle(left_frame_copy, (bbox_left[0], bbox_left[1]), 
                (bbox_left[0] + bbox_left[2], bbox_left[1] + bbox_left[3]), (0, 255, 0), 2)
    cv2.rectangle(right_frame_copy, (bbox_right[0], bbox_right[1]), 
                (bbox_right[0] + bbox_right[2], bbox_right[1] + bbox_right[3]), (0, 255, 0), 2)

    # Visualize features
    left_with_features = cv2.drawKeypoints(left_frame_copy, keypoints_left, None, 
                                        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    right_with_features = cv2.drawKeypoints(right_frame_copy, keypoints_right, None, 
                                            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    combined_frame = cv2.hconcat([left_with_features, right_with_features])
    cv2.imshow("BBoxes and Features", combined_frame)
    cv2.waitKey(0)

    # feature matches
    matches_img = cv2.drawMatches(left_frame, keypoints_left, right_frame, keypoints_right, 
                                    good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imshow("SIFT Feature Matches", matches_img)
    cv2.waitKey(0)

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

    print("\nBenchmark Information:")
    for name, times in benchmark_times.items():
        if times:
            print_avg_time(name.capitalize(), times)

    for times in benchmark_times.values():
        times.clear()
