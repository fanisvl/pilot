This project implements a simple pipeline inspired by the Formula Student autonomous racing competition, \
it's based on **ROS** and it includes **perception**, **planning** and **control** modules.

[Stereo Pipeline](#stereo-pipeline) \
[Mono Pipeline](#monocular-pipeline) \
[Planning](#planning) \
[Control](#control) \
[Hardware](#hardware) \
[References](#references)


# Perception

## Stereo Pipeline
The stereo pipeline utilizes the epipolar constraints of the stereo camera to estimate the position of the cones,
thus the results depend on the quality of stereo calibration. 
In order to estimate the position of the cones relative to the vehicle, we first have to detect the cones in both frames.
We can then extract and match features between corresponding cones, and finally use triangulation to estimate the location of each point
according to the vehicle world frame.

Two approaches were explored:
### 1A. Cone Detection on Both Frames
   This is the simplest approach. \
   An **SSD-Mobilenet** network was trained on a custom dataset of ~50 images (augmented to ~100). \
   The model can achieve inference at ~10Hz for both frames on the Jetson Nano, with TensorRT acceleration.
   Since the IMX219-83 stereo camera does not have hardware synchronization, the TimeSyncrhonizer filter is used.
   Although functional for test pursposes, due to the low amount of data and the low amount of variance within the data (the cones were placed at a max of ~2m from the camera due to space constraints),
   the model does not generalize well. It's prediction confidence drops significantly as the distance of cones increases. A larger and more diverse dataset should be collected.
     
### 1B. Cone detection on left frame, and bounding box propagation to the right frame.
   Bounding box propagation can be achieved by:
   
   a. **Using a model like YOLOV8** _by ultralytics_, we can detect _pre-determined keypoints_[1] that define the cone.
        We can then utilize the epipolar constraints of our stereo camera to project the keypoints to the right
        frame, and create a new bounding box around them. While this approach was not chosen for the stereo pipeline,
        it was used in the [Monocular Pipeline](#monocular-pipeline).
       
   b. **Using an object tracker**, like **CSRT** to track the cone from the left frame to the right frame. \
       CSRT was able to track the cone bounding box to the right frame very accurately, but
       it's performance wasn't sufficient for real-time at ~4Hz on the Jetson Nano. \
       The **MOSSE** tracker was also evaluated, and while it was faster, it's accuracy wasn't sufficient. 
       It's important that bounding boxes are as accurate as possible, to avoid detecting background features
       in the next steps of the pipeline.

Cone detection on both frames was chosen for robustness and simplicity.

### 2. SIFT Feature Extraction & Feature matching between cone pairs in left and right frame
   SIFT Features were extracted for each bounding box in both frames, and then matched in order to obtain points that we can
   later use for triangulation. Although SIFT is accurate, it presents a bottleneck performing at ~7Hz on the Jetson Nano.
   Faster feature extraction methods like ORB and BRIEF should be explored.
   This process was parallelized for additional performance.
   <img src="https://github.com/user-attachments/assets/e63de212-2284-4f57-840d-143211ca656f" width="1000">


### 3. Triangulation using epipolar constraints.
The matched SIFT features for each cone pair are triangulated and the mean is the estimated cone position.
### Results (in cm)
To evaluate the results cones were placed at pre-measured locations and the stereo pipeline was executed for 300 iterations.

<img src="https://github.com/user-attachments/assets/c3cfe745-7417-4292-9954-2045746a0990" width="800"> \
After 300 iterations, a mean euclidean distance error of ~12cm is observed, mostly attributed to the Z-axis.
The mean X axis error is ~0.7cm and the mean Z axis error is ~12cm. \
This could be due to the fact that the Z-axis distances that we're trying to estimate are greater,
or it could be due biased data in the stereo calibration process.

A more diverse set of test examples should be considered in order to make any conclusions and thus a better ground truth data collection pipeline should be established. 
Ground truth data can be obtained using LiDAR measurments, or a simulation.

Less SIFT features could be correlated with less accurate estimations, as they are more prone to calibration inaccuracies and background features, 
but again more examples would have to be gathered to make that conclusion. \
eg. bottom left cone
<img src="https://github.com/user-attachments/assets/81dadaea-1547-49aa-8662-9f1460072657" width="1000">

## Monocular Pipeline
The information contained in one camera frame is not enough to estimate the position of the cone relative to the camera. However knowing the cone dimensions we can find a solution. \
The [Perspective n-point (PnP)](https://en.wikipedia.org/wiki/Perspective-n-Point) problem is defined as
> "Estimating the pose of a **calibrated camera** given a set of **n 3D points in the world frame** and their **corresponding 2D projections** in the image".

So given a calibrated camera, and by assuming that the world frame is at the base of the cone
we can use the cone model to calculate a set of pre-determined 3D points relative to the base.
Then we can use keypoint regression to detect the same corresponding 2D keypoints in our 2d image.
The solution of PnP contains a translation vector that calculates the pose of the camera relative to 
the world frame. We set the world frame as the base of the cone and so the translation vector
corresponds to the location of the cone relative to the camera.


<img src="https://github.com/user-attachments/assets/09043185-6169-49ae-a02e-dc264210cad9" width="200"> \
*AMZ Racing* [[1]](https://arxiv.org/abs/1905.05150) 

To detect the required keypoints a YOLOv8-nano model was pre-trained on the [FSOCO dataset](https://www.fsoco-dataset.com/overview/) (~600 images)
which represents WEMAS cones (used in FS-Germany), although it performed well on that type of cone it didn't generalize to mine, 
so it was later fine tuned to my cones on ~50 images, augmented to 100. An inference example can be found for both cone types below.

### Results
<img src="https://github.com/user-attachments/assets/576931ba-6bdb-4c15-ae1b-844753466909" width="800"> \
<img src="https://github.com/user-attachments/assets/d65215d2-fb7a-4734-a89b-758eed3a1b35" width="400"> \
We can observe that the left cone which has more compressed keypoints is estimated as further away,
while the other cones with expanded keypoints are perceived as closer. \
Robust and accurate keypoint regression is essential for this method to be reliable.


Same model:
<img src="https://github.com/user-attachments/assets/478c2cf6-b3e1-4069-98db-338efe47217a" width="800"> \
<img src="https://github.com/user-attachments/assets/c3d5f4ca-60cb-4667-8cf6-1081d43371d0" width="400"> 

### Simulation
Object detection was also tested in the Gazebo simulator using this open source project by a FS team: [eufs_sim](https://gitlab.com/eufs/eufs_sim)

<img src="https://github.com/user-attachments/assets/52d30007-f2fc-43c8-b0a8-9cff093bbb33" width="600"> 


## Planning
_Note: Planning and Control are still very much under development, and they're currently much simpler than perception.
Emphasis was first given to perception simply because [Garbage in, garbage Out](https://en.wikipedia.org/wiki/Garbage_in,_garbage_out)_ 

Having a set of cone estimates, Delaunay Triangulation is performed to find midpoints between the cone boundary pairs.
Points that represent a linear trajectory between midpoints are generated
and published for the Pure pusuit algorithm in the control module.
<img src="https://github.com/user-attachments/assets/00cb95b2-7a16-4a48-8b5b-f542802cc04d" width="420"> 
<img src="https://github.com/user-attachments/assets/d12d0292-5f57-4056-98f1-09db5de493e4" width="400"> 

Clustering was also tested to remove background cone detections and noisy estimates.
<img src="https://github.com/user-attachments/assets/51ac47a8-9135-4e4a-8f61-1ab79e3b35d1" width="400">

_Utilizing SLAM (eg. GraphSLAM) would greatly improve the planning module. [ChalmersFS](https://arxiv.org/pdf/2210.10933)_

## Control
Given a set of trajectory points from planning, the control module uses the [Pure Pursuit algorithm](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) to pick a target point, and calculate the steering angle required.
Pure Pursuit has only one parameter L (Lookahead Distance), which defines the distance of our target waypoint.
The target point is calculated by interpolating between trajectory points inside and outside the circle of radius L, as can be seen in the image below.
<img src="https://github.com/user-attachments/assets/1910d202-9851-481a-bcb6-d359627af8bf" width="400"> \
[Penn Engineering](https://www.youtube.com/watch?v=x9s8J4ucgO0)


If the lookahead distance is set too small, it can cause aggressive steering as the vehicle constantly adjusts to reach the nearby target point,
and if the distance is too large, the path may be smoother, but it can result in trajectory tracking errors.

If our environment follows a set of constraints, we can get an idea for what our lookahead parameter should be set to.
For example [FS-Germany](https://www.formulastudent.de/fileadmin/user_upload/all/2024/important_docs/FSG24_Competition_Handbook_v1.2.pdf) 
has these conditions for skidpad with tight turns, a smaller lookahead will be required. \
<img src="https://github.com/user-attachments/assets/a8cb05d3-bfdd-4f96-9d8b-65e9c83e2a77" width="200"> \

The steering angle is normalized to [-1,1] and translated to the corresponding
PWM value to be sent to steering via the PCA9685 driver.
<img src="https://github.com/user-attachments/assets/cec1f981-d24d-4803-918f-8af691a10dac" width="400"> 

_Controllers like PID and MPC should be explored in the future._

## Hardware
<img src="https://github.com/user-attachments/assets/9d8afad6-7312-4c58-9f20-aa7e0c0533fa" width="800"> 
<img src="https://github.com/user-attachments/assets/d9bb99e3-5133-4e74-82e4-f53fb80faec9" width="600"> 

The RC car is a Wltoys 144001. 
The ESC (Electronic Speed Controller) and steering servo are connected to a PCA9685 driver, which communicates using PWM signals to control the vehicle. 
The PCA9685 is connected to a Jetson Nano, enabling programmatic control of the car. 
The IMX219-83, is used for stereo vision.
The top section of the car was 3D-printed. [(thingiverse)](https://www.thingiverse.com/thing:2566276)

## References 
AMZ Racing [[1]](https://arxiv.org/abs/1905.05150), [[2]](https://arxiv.org/pdf/1902.02394), \
Chalmers FS [[1]](https://arxiv.org/pdf/2210.10933), \
KA-Racing [[1]](https://arxiv.org/pdf/2210.10933), [[2]](https://arxiv.org/pdf/2010.02828) \
[FSOCO-Dataset](https://www.fsoco-dataset.com/overview/)
