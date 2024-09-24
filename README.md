This project implements a simple pipeline inspired by the Formula Student autonomous racing competition, \
based on **ROS** that includes **perception**, **planning** and **control** modules.

# Perception

## Stereo Pipeline
The stereo pipeline utilizes the epipolar constraints of the stereo camera to estimate the position of the cones.
Thus, the results depend on the accuracy of stereo calibration. 
In order to estimate the position of the cones relative to the vehicle, we first have to detect the cones in both frames.
We can then extract and match features between corresponding cones, and finally use triangulation to estimate the location of each point
according to the vehicle world frame. \
Parallelization was utilized for SIFT Feature extraction, feature matching and triangulation.

Two approaches were explored:
### 1A. Cone Detection on Both Frames
   This is the simplest approach. \
   An **SSD-Mobilenet** network was trained on a custom dataset of ~50 images (augmented to ~100). \
   The model can achieve inference at ~10Hz for both frames on the Jetson Nano, with TensorRT acceleration.
   Although functional for test pursposes, due to the low amount of data and the low amount of variance within the data (the cones were placed at a max of ~2m from the camera due to space constraints),
   the model does not generalize well. It's prediction confidence drops significantly as the distance of cones grows. A more diverse dataset should be collected.
     
### 1B. Cone detection on left frame, and bounding box propagation to the right frame. \
   Bounding box propagation can be achieved by:
   
   a. **Using a model like YOLOV8** _by ultralytics_, we can detect _pre-determined keypoints_[1] that define the cone.
        We can then utilize the epipolar constraints of our stereo camera to project the keypoints to the right
        frame, and create a new bounding box around them. Due to a low amount of data, keypoint regression was not robust enough and this approach was not chosen for the stereo pipeline. \
        However, this approach was used for the [Monocular Pipeline](#monocular-pipeline). \
       <img src="https://github.com/user-attachments/assets/09043185-6169-49ae-a02e-dc264210cad9" width="200"> 
       *AMZ Racing [[1]](https://arxiv.org/abs/1905.05150)*

       
   b. **Using an object tracker**, like **CSRT** to track the cone from the left frame to the right frame. \
       While CSRT was able to track the cone bounding box to the right frame very accurately,
       it's performance wasn't sufficient for real-time at ~2Hz on the Jetson Nano. \
       The **MOSSE** tracker was also evaluated, and while it was faster, it's accuracy wasn't sufficient. \
       It's important that bounding boxes are as accurate as possible, to avoid detecting background features
       in the next steps of the pipeline.


### 2. SIFT Feature Extraction & Feature matching between cone pairs on left and right frame.
   SIFT Features were extracted for each frame, and then matched in order to obtain points that we can
   later use for triangulation. Although SIFT is accurate, it presents a bottleneck performing at ~7Hz on the Jetson Nano.
   Faster feature extraction methods like ORB and BRIEF should be explored.
<img src="https://github.com/user-attachments/assets/8dcf6489-5ad1-47bc-9978-2513570d6220" width="1000">


### 3. Triangulation using epipolar constraints.
### Results (in cm)
<img src="https://github.com/user-attachments/assets/8ab8eeb0-61f6-4dbd-82c6-5eb94e9645fb" width="400"> 


## Monocular Pipeline
> Image
> Keypoint Regression on cones with YOLO
> 2d image keypoints are fed into PnP along with their 3d correpondences calculated with a world frame 
  at the base of the cone, as well as the camera intrinsics
  Knowing that the 3d points have a world frame that's at the base of the cone,
  the translation vector output of PnP will correspond to the position of the cone
  relative to the camera.

Limitations:
The cone detection model (YOLOv8-nano) was pre-trained on the FSOCO dataset,
which represents WEMAS cones (used in FS Germany).
The model was later fine tuned to my cones,
but due to the low amount of labeled data (~50 images, augmented to ~100)
the model isn't as robust as it should be.

### Planning
Deulaunay Triangulation to find mid points **TODO: Get a delaunay plot example**
Points that represent a linear trajectory between midpoints are generated
and published for the Pure pusuit algorithm.

### Control
**TODO: Add the map plot**
Pure Pursuit
Given a set of points that represent a trajectory, and a Lookahead Distance,
a target point is calculated and used to find the necessary steering angle.
The steering angle is normalized to [-1,1] and translated to the corresponding
PWM value to be sent to the steering servo.


# Hardware
RC car img
StereoCam - imu - > Jetson Nano - > PCA9685 PWM Driver - > ESC & Steering


\
references \
AMZ Racing [[1]](https://arxiv.org/abs/1905.05150), [[2]](https://arxiv.org/pdf/1902.02394), \
Chalmers FS [[1]](https://arxiv.org/pdf/2210.10933), \
KA-Racing [[1]](https://arxiv.org/pdf/2210.10933), [[2]](https://arxiv.org/pdf/2010.02828) \
[FSOCO-Dataset](https://www.fsoco-dataset.com/overview/)
