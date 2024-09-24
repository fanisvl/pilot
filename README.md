This project implements a simple pipeline inspired by the Formula Student autonomous racing competition, \
based on **ROS** that includes **perception**, **planning** and **control** modules.

### Perception

#### Stereo
> Left Frame, Right Frame
> Cone Detection with Mobilenet
2 design options were explored:
a) Do cone detection on left frame only and then do bounding box propagation using
   either a tracker like CSRT (accurate but slow) to find the bbox in the right image or
   by detecting SIFT Features and projecting them to the right frame, knowing the 
   camera intrinsics, then drawing a bounding box around the projected features.

b) Do cone detection on both frames

> SIFT Feature Extraction & Feature matching between cone pairs on left and right frame.
> Triangulation using the stereo camera intrinsics

#### Mono Pipeline
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
