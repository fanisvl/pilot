sensor_msgs/Image left_image
sensor_msgs/Image right_image

# per camera
uint16 resolution_x
uint16 resolution_y

# use these to calculate pixel_size
float32 sensor_width
float32 sensor_height

# lens
uint16 baseline # in mm
float32 focal_length # in mm

# in degrees
uint16 angle_of_view_horizontal 
uint16 angle_of_view_vertical
uint16 angle_of_view_diagonal

float32 pixel_size # in mm
