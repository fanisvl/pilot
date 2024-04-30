Autopilot Demo  

1) Install package dependencies with rosdep (inside autopilot directory)

```
#For first time running rosdep in your system you have to run    
sudo rosdep init
rosdep update
#If you have run rosdep before do not run the above commmands
rosdep install --from-paths src -y --ignore-src
```
2) Build required packages with colcon (inside autopilot directory)
```
colcon build --packages-up-to vision operations
```
3) Source bash from install directory (inside autopilot directory)\
```
source ./install/setup.bash
```
4) Launch .launch.py file with ros
```
ros2 launch vision autopilot.launch.py
```
5) Open rqt (in another terminal)
```
rqt
```
Inside rqt window from the menu choose Plugins/Introspection/Node Graph\
Output:

![image](https://github.com/AUEB-CS-Autonomous-Racing/autopilot/assets/80046016/0b8daff1-83f5-4c5e-8c6a-b56e64af75fe)

6) (in another terminal)
```
ros2 param list
#select a parameter you like for example
ros2 param get /system PI
```
Output:

![image](https://github.com/AUEB-CS-Autonomous-Racing/autopilot/assets/80046016/6d274091-4ea2-4dac-a1fb-080b1df69840)

7) Terminate execution correctly:
```
#Press ctrl-c on the terminal you launched the .launch.py file
```
