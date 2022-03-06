# dron_control_node
dron_control_node
## Node ROS Topics
### Input of the Node
`/mavros/local_position/pose`, `PoseStamped` get the currnte postion of the drone               
`/drone/input_postion/pose`, `PoseStamped` get the pos from the vision nagvation system         
`/drone/ground_control/pose`, `PoseStamped` (porposted) get the pos from the Ground Control manual input(for Ground Control use)                  
`/drone/set/control_mode`, `String` (porposted) set the control mode(for Ground Control use)                  
`/drone/set/px4_mode`, `String` (porposted) set the control mode(for Ground Control use)                      
### Output of the Node  
`/mavros/setpoint_position/local`, `PoseStamped` (>30Hz) pos pass to the mavros to excite       
`"/auto_mode/status"`, `Bool` let the navgation program know whether start the program or stop it
`/drone/current/control_mode`, `String` (>30Hz will chagne to 20hz later) send the current control mode out (`"auto"` or `"manual"`)                       
`/drone/nagvation/pos`, `PoseStamped` (porposted)(>20Hz) pos pass to the vision nagvation to excite                 
## {riskly}Setup Workspace(catkin make)
```bash
pip3 uninstall em
pip3 install empy
source devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
# x86
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
# jetson
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
```
after you build the package in the catkin ws, now the python and python3 system path are both fuck up       
(Told you already this is riskly)      
to fix this we must go the py and py3 to fix sys path ourselves                     
### opt1: do everytime after source deve
```python
#python3
python3 
>>> import sys
>>> sys.path
```
READ this very carefully
**for Python3** enusre the tf package's sys path is on top of the `'/opt/ros/melodic/lib/python2.7/dist-packages'`
sys.path.remove(python2_path)
sys.path.append(python2_path)
### other opt [read](https://stackoverflow.com/questions/31414041/how-to-prepend-a-path-to-sys-path-in-python?answertab=votes)



## install
```bash
sudo apt-get install python-tk
sudo apt-get install python3-tk
pip3 install netifaces
cd <catkin_ws>
git clone https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node.git
roslaunch dron_control_node lauchDroneControlNode.launch 
```

## Procedure of using control node gui
1. Click arm button to arm the drone
2. Enter the desired position and angle
3. Click Offboard button to change the mode

## Update Blog
2022-3-7-01:21 Upload the new version of `control_node.py` aim to let the navgation program know whether start the program or stop it


![image](https://user-images.githubusercontent.com/45313904/156055261-3e544232-645b-46f3-a5bf-8a51c7afede9.png)

![image](https://user-images.githubusercontent.com/45313904/156056672-ade1b6a5-aaa9-44fb-851d-aa2ca0b6d274.png)
