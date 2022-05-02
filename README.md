# dron_control_node
this is a GUI for youe to control the px4 system drone on ROS
## Node ROS Topics
### Onboard dron_control_node
|Type|Topic|Data_type|
|:---:|:---:|:---:|
|Publisher| /drone/nagvation/pos| [PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)|
|Publisher| /auto_mode/status| [BoolStamped](http://docs.ros.org/en/indigo/api/jsk_recognition_msgs/html/msg/BoolStamped.html)|
|Publisher| /mavros/setpoint_position/local| [PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)|
|Publisher| /drone/current/control_status| Custom Message [GetDroneState](https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node/tree/master/drone_control_msgs)|
|Publisher| /servo/angle| [AllServoAngle](https://github.com/Drone-FYP2021-PolyU-EIE/ROS_CircuitPython_ServoKit)|
|Subscriber| /drone/input_posistion/pose| [PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)|
|Subscriber| /mavros/local_position/pose| [PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)|
|Subscriber| /mavros/state| [State](http://docs.ros.org/en/melodic/api/mavros_msgs/html/msg/State.html)|
|Subscriber| /drone/set/control_status| Custom Message [SetDroneState](https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node/tree/master/drone_control_msgs)|
### Offboard offboard/dron_control_node
|Type|Topic|Data_type|
|:---:|:---:|:---:|
|Subscriber| /drone/current/control_status| Custom Message [GetDroneState](https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node/tree/master/drone_control_msgs)|
|Publisher| /drone/set/control_status| Custom Message [SetDroneState](https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node/tree/master/drone_control_msgs)|
### Input of the Node
`/mavros/local_position/pose`, `PoseStamped` get the currnte postion of the drone               
`/drone/input_postion/pose`, `PoseStamped` get the pos from the vision nagvation system                         
`/detection_result/image`, `Image` (porposted) get the detection result and show in the GUI
                    
### Output of the Node  
`/mavros/setpoint_position/local`, `PoseStamped` (>30Hz) pos pass to the mavros to excite       
`/auto_mode/status`, `BoolStamped` let the navgation program know whether start the program or stop it                       
`/drone/current/control_mode`, `String` (>30Hz will chagne to 20hz later) send the current control mode out (`"auto"` or `"manual"`)                       
`/drone/nagvation/pos`, `PoseStamped` (>30Hz) pos pass to the vision nagvation to excite

# For ubuntu 18.04
## Prerequisites
* Python 3
* ROS 1
* mavros
* gazebo(optional)
## install
```bash
sudo apt-get install -y python-tk python3-tk python3-tk-dbg python3-pil python3-pil.imagetk
pip3 install netifaces Pillow imutils
sudo apt-get install -y ros-$ROS_DISTRO-jsk-pcl-ros ros-$ROS_DISTRO-jsk-rviz-plugins ros-$ROS_DISTRO-ros-numpy
$cd <catkin_ws>/src #cd to your catkin workspace src here
git clone https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node.git
```
## Setup Workspace(catkin make)
```bash
$cd to your catkin ws frist
source devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
# for x86 CPU only
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
# jetson or ARM devices Only 
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
```
after you build the package in the catkin ws, now the python and python3 system path may have some problem    
(Told you already this is riskly)      
the program will try to auto fix this issuess  
but if it does not work do the following
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

### Setup ROS for Offboard Control
#### On ROS Master
add the following into the `.bashrc`                        
```bash
ip=$(hostname -I | cut -f1 -d' ')
echo 'ROS Master IP:'$ip
export ROS_MASTER_URI='http://'$ip':11311'
export ROS_HOSTNAME=$ip
export ROS_IP=$ip
```
#### On ROS Slave
add the following into the `.bashrc`   
replace `192.168.1.2` with your rosmaster ip    
```bash
master_ip=$('192.168.1.2')
ip=$(hostname -I | cut -f1 -d' ')
echo 'ROS Master IP:'$master_ip
echo 'ROS Local IP:'$ip
export ROS_MASTER_URI='http://'$master_ip':11311'
export ROS_HOSTNAME=$ip
export ROS_IP=$master_ip
```
## Run the ROS node
### Onboard(on the drone)
```bash
roslaunch drone_control_node exampleOnboardHasGripper.launch
```
### Offboard(Ground Station)
```bash
roslaunch drone_control_node exampleOffboardHasGripper.launch
```

## Procedure of using control node gui
1. Click arm button to arm the drone
2. Enter the desired position and angle
3. Click Offboard button to change the mode

## Update Blog
2022-3-7-01:21 Upload the new version of `control_node.py` aim to let the navgation program know whether start the program or stop it <br />
2022-3-11-02:15 Update `control_node.py`, change the message type of `/auto_mode/status` to BoolStamped as the navigation node message filter time synchronizer require header. The BoolStamped message type require `jsk_recognition_msgs` which can install by  <br />                        
`sudo apt-get install ros-noetic-jsk-pcl-ros ros-noetic-jsk-rviz-plugins ros-noetic-ros-numpy`              
2022-3-11-04:xx upadate GUI switch to auto mode also set position once              
2022-3-13: fix issues in take local rotation related          
2022-3-15: added colorfull button

## Screen
![image](https://user-images.githubusercontent.com/45313904/160270885-29227eef-3b75-47e8-a2c8-ec4a1acfe1b0.png)
![image](https://user-images.githubusercontent.com/45313904/160272915-c6def331-d825-4f6c-af4b-eeddae91650c.png)
![image](https://user-images.githubusercontent.com/45313904/161622382-4498c14f-d986-4040-a0fc-77b88f0c1557.png)

## Demo Video
https://youtu.be/6EUKs0d83WY
![image](https://user-images.githubusercontent.com/45313904/164148562-26323d6c-7d7a-4c17-b76c-3a00af24523f.png)
