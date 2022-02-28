# dron_control_node
dron_control_node
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

![image](https://user-images.githubusercontent.com/45313904/156055261-3e544232-645b-46f3-a5bf-8a51c7afede9.png)

