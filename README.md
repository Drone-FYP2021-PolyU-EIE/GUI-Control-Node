# dron_control_node
dron_control_node
## [riskly]Setup Workspace(catkin make)
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
## install
```bash
sudo apt-get install python-tk
sudo apt-get install python3-tk
pip3 install netifaces
cd <catkin_ws>
git clone https://github.com/Drone-FYP2021-PolyU-EIE/dron_control_node.git
```
