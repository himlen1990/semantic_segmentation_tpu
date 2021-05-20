# semantic_segmentation_tpu
ros package of the edge tpu semantic segmentation example
---
## Environment
---
- Ubuntu18.04 and ROS Melodic

## Coral USB Accelerator Installation
(https://coral.ai/docs/accelerator/get-started/#requirements)
```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install libedgetpu1-std
```

## Setup ROS Environment and Check if the Coral EdgeTPU work properly

```bash
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install python3-opencv
sudo apt-get install ros-melodic-catkin
sudo apt-get install ros-melodic-uvc-camera
source /opt/ros/melodic/setup.bash
mkdir -p ~/aerov_grasp_ws/src
cd ~/aerov_grasp_ws/src
git clone https://github.com/himlen1990/semantic_segmentation_tpu.git
wstool init
wstool merge semantic_segmentation_tpu/.rosinstall
wstool update
rosdep install --from-paths . --ignore-src -y -r
cd ~/aerov_grasp_ws
catkin init
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build
```

## Run Demo

```bash
source ~/aerov_grasp_ws/devel/setup.bash
roslaunch semantic_segmentation_tpu semantic_segmentation.launch
```