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
source /opt/ros/melodic/setup.bash
mkdir -p ~/aerov_grasp_ws/src
```

