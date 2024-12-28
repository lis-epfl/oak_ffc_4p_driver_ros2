# OAK FFC 4P Driver Package for ROS2
This package wraps the driver of OAK FFC 4P which is connected to 4 fisheye cameras. It allows to set the configs of the cam (`rgb`, `resolution` ...), calibrate the sharpness/focus of the cams (`sharpness_calibration_mode`), assemble the 4 fisheye images and publish them on `/oak_ffc_4p_driver/image_raw` using `image_transport`. It is inspired from this [repo](https://github.com/D2SLAM-Fusion/driver-oak_ffc_4p_ros/).

It has been tested on **Ubuntu 22.04** and **ROS2 Humble**.

## Getting Started
### Install Dependencies 
Make sure that `OpenCV` is installed (tested with version 4.5.4), and the ROS2 packages `image_transport` and `cv_bridge`. 
``` shell script
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3-opencv libopencv-dev
sudo apt install ros-humble-image-transport 
sudo apt install ros-humble-image-transport-plugins
sudo apt install ros-humble-cv-bridge
```

### Install DepthAI
Download depth core AI, tar, build and install:
``` shell script
cd ~/Downloads
wget https://github.com/luxonis/depthai-core/releases/download/v2.29.0/depthai-core-v2.29.0.tar.gz 
tar -xvf depthai-core-v2.29.0.tar.gz
rm depthai-core-v2.29.0.tar.gz
cd depthai-core-v2.29.0
mkdir build
cd build
cmake ..
make -j$(nproc)
make install # this doesn’t copy the include and library files into /usr/local/, so we will do that next
cd /build/install/include
sudo cp -r * /usr/local/include/
cd ../lib/
sudo cp -r * /usr/local/lib
```

Then enable access to the device without needing to sudo, unplug the OAK FFC device, run the following commands, and then replug the device:
``` shell script
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then update the bootloader:
``` shell script
wget https://github.com/luxonis/depthai-python/archive/refs/tags/v2.29.0.0.tar.gz
tar -xvf depthai-python-2.29.0.0.tar.gz
cd depthai-python-2.29.0.0/examples/bootloader/
python3 flash_bootloader.py # choose 0 to flash the device if no other luxonis device is connected
```

### Create and Build Workspace
Create a ROS2 workspace if it doesn't exist and clone the repo inside the `src` folder of the workspace (or simply clone it inside an existing workspace), then build it: 
``` shell script
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/lis-epfl/oak_ffc_4p_driver_ros2
cd ..
colcon build --symlink-install --packages-select oak_ffc_4p_driver_ros2
```

### Check Cameras for Syncing
Modify `/config/cam_config.yaml` and set `syncing_master` to `"CAM_A"` and `sharpness_calibration_mode` to `true` and launch the node: 
``` shell script
cd ~/ros2_ws/src
source install/setup.bash
ros2 launch oak_ffc_4p_driver_ros2 oak_ffc_4p_driver.launch.py
```
If the images display on the screen, then `CAM_A` can be synced. Otherwise, the connection to the board from the camera is faulty and needs to be changed/fixed. Modify the `syncing_master` to `"CAM_B"` and run again the launch file, and then `"CAM_C"` and finally `"CAM_D"` to check all cameras. 
you can also set `image_info` to true in the config file `/config/cam_config.yaml` to see the timestamp of each image and make sure that they drift less then 1ms over time (it should be in the order of tens of microseconds).

### Calibrate the Cameras' Sharpness/Focus
Modify `/config/cam_donfig.yaml` and set `sharpness_calibration_mode` to `true`. Then run the launch file and rotate the lens of each camera until the clearness value reaches a max (the clearness it will go up and down as you rotate and you will see the focus get better/worse):
``` shell script
cd ~/ros2_ws/src
source install/setup.bash
ros2 launch oak_ffc_4p_driver_ros2 oak_ffc_4p_driver.launch.py
```
The images will also display the latency up to the display point in the code and the fps to check if it matches the fps set in the config file.

## Test Latency
To test the latency of the whole pipeline, change the config file `/config/cam_config.yaml` and set `sharpness_calibration_mode` to `false` and `image_info` to `true`. Then in one terminal run:
``` shell script
cd ~/ros2_ws/src
source install/setup.bash
ros2 launch oak_ffc_4p_driver_ros2 oak_ffc_4p_driver.launch.py
```
In another terminal run:
``` shell script
cd ~/ros2_ws/src
source install/setup.bash
ros2 run oak_ffc_4p_driver_ros2 image_latency_node
```

The total latency of the pipeline is the latency of the assembled image displayed in the terminal where we ran the driver launch file and the ROS2 transport latency displayed on the terminal where we ran `image_latency_node`. 

To see the published images you can use `rqt_image_view`:
``` shell script
ros2 run rqt_image_view rqt_image_view
```

