# OAK FFC 4P Driver Package for ROS2
This package wraps the driver of OAK FFC 4P which is connected to 4 fisheye cameras. It allows to set the configs of the cam (`rgb`, `resolution` ...), calibrate the sharpness/focus of the cams (`sharpness_calibration_mode`), assemble the 4 fisheye images and publish them on `/oak_ffc_4p_driver/image_raw` using `image_transport`. It is inspired from this [repo](https://github.com/D2SLAM-Fusion/driver-oak_ffc_4p_ros/).

It has been tested on **Ubuntu 22.04** and **ROS2 Humble**.

## Getting Started
### Install Dependencies 
Make sure that `OpenCV` is installed (tested with version 4.8.0, specifically for the Orin), and the ROS2 packages `image_transport` and `cv_bridge`. 
``` shell script
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3-opencv libopencv-dev
sudo apt install ros-humble-image-transport 
sudo apt install ros-humble-image-transport-plugins
sudo apt install ros-humble-cv-bridge
```

### OpenCV installation
Usually OpenCV 4.8.0 comes with jetpack 6.2.1, but if it doesn't/you're using your host pc, you need to install it from source:

``` shell script
# Remove existing OpenCV if present (optional but recommended to avoid conflicts)
sudo apt remove libopencv-dev python3-opencv

# Download Sources
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.8.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.8.0.zip
unzip opencv.zip
unzip opencv_contrib.zip

# Create Build Directory
cd opencv-4.8.0
mkdir build
cd build

# Configure with CMake (Standard configuration for Jetson/Ubuntu 22.04)
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.8.0/modules \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D WITH_CUBLAS=ON \
-D WITH_TBB=ON \
-D WITH_V4L=ON \
-D WITH_GSTREAMER=ON \
-D WITH_GTK=ON \
-D WITH_OPENGL=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_ENABLE_NONFREE=ON \
-D BUILD_EXAMPLES=OFF ..

# Build and Install (this may take some time)
make -j$(nproc)
sudo make install
sudo ldconfig
```

### `cv_bridge` installation
Since we installed a custom version of OpenCV, the standard ros-humble-cv-bridge (which links against OpenCV 4.5.4) will not work. We must rebuild it from source in your workspace.

``` script shell
# Create a workspace for cv_bridge if you don't have one, or use your main workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the vision_opencv repository (Humble branch)
git clone -b humble https://github.com/ros-perception/vision_opencv.git

# Build cv_bridge forcing it to find the new OpenCV 4.8.0
cd ~/ros2_ws
colcon build --symlink-install --packages-select cv_bridge --cmake-args -DOpenCV_DIR=/usr/local/lib/cmake/opencv4

# Source the workspace
source install/setup.bash
```

### Install DepthAI
Download depth core AI, tar, build and install:
**Note:** We build it as a shared library (`BUILD_SHARED_LIBS=ON`) in Release mode to ensure compatibility with the ROS 2 driver and avoid linkage errors.
``` shell script
cd ~/Downloads
wget https://github.com/luxonis/depthai-core/releases/download/v3.2.1/depthai-core-v3.2.1.tar.gz 
tar -xvf depthai-core-v3.2.1.tar.gz
rm depthai-core-v3.2.1.tar.gz
cd depthai-core-v3.2.1
mkdir build
cd build
# IMPORTANT: Build as SHARED libs to avoid CMake linkage errors later
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
make -j$(nproc)
sudo make install 
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
The images will also display the latency up to the display point in the code and the fps to check if it matches the fps set in the config file `/config/cam_donfig.yaml`.

### Publish each Cam Individually
To publish each cam individually on a different topic in place of the assembled image, change the config file `/config/cam_config.yaml` and set `sharpness_calibration_mode` to `false` and `publish_cams_individually` to `true` then launch:
``` shell script
cd ~/ros2_ws/src
source install/setup.bash
ros2 launch oak_ffc_4p_driver_ros2 oak_ffc_4p_driver.launch.py
```

## Test Latency
To test the latency of the whole pipeline (only for the assembled image mode), change the config file `/config/cam_config.yaml` and set `sharpness_calibration_mode` to `false`, `image_info` to `true` and `publish_cams_individually` to `false`. Then in one terminal run:
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

