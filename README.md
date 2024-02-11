[![Build && Tests](https://github.com/aubino/viso_imu/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/aubino/viso_imu/actions/workflows/c-cpp.yml)

## VISO_IMU Project
    This project is a visual odometry implementation that try to solve translation ambiguity with imu datas. 
    It is supposed to work on jetson nano with an IMX219-83 camera and it's on board IMU 

### Dependencies 
    Opencv4 : 
    Eigen3  :
    Boost : 
    cmake (needed for build):


### Compilation instrunctions
````bash
git clone https://github.com/aubino/viso_imu.git
cd viso_imu
mkdir build
cd build 
cmake ..
make -j4
````  

### Execution instrunctions and option
- Executable :   viso_imu __the visual odometry calculator__ . 
- Options
    - -c  configuration file containing the camera intrinsics parameters, supposed to be obtained after calibration. *Absolute yaml or xml file path*
    - -p  IMU data sampling period


- Executable  viso_imu_test __The testing executable__. Uses known binocular (stereo) camera data to test the accuracy of the odometry  
    - Arguments  (in order)  
        - left_root  left images root __absolute directory__ containing left images of a stereo camera 
        - right_root  right imaes root __absolute directory__ containing right images of a stereo camera
        - config_file  configuration file __absolute path__ containong calibration data for both left and right images and Transformation between the two camera  frames  

- Executable  imu_test __the imu testing executable__  for testing the imu connection. No options

- Executable  simple_camera __A simple executable to test the camera connection__. Provided by JetsonHacksNano. See more details at https://github.com/JetsonHacksNano/CSI-Camera

### Additional information 
If you want to be able to see every step of the odometry calculation (key points, matching ....etc) and enable verbose options you should uncomment the line 7 in the matcher.hpp file.
robot upstart tutorial : https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/

**Warning** 
The translation estimation is dommed to drift because the data provided by the imu is not accurate. A significant improvement will be to filter those data (ie kalman filter) before using them. This filter will be integrated in the next version of this project.
This project is totally opensource and can be modified by whoever wants to. For any pull request, contact the maintainer at : aubin.adjanohoun@gmail.com.
