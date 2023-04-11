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
`git clone https://github.com/aubino/viso_imu.git` 
`mkdir build`
`cd build`  
`cmake ..`
`make -j4`  

### Execution instrunctions and option
Executable  viso_imu *the visual odometry calculator*. 
Options
    -c  configuration file containing the camera intrinsics parameters, supposed to be obtained after calibration. *Absolute yaml or xml file path*
    -p  IMU data sampling period


Executable  viso_imu_test *The testing executable. Uses known binocular (stereo) camera data to test the accuracy of the odometr*y  
Arguments  (in order)  
        left_root  left images root *absolute directory* containing left images of a stereo camera 
        right_root  right imaes root *absolute directory* containing right images of a stereo camera
        config_file  configuration file *absolute path* containong calibration data for both left and right images and Transformation between the two camera  frames  

Executable  imu_test *the imu testing executable*  for testing the imu connection. No options

Executable  simple_camera *A simple executable to test the camera connection. Provided by JetsonHacksNano. See more details at https://github.com/JetsonHacksNano/CSI-Camera*

### Additional information 
If you want to be able to see every step of the odometry calculation (key points, matching ....etc) and enable verbose options you should uncomment the line 7 in the matcher.hpp file. 

**Warning** 
The translation estimation is dommed to drift because the data provided by the imu is not accurate. A significant improvement will be to filter those data (ie kalman filter) before using them. This filter will be integrated in the next version of this project.
This project is totally opensource and can be modified by whoever wants to. For any pull request, contact the maintainer at : aubin.adjanohoun@gmail.com.
