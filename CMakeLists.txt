cmake_minimum_required(VERSION 3.7)
project(viso_imu)
set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#################################Flags related to the use of threads #################################""
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)
####################### Debug options ##########################
set(CMAKE_BUILD_TYPE  Debug)

add_compile_options(-std=c++17 -Wall -lm  -lboost_program_options)
set(CMAKE_CXX_STANDARD 17)
set( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)







find_package( OpenCV  )
find_package(Boost REQUIRED program_options filesystem)
find_package(Eigen3 )

add_definitions(${OpenCV_DEFINITIONS} 
    ${EIGEN3_DEFINITIONS}
    ${Boost_DEFINITIONS})
#add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${Boost_INCLUDE_DIR})
include_directories(
    ${catkin_INCLUDE_DIRS}
)
include_directories( include 
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

add_executable(imu_test src/ICM20948.cpp src/main.cpp)
add_executable(usb_stereo_launcher src/usb_stereo.cpp  src/cloud_computer.cpp src/main_usb_cam.cpp)
add_executable(viso_imu src/detector.cpp src/ICM20948.cpp src/imu.cpp src/main_inception.cpp src/matcher.cpp src/transform_computer.cpp)
add_executable(viso_imu_test src/detector.cpp src/ICM20948.cpp src/imu.cpp src/main_unitary_test.cpp src/matcher.cpp src/transform_computer.cpp)
add_executable(simple_camera src/simple_camera.cpp)
add_executable(test_usb_cam src/test_usb_cam)

target_link_libraries(simple_camera ${OpenCV_LIBRARIES})

target_link_libraries(test_usb_cam ${OpenCV_LIBRARIES})

target_link_libraries(usb_stereo_launcher ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

target_link_libraries(viso_imu ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})

target_link_libraries(viso_imu_test ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})



# target_link_libraries(usb_stereo_launcher
# ${OpenCV_LIBRARIES}
# ${Boost_LIBRARIES})

#############
## Install ##
#############
# catkin_install_python(PROGRAMS
    # scripts/usb_cam
    # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
