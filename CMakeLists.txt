cmake_minimum_required(VERSION 3.7)
project(viso_imu)
#set(CMAKE_C_COMPILER "/usr/bin/clang")
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
#################################Flags related to the use of threads #################################""
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)
#######################Debug options ##########################
set(CMAKE_BUILD_TYPE  Debug)

add_compile_options(-std=c++17 -Wall -lm)
set( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")

find_package( OpenCV REQUIRED
    #Eigen3 REQUIRED 
    )

add_definitions(${OpenCV_DEFINITIONS} 
    ${EIGEN3_DEFINITIONS})

include_directories( "${EIGEN3_INCLUDE_DIR}" )

include_directories( include 
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )
add_executable(viso_imu src/detector.cpp src/ICM20948.cpp src/imu.cpp src/main_inception.cpp src/matcher.cpp src/transform_computer.cpp)
#add_executable(viso_imu_test src/detector.cpp src/ICM20948.cpp src/imu.cpp src/main_unitary_test.cpp src/matcher.cpp src/transform_computer.cpp)
add_executable(imu_test src/ICM20948.cpp src/main.cpp)

target_link_libraries(viso_imu 
${OpenCV_LIBRARIES}
${EIGEN3_LIBRARIES})

#target_link_libraries(viso_imu_test 
#${OpenCV_LIBRARIES}
#${EIGEN3_LIBRARIES})
