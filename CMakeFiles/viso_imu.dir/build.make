# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/aubin/viso_imu

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aubin/viso_imu

# Include any dependencies generated for this target.
include CMakeFiles/viso_imu.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/viso_imu.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/viso_imu.dir/flags.make

CMakeFiles/viso_imu.dir/src/detector.cpp.o: CMakeFiles/viso_imu.dir/flags.make
CMakeFiles/viso_imu.dir/src/detector.cpp.o: src/detector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aubin/viso_imu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/viso_imu.dir/src/detector.cpp.o"
	/usr/bin/clang++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/viso_imu.dir/src/detector.cpp.o -c /home/aubin/viso_imu/src/detector.cpp

CMakeFiles/viso_imu.dir/src/detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso_imu.dir/src/detector.cpp.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aubin/viso_imu/src/detector.cpp > CMakeFiles/viso_imu.dir/src/detector.cpp.i

CMakeFiles/viso_imu.dir/src/detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso_imu.dir/src/detector.cpp.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aubin/viso_imu/src/detector.cpp -o CMakeFiles/viso_imu.dir/src/detector.cpp.s

CMakeFiles/viso_imu.dir/src/ICM20948.cpp.o: CMakeFiles/viso_imu.dir/flags.make
CMakeFiles/viso_imu.dir/src/ICM20948.cpp.o: src/ICM20948.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aubin/viso_imu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/viso_imu.dir/src/ICM20948.cpp.o"
	/usr/bin/clang++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/viso_imu.dir/src/ICM20948.cpp.o -c /home/aubin/viso_imu/src/ICM20948.cpp

CMakeFiles/viso_imu.dir/src/ICM20948.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso_imu.dir/src/ICM20948.cpp.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aubin/viso_imu/src/ICM20948.cpp > CMakeFiles/viso_imu.dir/src/ICM20948.cpp.i

CMakeFiles/viso_imu.dir/src/ICM20948.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso_imu.dir/src/ICM20948.cpp.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aubin/viso_imu/src/ICM20948.cpp -o CMakeFiles/viso_imu.dir/src/ICM20948.cpp.s

CMakeFiles/viso_imu.dir/src/imu.cpp.o: CMakeFiles/viso_imu.dir/flags.make
CMakeFiles/viso_imu.dir/src/imu.cpp.o: src/imu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aubin/viso_imu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/viso_imu.dir/src/imu.cpp.o"
	/usr/bin/clang++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/viso_imu.dir/src/imu.cpp.o -c /home/aubin/viso_imu/src/imu.cpp

CMakeFiles/viso_imu.dir/src/imu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso_imu.dir/src/imu.cpp.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aubin/viso_imu/src/imu.cpp > CMakeFiles/viso_imu.dir/src/imu.cpp.i

CMakeFiles/viso_imu.dir/src/imu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso_imu.dir/src/imu.cpp.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aubin/viso_imu/src/imu.cpp -o CMakeFiles/viso_imu.dir/src/imu.cpp.s

CMakeFiles/viso_imu.dir/src/main_inception.cpp.o: CMakeFiles/viso_imu.dir/flags.make
CMakeFiles/viso_imu.dir/src/main_inception.cpp.o: src/main_inception.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aubin/viso_imu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/viso_imu.dir/src/main_inception.cpp.o"
	/usr/bin/clang++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/viso_imu.dir/src/main_inception.cpp.o -c /home/aubin/viso_imu/src/main_inception.cpp

CMakeFiles/viso_imu.dir/src/main_inception.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso_imu.dir/src/main_inception.cpp.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aubin/viso_imu/src/main_inception.cpp > CMakeFiles/viso_imu.dir/src/main_inception.cpp.i

CMakeFiles/viso_imu.dir/src/main_inception.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso_imu.dir/src/main_inception.cpp.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aubin/viso_imu/src/main_inception.cpp -o CMakeFiles/viso_imu.dir/src/main_inception.cpp.s

CMakeFiles/viso_imu.dir/src/matcher.cpp.o: CMakeFiles/viso_imu.dir/flags.make
CMakeFiles/viso_imu.dir/src/matcher.cpp.o: src/matcher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aubin/viso_imu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/viso_imu.dir/src/matcher.cpp.o"
	/usr/bin/clang++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/viso_imu.dir/src/matcher.cpp.o -c /home/aubin/viso_imu/src/matcher.cpp

CMakeFiles/viso_imu.dir/src/matcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso_imu.dir/src/matcher.cpp.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aubin/viso_imu/src/matcher.cpp > CMakeFiles/viso_imu.dir/src/matcher.cpp.i

CMakeFiles/viso_imu.dir/src/matcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso_imu.dir/src/matcher.cpp.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aubin/viso_imu/src/matcher.cpp -o CMakeFiles/viso_imu.dir/src/matcher.cpp.s

CMakeFiles/viso_imu.dir/src/transform_computer.cpp.o: CMakeFiles/viso_imu.dir/flags.make
CMakeFiles/viso_imu.dir/src/transform_computer.cpp.o: src/transform_computer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aubin/viso_imu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/viso_imu.dir/src/transform_computer.cpp.o"
	/usr/bin/clang++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/viso_imu.dir/src/transform_computer.cpp.o -c /home/aubin/viso_imu/src/transform_computer.cpp

CMakeFiles/viso_imu.dir/src/transform_computer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso_imu.dir/src/transform_computer.cpp.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aubin/viso_imu/src/transform_computer.cpp > CMakeFiles/viso_imu.dir/src/transform_computer.cpp.i

CMakeFiles/viso_imu.dir/src/transform_computer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso_imu.dir/src/transform_computer.cpp.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aubin/viso_imu/src/transform_computer.cpp -o CMakeFiles/viso_imu.dir/src/transform_computer.cpp.s

# Object files for target viso_imu
viso_imu_OBJECTS = \
"CMakeFiles/viso_imu.dir/src/detector.cpp.o" \
"CMakeFiles/viso_imu.dir/src/ICM20948.cpp.o" \
"CMakeFiles/viso_imu.dir/src/imu.cpp.o" \
"CMakeFiles/viso_imu.dir/src/main_inception.cpp.o" \
"CMakeFiles/viso_imu.dir/src/matcher.cpp.o" \
"CMakeFiles/viso_imu.dir/src/transform_computer.cpp.o"

# External object files for target viso_imu
viso_imu_EXTERNAL_OBJECTS =

viso_imu: CMakeFiles/viso_imu.dir/src/detector.cpp.o
viso_imu: CMakeFiles/viso_imu.dir/src/ICM20948.cpp.o
viso_imu: CMakeFiles/viso_imu.dir/src/imu.cpp.o
viso_imu: CMakeFiles/viso_imu.dir/src/main_inception.cpp.o
viso_imu: CMakeFiles/viso_imu.dir/src/matcher.cpp.o
viso_imu: CMakeFiles/viso_imu.dir/src/transform_computer.cpp.o
viso_imu: CMakeFiles/viso_imu.dir/build.make
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
viso_imu: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
viso_imu: CMakeFiles/viso_imu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aubin/viso_imu/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable viso_imu"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/viso_imu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/viso_imu.dir/build: viso_imu

.PHONY : CMakeFiles/viso_imu.dir/build

CMakeFiles/viso_imu.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/viso_imu.dir/cmake_clean.cmake
.PHONY : CMakeFiles/viso_imu.dir/clean

CMakeFiles/viso_imu.dir/depend:
	cd /home/aubin/viso_imu && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aubin/viso_imu /home/aubin/viso_imu /home/aubin/viso_imu /home/aubin/viso_imu /home/aubin/viso_imu/CMakeFiles/viso_imu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/viso_imu.dir/depend

