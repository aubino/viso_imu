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
CMAKE_BINARY_DIR = /home/aubin/viso_imu/build

# Include any dependencies generated for this target.
include CMakeFiles/imu_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/imu_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/imu_test.dir/flags.make

CMakeFiles/imu_test.dir/src/ICM20948.cpp.o: CMakeFiles/imu_test.dir/flags.make
CMakeFiles/imu_test.dir/src/ICM20948.cpp.o: ../src/ICM20948.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aubin/viso_imu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/imu_test.dir/src/ICM20948.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu_test.dir/src/ICM20948.cpp.o -c /home/aubin/viso_imu/src/ICM20948.cpp

CMakeFiles/imu_test.dir/src/ICM20948.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_test.dir/src/ICM20948.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aubin/viso_imu/src/ICM20948.cpp > CMakeFiles/imu_test.dir/src/ICM20948.cpp.i

CMakeFiles/imu_test.dir/src/ICM20948.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_test.dir/src/ICM20948.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aubin/viso_imu/src/ICM20948.cpp -o CMakeFiles/imu_test.dir/src/ICM20948.cpp.s

CMakeFiles/imu_test.dir/src/main.cpp.o: CMakeFiles/imu_test.dir/flags.make
CMakeFiles/imu_test.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aubin/viso_imu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/imu_test.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu_test.dir/src/main.cpp.o -c /home/aubin/viso_imu/src/main.cpp

CMakeFiles/imu_test.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_test.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aubin/viso_imu/src/main.cpp > CMakeFiles/imu_test.dir/src/main.cpp.i

CMakeFiles/imu_test.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_test.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aubin/viso_imu/src/main.cpp -o CMakeFiles/imu_test.dir/src/main.cpp.s

# Object files for target imu_test
imu_test_OBJECTS = \
"CMakeFiles/imu_test.dir/src/ICM20948.cpp.o" \
"CMakeFiles/imu_test.dir/src/main.cpp.o"

# External object files for target imu_test
imu_test_EXTERNAL_OBJECTS =

imu_test: CMakeFiles/imu_test.dir/src/ICM20948.cpp.o
imu_test: CMakeFiles/imu_test.dir/src/main.cpp.o
imu_test: CMakeFiles/imu_test.dir/build.make
imu_test: CMakeFiles/imu_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aubin/viso_imu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable imu_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imu_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/imu_test.dir/build: imu_test

.PHONY : CMakeFiles/imu_test.dir/build

CMakeFiles/imu_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imu_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imu_test.dir/clean

CMakeFiles/imu_test.dir/depend:
	cd /home/aubin/viso_imu/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aubin/viso_imu /home/aubin/viso_imu /home/aubin/viso_imu/build /home/aubin/viso_imu/build /home/aubin/viso_imu/build/CMakeFiles/imu_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imu_test.dir/depend

