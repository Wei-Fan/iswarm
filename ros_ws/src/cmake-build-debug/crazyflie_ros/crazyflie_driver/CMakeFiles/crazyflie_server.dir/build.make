# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /home/chengque/clion-2018.1.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/chengque/clion-2018.1.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug

# Include any dependencies generated for this target.
include crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/depend.make

# Include the progress variables for this target.
include crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/progress.make

# Include the compile flags for this target's objects.
include crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/flags.make

crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o: crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/flags.make
crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o: ../crazyflie_ros/crazyflie_driver/src/crazyflie_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o"
	cd /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug/crazyflie_ros/crazyflie_driver && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o -c /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/crazyflie_ros/crazyflie_driver/src/crazyflie_server.cpp

crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.i"
	cd /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug/crazyflie_ros/crazyflie_driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/crazyflie_ros/crazyflie_driver/src/crazyflie_server.cpp > CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.i

crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.s"
	cd /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug/crazyflie_ros/crazyflie_driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/crazyflie_ros/crazyflie_driver/src/crazyflie_server.cpp -o CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.s

crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.requires:

.PHONY : crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.requires

crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.provides: crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.requires
	$(MAKE) -f crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/build.make crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.provides.build
.PHONY : crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.provides

crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.provides.build: crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o


# Object files for target crazyflie_server
crazyflie_server_OBJECTS = \
"CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o"

# External object files for target crazyflie_server
crazyflie_server_EXTERNAL_OBJECTS =

devel/lib/crazyflie_driver/crazyflie_server: crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o
devel/lib/crazyflie_driver/crazyflie_server: crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/build.make
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/libtf.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/libtf2_ros.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/libactionlib.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/libmessage_filters.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/libroscpp.so
devel/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/libxmlrpcpp.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/libtf2.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/libroscpp_serialization.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/librosconsole.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/librosconsole_log4cxx.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/librosconsole_backend_interface.so
devel/lib/crazyflie_driver/crazyflie_server: /usr/lib/liblog4cxx.so
devel/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/librostime.so
devel/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/crazyflie_driver/crazyflie_server: /opt/ros/indigo/lib/libcpp_common.so
devel/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/crazyflie_driver/crazyflie_server: devel/lib/libcrazyflie_cpp.so
devel/lib/crazyflie_driver/crazyflie_server: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
devel/lib/crazyflie_driver/crazyflie_server: crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../devel/lib/crazyflie_driver/crazyflie_server"
	cd /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug/crazyflie_ros/crazyflie_driver && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/crazyflie_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/build: devel/lib/crazyflie_driver/crazyflie_server

.PHONY : crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/build

crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/requires: crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/src/crazyflie_server.cpp.o.requires

.PHONY : crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/requires

crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/clean:
	cd /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug/crazyflie_ros/crazyflie_driver && $(CMAKE_COMMAND) -P CMakeFiles/crazyflie_server.dir/cmake_clean.cmake
.PHONY : crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/clean

crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/depend:
	cd /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/crazyflie_ros/crazyflie_driver /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug/crazyflie_ros/crazyflie_driver /home/chengque/workspace/catkin_ws/src/crazyswarm/ros_ws/src/cmake-build-debug/crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : crazyflie_ros/crazyflie_driver/CMakeFiles/crazyflie_server.dir/depend

