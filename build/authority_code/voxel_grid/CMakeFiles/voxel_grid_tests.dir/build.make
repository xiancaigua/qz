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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cquer/2023_qingzhou/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cquer/2023_qingzhou/build

# Include any dependencies generated for this target.
include authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/depend.make

# Include the progress variables for this target.
include authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/progress.make

# Include the compile flags for this target's objects.
include authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/flags.make

authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o: authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/flags.make
authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o: /home/cquer/2023_qingzhou/src/authority_code/voxel_grid/test/voxel_grid_tests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cquer/2023_qingzhou/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o"
	cd /home/cquer/2023_qingzhou/build/authority_code/voxel_grid && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o -c /home/cquer/2023_qingzhou/src/authority_code/voxel_grid/test/voxel_grid_tests.cpp

authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.i"
	cd /home/cquer/2023_qingzhou/build/authority_code/voxel_grid && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cquer/2023_qingzhou/src/authority_code/voxel_grid/test/voxel_grid_tests.cpp > CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.i

authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.s"
	cd /home/cquer/2023_qingzhou/build/authority_code/voxel_grid && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cquer/2023_qingzhou/src/authority_code/voxel_grid/test/voxel_grid_tests.cpp -o CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.s

authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.requires:

.PHONY : authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.requires

authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.provides: authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.requires
	$(MAKE) -f authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/build.make authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.provides.build
.PHONY : authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.provides

authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.provides.build: authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o


# Object files for target voxel_grid_tests
voxel_grid_tests_OBJECTS = \
"CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o"

# External object files for target voxel_grid_tests
voxel_grid_tests_EXTERNAL_OBJECTS =

/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/build.make
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: gtest/googlemock/gtest/libgtest.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /home/cquer/2023_qingzhou/devel/lib/libvoxel_grid.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/libroscpp.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/librosconsole.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/librostime.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/libcpp_common.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests: authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cquer/2023_qingzhou/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests"
	cd /home/cquer/2023_qingzhou/build/authority_code/voxel_grid && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/voxel_grid_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/build: /home/cquer/2023_qingzhou/devel/lib/voxel_grid/voxel_grid_tests

.PHONY : authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/build

authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/requires: authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.requires

.PHONY : authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/requires

authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/clean:
	cd /home/cquer/2023_qingzhou/build/authority_code/voxel_grid && $(CMAKE_COMMAND) -P CMakeFiles/voxel_grid_tests.dir/cmake_clean.cmake
.PHONY : authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/clean

authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/depend:
	cd /home/cquer/2023_qingzhou/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cquer/2023_qingzhou/src /home/cquer/2023_qingzhou/src/authority_code/voxel_grid /home/cquer/2023_qingzhou/build /home/cquer/2023_qingzhou/build/authority_code/voxel_grid /home/cquer/2023_qingzhou/build/authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : authority_code/voxel_grid/CMakeFiles/voxel_grid_tests.dir/depend

