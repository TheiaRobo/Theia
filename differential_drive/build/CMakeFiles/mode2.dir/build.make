# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robo/DD2425_2013/Theia/differential_drive

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robo/DD2425_2013/Theia/differential_drive/build

# Include any dependencies generated for this target.
include CMakeFiles/mode2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mode2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mode2.dir/flags.make

CMakeFiles/mode2.dir/src/mode2.o: CMakeFiles/mode2.dir/flags.make
CMakeFiles/mode2.dir/src/mode2.o: ../src/mode2.cpp
CMakeFiles/mode2.dir/src/mode2.o: ../manifest.xml
CMakeFiles/mode2.dir/src/mode2.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/mode2.dir/src/mode2.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/mode2.dir/src/mode2.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/mode2.dir/src/mode2.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/mode2.dir/src/mode2.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/robo/DD2425_2013/Theia/differential_drive/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mode2.dir/src/mode2.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/mode2.dir/src/mode2.o -c /home/robo/DD2425_2013/Theia/differential_drive/src/mode2.cpp

CMakeFiles/mode2.dir/src/mode2.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mode2.dir/src/mode2.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/robo/DD2425_2013/Theia/differential_drive/src/mode2.cpp > CMakeFiles/mode2.dir/src/mode2.i

CMakeFiles/mode2.dir/src/mode2.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mode2.dir/src/mode2.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/robo/DD2425_2013/Theia/differential_drive/src/mode2.cpp -o CMakeFiles/mode2.dir/src/mode2.s

CMakeFiles/mode2.dir/src/mode2.o.requires:
.PHONY : CMakeFiles/mode2.dir/src/mode2.o.requires

CMakeFiles/mode2.dir/src/mode2.o.provides: CMakeFiles/mode2.dir/src/mode2.o.requires
	$(MAKE) -f CMakeFiles/mode2.dir/build.make CMakeFiles/mode2.dir/src/mode2.o.provides.build
.PHONY : CMakeFiles/mode2.dir/src/mode2.o.provides

CMakeFiles/mode2.dir/src/mode2.o.provides.build: CMakeFiles/mode2.dir/src/mode2.o

# Object files for target mode2
mode2_OBJECTS = \
"CMakeFiles/mode2.dir/src/mode2.o"

# External object files for target mode2
mode2_EXTERNAL_OBJECTS =

../bin/mode2: CMakeFiles/mode2.dir/src/mode2.o
../bin/mode2: CMakeFiles/mode2.dir/build.make
../bin/mode2: CMakeFiles/mode2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/mode2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mode2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mode2.dir/build: ../bin/mode2
.PHONY : CMakeFiles/mode2.dir/build

CMakeFiles/mode2.dir/requires: CMakeFiles/mode2.dir/src/mode2.o.requires
.PHONY : CMakeFiles/mode2.dir/requires

CMakeFiles/mode2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mode2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mode2.dir/clean

CMakeFiles/mode2.dir/depend:
	cd /home/robo/DD2425_2013/Theia/differential_drive/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robo/DD2425_2013/Theia/differential_drive /home/robo/DD2425_2013/Theia/differential_drive /home/robo/DD2425_2013/Theia/differential_drive/build /home/robo/DD2425_2013/Theia/differential_drive/build /home/robo/DD2425_2013/Theia/differential_drive/build/CMakeFiles/mode2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mode2.dir/depend

