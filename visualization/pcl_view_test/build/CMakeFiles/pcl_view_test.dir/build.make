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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/leus/pcl_tutiorials/visualization

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leus/pcl_tutiorials/visualization/build

# Include any dependencies generated for this target.
include CMakeFiles/pcl_view_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pcl_view_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pcl_view_test.dir/flags.make

CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o: CMakeFiles/pcl_view_test.dir/flags.make
CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o: ../pcl_view_test.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/leus/pcl_tutiorials/visualization/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o -c /home/leus/pcl_tutiorials/visualization/pcl_view_test.cpp

CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/leus/pcl_tutiorials/visualization/pcl_view_test.cpp > CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.i

CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/leus/pcl_tutiorials/visualization/pcl_view_test.cpp -o CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.s

CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o.requires:
.PHONY : CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o.requires

CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o.provides: CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o.requires
	$(MAKE) -f CMakeFiles/pcl_view_test.dir/build.make CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o.provides.build
.PHONY : CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o.provides

CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o.provides.build: CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o

# Object files for target pcl_view_test
pcl_view_test_OBJECTS = \
"CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o"

# External object files for target pcl_view_test
pcl_view_test_EXTERNAL_OBJECTS =

pcl_view_test: CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o
pcl_view_test: /usr/lib/libboost_system-mt.so
pcl_view_test: /usr/lib/libboost_filesystem-mt.so
pcl_view_test: /usr/lib/libboost_thread-mt.so
pcl_view_test: /usr/lib/libboost_date_time-mt.so
pcl_view_test: /usr/lib/libboost_iostreams-mt.so
pcl_view_test: /usr/lib/libboost_serialization-mt.so
pcl_view_test: /usr/lib/libpcl_common.so
pcl_view_test: /opt/ros/groovy/lib/libflann_cpp_s.a
pcl_view_test: /usr/lib/libpcl_kdtree.so
pcl_view_test: /usr/lib/libpcl_octree.so
pcl_view_test: /usr/lib/libpcl_search.so
pcl_view_test: /usr/lib/libOpenNI.so
pcl_view_test: /usr/lib/libvtkCommon.so.5.8.0
pcl_view_test: /usr/lib/libvtkRendering.so.5.8.0
pcl_view_test: /usr/lib/libvtkHybrid.so.5.8.0
pcl_view_test: /usr/lib/libvtkCharts.so.5.8.0
pcl_view_test: /usr/lib/libpcl_io.so
pcl_view_test: /usr/lib/libpcl_sample_consensus.so
pcl_view_test: /usr/lib/libpcl_filters.so
pcl_view_test: /usr/lib/libpcl_visualization.so
pcl_view_test: /usr/lib/libpcl_outofcore.so
pcl_view_test: /usr/lib/libpcl_features.so
pcl_view_test: /usr/lib/libpcl_segmentation.so
pcl_view_test: /usr/lib/libpcl_people.so
pcl_view_test: /usr/lib/libpcl_registration.so
pcl_view_test: /usr/lib/libpcl_recognition.so
pcl_view_test: /usr/lib/libpcl_keypoints.so
pcl_view_test: /usr/lib/libqhull.so
pcl_view_test: /usr/lib/libpcl_surface.so
pcl_view_test: /usr/lib/libpcl_tracking.so
pcl_view_test: /usr/lib/libpcl_apps.so
pcl_view_test: /usr/lib/libboost_system-mt.so
pcl_view_test: /usr/lib/libboost_filesystem-mt.so
pcl_view_test: /usr/lib/libboost_thread-mt.so
pcl_view_test: /usr/lib/libboost_date_time-mt.so
pcl_view_test: /usr/lib/libboost_iostreams-mt.so
pcl_view_test: /usr/lib/libboost_serialization-mt.so
pcl_view_test: /usr/lib/libqhull.so
pcl_view_test: /usr/lib/libOpenNI.so
pcl_view_test: /opt/ros/groovy/lib/libflann_cpp_s.a
pcl_view_test: /usr/lib/libvtkCommon.so.5.8.0
pcl_view_test: /usr/lib/libvtkRendering.so.5.8.0
pcl_view_test: /usr/lib/libvtkHybrid.so.5.8.0
pcl_view_test: /usr/lib/libvtkCharts.so.5.8.0
pcl_view_test: /usr/lib/libpcl_common.so
pcl_view_test: /usr/lib/libpcl_kdtree.so
pcl_view_test: /usr/lib/libpcl_octree.so
pcl_view_test: /usr/lib/libpcl_search.so
pcl_view_test: /usr/lib/libpcl_io.so
pcl_view_test: /usr/lib/libpcl_sample_consensus.so
pcl_view_test: /usr/lib/libpcl_filters.so
pcl_view_test: /usr/lib/libpcl_visualization.so
pcl_view_test: /usr/lib/libpcl_outofcore.so
pcl_view_test: /usr/lib/libpcl_features.so
pcl_view_test: /usr/lib/libpcl_segmentation.so
pcl_view_test: /usr/lib/libpcl_people.so
pcl_view_test: /usr/lib/libpcl_registration.so
pcl_view_test: /usr/lib/libpcl_recognition.so
pcl_view_test: /usr/lib/libpcl_keypoints.so
pcl_view_test: /usr/lib/libpcl_surface.so
pcl_view_test: /usr/lib/libpcl_tracking.so
pcl_view_test: /usr/lib/libpcl_apps.so
pcl_view_test: /usr/lib/libvtkViews.so.5.8.0
pcl_view_test: /usr/lib/libvtkInfovis.so.5.8.0
pcl_view_test: /usr/lib/libvtkWidgets.so.5.8.0
pcl_view_test: /usr/lib/libvtkHybrid.so.5.8.0
pcl_view_test: /usr/lib/libvtkParallel.so.5.8.0
pcl_view_test: /usr/lib/libvtkVolumeRendering.so.5.8.0
pcl_view_test: /usr/lib/libvtkRendering.so.5.8.0
pcl_view_test: /usr/lib/libvtkGraphics.so.5.8.0
pcl_view_test: /usr/lib/libvtkImaging.so.5.8.0
pcl_view_test: /usr/lib/libvtkIO.so.5.8.0
pcl_view_test: /usr/lib/libvtkFiltering.so.5.8.0
pcl_view_test: /usr/lib/libvtkCommon.so.5.8.0
pcl_view_test: /usr/lib/libvtksys.so.5.8.0
pcl_view_test: CMakeFiles/pcl_view_test.dir/build.make
pcl_view_test: CMakeFiles/pcl_view_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pcl_view_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcl_view_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pcl_view_test.dir/build: pcl_view_test
.PHONY : CMakeFiles/pcl_view_test.dir/build

CMakeFiles/pcl_view_test.dir/requires: CMakeFiles/pcl_view_test.dir/pcl_view_test.cpp.o.requires
.PHONY : CMakeFiles/pcl_view_test.dir/requires

CMakeFiles/pcl_view_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcl_view_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcl_view_test.dir/clean

CMakeFiles/pcl_view_test.dir/depend:
	cd /home/leus/pcl_tutiorials/visualization/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leus/pcl_tutiorials/visualization /home/leus/pcl_tutiorials/visualization /home/leus/pcl_tutiorials/visualization/build /home/leus/pcl_tutiorials/visualization/build /home/leus/pcl_tutiorials/visualization/build/CMakeFiles/pcl_view_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcl_view_test.dir/depend
