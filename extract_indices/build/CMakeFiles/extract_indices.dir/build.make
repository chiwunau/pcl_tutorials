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
CMAKE_SOURCE_DIR = /home/leus/pcl_tutiorials/extract_indices

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leus/pcl_tutiorials/extract_indices/build

# Include any dependencies generated for this target.
include CMakeFiles/extract_indices.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/extract_indices.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/extract_indices.dir/flags.make

CMakeFiles/extract_indices.dir/extract_indices.cpp.o: CMakeFiles/extract_indices.dir/flags.make
CMakeFiles/extract_indices.dir/extract_indices.cpp.o: ../extract_indices.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/leus/pcl_tutiorials/extract_indices/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/extract_indices.dir/extract_indices.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/extract_indices.dir/extract_indices.cpp.o -c /home/leus/pcl_tutiorials/extract_indices/extract_indices.cpp

CMakeFiles/extract_indices.dir/extract_indices.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/extract_indices.dir/extract_indices.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/leus/pcl_tutiorials/extract_indices/extract_indices.cpp > CMakeFiles/extract_indices.dir/extract_indices.cpp.i

CMakeFiles/extract_indices.dir/extract_indices.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/extract_indices.dir/extract_indices.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/leus/pcl_tutiorials/extract_indices/extract_indices.cpp -o CMakeFiles/extract_indices.dir/extract_indices.cpp.s

CMakeFiles/extract_indices.dir/extract_indices.cpp.o.requires:
.PHONY : CMakeFiles/extract_indices.dir/extract_indices.cpp.o.requires

CMakeFiles/extract_indices.dir/extract_indices.cpp.o.provides: CMakeFiles/extract_indices.dir/extract_indices.cpp.o.requires
	$(MAKE) -f CMakeFiles/extract_indices.dir/build.make CMakeFiles/extract_indices.dir/extract_indices.cpp.o.provides.build
.PHONY : CMakeFiles/extract_indices.dir/extract_indices.cpp.o.provides

CMakeFiles/extract_indices.dir/extract_indices.cpp.o.provides.build: CMakeFiles/extract_indices.dir/extract_indices.cpp.o

# Object files for target extract_indices
extract_indices_OBJECTS = \
"CMakeFiles/extract_indices.dir/extract_indices.cpp.o"

# External object files for target extract_indices
extract_indices_EXTERNAL_OBJECTS =

extract_indices: CMakeFiles/extract_indices.dir/extract_indices.cpp.o
extract_indices: /usr/lib/libboost_system-mt.so
extract_indices: /usr/lib/libboost_filesystem-mt.so
extract_indices: /usr/lib/libboost_thread-mt.so
extract_indices: /usr/lib/libboost_date_time-mt.so
extract_indices: /usr/lib/libboost_iostreams-mt.so
extract_indices: /usr/lib/libboost_serialization-mt.so
extract_indices: /usr/lib/libpcl_common.so
extract_indices: /opt/ros/groovy/lib/libflann_cpp_s.a
extract_indices: /usr/lib/libpcl_kdtree.so
extract_indices: /usr/lib/libpcl_octree.so
extract_indices: /usr/lib/libpcl_search.so
extract_indices: /usr/lib/libOpenNI.so
extract_indices: /usr/lib/libvtkCommon.so.5.8.0
extract_indices: /usr/lib/libvtkRendering.so.5.8.0
extract_indices: /usr/lib/libvtkHybrid.so.5.8.0
extract_indices: /usr/lib/libvtkCharts.so.5.8.0
extract_indices: /usr/lib/libpcl_io.so
extract_indices: /usr/lib/libpcl_sample_consensus.so
extract_indices: /usr/lib/libpcl_filters.so
extract_indices: /usr/lib/libpcl_visualization.so
extract_indices: /usr/lib/libpcl_outofcore.so
extract_indices: /usr/lib/libpcl_features.so
extract_indices: /usr/lib/libpcl_segmentation.so
extract_indices: /usr/lib/libpcl_people.so
extract_indices: /usr/lib/libpcl_registration.so
extract_indices: /usr/lib/libpcl_recognition.so
extract_indices: /usr/lib/libpcl_keypoints.so
extract_indices: /usr/lib/libqhull.so
extract_indices: /usr/lib/libpcl_surface.so
extract_indices: /usr/lib/libpcl_tracking.so
extract_indices: /usr/lib/libpcl_apps.so
extract_indices: /usr/lib/libboost_system-mt.so
extract_indices: /usr/lib/libboost_filesystem-mt.so
extract_indices: /usr/lib/libboost_thread-mt.so
extract_indices: /usr/lib/libboost_date_time-mt.so
extract_indices: /usr/lib/libboost_iostreams-mt.so
extract_indices: /usr/lib/libboost_serialization-mt.so
extract_indices: /usr/lib/libqhull.so
extract_indices: /usr/lib/libOpenNI.so
extract_indices: /opt/ros/groovy/lib/libflann_cpp_s.a
extract_indices: /usr/lib/libvtkCommon.so.5.8.0
extract_indices: /usr/lib/libvtkRendering.so.5.8.0
extract_indices: /usr/lib/libvtkHybrid.so.5.8.0
extract_indices: /usr/lib/libvtkCharts.so.5.8.0
extract_indices: /usr/lib/libpcl_common.so
extract_indices: /usr/lib/libpcl_kdtree.so
extract_indices: /usr/lib/libpcl_octree.so
extract_indices: /usr/lib/libpcl_search.so
extract_indices: /usr/lib/libpcl_io.so
extract_indices: /usr/lib/libpcl_sample_consensus.so
extract_indices: /usr/lib/libpcl_filters.so
extract_indices: /usr/lib/libpcl_visualization.so
extract_indices: /usr/lib/libpcl_outofcore.so
extract_indices: /usr/lib/libpcl_features.so
extract_indices: /usr/lib/libpcl_segmentation.so
extract_indices: /usr/lib/libpcl_people.so
extract_indices: /usr/lib/libpcl_registration.so
extract_indices: /usr/lib/libpcl_recognition.so
extract_indices: /usr/lib/libpcl_keypoints.so
extract_indices: /usr/lib/libpcl_surface.so
extract_indices: /usr/lib/libpcl_tracking.so
extract_indices: /usr/lib/libpcl_apps.so
extract_indices: /usr/lib/libvtkViews.so.5.8.0
extract_indices: /usr/lib/libvtkInfovis.so.5.8.0
extract_indices: /usr/lib/libvtkWidgets.so.5.8.0
extract_indices: /usr/lib/libvtkHybrid.so.5.8.0
extract_indices: /usr/lib/libvtkParallel.so.5.8.0
extract_indices: /usr/lib/libvtkVolumeRendering.so.5.8.0
extract_indices: /usr/lib/libvtkRendering.so.5.8.0
extract_indices: /usr/lib/libvtkGraphics.so.5.8.0
extract_indices: /usr/lib/libvtkImaging.so.5.8.0
extract_indices: /usr/lib/libvtkIO.so.5.8.0
extract_indices: /usr/lib/libvtkFiltering.so.5.8.0
extract_indices: /usr/lib/libvtkCommon.so.5.8.0
extract_indices: /usr/lib/libvtksys.so.5.8.0
extract_indices: CMakeFiles/extract_indices.dir/build.make
extract_indices: CMakeFiles/extract_indices.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable extract_indices"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/extract_indices.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/extract_indices.dir/build: extract_indices
.PHONY : CMakeFiles/extract_indices.dir/build

CMakeFiles/extract_indices.dir/requires: CMakeFiles/extract_indices.dir/extract_indices.cpp.o.requires
.PHONY : CMakeFiles/extract_indices.dir/requires

CMakeFiles/extract_indices.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/extract_indices.dir/cmake_clean.cmake
.PHONY : CMakeFiles/extract_indices.dir/clean

CMakeFiles/extract_indices.dir/depend:
	cd /home/leus/pcl_tutiorials/extract_indices/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leus/pcl_tutiorials/extract_indices /home/leus/pcl_tutiorials/extract_indices /home/leus/pcl_tutiorials/extract_indices/build /home/leus/pcl_tutiorials/extract_indices/build /home/leus/pcl_tutiorials/extract_indices/build/CMakeFiles/extract_indices.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/extract_indices.dir/depend

