# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /home/leo/Downloads/clion-2017.1.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/leo/Downloads/clion-2017.1.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/rgbd_tum_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rgbd_tum_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rgbd_tum_test.dir/flags.make

CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o: CMakeFiles/rgbd_tum_test.dir/flags.make
CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o: ../Examples/RGB-D/rgbd_tum_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o -c /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2/Examples/RGB-D/rgbd_tum_test.cpp

CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2/Examples/RGB-D/rgbd_tum_test.cpp > CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.i

CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2/Examples/RGB-D/rgbd_tum_test.cpp -o CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.s

CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o.requires:

.PHONY : CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o.requires

CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o.provides: CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o.requires
	$(MAKE) -f CMakeFiles/rgbd_tum_test.dir/build.make CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o.provides.build
.PHONY : CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o.provides

CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o.provides.build: CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o


# Object files for target rgbd_tum_test
rgbd_tum_test_OBJECTS = \
"CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o"

# External object files for target rgbd_tum_test
rgbd_tum_test_EXTERNAL_OBJECTS =

../Examples/RGB-D/rgbd_tum_test: CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o
../Examples/RGB-D/rgbd_tum_test: CMakeFiles/rgbd_tum_test.dir/build.make
../Examples/RGB-D/rgbd_tum_test: ../lib/libORB_SLAM2.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../Examples/RGB-D/rgbd_tum_test: /usr/local/lib/libpangolin.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libdc1394.so
../Examples/RGB-D/rgbd_tum_test: /usr/local/lib/x86_64-linux-gnu/libuvc.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libtiff.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../Examples/RGB-D/rgbd_tum_test: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/RGB-D/rgbd_tum_test: ../Thirdparty/g2o/lib/libg2o.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libpcl_ros_filters.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libpcl_ros_io.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libpcl_ros_tf.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_common.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_octree.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_io.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_kdtree.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_search.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_sample_consensus.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_filters.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_features.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_keypoints.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_segmentation.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_visualization.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_outofcore.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_registration.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_recognition.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_surface.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_people.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_tracking.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libpcl_apps.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libqhull.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libOpenNI.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libvtkCommon.so.5.8.0
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libvtkRendering.so.5.8.0
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libvtkHybrid.so.5.8.0
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libvtkCharts.so.5.8.0
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libnodeletlib.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libbondcpp.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libuuid.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libclass_loader.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/libPocoFoundation.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libdl.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libroslib.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/librospack.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libpython2.7.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libtinyxml.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/librosbag.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/librosbag_storage.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libroslz4.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/liblz4.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libtopic_tools.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libtf.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libtf2_ros.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libactionlib.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libmessage_filters.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libroscpp.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libboost_signals.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libxmlrpcpp.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libtf2.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libroscpp_serialization.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/librosconsole.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/librosconsole_log4cxx.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/librosconsole_backend_interface.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/liblog4cxx.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/librostime.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../Examples/RGB-D/rgbd_tum_test: /opt/ros/indigo/lib/libcpp_common.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libpthread.so
../Examples/RGB-D/rgbd_tum_test: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
../Examples/RGB-D/rgbd_tum_test: CMakeFiles/rgbd_tum_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/RGB-D/rgbd_tum_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rgbd_tum_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rgbd_tum_test.dir/build: ../Examples/RGB-D/rgbd_tum_test

.PHONY : CMakeFiles/rgbd_tum_test.dir/build

CMakeFiles/rgbd_tum_test.dir/requires: CMakeFiles/rgbd_tum_test.dir/Examples/RGB-D/rgbd_tum_test.cpp.o.requires

.PHONY : CMakeFiles/rgbd_tum_test.dir/requires

CMakeFiles/rgbd_tum_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rgbd_tum_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rgbd_tum_test.dir/clean

CMakeFiles/rgbd_tum_test.dir/depend:
	cd /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2 /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2 /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2/cmake-build-debug /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2/cmake-build-debug /home/leo/Desktop/ORB_SLAM2_Changing/ORB_SLAM2/cmake-build-debug/CMakeFiles/rgbd_tum_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rgbd_tum_test.dir/depend
