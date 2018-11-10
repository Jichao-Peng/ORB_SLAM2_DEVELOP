cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
#set(CMAKE_CXX_FLAGS "-std=c++0x -Wno-deprecated")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wno-deprecated")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/ORB_SLAM2/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

set(OpenCV_DIR  "/usr/local/share/OpenCV")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(catkin REQUIRED pcl_ros OpenCV)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
        ORB_SLAM2/src/System.cc
        ORB_SLAM2/src/Tracking.cc
        ORB_SLAM2/src/LocalMapping.cc
        ORB_SLAM2/src/LoopClosing.cc
        ORB_SLAM2/src/ORBextractor.cc
        ORB_SLAM2/src/ORBmatcher.cc
        ORB_SLAM2/src/FrameDrawer.cc
        ORB_SLAM2/src/Converter.cc
        ORB_SLAM2/src/MapPoint.cc
        ORB_SLAM2/src/KeyFrame.cc
        ORB_SLAM2/src/Map.cc
        ORB_SLAM2/src/MapDrawer.cc
        ORB_SLAM2/src/Optimizer.cc
        ORB_SLAM2/src/PnPsolver.cc
        ORB_SLAM2/src/Frame.cc
        ORB_SLAM2/src/KeyFrameDatabase.cc
        ORB_SLAM2/src/Sim3Solver.cc
        ORB_SLAM2/src/Initializer.cc
        ORB_SLAM2/src/Viewer.cc
        ORB_SLAM2/src/PointCloudMapping.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
        ${catkin_LIBRARIES}
        )

#add_executable(rgbd_orb_slam2_ros_test
#         src/rgbd_orb_slam2_ros_test.cpp src/rgbd_orb_slam2_ros_test.h)
#target_link_libraries(rgbd_orb_slam2_ros_test ${PROJECT_NAME})

add_executable(rgbd_orb_slam2_test
        src/rgbd_orb_slam2_test.cpp)
target_link_libraries(rgbd_orb_slam2_test ${PROJECT_NAME})

#add_executable(rgbd_orb_slam2
#        src/rgbd_orb_slam2.cpp src/rgbd_orb_slam2.h )
#target_link_libraries(rgbd_orb_slam2 ${PROJECT_NAME})




