project(cppUtilsRGBD)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_definitions(-std=c++11)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)

include_directories(
        # include
        /usr/lib/x86_64-linux-gnu/cmake/pcl
)

include_directories(
        # include
        /usr/include/boost
)

set(Boost_DEBUG ON)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
ADD_DEFINITIONS(-DBoost_USE_STATIC_LIBS=ON)
set(Boost_INCLUDE_DIR /usr/include/)
set(Boost_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)

# set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev )
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(" OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS}")

#add_library(PCLutils SHARED
#PCLutils.cpp
#)
#
#target_link_libraries(PCLutils
#${OpenCV_LIBS}
#${PCL_LIBRARIES}
#)
#
#add_executable (timeSync timeSync.cpp)
#target_link_libraries (timeSync)
#
#add_executable (projCloud projCloud.cpp)
#target_link_libraries (projCloud PCLutils)
#
add_executable (projDepth projDepth.cpp)
target_link_libraries (projDepth  ${OpenCV_LIBS})
#
#add_executable (projInverse projInverse.cpp)
#target_link_libraries (projInverse PCLutils)
#
#add_executable (grey2rgb grey2rgb.cpp)
#target_link_libraries (grey2rgb PCLutils)
#
#add_executable (createXML createXML.cpp)
#target_link_libraries (createXML PCLutils)
#
#add_executable (undistort undistort.cpp)
#target_link_libraries (undistort PCLutils)
#add_executable (stereo_calib stereo_calib.cpp)
#target_link_libraries (stereo_calib PCLutils)

#add_executable( feature_ext feature_ext.cpp  )
#target_link_libraries( feature_ext ${OpenCV_LIBS} )
