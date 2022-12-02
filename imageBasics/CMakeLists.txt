cmake_minimum_required( VERSION 2.8 )
project(imageBasics)
 
set( CMAKE_CXX_FLAGS "-std=c++11 -O3")
find_package(OpenCV)
include_directories(${OpenCV})
 
add_executable(imageBasics imageBasics.cpp)
# 链接OpenCV库
target_link_libraries(imageBasics ${OpenCV_LIBS})
 
add_executable(undistortImage undistortImage.cpp)
# 链接OpenCV库
target_link_libraries(undistortImage ${OpenCV_LIBS})


