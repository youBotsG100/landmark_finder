cmake_minimum_required(VERSION 2.8)
project(landmark_finder)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs std_msgs 
cv_bridge image_transport) # genmsg
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

## Declare a catkin package
catkin_package()

include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)


set(SOURCE
   main.cpp   
)

set_source_files_properties(${SOURCE} PROPERTIES LANGUAGE CXX )

add_executable(${PROJECT_NAME} ${SOURCE})	
	
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})
