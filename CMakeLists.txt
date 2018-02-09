cmake_minimum_required(VERSION 2.8.3)
project(sift)

SET(CMAKE_CXX_FLAGS "-std=c++11")
find_package(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge image_transport camera_info_manager sensor_msgs)


find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(sift
  include/sift/sift_main.hpp
  src/sift_main.cpp
  src/sift.cpp)
target_link_libraries(sift ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS sift
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(orb
  src/orb.cpp
  include/sift/orb_main.hpp
  src/orb_main.cpp
)
add_dependencies(orb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(orb
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)