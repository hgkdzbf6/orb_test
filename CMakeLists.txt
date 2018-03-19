cmake_minimum_required(VERSION 2.8.3)
project(sift)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(CMAKE_CXX_FLAGS "-std=c++11")
find_package(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge image_transport camera_info_manager sensor_msgs geometry_msgs)


find_package(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

catkin_package(
  DEPENDS Eigen OpenCV  
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(sift
  include/sift/sift_main.hpp
  src/sift_main.cpp
  src/sift.cpp)
target_link_libraries(sift ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS sift
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(orb
  include/sift/common.hpp
  src/orb.cpp
  include/sift/orb_main.hpp
  src/orb_main.cpp
)
add_dependencies(orb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(orb
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)