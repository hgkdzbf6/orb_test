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
  ${Eigen_LIBRARIES}
)

# add_executable(sift
#   include/sift/sift_main.hpp
#   src/sift_main.cpp
#   src/sift.cpp)
# add_dependencies(sift ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(sift ${LINK_LIBS})

# install(TARGETS sift
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(orb
  include/sift/lib/orb_main.hpp
  include/sift/node/orb_node.hpp
  src/orb_node.cpp
  src/orb_main.cpp
)
add_dependencies(orb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(orb
  ${LINK_LIBS}
)

# add_executable(surf
#   include/sift/surf_main.hpp
#   src/surf.cpp
#   src/surf_main.cpp
# )
# add_dependencies(surf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(surf
#   ${LINK_LIBS}
# )