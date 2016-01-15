cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        cv_bridge 
                                        image_transport
                                        image_geometry
                                        tf2_ros
                                        nav_msgs
                                        visualization_msgs)
                                        
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

project(simple_visual_odometry)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include five-point-nister ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_subdirectory(five-point-nister)

#build the tracking node
file(GLOB_RECURSE VO_SOURCE 
          src/*.cpp) 

add_executable(${PROJECT_NAME} ${VO_SOURCE})

add_executable(five_points_test test/testFivePoints.cpp)
add_executable(pnp_test test/testPnP.cpp)                    
                                 
#link libriaries
target_link_libraries(${PROJECT_NAME} five-point-nister ${catkin_LIBRARIES})
target_link_libraries(five_points_test five-point-nister ${catkin_LIBRARIES})
target_link_libraries(pnp_test ${catkin_LIBRARIES})
                       
#add dependencies to generated messages
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
