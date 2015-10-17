cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        cv_bridge 
                                        image_transport
                                        image_geometry
                                        tf)
                                        
find_package(OpenCV REQUIRED)

project(simple_visual_odometry)

catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS})

#build the tracking node
file(GLOB_RECURSE VO_SOURCE 
                  src/*.cpp) 

add_executable(${PROJECT_NAME} ${VO_SOURCE})
                            
                    
                                 
#link libriaries
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
                       
#add dependencies to generated messages
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
