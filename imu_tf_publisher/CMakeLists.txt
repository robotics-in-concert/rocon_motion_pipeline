cmake_minimum_required(VERSION 2.8.3)
project(imu_tf_publisher)

find_package(catkin REQUIRED roscpp
                             tf
                             geometry_msgs)
                             
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(imu_tf_publisher src/imu_tf_publisher.cpp)
target_link_libraries(imu_tf_publisher ${catkin_LIBRARIES})

install(TARGETS imu_tf_publisher
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})