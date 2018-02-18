cmake_minimum_required(VERSION 2.8.3)
project(hockey_game)


set( CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        cv_bridge
        image_transport
        sensor_msgs
        geometry_msgs
        pcl_ros
)

#c++11
if (CMAKE_VERSION VERSION_LESS "3.1")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else()
        CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
        # -std=c++0x is deprecated but some tools e.g. qmake or older gcc are still using it
        if(COMPILER_SUPPORTS_CXX0X)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
        else()
            message(FATAL_ERROR "Compiler ${CMAKE_CXX_COMPILER} does not support c++11/c++0x")
        endif()
    endif()
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

#link liberary
set( PROJECT_LINK_LIBS libreferee.a)
LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES tf_corner
# CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf
# DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
   # ${OpenCV_INCLUDE_DIR}
)

set( QT_USE_QTNETWORK TRUE )
set( QT_USE_QTXML TRUE )

find_package( Qt4 REQUIRED )
find_package(OpenCV REQUIRED)

include( ${QT_USE_FILE} )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_LIST)
#INCLUDE_DIRECTORIES(
#    ${catkin_INCLUDE_DIRS}
#    ${OpenCV_INCLUDE_DIRS}
#)


set(MOC_H
  ${CMAKE_CURRENT_SOURCE_DIR}/src/decider.h
)

QT4_WRAP_CPP( MOC_CPP ${MOC_H} )


#add_executable(${PROJECT_NAME} ${SRC_LIST} ${MOC_CPP})
add_executable(game ${SRC_LIST} ${MOC_CPP})

#add_executable(lidar src/lidar.h src/lidar.cpp)
#add_executable(fsm src/FSM.h src/FSM.cpp src/lidar.h src/lidar.cpp hermes/referee.h hermes/referee.cpp
#src/classification.cpp src/classification.h src/connect.h src/connect.cpp src/sendalive.cpp src/sendalive.h
#src/sendxy.h src/sendxy.cpp)
#add_executable(camera src/classification.h src/classification.cpp)
#add_executable(rbtpos src/robot_pos.h src/robot_pos.cpp)
#add_executable(schiri hermes/referee.h hermes/referee.cpp)

#target_link_libraries(lidar ${catkin_INCLUDE_LIBRARIES} ${QT_LIBRARIES} )
#target_link_libraries(fsm ${catkin_INCLUDE_LIBRARIES} ${QT_LIBRARIES} ${OpenCV_INCLUDE_LIBRARIES} )
#target_link_libraries(camera ${catkin_INCLUDE_LIBRARIES} ${QT_LIBRARIES} ${OpenCV_INCLUDE_LIBRARIES} )
#target_link_libraries(rbtpos ${catkin_INCLUDE_LIBRARIES} ${QT_LIBRARIES} )
#target_link_libraries(schiri ${catkin_INCLUDE_LIBRARIES} ${QT_LIBRARIES} ${OpenCV_INCLUDE_LIBRARIES})

#target_link_libraries(${PROJECT_NAME}
target_link_libraries(game
        referee
  ${PROJECT_LINK_LIBS}
        ${catkin_LIBRARIES}
        ${QT_LIBRARIES}
        ${OpenCV_LIBS}
)
