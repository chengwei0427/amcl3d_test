project(amcl3d)
cmake_minimum_required(VERSION 3.5.1)

# add_compile_options(-std=c++11)
set (CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -o3 -w -fpermissive -fPIC")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
##################################
## Configure CATKIN dependecies ##
##################################

find_package(catkin REQUIRED
  COMPONENTS
    visualization_msgs
    nav_msgs
    # octomap_ros
    pcl_ros
    rosinrange_msg
)

###################################
## CATKIN specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    visualization_msgs
    nav_msgs
    # octomap_ros
    pcl_ros
    rosinrange_msg
)

#######################
## Configuring GMock ##
#######################

find_package(GMock)

#######################
## Configuring 3d party ##
#######################
set(3RDPARTY_DIR ${PROJECT_SOURCE_DIR}/../../../../workspace/3rdparty)
#algo inc & lib
set(ALGO_HEADER_DIR ${3RDPARTY_DIR}/../vscommon/inc)
set(ALGO_LIB_DIR ${3RDPARTY_DIR}/../vscommon/lib)
#eigen
set(EIGEN_HEADER_DIR ${3RDPARTY_DIR}/Eigen/include/eigen3)
#boost
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS log thread system regex date_time serialization filesystem program_options timer chrono REQUIRED)
set(BOOST_HEADER_DIR ${Boost_INCLUDE_DIRS})
#flann
set(FLANN_HEADER_DIR ${3RDPARTY_DIR}/flann-1.8.4/include)
set(FLANN_LIB_DIR ${3RDPARTY_DIR}/flann-1.8.4/lib)
#log4cpp
set(LOG4CPP_HEADER_DIR ${3RDPARTY_DIR}/log4cpp/include)
set(LOG4CPP_LIB_DIR ${3RDPARTY_DIR}/log4cpp/lib)
#tinyxml2
set(TINYXML2_HEADER_DIR ${3RDPARTY_DIR}/tinyxml2/include)
set(TINYXML2_LIB_DIR ${3RDPARTY_DIR}/tinyxml2/lib)
#include opencv
find_package( OpenCV REQUIRED )
set(OPENCV_HEADER_DIR ${OpenCV_INCLUDE_DIRS})
###########
## Build ##
###########

include_directories(SYSTEM 
  ${catkin_INCLUDE_DIRS}
  ${ALGO_HEADER_DIR}
  ${ALGO_HEADER_DIR}/robcommon
  ${EIGEN_HEADER_DIR}
  ${BOOST_HEADER_DIR}
  ${FLANN_HEADER_DIR}
  ${LOG4CPP_HEADER_DIR}
  # ${GTEST_HEADER_DIR}
  ${TINYXML2_HEADER_DIR}
  ${OPENCV_HEADER_DIR}
  )

  link_directories(
                ${Boost_LIBRARIES}
                ${OpenCV_LIBRARIES}
                ${ALGO_LIB_DIR}
                ${FLANN_LIB_DIR}
                ${LOG4CPP_LIB_DIR}
                # ${GTEST_LIB_DIR}
                ${TINYXML2_LIB_DIR}
)

link_libraries(
              libtinyxml2.a
              liblog4cpp.a
              # libgtest.a
              libflann_s.a
              libflann_cpp_s.a
              ${Boost_LIBRARIES}
              ${OpenCV_LIBRARIES}
              libvscommon.a)

add_library(${PROJECT_NAME} STATIC
  src/Grid3d.cpp
  src/Grid3d.h
  src/amcl3d.cpp
  src/amcl3d.h
  src/ParticleFilter.cpp
  src/ParticleFilter.h
  src/PointCloudTools.cpp
  src/PointCloudTools.h)
add_dependencies(${PROJECT_NAME}      ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/Node.cpp
  src/Node.h
  # src/Parameters.cpp
  src/Parameters.h)
add_dependencies(${PROJECT_NAME}_node       ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node  
    ${catkin_LIBRARIES} ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    libtinyxml2.a
    liblog4cpp.a
    # libgtest.a
    libflann_s.a
    libflann_cpp_s.a
    libvscommon.a
    liblocation.a
    )

#############
## floam ##
#############
add_subdirectory(${PROJECT_SOURCE_DIR}/src/floam)
#############
## Testing ##
#############

if(GMOCK_FOUND)
  include_directories(src)
  include_directories(SYSTEM ${GMOCK_INCLUDE_DIRS})

  file(GLOB_RECURSE TESTS_SRCS  "tests/*.c*")
  file(GLOB_RECURSE TESTS_HDRS  "tests/*.h*")

  catkin_add_gtest(${PROJECT_NAME}_tests
    ${TESTS_SRCS} ${TESTS_HDRS})
  if(TARGET ${PROJECT_NAME}_tests)
    target_compile_definitions(${PROJECT_NAME}_tests PRIVATE
      PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
    target_link_libraries(${PROJECT_NAME}_tests
      ${GMOCK_BOTH_LIBRARIES} ${PROJECT_NAME})
  endif()
endif()
