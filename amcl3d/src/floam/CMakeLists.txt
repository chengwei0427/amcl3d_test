# csparse
LIST(APPEND CMAKE_MODULE_PATH ${3RDPARTY_DIR}/g2o/g2o2017_CSparse/cmake_modules)
find_package( CSparse REQUIRED)
find_package(Ceres REQUIRED)
include_directories( 
    include
    ${CSPARSE_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS})

link_directories(
    include
    ${CERES_LIBRARY_DIRS} )


add_executable(floam_laserProcessing_amcl3d src/laserProcessingNode.cpp src/laserProcessingClass.cpp src/lidar.cpp)
target_link_libraries(floam_laserProcessing_amcl3d ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(floam_odomEstimation_amcl3d src/odomEstimationNode.cpp src/lidarOptimization.cpp src/lidar.cpp src/odomEstimationClass.cpp)
target_link_libraries(floam_odomEstimation_amcl3d ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})
