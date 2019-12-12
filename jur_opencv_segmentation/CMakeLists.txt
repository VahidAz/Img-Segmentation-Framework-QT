cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release)
project(jur_opencv_segmentation)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             cv_bridge
             jur_gui_segmentation
)

find_package(OpenCV)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        juropencvmeanshift
        juropencvwatershed
        juropencvsegmentation
    CATKIN_DEPENDS
        roscpp
        cv_bridge
        jur_gui_segmentation
)

## Declare a cpp library
add_library(juropencvmeanshift src/JUROpenCVMeanShift.cpp)
target_link_libraries(juropencvmeanshift ${OpenCV_LIBRARIES})

add_library(juropencvwatershed src/JUROpenCVWatershed.cpp)
target_link_libraries(juropencvwatershed ${OpenCV_LIBRARIES})

add_library(juropencvsegmentation src/JUROpenCVSegmentation.cpp)
target_link_libraries(juropencvsegmentation juropencvmeanshift juropencvwatershed ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(jur_opencv_segmentation_node src/JUROpenCVSegmentationNode.cpp)
target_link_libraries(jur_opencv_segmentation_node juropencvsegmentation ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS
            jur_opencv_segmentation_node
            juropencvsegmentation
            juropencvmeanshift
            juropencvwatershed
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
