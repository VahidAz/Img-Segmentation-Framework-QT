cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release)
project(jur_gui_segmentation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             sensor_msgs
             message_generation
             cv_bridge
)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

find_package(OpenCV)

find_package(OpenMP QUIET)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Qwt6 REQUIRED)

include(${QT_USE_FILE})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${QWT6_INCLUDE_DIR}
    ${OpenMP_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

#LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/glnx86/)

add_service_files(FILES jur_segmentation_service.srv)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
    CATKIN_DEPENDS
        sensor_msgs
        message_runtime
)

## Qt
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
set(QT_MOC
    include/jur_gui_segmentation/JURGUISubClassQGV.h
    include/jur_gui_segmentation/JURGUI2DWindow.h
    include/jur_gui_segmentation/JURGUIMainWindow.h
    include/jur_gui_segmentation/JURGUIPlotWidget.h
    include/jur_gui_segmentation/JURGUIROSNode.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

## Declare a cpp executable
add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_LIBRARIES} ${Qwt6_Qt4_LIBRARY} ${Qwt6_Qt3_LIBRARY})
add_dependencies(${PROJECT_NAME} jur_gui_segmentation_gencpp)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
)
