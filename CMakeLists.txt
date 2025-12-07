cmake_minimum_required(VERSION 3.5)
project(orbslam3)

set(ENV{PYTHONPATH} "/opt/ros/kilted/lib/python3.12/site-packages/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Allow CMake to locate our custom Find modules (e.g. FindORB_SLAM3.cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")


include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}     
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${DBoW2_INCLUDE_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

set(OPENCV_LIBS
  ${OpenCV_LIBS}
  opencv_calib3d
  opencv_core
  opencv_imgproc
  opencv_highgui
)

### Monocular
ament_auto_add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
target_link_libraries(mono ${EXTRA_LIBS} ${OPENCV_LIBS})

### Monocular inertial
ament_auto_add_executable(monocular_inertial
  src/monocular-inertial/mono-inertial.cpp
  src/monocular-inertial/monocular-inertial-node.cpp
)
target_link_libraries(monocular_inertial ${EXTRA_LIBS} ${OPENCV_LIBS})

### rgbd
ament_auto_add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
target_link_libraries(rgbd ${EXTRA_LIBS} ${OPENCV_LIBS})

### stereo
ament_auto_add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
target_link_libraries(stereo ${EXTRA_LIBS} ${OPENCV_LIBS})

### stereo-inertial
ament_auto_add_executable(stereo_inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
target_link_libraries(stereo_inertial ${EXTRA_LIBS} ${OPENCV_LIBS})

install(DIRECTORY launch config vocabulary
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS mono monocular_inertial rgbd stereo stereo_inertial
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
