cmake_minimum_required(VERSION 2.8)
project(AntennaDetect-SemiDenseReconstruction)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/Config
${PROJECT_SOURCE_DIR}/src/ComData
${PROJECT_SOURCE_DIR}/src/AntennaDetect
${PROJECT_SOURCE_DIR}/src/MoveSense
${PROJECT_SOURCE_DIR}/src/PointCloudProcessing
${PROJECT_SOURCE_DIR}/src/VSLAM/include
${PROJECT_SOURCE_DIR}/src/VSLAM/imu
${PROJECT_SOURCE_DIR}/src/VSLAM
${PROJECT_SOURCE_DIR}/src/DepthEstimation
${PROJECT_SOURCE_DIR}/src/DepthEstimation/include
${PROJECT_SOURCE_DIR}/Thirdparty/vikit_common/include
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus/sophus
${PROJECT_SOURCE_DIR}/thread/AntennaDetectThread
${PROJECT_SOURCE_DIR}/thread/PointCloudProcessingThread
${PROJECT_SOURCE_DIR}/thread/VSLAMThread
${PROJECT_SOURCE_DIR}/thread/ViewThread
${PROJECT_SOURCE_DIR}/thread/DepthEstimationThread
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}

)
################################ main thread ################################
add_library(${PROJECT_NAME} SHARED
src/ComData/ComData.cpp
Config/Config.cpp

#AntennaDetectThread
src/AntennaDetect/AntennaDetect.cpp
thread/AntennaDetectThread/AntennaDetectThread.cpp

#PointCloudProcessingThread
src/PointCloudProcessing/BilateralInterPolation.cpp
src/PointCloudProcessing/Denoise3DPoint.cpp
src/PointCloudProcessing/Method_DensityCluster.cpp
src/PointCloudProcessing/Method_DepthConnectedcheck.cpp
thread/PointCloudProcessingThread/PointCloudProcessingThread.cpp

#VSLAMThread
src/VSLAM/src/Converter.cc
src/VSLAM/src/fast.cpp
src/VSLAM/src/fast_score.cpp
src/VSLAM/src/Frame.cc
src/VSLAM/src/FrameDrawer.cc
src/VSLAM/src/Initializer.cc
src/VSLAM/src/KeyFrame.cc
src/VSLAM/src/KeyFrameDatabase.cc
src/VSLAM/src/LocalMapping.cc
src/VSLAM/src/LoopClosing.cc
src/VSLAM/src/Map.cc
src/VSLAM/src/MapDrawer.cc
src/VSLAM/src/MapPoint.cc
src/VSLAM/src/Optimizer.cc
src/VSLAM/src/ORBextractor.cc
src/VSLAM/src/ORBmatcher.cc
src/VSLAM/src/PnPsolver.cc
src/VSLAM/src/Sim3Solver.cc
src/VSLAM/src/System.cc
src/VSLAM/src/Tracking.cc
src/VSLAM/src/Viewer.cc
src/VSLAM/src/VSLAM.cc
src/VSLAM/imu/eigen_utils.cpp
src/VSLAM/imu/IMU.cpp
thread/VSLAMThread/VSLAMThread.cpp

#DepthEstimation
src/DepthEstimation/DepthEstimation.cpp
src/DepthEstimation/DynamicEstimation.cpp
src/DepthEstimation/StaticEstimation.cpp
src/DepthEstimation/src/feature_alignment.cpp
src/DepthEstimation/src/frame.cpp
src/DepthEstimation/src/matcher.cpp
src/DepthEstimation/src/point.cpp
src/DepthEstimation/src/config.cpp
thread/DepthEstimationThread/DepthEstimationThread.cpp
#Viewthread
thread/ViewThread/ViewThread.cpp

#MoveSense
src/MoveSense/Camera.cpp
src/MoveSense/CameraCtrl.cpp
src/MoveSense/CtrlInf.cpp
src/MoveSense/MoveSenseCamera.cpp
src/MoveSense/runCamera2.cpp
src/MoveSense/VL4IF.cpp
)


target_link_libraries(  ${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus/lib/libSophus.so
${PROJECT_SOURCE_DIR}/Thirdparty/fast/lib/libfast.so
${PROJECT_SOURCE_DIR}/Thirdparty/vikit_common/lib/libvikit_common.so
/usr/lib/x86_64-linux-gnu/libboost_thread.so
/usr/lib/x86_64-linux-gnu/libboost_system.so
/usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/usr/lib/x86_64-linux-gnu/libusb-1.0.so
/usr/lib/x86_64-linux-gnu/libudev.so
/usr/lib/x86_64-linux-gnu/libusb.so
/usr/lib/x86_64-linux-gnu/libv4l1.so
)

add_executable(mainThread thread/mainThread.cpp)
target_link_libraries(mainThread ${PROJECT_NAME})

add_executable(mainThread-offline thread/mainThread-offline.cpp)
target_link_libraries(mainThread-offline ${PROJECT_NAME})

################################    test AntennaDetect  #################################
#test AntennaDetect
add_library(AntennaDetect SHARED
src/AntennaDetect/AntennaDetect.cpp
#MoveSense
src/MoveSense/Camera.cpp
src/MoveSense/CameraCtrl.cpp
src/MoveSense/CtrlInf.cpp
src/MoveSense/MoveSenseCamera.cpp
src/MoveSense/runCamera2.cpp
src/MoveSense/VL4IF.cpp
)

target_link_libraries(AntennaDetect
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
/usr/lib/x86_64-linux-gnu/libboost_system.so
/usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/usr/lib/x86_64-linux-gnu/libusb-1.0.so
/usr/lib/x86_64-linux-gnu/libudev.so
/usr/lib/x86_64-linux-gnu/libusb.so
/usr/lib/x86_64-linux-gnu/libv4l1.so
)
add_executable(
test_AntennaDetect src/AntennaDetect/testdetect.cpp
)
target_link_libraries(
test_AntennaDetect AntennaDetect
)

################################    test PointCloudProcessing  #################################
add_library(PointCloudProcessing SHARED
src/PointCloudProcessing/BilateralInterPolation.cpp
src/PointCloudProcessing/Denoise3DPoint.cpp
src/PointCloudProcessing/Method_DensityCluster.cpp
src/PointCloudProcessing/Method_DepthConnectedcheck.cpp
#MoveSense
src/MoveSense/Camera.cpp
src/MoveSense/CameraCtrl.cpp
src/MoveSense/CtrlInf.cpp
src/MoveSense/MoveSenseCamera.cpp
src/MoveSense/runCamera2.cpp
src/MoveSense/VL4IF.cpp
)

target_link_libraries(PointCloudProcessing
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
/usr/lib/x86_64-linux-gnu/libboost_system.so
/usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/usr/lib/x86_64-linux-gnu/libusb-1.0.so
/usr/lib/x86_64-linux-gnu/libudev.so
/usr/lib/x86_64-linux-gnu/libusb.so
/usr/lib/x86_64-linux-gnu/libv4l1.so
)
add_executable(
test_PointCloudProcessing src/PointCloudProcessing/testPoint3DProcesing.cpp
)
target_link_libraries(
test_PointCloudProcessing PointCloudProcessing
)
################################# test VSLAM ################################
add_library(VSLAM SHARED
#ORB
src/VSLAM/src/Converter.cc
src/VSLAM/src/fast.cpp
src/VSLAM/src/fast_score.cpp
src/VSLAM/src/Frame.cc
src/VSLAM/src/FrameDrawer.cc
src/VSLAM/src/Initializer.cc
src/VSLAM/src/KeyFrame.cc
src/VSLAM/src/KeyFrameDatabase.cc
src/VSLAM/src/LocalMapping.cc
src/VSLAM/src/LoopClosing.cc
src/VSLAM/src/Map.cc
src/VSLAM/src/MapDrawer.cc
src/VSLAM/src/MapPoint.cc
src/VSLAM/src/Optimizer.cc
src/VSLAM/src/ORBextractor.cc
src/VSLAM/src/ORBmatcher.cc
src/VSLAM/src/PnPsolver.cc
src/VSLAM/src/Sim3Solver.cc
src/VSLAM/src/System.cc
src/VSLAM/src/Tracking.cc
src/VSLAM/src/Viewer.cc
src/VSLAM/src/VSLAM.cc
src/VSLAM/imu/eigen_utils.cpp
src/VSLAM/imu/IMU.cpp
#MoveSense
src/MoveSense/Camera.cpp
src/MoveSense/CameraCtrl.cpp
src/MoveSense/CtrlInf.cpp
src/MoveSense/MoveSenseCamera.cpp
src/MoveSense/runCamera2.cpp
src/MoveSense/VL4IF.cpp
)

target_link_libraries(VSLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}//Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus/lib/libSophus.so
${PROJECT_SOURCE_DIR}/Thirdparty/fast/lib/libfast.so
/usr/lib/x86_64-linux-gnu/libboost_thread.so
/usr/lib/x86_64-linux-gnu/libboost_system.so
/usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/usr/lib/x86_64-linux-gnu/libusb-1.0.so
/usr/lib/x86_64-linux-gnu/libudev.so
/usr/lib/x86_64-linux-gnu/libusb.so
/usr/lib/x86_64-linux-gnu/libv4l1.so
)
add_executable(
test_VSLAM src/VSLAM/tests/online_test.cc
)
target_link_libraries(
test_VSLAM VSLAM
)
