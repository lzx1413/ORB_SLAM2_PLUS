cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

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
set(CMAKE_PREFIX_PATH "/usr/local/opencv3/share/OpenCV")
find_package(OpenCV 3.4.3 REQUIRED)
#if(NOT OpenCV_FOUND)
#   find_package(OpenCV 2.4.3 QUIET)
#   if(NOT OpenCV_FOUND)
#      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#   endif()
#endif()
#set(OPENCV_LIB_PATH "/usr/local/opencv3/lib")

#set(OPENCV_INCLUDE_PATH "/usr/local/opencv3/include")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
#${OPENCV_INCLUDE_PATH}
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS}
 #       ${OPENCV_LIB_PATH}
        )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
        src/PointCloudMapping.cc src/InstanceSeg.cc include/InstanceSeg.h include/mxnet_utils.h)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
 ${MXNET_LIBS} ${CUDA_LIBRARIES} ${CUDA_CUFFT_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES}       ${PCL_LIBRARIES}
)

message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc Examples/RGB-D/instance_seg_test.cpp)
target_link_libraries(rgbd_tum ${PROJECT_NAME})
add_executable(rgbd_tum_dense
        Examples/RGB-D/rgbd_my.cc)
target_link_libraries(rgbd_tum_dense ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})


add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(live_demo Examples/Monocular/live_demo.cpp)
target_link_libraries(live_demo ${PROJECT_NAME})
add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary)
add_executable(bin_vocabulary Vocabulary/bin_vocabulary.cpp)
target_link_libraries(bin_vocabulary ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so ${OpenCV_LIBS})

set(MXNET_ROOT /home/zuoxin/workspace/incubator-mxnet/)
FIND_PACKAGE(CUDA REQUIRED)
include_directories(
        ${MXNET_ROOT}/include
        ${MXNET_ROOT}/cpp-package/include
        ${MXNET_ROOT}/3rdparty/dlpack/include
        ${MXNET_ROOT}/3rdparty/dmlc-core/include
        ${MXNET_ROOT}/3rdparty/mkldnn/include
        ${MXNET_ROOT}/3rdparty/mshadow
        ${MXNET_ROOT}/3rdparty/tvm/nnvm/include
        Examples/RGB-D/
)
SET(CMAKE_FIND_LIBRARY_PREFIXES "lib")
find_library(MXNET_LIBS NAMES mxnet HINTS "${MXNET_ROOT}/lib")
message(STATUS MXNET_LIBS: ${MXNET_LIBS})
add_executable(mx_instance Examples/RGB-D/mx_instance_seg.cpp )
target_link_libraries(mx_instance ${OpenCV_LIBS} ${MXNET_LIBS} ${CUDA_LIBRARIES} ${CUDA_CUFFT_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES})
#test case
add_executable(instance_seg_test
        Examples/RGB-D/instance_seg_test.cpp src/InstanceSeg.cc)
target_link_libraries(instance_seg_test ${OpenCV_LIBS} ${MXNET_LIBS} ${CUDA_LIBRARIES} ${CUDA_CUFFT_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES})

add_executable(rgbd_tum_obj
        Examples/OBJSLAM/tum_rgbd_obj.cc
        src/ObjectInstance.cc include/ObjectInstance.h src/ObjectManager.cc include/ObjectManager.h include/utils.h src/utils.cc src/InstanceSeg.cc include/InstanceSeg.h)
target_link_libraries(rgbd_tum_obj ${PROJECT_NAME} jsoncpp ${OpenCV_LIBS} ${MXNET_LIBS} ${CUDA_LIBRARIES} ${CUDA_CUFFT_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES})

if(USE_EXAMPLE_WEBCAM)
    find_package(PkgConfig REQUIRED)
    pkg_search_module(CURL REQUIRED libcurl)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    add_executable(mono_uvc Examples/mono_uvc.cpp)
    target_link_libraries(mono_uvc ${PROJECT_NAME})
    add_executable(mono_android_ipcam Examples/mono_android_ipcam.cpp)
    target_link_libraries(mono_android_ipcam ${PROJECT_NAME} ${CURL_LIBRARIES})
    add_executable(mono_android_ipcam_ar Examples/mono_android_ipcam_ar.cpp Examples/AR/ViewerAR.cc)
    target_include_directories(mono_android_ipcam_ar PUBLIC Examples/AR/)
    target_link_libraries(mono_android_ipcam_ar ${PROJECT_NAME} ${CURL_LIBRARIES})
endif()