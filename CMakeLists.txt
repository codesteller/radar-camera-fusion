cmake_minimum_required(VERSION 3.20)
project(adas_fusion)

# cmake_policy(SET CMP0146 OLD)

set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86 89 90)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Release)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# CUDA
find_package(CUDA REQUIRED)
# message(STATUS "CUDA Libs: \n${CUDA_LIBRARIES}\n")
get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
# message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

# Opencv
set(OpenCV_DIR /home/pallab/Apsoft/dev-utils/cv2/installation/OpenCV-4.8.1/lib/cmake/opencv4/)
find_package(OpenCV REQUIRED)
# message(STATUS "OpenCV Libs: \n${OpenCV_LIBS}\n")
# message(STATUS "OpenCV Libraries: \n${OpenCV_LIBRARIES}\n")
# message(STATUS "OpenCV Headers: \n${OpenCV_INCLUDE_DIRS}\n")

# TensorRT
set(TensorRT_DIR /opt/nvidia/TensorRT-8.6.1.6)
set(TensorRT_INCLUDE_DIRS ${TensorRT_DIR}/include)
set(TensorRT_LIBRARIES ${TensorRT_DIR}/lib)
# message(STATUS "TensorRT Libs: \n${TensorRT_LIBRARIES}\n")
# message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")

# YAML-CPP
set(YAML-CPP_DIR /home/pallab/Apsoft/dev-utils/yaml-parser/yaml-cpp/install/lib/cmake/yaml-cpp/)
find_package(YAML-CPP REQUIRED)
message(STATUS "YAML-CPP Libs: ${YAML_CPP_LIBRARIES}\n")

# Eigen
set(EIGEN_VERSION 3.3)
find_package(Eigen3 ${EIGEN_VERSION} QUIET)
if(NOT EIGEN3_FOUND)
  set(BUILD_TESTING OFF CACHE INTERNAL "")
  FetchContent_Declare(eigen
    GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
    GIT_TAG         ${EIGEN_VERSION}
    GIT_SHALLOW     ON)
  FetchContent_MakeAvailable(eigen)
  unset(BUILD_TESTING CACHE)
endif()


# TBB
set(TBB_DIR /opt/intel/oneTBB/libs/cmake/TBB/)
find_package(TBB CONFIG REQUIRED tbb)
set(TBB_LIBS TBB::tbb)
# message(STATUS "TBB Libs: ${TBB_LIBS}\n")
set(TBB_INSTALL_DIR /opt/intel/oneTBB)
set(TBB_INCLUDE_DIRS ${TBB_INSTALL_DIR}/include)
set(TBB_LIBRARIES ${TBB_INSTALL_DIR}/lib)

list(APPEND INCLUDE_DIRS
        ${CUDA_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        ${TBB_INCLUDE_DIRS}
        include
        )

list(APPEND ALL_LIBS
        ${CUDA_LIBRARIES}
        ${CUDA_LIB_DIR}
        ${OpenCV_LIBRARIES}
        ${TensorRT_LIBRARIES}
        ${TBB_LIBRARIES}
        )

include_directories(${INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
        src/main.cpp
        include/yolov8.hpp
        include/common.hpp
        src/BYTETracker.cpp
        src/Object.cpp 
        src/Rect.cpp 
        src/STrack.cpp 
        src/lapjv.cpp
        src/KalmanFilter.cpp
        )

target_link_directories(${PROJECT_NAME} PUBLIC ${ALL_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE nvinfer nvinfer_plugin cudart ${TBB_LIBS} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} Eigen3::Eigen)
