cmake_minimum_required(VERSION 3.20)
project(adas_fusion)
include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86 89 90)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(TBB CONFIG REQUIRED tbb)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Add TensorRT if separate installation
# TensorRT
set(TensorRT_DIR /opt/nvidia/TensorRT-8.6.1.6)
set(TensorRT_INCLUDE_DIRS ${TensorRT_DIR}/include)
set(TensorRT_LIBRARIES ${TensorRT_DIR}/lib)
# message(STATUS "TensorRT Libs: \n${TensorRT_LIBRARIES}\n")
# message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")

get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)

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
        ${TBB_LIBS}
        )

        
link_libraries(stdc++fs)

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
        src/birdview_model.cpp 
        src/camera_model.cpp
        )

target_link_directories(${PROJECT_NAME} PUBLIC ${ALL_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE nvinfer nvinfer_plugin cudart ${TBB_LIBS} ${TBB_LIBRARIES} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} Eigen3::Eigen)
