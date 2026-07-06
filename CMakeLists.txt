cmake_minimum_required(VERSION 3.14)
project(fire_detect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# ONNXRuntime 请根据你的实际路径修改
include_directories(/usr/local/onnxruntime/include)
link_directories(/usr/local/onnxruntime/lib)

# gpiod
link_libraries(gpiod pthread)

# 源文件
file(GLOB SRC_LIST
    main.cpp
    common.cpp
    hardware.cpp
    yolov8_infer.cpp
    bg_subtract.cpp
    ui_draw.cpp
)

add_executable(fire_detect ${SRC_LIST})
target_link_libraries(fire_detect ${OpenCV_LIBS} onnxruntime)