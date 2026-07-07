cmake_minimum_required(VERSION 3.10)
project(fire_detect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找 OpenCV
find_package(OpenCV REQUIRED)

# 手动指定 ONNX Runtime 路径（在 thirdparty 里）
set(ONNX_ROOT ${CMAKE_SOURCE_DIR}/thirdparty/onnxruntime)
set(ONNX_INCLUDE_DIR ${ONNX_ROOT}/include)
set(ONNX_LIB_DIR ${ONNX_ROOT}/lib)

# 所有源文件
set(SOURCES
    main.cpp
    common.cpp
    hardware.cpp
    bg_subtract.cpp
    ui_draw.cpp
    yolov8_infer.cpp
)

# 头文件路径
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${ONNX_INCLUDE_DIR}
)

# 库文件路径
link_directories(${ONNX_LIB_DIR})

# 生成可执行文件
add_executable(fire_detect ${SOURCES})

# 链接库
target_link_libraries(fire_detect
    ${OpenCV_LIBS}
    onnxruntime
    gpiod
    pthread
)