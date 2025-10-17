cmake_minimum_required(VERSION 3.8)
project(mos_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Explicitly tell CMake where to find the gRPC and Protobuf packages
list(APPEND CMAKE_PREFIX_PATH "/vcpkg/installed/arm64-linux")
find_package(gRPC CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)

# ===================================================================
# Protobuf and gRPC Code Generation
# ===================================================================

# 1. Find the gRPC C++ plugin executable
list(APPEND CMAKE_PREFIX_PATH "/vcpkg/installed/arm64-linux/tools/grpc")
find_program(GRPC_CPP_PLUGIN NAMES grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN)
    message(FATAL_ERROR "grpc_cpp_plugin not found!")
endif()

# 2. Define file paths
set(PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../mos-core/proto/robot_controller.v1.proto)
get_filename_component(PROTO_BASENAME ${PROTO_FILE} NAME_WE)
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)

# 创建生成目录
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

# 3. 生成 Protobuf 消息文件
protobuf_generate(
    LANGUAGE cpp
    OUT_VAR PROTO_SRC
    PROTOS ${PROTO_FILE}
    IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../mos-core/proto
    APPEND_PATH
)

# 4. 生成 gRPC 服务文件
protobuf_generate(
    LANGUAGE grpc
    OUT_VAR GRPC_SRC
    PROTOS ${PROTO_FILE}
    IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../mos-core/proto
    PLUGIN "protoc-gen-grpc=${GRPC_CPP_PLUGIN}"
    GENERATE_EXTENSIONS .grpc.pb.cc .grpc.pb.h
    APPEND_PATH
)

# 5. 合并生成的源文件
set(PROTO_GENERATED_SOURCES ${PROTO_SRC} ${GRPC_SRC})

# 打印调试信息
message(STATUS "Generated Protobuf files: ${PROTO_SRC}")
message(STATUS "Generated gRPC files: ${GRPC_SRC}")

# ===================================================================
# 可执行文件定义
# ===================================================================

add_executable(ros2_grpc_server
    src/ros2_grpc_server.cpp
    ${PROTO_GENERATED_SOURCES}
)

# 添加包含路径（这是关键！）
target_include_directories(ros2_grpc_server PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Link ROS dependencies using ament
ament_target_dependencies(ros2_grpc_server
    rclcpp
    trajectory_msgs
)

# Link system dependencies (from vcpkg)
target_link_libraries(ros2_grpc_server
    gRPC::grpc++
    protobuf::libprotobuf
)

# 安装
install(TARGETS
    ros2_grpc_server
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()