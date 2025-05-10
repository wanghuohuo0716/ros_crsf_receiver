cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(crsf_receiver)
#list(APPEND CMAKE_PREFIX_PATH "/usr/local")
#list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/CppLinuxSerial")
# 设置 C++ 标准（ROS1 Melodic 默认支持 C++14，但可以手动指定更高版本）
# set(CMAKE_CXX_STANDARD 14)  # ROS1 Melodic 默认不支持 C++20
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

# 编译器警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# 查找 ROS1 核心依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  crsf_receiver_msg  # 你的消息包
)

# 查找第三方库（如 CppLinuxSerial）
find_package(CppLinuxSerial REQUIRED)

# 添加可执行文件
add_executable(crsf_receiver_node
  src/crsf_receiver_node.cpp
  src/crsf_receiver.cpp
  src/crsf_parser.cpp
  src/crc8.cpp
  src/utils.cpp
)

# 包含头文件目录
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CppLinuxSerial_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(crsf_receiver_node
  ${catkin_LIBRARIES}
  CppLinuxSerial::CppLinuxSerial  # 使用命名空间目标
)

# 安装规则（ROS1 使用 CATKIN_PACKAGE_BIN_DESTINATION）
install(TARGETS crsf_receiver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 生成 catkin 包
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs crsf_receiver_msg
)