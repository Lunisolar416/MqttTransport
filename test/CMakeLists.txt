cmake_minimum_required(VERSION 3.10)

# 项目名称和语言
project(PolygonPointTest LANGUAGES CXX)

# C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Eigen3 3.3 REQUIRED NO_MODULE)


find_package(PahoMqttCpp REQUIRED)

# 添加可执行文件 - 测试多边形
add_executable(test_polygon
    example.cc
    polygon_point_singleton.cc
)

# 添加可执行文件 - 发布者
add_executable(publisher_0042
    pub.cc
    polygon_point_singleton.cc
)

# 包含头文件路径
target_include_directories(test_polygon PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

target_include_directories(publisher_0042 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PAHO_MQTT_CPP_INCLUDE_DIRS}  # Paho MQTT C++ 头文件
)

target_link_libraries(test_polygon PRIVATE
    Eigen3::Eigen
    paho-mqttpp3
    paho-mqtt3as
    pthread
)
# 链接外部库
target_link_libraries(publisher_0042 PRIVATE
    Eigen3::Eigen
    paho-mqttpp3
    paho-mqtt3as
    pthread
)
# （可选）编译选项
target_compile_options(test_polygon PRIVATE -Wall -Wextra -O2)
target_compile_options(publisher_0042 PRIVATE -Wall -Wextra -O2)
