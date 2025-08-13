cmake_minimum_required(VERSION 3.16)
project(MqttTransport)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 源文件（全部在 src 目录下）
set(SOURCES
    GeoUtils.cc
    main.cc
    MqttTransport.cc
    protocal.cc
    polygon_point_singleton.cc
)

# 生成可执行文件
add_executable(MqttTransport ${SOURCES})

# 添加 include 路径（先 add_executable，再 include）
target_include_directories(MqttTransport PRIVATE
    ${CMAKE_SOURCE_DIR}      # 当前 src 目录
    /usr/local/include       # paho-mqtt, GeographicLib 等
)

# 找 Eigen
find_package(Eigen3 REQUIRED)
target_link_libraries(MqttTransport PRIVATE Eigen3::Eigen)

# 链接外部库
target_link_libraries(MqttTransport PRIVATE
    paho-mqttpp3
    paho-mqtt3as
    yaml-cpp
    pthread
)

# 复制配置文件到构建目录
file(COPY
    ${CMAKE_SOURCE_DIR}/szo.yml
    ${CMAKE_SOURCE_DIR}/szo_sanya_vessel.yml
    DESTINATION ${CMAKE_BINARY_DIR}
)
