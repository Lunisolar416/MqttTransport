cmake_minimum_required(VERSION 3.16)
project(MqttTransport)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    GeoUtils.cc
    main.cc
    MqttTransport.cc
    protocal.cc
)


add_executable(MqttTransport ${SOURCES})


include_directories(/usr/local/include)
link_directories(/usr/local/lib)

target_link_libraries(MqttTransport
    paho-mqttpp3
    paho-mqtt3as
    yaml-cpp
    pthread
)

file(COPY 
    ${CMAKE_SOURCE_DIR}/szo.yml 
    ${CMAKE_SOURCE_DIR}/szo_sanya_vessel.yml
    DESTINATION ${CMAKE_BINARY_DIR}
)