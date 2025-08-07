FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    libssl-dev \
    libyaml-cpp-dev \
    unzip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY paho.mqtt.c /app/paho.mqtt.c
COPY paho.mqtt.cpp /app/paho.mqtt.cpp
COPY json /app/json

# 修正后的 JSON 库安装命令
RUN cd json && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig

# Paho C 库安装
RUN cd paho.mqtt.c && \
    cmake -B build -H. -DPAHO_WITH_SSL=ON -DPAHO_ENABLE_TESTING=OFF && \
    cmake --build build --target install && \
    ldconfig

# Paho C++ 库安装
RUN cd paho.mqtt.cpp && \
    cmake -B build -H. -DPAHO_BUILD_DOCUMENTATION=OFF -DPAHO_BUILD_SAMPLES=OFF && \
    cmake --build build --target install && \
    ldconfig

# 复制应用程序源代码
COPY . .

# 构建应用
RUN rm -rf build && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j$(nproc)

CMD ["/app/build/MqttTransport"]