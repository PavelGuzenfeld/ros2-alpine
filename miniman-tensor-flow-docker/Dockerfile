# ==========================================
# builder stage for gstreamer + opencv
# ==========================================
FROM dustynv/l4t-pytorch:r35.2.1 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# 1. install base build deps
RUN apt-get update --allow-insecure-repositories && apt-get install -y --no-install-recommends \
    build-essential git cmake ninja-build wget ca-certificates python3-dev pkg-config \
    flex bison libglib2.0-dev libcairo2-dev libgirepository1.0-dev libavcodec-dev \
    libavformat-dev libavutil-dev libswscale-dev liborc-0.4-dev libxml2-dev \
    libpopt-dev libffi-dev libpcre2-dev zlib1g-dev libssl-dev libv4l-dev \
    libjpeg-dev libpng-dev libudev-dev libasound2-dev libdrm-dev libgudev-1.0-dev \
    libgles2-mesa-dev libegl1-mesa-dev libmount-dev \
    curl python3-pip python3-setuptools software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# python tools for gstreamer build
RUN pip3 install --no-cache-dir meson==1.2.0

# 2. build gstreamer 1.24.13
RUN mkdir -p /tmp/gst && cd /tmp/gst && \
    git clone https://gitlab.freedesktop.org/gstreamer/gstreamer.git && \
    cd gstreamer && git checkout 1.24.13 && \
    meson setup builddir --prefix=/usr --libdir=/usr/lib/aarch64-linux-gnu --libexecdir=/usr/lib/aarch64-linux-gnu \
    -Dpython=enabled -Dcustom_subprojects="gst-libav,gst-plugins-base,gst-plugins-good,gst-plugins-ugly,gst-python" \
    -Dexamples=disabled -Dtests=disabled -Ddoc=disabled && \
    meson compile -C builddir && meson install -C builddir && rm -rf /tmp/gst

# 3. build opencv 4.8.1
RUN mkdir -p /tmp/opencv && cd /tmp/opencv && \
    git clone --depth 1 --branch 4.8.1 https://github.com/opencv/opencv.git . && \
    cmake -S . -B build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_LIST=core,imgproc,imgcodecs,videoio,python3 -DBUILD_opencv_python3=ON \
        -DWITH_CUDA=OFF -DWITH_GSTREAMER=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF -DWITH_GTK=OFF -DWITH_QT=OFF && \
    cmake --build build -j"$(nproc)" && cmake --install build && rm -rf /tmp/opencv

# ==========================================
# final stage: pytorch + ros2 (copied) + libs
# ==========================================
FROM dustynv/l4t-pytorch:r35.2.1 AS final

ENV DEBIAN_FRONTEND=noninteractive

# copy ros2 runtime from prebuilt ros-core image
COPY --from=dustynv/ros:humble-ros-core-l4t-r35.2.1 /opt/ros/humble/ /opt/ros/humble/

# install runtime deps for gstreamer/opencv
RUN apt-get update --allow-insecure-repositories && apt-get install -y --no-install-recommends \
    python3-pip python3-yaml python3-setuptools python3-gi python3-gi-cairo \
    libglib2.0-0 libcairo2 libgirepository-1.0-1 libavcodec58 libavformat58 \
    libavutil56 libswscale5 liborc-0.4-0 libxml2 libpopt0 libffi7 libpcre2-8-0 \
    libssl1.1 libv4l-0 v4l-utils libjpeg8 libpng16-16 && \
    rm -rf /var/lib/apt/lists/*

# python packages required
RUN pip3 install --no-cache-dir lark empy==3.3.4 catkin_pkg rosdistro PyGObject

# Install PyTorch
RUN wget -q https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
    pip3 install --no-cache-dir torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
    rm torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
COPY --from=builder /usr/lib/aarch64-linux-gnu/libgst* /usr/lib/aarch64-linux-gnu/
COPY --from=builder /usr/lib/aarch64-linux-gnu/gstreamer-1.0/ /usr/lib/aarch64-linux-gnu/gstreamer-1.0/
COPY --from=builder /usr/include/gstreamer-1.0/ /usr/include/gstreamer-1.0/
COPY --from=builder /usr/local/lib/libopencv* /usr/local/lib/
COPY --from=builder /usr/local/lib/python3.8/site-packages/cv2/ /usr/local/lib/python3.8/site-packages/cv2/

# env setup for ros2
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/humble
ENV PATH=$ROS_ROOT/install/bin:$PATH
ENV LD_LIBRARY_PATH=$ROS_ROOT/install/lib:/usr/local/lib:/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
ENV PYTHONPATH=$ROS_ROOT/install/lib/python3.8/site-packages:/usr/local/lib/python3.8/site-packages:$PYTHONPATH
ENV AMENT_PREFIX_PATH=$ROS_ROOT
ENV CMAKE_PREFIX_PATH=$ROS_ROOT
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}"
ENV PKG_CONFIG_PATH="/usr/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH}"
ENV PYTHON=/usr/bin/python3
ENV GI_TYPELIB_PATH="/usr/lib/aarch64-linux-gnu/girepository-1.0"
ENV GST_PLUGIN_PATH="/usr/lib/aarch64-linux-gnu/gstreamer-1.0:/workspace/install/lib/gstreamer-1.0"
ENV PYTHONPATH="/usr/lib/python3/site-packages"
ENV LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libGLdispatch.so"

WORKDIR /workspace
# RUN printf '#!/bin/bash\nset -e\n. /opt/ros/humble/setup.bash\n. /workspace/install/setup.bash || true\nexec "$@"\n' > /ros_entrypoint.sh && \
#     chmod +x /ros_entrypoint.sh

# ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
