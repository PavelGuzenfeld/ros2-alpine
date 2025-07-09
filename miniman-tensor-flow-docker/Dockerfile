# Minimal Jetson Xavier: PyTorch 2.1 + OpenCV 4.8.1 + ROS2 Humble + GStreamer 1.24.9
FROM dustynv/l4t-pytorch:r35.2.1 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install base build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git cmake ninja-build wget ca-certificates python3-dev pkg-config \
    flex bison libglib2.0-dev libcairo2-dev libgirepository1.0-dev libavcodec-dev \
    libavformat-dev libavutil-dev libswscale-dev liborc-0.4-dev libxml2-dev \
    libpopt-dev libffi-dev libpcre2-dev zlib1g-dev libssl-dev libv4l-dev \
    libjpeg-dev libpng-dev libudev-dev libasound2-dev libdrm-dev libgudev-1.0-dev \
    libgles2-mesa-dev libegl1-mesa-dev libmount-dev \
    curl python3-pip python3-setuptools software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# Proactively install all Python-based ROS2 build tools using pip3
RUN pip3 install --no-cache-dir \
    rosdep \
    vcstool \
    colcon-common-extensions \
    catkin_pkg \
    lark \
    empy \
    numpy \
    rosinstall_generator

# Install Meson for GStreamer build
RUN pip3 install meson==1.2.0

# 2. Build and install GStreamer 1.24.9
RUN mkdir -p /tmp/gst && cd /tmp/gst && \
    git clone https://gitlab.freedesktop.org/gstreamer/gstreamer.git && \
    cd gstreamer && git checkout 1.24.9 && \
    meson setup builddir --prefix=/usr --libdir=/usr/lib/aarch64-linux-gnu --libexecdir=/usr/lib/aarch64-linux-gnu \
    -Dpython=enabled -Dcustom_subprojects="gst-libav,gst-plugins-base,gst-plugins-good,gst-plugins-ugly,gst-python" \
    -Dexamples=disabled -Dtests=disabled -Ddoc=disabled && \
    meson compile -C builddir && meson install -C builddir && rm -rf /tmp/gst

# 3. Build minimal OpenCV 4.8.1
RUN mkdir -p /tmp/opencv && cd /tmp/opencv && \
    git clone --depth 1 --branch 4.8.1 https://github.com/opencv/opencv.git . && \
    cmake -S . -B build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_LIST=core,imgproc,imgcodecs,videoio,python3 -DBUILD_opencv_python3=ON \
        -DWITH_CUDA=OFF -DWITH_GSTREAMER=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF -DWITH_GTK=OFF -DWITH_QT=OFF && \
    cmake --build build -j"$(nproc)" && cmake --install build && rm -rf /tmp/opencv

# 4. Build ROS2 Humble from source (ros-base variant only)
# Enable the 'universe' repository to find libgtest-dev
RUN add-apt-repository universe && apt-get update

RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update && \
    rosdep fix-permissions

RUN mkdir -p /ros2_ws/src && cd /ros2_ws && \
    # MODIFIED: Use rosinstall_generator to get only ros-base sources
    rosinstall_generator ros_base --rosdistro humble --deps --tar > humble-ros-base.repos && \
    vcs import src < humble-ros-base.repos

# Expanded --skip-keys to prevent rosdep from trying to apt-install Python packages
RUN cd /ros2_ws && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    --skip-keys "fastcdr fastrtps rti-connext-dds-6.0.1 urdfdom_headers python3-lark python3-vcstool python3-colcon-common-extensions python3-catkin-pkg python3-rosdep python3-empy python3-numpy"

RUN cd /ros2_ws && colcon build --symlink-install --mixin release


# =========================================================================
# FINAL STAGE
# Start from the same clean base and copy only the runtime artifacts.
# =========================================================================
FROM dustynv/l4t-pytorch:r35.2.1

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}"
ENV PKG_CONFIG_PATH="/usr/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH}"
ENV PYTHON=/usr/bin/python3
ENV GI_TYPELIB_PATH="/usr/lib/aarch64-linux-gnu/girepository-1.0"
ENV GST_PLUGIN_PATH="/usr/lib/aarch64-linux-gnu/gstreamer-1.0:/workspace/install/lib/gstreamer-1.0"
ENV PYTHONPATH="/usr/lib/python3/site-packages"
ENV LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libGLdispatch.so"

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-yaml python3-setuptools python3-gi python3-gi-cairo \
    libglib2.0-0 libcairo2 libgirepository-1.0-1 libavcodec58 libavformat58 \
    libavutil56 libswscale5 liborc-0.4-0 libxml2 libpopt0 libffi7 libpcre2-8-0 \
    libssl1.1 libv4l-0 v4l-utils libjpeg8 libpng16-16 \
    python3-lark python3-empy python3-catkin-pkg && \
    rm -rf /var/lib/apt/lists/*

# Install PyTorch
RUN wget -q https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
    pip3 install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
    rm torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl


# 2. Copy all compiled artifacts from the builder stage
COPY --from=builder /ros2_ws/install/ /opt/ros/humble/
COPY --from=builder /usr/local/ /usr/local/

# Create CUDA library symlinks for PyTorch compatibility
RUN ln -sf /usr/local/cuda/targets/aarch64-linux/lib/libcublas.so /usr/lib/aarch64-linux-gnu/libcublas.so || true && \
    ln -sf /usr/local/cuda/targets/aarch64-linux/lib/libcublasLt.so /usr/lib/aarch64-linux-gnu/libcublasLt.so || true && \
    ln -sf /usr/local/cuda/targets/aarch64-linux/lib/libcurand.so /usr/lib/aarch64-linux-gnu/libcurand.so || true && \
    ln -sf /usr/local/cuda/targets/aarch64-linux/lib/libcusparse.so /usr/lib/aarch64-linux-gnu/libcusparse.so || true && \
    ln -sf /usr/local/cuda/targets/aarch64-linux/lib/libcusolver.so /usr/lib/aarch64-linux-gnu/libcusolver.so || true && \
    ldconfig

# Install additional Python dependencies
RUN pip3 install --no-cache-dir click PyGObject

# Copy built libraries from builder stage
COPY --from=builder /usr/lib/aarch64-linux-gnu/libgst* /usr/lib/aarch64-linux-gnu/
COPY --from=builder /usr/lib/aarch64-linux-gnu/gstreamer-1.0/ /usr/lib/aarch64-linux-gnu/gstreamer-1.0/
COPY --from=builder /usr/lib/aarch64-linux-gnu/pkgconfig/gstreamer* /usr/lib/aarch64-linux-gnu/pkgconfig/
COPY --from=builder /usr/lib/aarch64-linux-gnu/girepository-1.0/Gst* /usr/lib/aarch64-linux-gnu/girepository-1.0/
COPY --from=builder /usr/bin/gst-* /usr/bin/
COPY --from=builder /usr/include/gstreamer-1.0/ /usr/include/gstreamer-1.0/
COPY --from=builder /usr/local/lib/libopencv* /usr/local/lib/
COPY --from=builder /usr/local/lib/python3.8/site-packages/cv2/ /usr/local/lib/python3.8/site-packages/cv2/

# Update library cache with proper ordering
RUN echo "/opt/ros/humble/install/lib" > /etc/ld.so.conf.d/000-ros2.conf && \
    echo "/usr/local/cuda/lib64" > /etc/ld.so.conf.d/001-cuda.conf && \
    echo "/usr/local/cuda/targets/aarch64-linux/lib" > /etc/ld.so.conf.d/002-cuda-targets.conf && \
    echo "/usr/local/lib" > /etc/ld.so.conf.d/003-opencv.conf && \
    echo "/usr/lib/aarch64-linux-gnu" > /etc/ld.so.conf.d/004-gstreamer.conf && \
    ldconfig

# Set up ROS2 environment with fixed library paths
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/humble
ENV PATH=$ROS_ROOT/bin:$PATH
ENV LD_LIBRARY_PATH=$ROS_ROOT/lib:/usr/local/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=$ROS_ROOT/lib/python3.8/site-packages:/usr/local/lib/python3.8/site-packages:$PYTHONPATH
ENV AMENT_PREFIX_PATH=$ROS_ROOT
ENV CMAKE_PREFIX_PATH=$ROS_ROOT
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 4. Create workspace and entrypoint
WORKDIR /workspace
RUN printf '#!/bin/bash\nset -e\n. /opt/ros/humble/setup.bash\n. /workspace/install/setup.bash || true\nexec "$@"\n' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]