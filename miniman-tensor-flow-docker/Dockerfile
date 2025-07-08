# Minimal Jetson Xavier: PyTorch 2.1 + OpenCV 4.8.1 + ROS2 Humble + GStreamer 1.24.9
FROM dustynv/l4t-pytorch:r35.2.1 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Install minimal build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    cmake \
    ninja-build \
    wget \
    ca-certificates \
    python3-dev \
    pkg-config \
    flex \
    bison \
    libglib2.0-dev \
    libcairo2-dev \
    libgirepository1.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    liborc-0.4-dev \
    libxml2-dev \
    libpopt-dev \
    libffi-dev \
    libpcre2-dev \
    zlib1g-dev \
    libssl-dev \
    libv4l-dev \
    libjpeg-dev \
    libpng-dev \
    libudev-dev \
    libasound2-dev \
    libdrm-dev \
    libgudev-1.0-dev \
    libgles2-mesa-dev \
    libegl1-mesa-dev \
    libmount-dev \
    libavcodec58 \
    libavformat58 \
    libavutil56 \
    libswscale5 \
    libtbb2 \
    libtesseract4 \
    libpangocairo-1.0-0 && \
    rm -rf /var/lib/apt/lists/*

# Install Meson for GStreamer build
RUN pip3 install meson==1.2.0

# Remove any existing GStreamer packages and build GStreamer 1.24.9 from source
RUN apt-get update && \
    pip3 install --upgrade pip && \
    apt-get clean && \
    apt-get remove -y *gstreamer* && \
    rm -rf /var/lib/apt/lists/* && \
    apt autoremove -y

# Build and install GStreamer 1.24.9
RUN mkdir -p /tmp/gst-1.24 && \
    cd /tmp/gst-1.24 && \
    git clone https://gitlab.freedesktop.org/gstreamer/gstreamer.git && \
    cd gstreamer && \
    git checkout 1.24.9

WORKDIR /tmp/gst-1.24/gstreamer

# Base GStreamer setup
RUN meson setup builddir \
    --libdir=/usr/lib/aarch64-linux-gnu \
    --libexecdir=/usr/lib/aarch64-linux-gnu \
    --prefix=/usr \
    --buildtype=plain \
    -Dexamples=disabled \
    -Dtests=disabled \
    -Ddoc=disabled \
    -Dintrospection=enabled \
    -Dgtk_doc=disabled \
    -Dpython=enabled \
    -Dbad=disabled \
    -Dcustom_subprojects="gst-libav,gst-plugins-base,gst-plugins-good,gst-plugins-ugly,gst-python" \
    -Dlibsoup:sysprof=disabled \
    -Dgpl=enabled && \
    meson compile -C builddir && \
    meson install -C builddir

# Bad plugins rebuild
RUN meson setup --wipe builddir \
    --libdir=/usr/lib/aarch64-linux-gnu \
    --libexecdir=/usr/lib/aarch64-linux-gnu \
    --prefix=/usr \
    -Dbad=enabled \
    -Dauto_features=disabled \
    -Dgstreamer:tools=enabled \
    -Dcustom_subprojects="gst-plugins-bad" \
    -Dgst-plugins-bad:webrtc=enabled \
    -Dgst-plugins-bad:dtls=enabled \
    -Dgst-plugins-bad:sctp=enabled \
    -Dgst-plugins-bad:srtp=enabled \
    -Dgst-plugins-bad:openh264=enabled \
    -Dgst-plugins-bad:mpegtsmux=enabled \
    -Dgst-plugins-bad:videoparsers=enabled \
    -Dgst-plugins-bad:shm=enabled && \
    meson compile -C builddir && \
    meson install -C builddir

# Install Python dependencies for GStreamer
RUN pip3 install click PyGObject

# Install additional GI dependencies in builder
RUN apt-get update && apt-get install -y --no-install-recommends \
    gir1.2-glib-2.0 \
    gir1.2-gstreamer-1.0 && \
    rm -rf /var/lib/apt/lists/*

# Clean up build directory
RUN cd / && rm -rf /tmp/gst-1.24

# Build minimal OpenCV 4.8.1
RUN mkdir -p /tmp/opencv && cd /tmp/opencv && \
    git clone --depth 1 --branch 4.8.1 https://github.com/opencv/opencv.git . && \
    PKG_CONFIG_PATH="/usr/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH}" \
    cmake -S . -B build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_LIST=core,imgproc,imgcodecs,videoio,highgui,python3 \
        -DBUILD_opencv_python3=ON \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCS=OFF \
        -DBUILD_opencv_apps=OFF \
        -DWITH_CUDA=OFF \
        -DWITH_GSTREAMER=OFF \
        -DWITH_FFMPEG=ON \
        -DWITH_V4L=ON \
        -DWITH_LIBV4L=ON \
        -DWITH_GTK=OFF \
        -DWITH_QT=OFF \
        -DWITH_1394=OFF \
        -DWITH_VTK=OFF \
        -DWITH_OPENEXR=OFF \
        -DWITH_WEBP=OFF \
        -DWITH_TIFF=OFF && \
    cmake --build build -j"$(nproc)" && \
    cmake --install build && \
    cd / && rm -rf /tmp/opencv

# =========================================================================
# Copy ROS2 Humble from dustynv's working container
# This is the most reliable approach for getting Humble on r35.2.1
FROM dustynv/ros:humble-ros-base-l4t-r35.2.1 AS ros2_source

# Final Stage: PyTorch 2.0+ + OpenCV 4.8.1 + ROS2 Humble + GStreamer 1.24.9
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
    ca-certificates \
    wget \
    gnupg \
    curl \
    lsb-release \
    # GStreamer runtime deps
    libglib2.0-0 \
    libcairo2 \
    libgirepository-1.0-1 \
    libavcodec58 \
    libavformat58 \
    libavutil56 \
    libswscale5 \
    liborc-0.4-0 \
    libxml2 \
    libpopt0 \
    libffi7 \
    libpcre2-8-0 \
    zlib1g \
    libssl1.1 \
    libv4l-0 \
    v4l-utils \
    python3-gi \
    python3-gi-cairo \
    gir1.2-glib-2.0 \
    gir1.2-gstreamer-1.0 \
    # OpenCV runtime deps
    libjpeg8 \
    libpng16-16 && \
    rm -rf /var/lib/apt/lists/*

# Install minimal runtime dependencies for ROS2
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-yaml \
    python3-setuptools \
    python3-dev \
    libyaml-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy ROS2 Humble installation from dustynv's working container
COPY --from=ros2_source /opt/ros/humble /opt/ros/humble

# CRITICAL FIX: DO NOT build custom spdlog - use what comes with ROS2
# Remove any system spdlog that might conflict and ensure ROS2's libraries take precedence
RUN apt-get update && apt-get remove -y libspdlog* || true && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/* && \
    # Ensure ROS2 libraries are found first
    echo "/opt/ros/humble/install/lib" > /etc/ld.so.conf.d/000-ros2.conf && \
    ldconfig

# Install additional ROS2 Python dependencies via pip if needed
RUN pip3 install --no-cache-dir \
    empy \
    lark \
    catkin-pkg \
    colcon-common-extensions \
    rosdep \
    || echo "Some ROS2 Python packages already available"

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update || echo "rosdep update failed, continuing anyway"

# Upgrade PyTorch to 2.1
RUN wget -q https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
    pip3 install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
    rm torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl

# CUDA Library Path Fix for PyTorch
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:/usr/local/cuda/targets/aarch64-linux/lib:/opt/ros/humble/install/lib:/usr/local/lib:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}"

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
ENV ROS_VERSION=2
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV PYTHONPATH="/usr/local/lib/python3.8/site-packages:/opt/ros/humble/install/local/lib/python3.8/site-packages:/opt/ros/humble/install/lib/python3.8/site-packages:${PYTHONPATH}"
ENV AMENT_PREFIX_PATH="/opt/ros/humble/install"
ENV CMAKE_PREFIX_PATH="/opt/ros/humble/install"
ENV PATH="/opt/ros/humble/install/bin:${PATH}"

# Create workspace
RUN mkdir -p /workspace/
WORKDIR /workspace

# Create simple entrypoint script that just sources ROS2 and executes
RUN printf '#!/bin/bash\nset -e\n\n# Source ROS2 environment\nif [ -f /opt/ros/humble/install/setup.bash ]; then\n  source /opt/ros/humble/install/setup.bash\nfi\n\n# Source local workspace if it exists\nif [ -f /workspace/install/setup.bash ]; then\n  source /workspace/install/setup.bash\nfi\n\nexec "$@"\n' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Copy test script
COPY jetson_test.py /test/jetson_test.py

ENV DEBIAN_FRONTEND=
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]