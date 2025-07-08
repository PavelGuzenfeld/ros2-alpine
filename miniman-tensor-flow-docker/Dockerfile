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

# Use the main build for final stage
FROM dustynv/l4t-pytorch:r35.2.1

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}"
ENV PKG_CONFIG_PATH="/usr/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH}"
ENV PYTHON=/usr/bin/python3
ENV GI_TYPELIB_PATH="/usr/lib/aarch64-linux-gnu/girepository-1.0"
ENV GST_PLUGIN_PATH="/usr/lib/aarch64-linux-gnu/gstreamer-1.0:/workspace/install/lib/gstreamer-1.0"
ENV PYTHONPATH="/usr/lib/python3/site-packages"
ENV LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libGLdispatch.so"

# Install runtime dependencies and ROS2 Humble base
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

# Build spdlog v1.8.5 from source (version that ROS2 Humble expects)
RUN apt-get update && apt-get install -y --no-install-recommends git cmake build-essential && \
    git clone --depth 1 --branch v1.8.5 https://github.com/gabime/spdlog.git /tmp/spdlog && \
    cd /tmp/spdlog && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX=/usr \
             -DSPDLOG_BUILD_SHARED=ON \
             -DSPDLOG_BUILD_EXAMPLE=OFF \
             -DSPDLOG_BUILD_TESTS=OFF \
             -DSPDLOG_BUILD_BENCH=OFF && \
    make -j$(nproc) && make install && \
    cd / && rm -rf /tmp/spdlog && \
    apt-get remove -y git cmake build-essential && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/* && \
    ldconfig

# Create a simple script to handle missing library symlinks at runtime
RUN echo '#!/bin/bash\n\
# Create symlinks for commonly missing libraries\n\
echo "Checking for missing ROS2 libraries..."\n\
\n\
# Look for spdlog library in ROS2 installation\n\
SPDLOG_LIB=$(find /opt/ros/humble -name "*spdlog*" -name "*.so*" 2>/dev/null | head -n1)\n\
if [ -n "$SPDLOG_LIB" ] && [ ! -f /usr/lib/aarch64-linux-gnu/libspdlog.so.1 ]; then\n\
  echo "Found spdlog library: $SPDLOG_LIB"\n\
  ln -sf "$SPDLOG_LIB" /usr/lib/aarch64-linux-gnu/libspdlog.so.1 2>/dev/null || true\n\
  ln -sf "$SPDLOG_LIB" /usr/lib/aarch64-linux-gnu/libspdlog.so 2>/dev/null || true\n\
fi\n\
\n\
# Look for fmt library in ROS2 installation\n\
FMT_LIB=$(find /opt/ros/humble -name "*fmt*" -name "*.so*" 2>/dev/null | head -n1)\n\
if [ -n "$FMT_LIB" ] && [ ! -f /usr/lib/aarch64-linux-gnu/libfmt.so ]; then\n\
  echo "Found fmt library: $FMT_LIB"\n\
  ln -sf "$FMT_LIB" /usr/lib/aarch64-linux-gnu/libfmt.so 2>/dev/null || true\n\
fi\n\
\n\
# Update library cache\n\
ldconfig 2>/dev/null || true\n\
echo "Library setup complete."\n\
' > /usr/local/bin/fix_ros_libs.sh && chmod +x /usr/local/bin/fix_ros_libs.sh

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

# Update library cache
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf && \
    echo "/usr/lib/aarch64-linux-gnu" > /etc/ld.so.conf.d/gstreamer.conf && \
    ldconfig

# Set up ROS2 environment
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV PYTHONPATH="/usr/local/lib/python3.8/site-packages:${PYTHONPATH}"
ENV AMENT_PREFIX_PATH="/opt/ros/humble/install"
ENV CMAKE_PREFIX_PATH="/opt/ros/humble/install"
ENV PATH="/opt/ros/humble/install/bin:${PATH}"
ENV LD_LIBRARY_PATH="/opt/ros/humble/install/lib:${LD_LIBRARY_PATH}"

# Create workspace
RUN mkdir -p /workspace/
WORKDIR /workspace

# Create entrypoint script using printf (more reliable than heredoc in Dockerfile)
RUN printf '#!/bin/bash\nset -e\n\n# Source ROS2 environment\nif [ -f /opt/ros/humble/setup.bash ]; then\n  source /opt/ros/humble/setup.bash\n  echo "ROS2 Humble environment sourced"\nfi\n\n# Source local workspace if it exists\nif [ -f /workspace/install/setup.bash ]; then\n  source /workspace/install/setup.bash\n  echo "Local workspace sourced"\nfi\n\n# Print environment info for debugging\nif [ "$1" = "bash" ] || [ "$1" = "/bin/bash" ]; then\n  echo "=== Environment Info ==="\n  \n  # PyTorch version\n  PYTORCH_VER=$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null || echo "NOT FOUND")\n  echo "PyTorch: $PYTORCH_VER"\n  \n  # OpenCV version\n  OPENCV_VER=$(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null || echo "NOT FOUND")\n  echo "OpenCV: $OPENCV_VER"\n  \n  # ROS2 distro\n  echo "ROS2: $ROS_DISTRO"\n  \n  # GStreamer version (with timeout)\n  GST_VER=$(timeout 3 gst-inspect-1.0 --version 2>/dev/null | head -n1 || echo "NOT FOUND")\n  echo "GStreamer: $GST_VER"\n  \n  # ROS2 command\n  ROS2_CMD=$(which ros2 2>/dev/null || echo "NOT FOUND")\n  echo "ros2 command: $ROS2_CMD"\n  \n  echo "========================"\nfi\n\nexec "$@"\n' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Create test directory for external test scripts
RUN mkdir -p /test

ENV DEBIAN_FRONTEND=
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]