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
        -DWITH_CUDA=OFF \
        -DWITH_GSTREAMER=ON \
        -DWITH_FFMPEG=ON \
        -DWITH_V4L=ON \
        -DWITH_LIBV4L=ON \
        -DWITH_GTK=OFF \
        -DWITH_QT=OFF && \
    cmake --build build -j"$(nproc)" && \
    cmake --install build && \
    cd / && rm -rf /tmp/opencv

# =========================================================================
# Final Stage
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

# Add ROS2 Humble repository and install base packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rmw-fastrtps-cpp \
    python3-rosdep \
    python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

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

# Create workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS2 environment\n\
if [ -f /opt/ros/humble/setup.bash ]; then\n\
  source /opt/ros/humble/setup.bash\n\
  echo "ROS2 Humble environment sourced"\n\
fi\n\
\n\
# Source local workspace if it exists\n\
if [ -f /workspace/install/setup.bash ]; then\n\
  source /workspace/install/setup.bash\n\
  echo "Local workspace sourced"\n\
fi\n\
\n\
# Print environment info for debugging\n\
if [ "$1" = "bash" ] || [ "$1" = "/bin/bash" ]; then\n\
  echo "=== Environment Info ==="\n\
  echo "PyTorch: $(python3 -c \"import torch; print(torch.__version__)\" 2>/dev/null || echo \"NOT FOUND\")"\n\
  echo "OpenCV: $(python3 -c \"import cv2; print(cv2.__version__)\" 2>/dev/null || echo \"NOT FOUND\")"\n\
  echo "ROS2: $ROS_DISTRO"\n\
  echo "GStreamer: $(gst-launch-1.0 --version 2>/dev/null | head -n1 || echo \"NOT FOUND\")"\n\
  echo "ros2 command: $(which ros2 2>/dev/null || echo \"NOT FOUND\")"\n\
  echo "========================"\n\
fi\n\
\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENV DEBIAN_FRONTEND=
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]