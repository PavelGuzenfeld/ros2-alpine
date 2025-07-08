# Jetson Xavier TensorFlow + OpenCV + ROS2 Jazzy (Base Level Only - Fast-DDS)
FROM nvcr.io/nvidia/l4t-base:r35.2.1 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# build-time deps and python venv setup
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    cmake \
    ninja-build \
    curl \
    wget \
    ca-certificates \
    python3-dev \
    python3-pip \
    python3-venv \
    libhdf5-dev \
    libopenblas-dev \
    liblapack-dev \
    gfortran \
    pkg-config \
    flex \
    bison \
    nasm \
    gettext \
    libudev-dev \
    libasound2-dev \
    libdrm-dev \
    libgudev-1.0-dev \
    libgles2-mesa-dev \
    libegl1-mesa-dev \
    libmount-dev \
    libglib2.0-dev \
    libcairo2-dev \
    libgirepository1.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libavresample-dev \
    liborc-0.4-dev \
    libxml2-dev \
    libpopt-dev \
    libffi-dev \
    libpcre2-dev \
    zlib1g-dev \
    libssl-dev \
    libasound2-dev \
    libpulse-dev \
    libx11-dev \
    libxext-dev \
    libxfixes-dev \
    libxi-dev \
    libxrender-dev \
    libxrandr-dev \
    libxss-dev \
    libgdk-pixbuf2.0-dev \
    libepoxy-dev \
    libxkbcommon-dev \
    libwayland-dev \
    libopus-dev \
    libvorbis-dev \
    libtheora-dev \
    libvpx-dev \
    libmp3lame-dev \
    libspeex-dev \
    libflac-dev \
    libpango1.0-dev \
    libjpeg-dev \
    libpng-dev \
    libv4l-dev && \
    rm -rf /var/lib/apt/lists/*

# Install newer Meson version required for GStreamer 1.24.9
RUN pip3 install meson==1.2.0

# create python venv and install deps
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

COPY requirements.txt ./
RUN pip install --no-cache-dir --upgrade pip wheel && \
    pip install --no-cache-dir -r requirements.txt

# Remove any existing GStreamer packages and build GStreamer 1.24.9 from source
RUN apt-get update && \
    apt-get install -y \
    libudev-dev libasound2-dev libdrm-dev libgudev-1.0-dev \
    libgles2-mesa-dev libegl1-mesa-dev libmount-dev flex bison \
    libglib2.0-dev libcairo2-dev libavcodec58 libavformat58 \
    libavutil56 libswscale5 libtbb2 libtesseract4 libpangocairo-1.0-0 libgirepository1.0-dev \
    && pip3 install --upgrade pip \
    && apt-get clean \
    && apt-get remove -y *gstreamer* \
    && rm -rf /var/lib/apt/lists/* \
    && apt autoremove -y

# Build and install GStreamer 1.24.9
RUN mkdir -p /tmp/gst-1.24 && \
    cd /tmp/gst-1.24 && \
    git clone https://gitlab.freedesktop.org/gstreamer/gstreamer.git && \
    cd gstreamer && \
    git checkout 1.24.9

WORKDIR /tmp/gst-1.24/gstreamer

# Base GStreamer setup (without bad plugins)
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

# Build OpenCV with custom GStreamer 1.24.9 integration
WORKDIR /tmp
RUN mkdir -p /tmp/opencv && cd /tmp/opencv && \
    git clone --depth 1 --branch 4.x https://github.com/opencv/opencv.git . && \
    PY_SITEPACKAGES=$(python3 -c "import site; print(site.getsitepackages()[0])") && \
    PKG_CONFIG_PATH="/usr/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH}" \
    cmake -S . -B build -G Ninja \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D CMAKE_PREFIX_PATH="/usr" \
    -D PKG_CONFIG_EXECUTABLE=$(which pkg-config) \
    -D BUILD_LIST=core,imgproc,imgcodecs,videoio,highgui,python3 \
    -D BUILD_opencv_python3=ON \
    -D OPENCV_PYTHON3_INSTALL_PATH="$PY_SITEPACKAGES" \
    -D BUILD_SHARED_LIBS=ON \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_JPEG=OFF \
    -D BUILD_PNG=OFF \
    -D BUILD_PROTOBUF=OFF \
    -D BUILD_ADE=ON \
    -D WITH_CUDA=OFF \
    -D WITH_GSTREAMER=ON \
    -D WITH_FFMPEG=ON \
    -D WITH_V4L=ON \
    -D WITH_LIBV4L=ON && \
    cmake --build build -j"$(nproc)" && \
    cmake --install build && \
    rm -rf /tmp/opencv

# =========================================================================
# Final Stage with ROS2 Jazzy (Base Level Only - Fast-DDS middleware)
# =========================================================================
FROM nvcr.io/nvidia/l4t-base:r35.2.1

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="/opt/venv/bin:$PATH"
ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}"
ENV PKG_CONFIG_PATH="/usr/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH}"
ENV GST_PLUGIN_PATH="/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
ENV PYTHON=/usr/bin/python3
ENV GI_TYPELIB_PATH=/usr/lib/aarch64-linux-gnu/girepository-1.0
ENV PYTHONPATH="/usr/lib/python3/site-packages"
ENV LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libgomp.so.1:/usr/lib/aarch64-linux-gnu/libGLdispatch.so"
ENV GST_REGISTRY_UPDATE=no
ENV GST_PLUGIN_SCANNER_TIMEOUT=5
ENV GST_PLUGIN_SYSTEM_PATH=""
ENV GST_DEBUG=0

# setup nvidia runtime deps and supporting libraries for GStreamer
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    wget \
    gnupg \
    git \
    build-essential \
    cmake \
    pkg-config \
    python3 \
    python3-pip \
    python3-dev \
    libopenblas-base \
    liblapack3 \
    cuda-runtime-11-4 \
    libcudnn8 \
    libnvinfer8 \
    libnvinfer-plugin8 \
    libnvonnxparsers8 \
    libnvparsers8 \
    libhdf5-dev \
    libudev1 \
    libasound2 \
    libdrm2 \
    libgudev-1.0-0 \
    libgles2-mesa \
    libegl1-mesa \
    libmount1 \
    libglib2.0-0 \
    libcairo2 \
    libgirepository-1.0-1 \
    libavcodec58 \
    libavformat58 \
    libavutil56 \
    libswscale5 \
    libtbb2 \
    libtesseract4 \
    libpangocairo-1.0-0 \
    liborc-0.4-0 \
    libxml2 \
    libpopt0 \
    libffi7 \
    libpcre2-8-0 \
    zlib1g \
    libssl1.1 \
    libpulse0 \
    libx11-6 \
    libxext6 \
    libxfixes3 \
    libxi6 \
    libxrender1 \
    libxrandr2 \
    libxss1 \
    libgdk-pixbuf2.0-0 \
    libepoxy0 \
    libxkbcommon0 \
    libwayland-client0 \
    libwayland-cursor0 \
    libwayland-egl1 \
    libopus0 \
    libvorbis0a \
    libvorbisenc2 \
    libtheora0 \
    libvpx6 \
    libmp3lame0 \
    libspeex1 \
    libflac8 \
    libpango-1.0-0 \
    libjpeg8 \
    libpng16-16 \
    libv4l-0 \
    v4l-utils \
    python3-gi \
    python3-gi-cairo \
    gir1.2-glib-2.0 \
    gir1.2-gstreamer-1.0 \
    software-properties-common \
    lsb-release && \
    rm -rf /var/lib/apt/lists/*

# Install a newer version of CMake
ENV CMAKE_VERSION=3.29.3
RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-aarch64.tar.gz -O /tmp/cmake.tar.gz && \
    tar -zxvf /tmp/cmake.tar.gz -C /usr/local --strip-components=1 && \
    rm /tmp/cmake.tar.gz


# Install minimal system build dependencies and build LTTng from source
RUN apt-get update && apt-get install -y --no-install-recommends \
    libtinyxml2-dev \
    libssl-dev \
    libyaml-cpp-dev \
    libacl1-dev \
    uuid-dev \
    libpopt-dev \
    libxml2-dev \
    libasio-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install LTTng from source
RUN mkdir -p /tmp/lttng && cd /tmp/lttng && \
    # Always build userspace-rcu from source (system version is too old)
    wget https://lttng.org/files/urcu/userspace-rcu-0.14.0.tar.bz2 && \
    tar -xjf userspace-rcu-0.14.0.tar.bz2 && \
    cd userspace-rcu-0.14.0 && \
    ./configure --prefix=/usr && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd .. && \
    # Build LTTng-UST (disable NUMA support to avoid libnuma dependency)
    wget https://lttng.org/files/lttng-ust/lttng-ust-2.13.6.tar.bz2 && \
    tar -xjf lttng-ust-2.13.6.tar.bz2 && \
    cd lttng-ust-2.13.6 && \
    ./configure --prefix=/usr --disable-numa && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd .. && \
    # Build LTTng-Tools
    wget https://lttng.org/files/lttng-tools/lttng-tools-2.13.11.tar.bz2 && \
    tar -xjf lttng-tools-2.13.11.tar.bz2 && \
    cd lttng-tools-2.13.11 && \
    ./configure --prefix=/usr && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && rm -rf /tmp/lttng

# Install Python build dependencies
RUN pip3 install --no-cache-dir --upgrade setuptools==58.2.0 \
    colcon-common-extensions \
    vcstool \
    rosdep \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    lark \
    empy \
    pyyaml \
    wheel \
    pybind11 \
    cython \
    catkin-pkg \
    distlib

# Set compiler flags
ENV MAKEFLAGS="-j2"
ENV CFLAGS="-Wno-int-conversion -Wno-incompatible-pointer-types -Wno-error"
ENV CXXFLAGS="-Wno-error -Wno-deprecated-declarations -Wno-stringop-overread"

# Create ROS2 workspace
RUN mkdir -p /opt/ros2_jazzy/src
WORKDIR /opt/ros2_jazzy

# Clone all required repositories
COPY ros2.repos /opt/ros2_jazzy/
RUN vcs import src < ros2.repos

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -y || echo "Some dependencies may not be available; this is expected."

# Set the default shell to bash for all subsequent RUN commands
SHELL ["/bin/bash", "-c"]

# Stage 1: Build vendor packages and Fast-DDS first
RUN echo "=== Stage 1: Building Vendor Packages and Fast-DDS ===" && \
    colcon build --symlink-install \
    --packages-up-to \
        ament_lint \
        ament_index_python \
        fastcdr \
        foonathan_memory_vendor \
        fastrtps \
        libyaml_vendor \
        pybind11_vendor \
        spdlog_vendor \
        tinyxml2_vendor \
        console_bridge_vendor \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --event-handlers console_direct+ || (echo "VENDOR BUILD FAILED" && exit 1)

# Stage 2: Build foundation packages
RUN echo "=== Stage 2: Building Foundation ===" && \
    source install/setup.bash && \
    colcon build --symlink-install \
    --packages-up-to \
        ament_lint \
        ament_index_python \
        ament_cmake_ros \
        python_cmake_module \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --event-handlers console_direct+ || (echo "FOUNDATION BUILD FAILED" && exit 1)

# Stage 3: Build ROSIDL system with all dependencies
RUN echo "=== Stage 3: Building ROSIDL System ===" && \
    source install/setup.bash && \
    colcon build --symlink-install \
    --packages-up-to \
        rosidl_generator_py \
        rosidl_dynamic_typesupport \
        rosidl_runtime_cpp \
        rosidl_typesupport_cpp \
        rosidl_typesupport_fastrtps_cpp \
        rosidl_typesupport_introspection_cpp \
        rosidl_typesupport_fastrtps_c \
        rosidl_typesupport_introspection_c \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --event-handlers console_direct+ || (echo "ROSIDL BUILD FAILED" && exit 1)

# Stage 4: Build RMW and Fast-RTPS packages  
RUN echo "=== Stage 4: Building RMW and Fast-RTPS ===" && \
    source install/setup.bash && \
    colcon build --symlink-install \
    --packages-up-to \
        rmw \
        rmw_dds_common \
        rmw_fastrtps_shared_cpp \
        rmw_fastrtps_cpp \
        rmw_implementation \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --event-handlers console_direct+ || (echo "RMW BUILD FAILED" && exit 1)

# Stage 5: Build core interfaces and RCL
RUN echo "=== Stage 5: Building Core Interfaces and RCL ===" && \
    source install/setup.bash && \
    colcon build --symlink-install \
    --packages-up-to \
        builtin_interfaces \
        unique_identifier_msgs \
        rcl_interfaces \
        common_interfaces \
        rcl \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --event-handlers console_direct+ || (echo "CORE RCL BUILD FAILED" && exit 1)

# Stage 6: Build all remaining packages with exclusions
RUN echo "=== Stage 6: Building All Remaining Packages ===" && \
    source install/setup.bash && \
    colcon build --symlink-install \
    --packages-skip \
        lttngpy \
        test_msgs \
        examples_rclcpp_minimal_action_client \
        examples_rclcpp_minimal_action_server \
        examples_rclcpp_minimal_client \
        examples_rclcpp_minimal_composition \
        examples_rclcpp_minimal_publisher \
        examples_rclcpp_minimal_service \
        examples_rclcpp_minimal_subscriber \
        examples_rclcpp_minimal_timer \
        examples_rclcpp_multithreaded_executor \
        examples_rclpy_executors \
        examples_rclpy_minimal_action_client \
        examples_rclpy_minimal_action_server \
        examples_rclpy_minimal_client \
        examples_rclpy_minimal_publisher \
        examples_rclpy_minimal_service \
        examples_rclpy_minimal_subscriber \
        demo_nodes_cpp \
        demo_nodes_py \
        quality_of_service_demo_cpp \
        quality_of_service_demo_py \
        lifecycle \
        pendulum_control \
        pendulum_msgs \
        topic_monitor \
        composition \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --event-handlers console_direct+ || (echo "FINAL BUILD FAILED" && exit 1)

# Set up ROS2 environment
ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros2_jazzy/install
ENV COLCON_PREFIX_PATH=/opt/ros2_jazzy/install
ENV LD_LIBRARY_PATH=/opt/ros2_jazzy/install/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/ros2_jazzy/install/bin:$PATH
ENV PYTHONPATH=/opt/ros2_jazzy/install/local/lib/python3.8/site-packages:/opt/ros2_jazzy/install/lib/python3.8/site-packages:$PYTHONPATH
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Revert to default shell
SHELL ["/bin/sh", "-c"]
ENV DEBIAN_FRONTEND=

# add Jetson repo keys
RUN apt-key adv --fetch-keys https://repo.download.nvidia.com/jetson/jetson-ota-public.asc && \
    echo "deb https://repo.download.nvidia.com/jetson/common r35.2 main" > /etc/apt/sources.list.d/nvidia-jetson.list && \
    echo "deb https://repo.download.nvidia.com/jetson/t234 r35.2 main" >> /etc/apt/sources.list.d/nvidia-jetson.list

# copy prebuilt python venv with OpenCV + deps and custom GStreamer 1.24.9
COPY --from=builder /opt/venv /opt/venv

# copy custom built GStreamer 1.24.9 from system locations
COPY --from=builder /usr/lib/aarch64-linux-gnu/libgst* /usr/lib/aarch64-linux-gnu/
COPY --from=builder /usr/lib/aarch64-linux-gnu/gstreamer-1.0/ /usr/lib/aarch64-linux-gnu/gstreamer-1.0/
COPY --from=builder /usr/lib/aarch64-linux-gnu/pkgconfig/gstreamer* /usr/lib/aarch64-linux-gnu/pkgconfig/
COPY --from=builder /usr/lib/aarch64-linux-gnu/girepository-1.0/Gst* /usr/lib/aarch64-linux-gnu/girepository-1.0/
COPY --from=builder /usr/bin/gst-* /usr/bin/
COPY --from=builder /usr/include/gstreamer-1.0/ /usr/include/gstreamer-1.0/

# copy OpenCV shared libraries from builder stage
COPY --from=builder /usr/local/lib/libopencv* /usr/local/lib/

# update library cache and prebuild GStreamer registry
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf && \
    echo "/usr/lib/aarch64-linux-gnu" > /etc/ld.so.conf.d/gstreamer.conf && \
    ldconfig && \
    export GST_REGISTRY_UPDATE=no && \
    export GST_DEBUG=0 && \
    timeout 30s gst-inspect-1.0 --print-all >/dev/null 2>&1 || true

# Create ROS2 workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# install nvidia tensorflow in final image
RUN pip install --no-cache-dir --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v51 tensorflow==2.11.0+nv23.1 && \
    pip install --no-cache-dir click PyGObject

# Create startup script that sources ROS2
RUN echo '#!/bin/bash\n\
set -e\n\
if [ -f /opt/ros2_jazzy/install/setup.bash ]; then\n\
  source /opt/ros2_jazzy/install/setup.bash\n\
fi\n\
if [ -f /workspace/install/setup.bash ]; then\n\
  source /workspace/install/setup.bash\n\
fi\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Copy test script
COPY ./ros2_test.py /test/ros2_test.py

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]