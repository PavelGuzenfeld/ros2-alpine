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

RUN apt purge -y python3.9 libpython3.9* || echo "python3.9 not found, skipping removal" && \
    ls -ll /usr/bin/python*

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
ENV LANG=en_US.UTF-8

# 1. runtime deps (spdlog included to fix rclpy errors)
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales python3-gi python3-gi-cairo libglib2.0-0 libcairo2 libgirepository-1.0-1 \
    libavcodec58 libavformat58 libavutil56 libswscale5 liborc-0.4-0 libxml2 libpopt0 \
    libffi7 libpcre2-8-0 libssl1.1 libv4l-0 v4l-utils libjpeg8 libpng16-16 libspdlog1 \
    build-essential cmake git python3-rosdep libacl1-dev \
    python3-rosinstall-generator python3-setuptools python3-numpy \
    python3-pip python3-dev libasio-dev libtinyxml2-dev libcunit1-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. locale setup
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 3. python tools
RUN pip3 install --index-url https://pypi.org/simple --no-cache-dir lark empy==3.3.4 colcon-common-extensions vcstool catkin_pkg rosdistro click PyGObject scikit-build && \
    pip3 install --index-url https://pypi.org/simple --upgrade cmake

# 4. copy gstreamer/opencv from builder
COPY --from=builder /usr/lib/aarch64-linux-gnu/libgst* /usr/lib/aarch64-linux-gnu/
COPY --from=builder /usr/lib/aarch64-linux-gnu/gstreamer-1.0/ /usr/lib/aarch64-linux-gnu/gstreamer-1.0/
COPY --from=builder /usr/include/gstreamer-1.0/ /usr/include/gstreamer-1.0/
COPY --from=builder /usr/local/lib/libopencv* /usr/local/lib/
COPY --from=builder /usr/local/lib/python3.8/site-packages/cv2/ /usr/local/lib/python3.8/site-packages/cv2/

# ==========================================
# 5. build ROS2 HUMBLE FROM SOURCE (last step)
# ==========================================
WORKDIR /tmp
# 1. generate rosinstall file
RUN mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT} && \
    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ros_base \
        launch_xml launch_yaml launch_testing launch_testing_ament_cmake \
        demo_nodes_cpp example_interfaces camera_calibration_parsers \
        vision_msgs image_geometry \
        image_pipeline image_transport compressed_image_transport compressed_depth_image_transport \
        > ros2.${ROS_DISTRO}.rosinstall

# 2. import repos
RUN cd ${ROS_ROOT} && vcs import src < ros2.${ROS_DISTRO}.rosinstall
# remove useless vendor packages causing cmake <3.5 failures
# nuke all useless vendor junk before build
RUN rm -rf ${ROS_ROOT}/src/osrf_testing_tools_cpp \
           ${ROS_ROOT}/src/iceoryx \
           ${ROS_ROOT}/src/uncrustify_vendor \
           ${ROS_ROOT}/src/google_benchmark_vendor \
           ${ROS_ROOT}/src/zstd_vendor \
           ${ROS_ROOT}/src/fastrtps \
           ${ROS_ROOT}/src/mimick_vendor



# 3. fix ament_cmake (remove and re‑clone cleanly)
RUN rm -rf ${ROS_ROOT}/src/ament_cmake && \
    git -C ${ROS_ROOT}/src/ clone https://github.com/ament/ament_cmake -b ${ROS_DISTRO}

# 4. init rosdep and update
RUN rosdep init || true && rosdep update

# 5. install dependencies (skip garbage packages jetson doesn’t have)
# RUN cd ${ROS_ROOT} && \
#     rosdep install -y --ignore-src --from-paths src --rosdistro ${ROS_DISTRO} \
#         --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev \
#         python-opencv python3-opencv \
#         libxml2-utils python3-pytest python3-pytest-timeout pydocstyle python3-importlib-metadata \
#         python3-argcomplete python3-flake8 libeigen3-dev python3-lxml libzstd-dev libssl-dev libconsole-bridge-dev \
#         libyaml-dev libboost-dev python3-mypy python3-empy uncrustify libspdlog-dev clang-format libeigen3-dev python3-catkin-pkg-modules \
#         libtinyxml-dev python3-cryptography libeigen3-dev python3-pykdl libsqlite3-dev libbenchmark-dev python3-pytest-mock \
#         cppcheck graphviz acl libyaml-cpp-dev libeigen3-dev libspdlog-dev python3-pycodestyle pybind11-dev libeigen3-dev \
#         libboost-python-dev libeigen3-dev python3-lark libbullet-dev rti-connext-dds-6.0.1 libboost-python1.71.0 libyaml-dev \
#         libeigen3-dev python3-psutil python3-lark libbenchmark-dev libtinyxml-dev libbullet-dev python3-rosdistro-modules \
#         python3-netifaces libspdlog-dev libbenchmark-dev libbullet-dev python3-packaging libtinyxml-dev"
RUN apt-get update && apt-get install -y --no-install-recommends \
    libxml2-utils python3-argcomplete python3-flake8 libeigen3-dev \
    python3-lxml libzstd-dev libconsole-bridge-dev libyaml-dev \
    libboost-dev python3-mypy python3-empy uncrustify libspdlog-dev \
    clang-format libtinyxml-dev \
    python3-cryptography python3-pykdl libsqlite3-dev libbenchmark-dev \
    python3-pytest python3-pytest-mock python3-pytest-timeout pydocstyle cppcheck graphviz acl libyaml-cpp-dev \
    pybind11-dev libboost-python-dev python3-psutil python3-packaging \
    libbullet-dev python3-lark python3-netifaces \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade --no-cache-dir cmake scikit-build ninja
ENV CMAKE_ARGS="-DBUILD_IDLC=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5"
RUN cd ${ROS_ROOT} && colcon build --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_IDLC=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5

# 7. cleanup build artifacts
RUN rm -rf ${ROS_ROOT}/src ${ROS_ROOT}/logs ${ROS_ROOT}/build *.rosinstall


# ==========================================
# env + entrypoint
# ==========================================
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


COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh && echo 'source /ros_entrypoint.sh' >> /root/.bashrc

WORKDIR /workspace
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
