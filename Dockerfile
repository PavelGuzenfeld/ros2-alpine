# ROS 2 Jazzy Alpine Linux - Multi-stage Build
#
# Builds ROS 2 Jazzy from source on Alpine Linux (musl libc).
# Final image is ~200MB vs ~2GB for Ubuntu-based ROS 2 images.

ARG ALPINE_VERSION=3.21
ARG ROS_DISTRO=jazzy
ARG PARALLEL_JOBS=2

# ===========================================================================
# Builder stage
# ===========================================================================
FROM alpine:${ALPINE_VERSION} AS builder

ARG PARALLEL_JOBS
ARG ROS_DISTRO

RUN apk add --no-cache \
    bash \
    build-base \
    cmake \
    ninja \
    python3-dev \
    py3-pip \
    git \
    curl \
    linux-headers \
    eigen-dev \
    tinyxml2-dev \
    yaml-cpp-dev \
    yaml-dev \
    pkgconfig \
    uncrustify \
    cppcheck \
    py3-flake8 \
    libxml2-utils \
    asio-dev \
    openssl-dev \
    lttng-ust-dev \
    swig \
    cython \
    py3-wheel \
    py3-pybind11-dev \
    libc-dev

ENV PYTHONWARNINGS=ignore::UserWarning:setuptools._distutils.dist
RUN pip3 install --break-system-packages --quiet \
    colcon-common-extensions \
    lark \
    empy \
    pyyaml \
    numpy \
    setuptools \
    wheel \
    pybind11 \
    cython \
    cpplint \
    pycodestyle \
    pydocstyle \
    catkin-pkg \
    distlib \
    vcstool

WORKDIR /ros2_ws

# Clone ROS 2 source packages pinned to the jazzy branch.
# Shallow clones (--depth 1) speed up the clone step significantly.
# eProsima repos don't have a jazzy branch; they use their default branch.
RUN mkdir -p src && cd src && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ament/ament_package.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ament/ament_cmake.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ament/ament_index.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ament/googletest.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ament/ament_lint.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/ament_cmake_ros.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rcutils.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rcpputils.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rmw.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rosidl.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rcl.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rclpy.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/ros2cli.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/common_interfaces.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/libyaml_vendor.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rosidl_python.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rpyutils.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rosidl_core.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rosidl_defaults.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rosidl_dynamic_typesupport.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rosidl_dynamic_typesupport_fastrtps.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rosidl_typesupport.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/unique_identifier_msgs.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rcl_interfaces.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rcl_logging.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rmw_implementation.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rmw_fastrtps.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rmw_dds_common.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/ros2_tracing.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/spdlog_vendor.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/pybind11_vendor.git && \
    git clone --depth 1 --branch ${ROS_DISTRO} https://github.com/ros2/rosidl_typesupport_fastrtps.git && \
    git clone --depth 1 https://github.com/eProsima/Fast-DDS.git && \
    git clone --depth 1 https://github.com/eProsima/Fast-CDR.git && \
    git clone --depth 1 https://github.com/eProsima/foonathan_memory_vendor.git

# Remove test/example/benchmark directories to reduce build scope
RUN cd src && \
    find . -name "*_test" -type d \
        ! -path "./ament_cmake/ament_cmake_test*" \
        ! -path "./ament_cmake/ament_cmake_gmock*" \
        ! -path "./ament_cmake_ros/rmw_test_fixture*" \
        -exec rm -rf {} + 2>/dev/null; \
    find . -name "*_tests" -type d -exec rm -rf {} + 2>/dev/null; \
    find . -name "test_*" -type d -exec rm -rf {} + 2>/dev/null; \
    find . -name "*example*" -type d -exec rm -rf {} + 2>/dev/null; \
    find . -name "*demo*" -type d -exec rm -rf {} + 2>/dev/null; \
    find . -name "*benchmark*" -type d -exec rm -rf {} + 2>/dev/null; \
    true

ENV MAKEFLAGS="-j${PARALLEL_JOBS}"
ENV CFLAGS="-Wno-int-conversion -Wno-incompatible-pointer-types -Wno-error"
ENV CXXFLAGS="-Wno-error -Wno-deprecated-declarations -Wno-stringop-overread"

# Build Stage 1: Foundation packages
RUN colcon build \
        --packages-select \
            gtest_vendor \
            gmock_vendor \
            ament_package \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DCMAKE_C_FLAGS="${CFLAGS}" \
            -DCMAKE_CXX_FLAGS="${CXXFLAGS}" \
        --event-handlers console_direct+

# Build Stage 2: Ament build system
RUN bash -c 'source install/setup.bash && \
    colcon build \
        --packages-up-to \
            ament_cmake \
            ament_lint_common \
            ament_mypy \
            ament_index_python \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPython3_EXECUTABLE=/usr/bin/python3 \
            -DCMAKE_C_FLAGS="${CFLAGS}" \
            -DCMAKE_CXX_FLAGS="${CXXFLAGS}" \
        --event-handlers console_direct+'

# Build Stage 3: External dependencies
RUN bash -c 'source install/setup.bash && \
    colcon build \
        --packages-up-to \
            foonathan_memory_vendor \
            fastcdr \
            libyaml_vendor \
            spdlog_vendor \
            pybind11_vendor \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DCMAKE_C_FLAGS="${CFLAGS}" \
            -DCMAKE_CXX_FLAGS="${CXXFLAGS}" \
        --event-handlers console_direct+'

# Build Stage 4: Fast-DDS middleware (heavy, separate layer for caching)
RUN bash -c 'source install/setup.bash && \
    colcon build \
        --packages-up-to \
            fastdds \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DBUILD_DOCUMENTATION=OFF \
            -DCOMPILE_EXAMPLES=OFF \
            -DCMAKE_C_FLAGS="${CFLAGS}" \
            -DCMAKE_CXX_FLAGS="${CXXFLAGS}" \
        --event-handlers console_direct+'

# Build Stage 5: rpyutils
RUN bash -c 'source install/setup.bash && \
    colcon build \
        --packages-select \
            rpyutils \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPython3_EXECUTABLE=/usr/bin/python3 \
            -DCMAKE_C_FLAGS="${CFLAGS}" \
            -DCMAKE_CXX_FLAGS="${CXXFLAGS}" \
        --event-handlers console_direct+'

# Build Stage 6: ROSIDL core
RUN bash -c 'source install/setup.bash && \
    colcon build \
        --packages-up-to \
            rosidl_generator_py \
            rosidl_adapter \
            rosidl_cmake \
            rosidl_parser \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPython3_EXECUTABLE=/usr/bin/python3 \
            -DCMAKE_C_FLAGS="${CFLAGS}" \
            -DCMAKE_CXX_FLAGS="${CXXFLAGS}" \
        --event-handlers console_direct+'

# Build Stage 7: RMW and all fastrtps-dependent packages
RUN bash -c 'source install/setup.bash && \
    colcon build \
        --packages-up-to \
            rmw_fastrtps_cpp \
            rmw_fastrtps_shared_cpp \
            rmw_fastrtps_dynamic_cpp \
            rosidl_typesupport_fastrtps_cpp \
            rosidl_typesupport_fastrtps_c \
            rosidl_dynamic_typesupport_fastrtps \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DCMAKE_C_FLAGS="${CFLAGS}" \
            -DCMAKE_CXX_FLAGS="${CXXFLAGS}" \
        --event-handlers console_direct+'

# Build Stage 8: Core ROS packages
RUN bash -c 'source install/setup.bash && \
    colcon build \
        --packages-up-to \
            rcutils \
            rmw \
            builtin_interfaces \
            std_msgs \
            rclpy \
            ros2cli \
        --packages-skip \
            libyaml_vendor \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_TESTING=OFF \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPython3_EXECUTABLE=/usr/bin/python3 \
            -DCMAKE_C_FLAGS="${CFLAGS}" \
            -DCMAKE_CXX_FLAGS="${CXXFLAGS}" \
        --event-handlers console_direct+'

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ===========================================================================
# Runtime stage - Clean Alpine with only runtime dependencies
# ===========================================================================
FROM alpine:${ALPINE_VERSION} AS runtime

ARG ROS_DISTRO

LABEL org.opencontainers.image.title="ROS 2 Jazzy on Alpine Linux"
LABEL org.opencontainers.image.description="Minimal ROS 2 Jazzy built from source on Alpine Linux"
LABEL org.opencontainers.image.source="https://github.com/PavelGuzenfeld/ros2-alpine"
LABEL org.opencontainers.image.licenses="MIT"
LABEL ros.distro="${ROS_DISTRO}"

RUN apk add --no-cache \
    python3 \
    py3-setuptools \
    py3-yaml \
    py3-numpy \
    py3-pip \
    py3-wheel \
    bash \
    libstdc++ \
    libgcc \
    tinyxml2 \
    yaml-cpp \
    yaml \
    openssl \
    lttng-ust

ENV PYTHONWARNINGS=ignore::UserWarning:setuptools._distutils.dist
RUN pip3 install --break-system-packages --quiet \
    lark \
    wheel \
    colcon-common-extensions \
    typing_extensions

RUN mkdir -p /opt/ros/jazzy
COPY --from=builder /ros2_ws/install /opt/ros/jazzy

ENV ROS_VERSION=2 \
    ROS_DISTRO=jazzy \
    ROS_PYTHON_VERSION=3 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN if [ -f /opt/ros/jazzy/setup.bash ]; then \
        chmod +x /opt/ros/jazzy/setup.bash; \
    else \
        printf '#!/bin/bash\n. /opt/ros/jazzy/local_setup.bash\n' > /opt/ros/jazzy/setup.bash; \
        chmod +x /opt/ros/jazzy/setup.bash; \
    fi

RUN mkdir -p /workspace/src
WORKDIR /workspace

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && echo 'ROS 2 Jazzy Alpine ready!' && /bin/bash"]
