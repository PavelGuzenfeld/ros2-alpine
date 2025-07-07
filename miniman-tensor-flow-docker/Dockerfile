# Minimal TensorFlow + OpenCV Dockerfile for Jetson (L4T r35.2.1 / JetPack 5.1)
FROM nvcr.io/nvidia/l4t-base:r35.2.1 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# build-time deps and python venv setup
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    cmake \
    ninja-build \
    curl \
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
RUN pip3 install click

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
# Final Stage
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
ENV LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libGLdispatch.so"

# setup nvidia runtime deps and supporting libraries for GStreamer
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    wget \
    gnupg \
    python3 \
    libopenblas-base \
    liblapack3 \
    cuda-runtime-11-4 \
    libcudnn8 \
    libnvinfer8 \
    libnvinfer-plugin8 \
    libnvonnxparsers8 \
    libnvparsers8 \
    python3-dev \
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
    python3-gi-cairo && \
    rm -rf /var/lib/apt/lists/*

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

# update library cache
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf && \
    echo "/usr/lib/aarch64-linux-gnu" > /etc/ld.so.conf.d/gstreamer.conf && \
    ldconfig

# copy app with enhanced tests
COPY ./test_gpu.py /app/test_gpu.py
WORKDIR /app

# install nvidia tensorflow in final image
RUN pip install --no-cache-dir --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v51 tensorflow==2.11.0+nv23.1 && \
    pip install --no-cache-dir click

CMD [ "python3", "test_gpu.py" ]