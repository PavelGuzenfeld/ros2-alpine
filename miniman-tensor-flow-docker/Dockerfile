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
    gfortran && \
    rm -rf /var/lib/apt/lists/*

# create python venv and install deps
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

COPY requirements.txt ./
RUN pip install --no-cache-dir --upgrade pip wheel && \
    pip install --no-cache-dir -r requirements.txt

# build OpenCV inside venv
RUN mkdir -p /tmp/opencv && cd /tmp/opencv && \
    git clone --depth 1 --branch 4.x https://github.com/opencv/opencv.git . && \
    PY_SITEPACKAGES=$(python3 -c "import site; print(site.getsitepackages()[0])") && \
    cmake -S . -B build -G Ninja \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D BUILD_LIST=core,imgproc,imgcodecs,python3 \
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
    -D WITH_CUDA=OFF && \
    cmake --build build -j"$(nproc)" && \
    cmake --install build && \
    rm -rf /tmp/opencv

# =========================================================================
# Final Stage
# =========================================================================
FROM nvcr.io/nvidia/l4t-base:r35.2.1

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="/opt/venv/bin:$PATH"
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# setup nvidia runtime deps
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
    libhdf5-dev && \
    rm -rf /var/lib/apt/lists/*

# add Jetson repo keys
RUN apt-key adv --fetch-keys https://repo.download.nvidia.com/jetson/jetson-ota-public.asc && \
    echo "deb https://repo.download.nvidia.com/jetson/common r35.2 main" > /etc/apt/sources.list.d/nvidia-jetson.list && \
    echo "deb https://repo.download.nvidia.com/jetson/t234 r35.2 main" >> /etc/apt/sources.list.d/nvidia-jetson.list

# copy prebuilt python venv with OpenCV + deps
COPY --from=builder /opt/venv /opt/venv

# copy OpenCV shared libraries from builder stage
COPY --from=builder /usr/local/lib/libopencv* /usr/local/lib/

# update library cache
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf && \
    ldconfig

# copy app
COPY ./test_gpu.py /app/test_gpu.py
WORKDIR /app

# install nvidia tensorflow in final image
RUN pip install --no-cache-dir --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v51 tensorflow==2.11.0+nv23.1

CMD [ "python3", "test_gpu.py" ]