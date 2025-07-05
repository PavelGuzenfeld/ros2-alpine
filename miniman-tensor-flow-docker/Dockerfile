# Dockerfile for a Minimal TensorFlow Image on Jetson (Ubuntu-based)
# Target: Jetson Xavier with JetPack 5.1 (L4T r35.2.1)

# =========================================================================
# Stage 1: The Builder
#
# This stage installs all build tools and development libraries needed to
# compile Python packages from source. None of these tools will be in the
# final image.
# =========================================================================
FROM nvcr.io/nvidia/l4t-base:r35.2.1 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Install essential build tools and Python development headers
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    cmake \
    curl \
    ca-certificates \
    python3-dev \
    python3-pip \
    python3-venv

# Install build-time dependencies for key Python packages
RUN apt-get install -y --no-install-recommends \
    libhdf5-dev \
    libopenblas-dev \
    liblapack-dev \
    gfortran && \
    rm -rf /var/lib/apt/lists/*

# Create a Python virtual environment to keep the installation clean
RUN python3 -m venv /opt/venv

# Activate the virtual environment for subsequent commands
ENV PATH="/opt/venv/bin:$PATH"

# Install Python dependencies into the virtual environment
# We install build tools like wheel and cython first
COPY requirements.txt .
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir wheel && \
    pip install --no-cache-dir -r requirements.txt

# =========================================================================
# Stage 2: The Final Image
#
# This stage starts from a clean base and installs only the essential
# runtime libraries, then copies the pre-built Python environment.
# =========================================================================
FROM nvcr.io/nvidia/l4t-base:r35.2.1

ENV DEBIAN_FRONTEND=noninteractive

# Install runtime dependencies for NVIDIA libraries and Python packages
# Note: We install the runtime packages, not the '-dev' versions
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    libhdf5-serial-dev \
    libopenblas-base \
    liblapack3 && \
    rm -rf /var/lib/apt/lists/*

# Copy the pre-built Python virtual environment from the builder stage
COPY --from=builder /opt/venv /opt/venv

# Copy the application and test script
COPY ./test_gpu.py /app/test_gpu.py

# Set the PATH to use the Python from our virtual environment
ENV PATH="/opt/venv/bin:$PATH"

# Install the official NVIDIA TensorFlow wheel for JetPack 5.1
# This ensures it links against the runtime libraries in this final image.
# The version 2.11.0 is the correct one for JetPack 5.1 (L4T r35.2.1).
RUN pip3 install --no-cache-dir --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v51 tensorflow==2.11.0+nv23.1

# Set up a non-root user for security
RUN adduser --disabled-password --gecos '' appuser
USER appuser
WORKDIR /app

# Set the default command to run the verification script
CMD [ "python3", "test_gpu.py" ]
