#!/bin/bash
set -e

# Build the Docker image
# Using DOCKER_BUILDKIT for a more efficient build process
DOCKER_BUILDKIT=1 docker build -t my-minimal-tf-ubuntu:jp51.

echo "Build complete. Image 'my-minimal-tf-ubuntu:jp51' is ready."
