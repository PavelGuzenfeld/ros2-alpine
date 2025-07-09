#!/bin/bash

# Run the container, ensuring GPU access with --runtime nvidia
# The container will run the test_gpu.py script and then exit.
sudo docker run \
  -it \
  --rm \
  --runtime nvidia \
  --network host \
  --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -e NVIDIA_REQUIRE_CUDA="cuda>=11.4" \
  my-minimal-tf-ubuntu:jp51
