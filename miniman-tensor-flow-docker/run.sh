#!/bin/bash

# Run the container, ensuring GPU access with --runtime nvidia
# The container will run the test_gpu.py script and then exit.
sudo docker run \
    -it \
    --rm \
    --runtime nvidia \
    --network host \
    my-minimal-tf-ubuntu:jp51
