# ROS 2 Jazzy on Alpine Linux

Minimal, production-ready Docker image for ROS 2 Jazzy built from source on Alpine Linux.
Final image size is ~200MB compared to ~2GB for Ubuntu-based ROS 2 images.

## Quick Start

### Pull Pre-built Image

```bash
# From GitHub Container Registry
docker pull ghcr.io/pavelguzenfeld/ros2-alpine:latest

# From Docker Hub
docker pull pavelguzenfeld/ros2-alpine:latest

# Run
docker run -it ghcr.io/pavelguzenfeld/ros2-alpine:latest
```

### Versioned Releases

```bash
# Specific version
docker pull ghcr.io/pavelguzenfeld/ros2-alpine:0.1.0

# Latest
docker pull ghcr.io/pavelguzenfeld/ros2-alpine:latest
```

### Build Locally

```bash
# Full pipeline (build + test)
./pipeline.sh

# Build only
./build.sh

# Test existing image
./test.sh
```

## Pipeline Options

```bash
./pipeline.sh                  # Build and test
./pipeline.sh --build-only     # Build without testing
./pipeline.sh --test-only      # Test existing image
./pipeline.sh --clean          # Clean cache and rebuild
./pipeline.sh --info           # Show system info
```

## Image Details

| Property | Value |
|----------|-------|
| Base | Alpine Linux 3.21 |
| Architectures | amd64, arm64 |
| ROS Distribution | Jazzy (built from source) |
| Middleware | FastRTPS (rmw_fastrtps_cpp) |
| Python | 3.12 |
| Final image size | ~200MB |
| Build time | 1-3 hours |

## What's Included

- Core ROS 2 packages (rcl, rclpy, rosidl)
- Standard message types (std_msgs, builtin_interfaces)
- ROS 2 CLI tools (ros2 topic, ros2 node, etc.)
- FastRTPS middleware
- Python bindings (rclpy)
- colcon build system

## Usage

```bash
# Run with host networking (for ROS 2 discovery)
docker run -it --network host ghcr.io/pavelguzenfeld/ros2-alpine:latest

# Mount a workspace
docker run -it -v $(pwd)/ws:/workspace ghcr.io/pavelguzenfeld/ros2-alpine:latest

# Inside the container
source /opt/ros/jazzy/setup.bash
python3 -c "
import rclpy
from rclpy.node import Node
rclpy.init()
node = Node('hello')
node.get_logger().info('Hello from Alpine ROS 2!')
rclpy.shutdown()
"
```

## Build Configuration

The Dockerfile supports build arguments for customization:

```bash
# Use more parallel jobs on a machine with more RAM
docker build --build-arg PARALLEL_JOBS=4 -t ros2-jazzy-alpine .

# Pin a specific Alpine version
docker build --build-arg ALPINE_VERSION=3.21 -t ros2-jazzy-alpine .
```

## System Requirements

- Docker 20.10+ with BuildKit
- 8GB+ RAM (for building from source)
- 10GB+ disk space
- Internet connection (for cloning ROS 2 sources)

## Project Structure

```
ros2-alpine/
├── Dockerfile           # Multi-stage build definition
├── build.sh             # Docker build script
├── test.sh              # Image validation tests
├── pipeline.sh          # Build + test orchestration
└── .github/
    └── workflows/
        ├── build.yml    # CI: lint, build, test, publish to GHCR + Docker Hub
        └── release.yml  # Create GitHub Releases on version tags
```

## CI/CD

GitHub Actions automatically:
- **On push to main**: builds, tests, and publishes to GHCR + Docker Hub
- **On tags (v\*)**: publishes versioned releases with GitHub Release notes
- **Weekly**: rebuilds to pick up Alpine security updates
- **On PRs**: runs linting (ShellCheck, Hadolint)

### Docker Hub Setup

To enable Docker Hub publishing, add these to your GitHub repo settings:
- **Variable** `DOCKERHUB_USERNAME`: your Docker Hub username
- **Secret** `DOCKERHUB_TOKEN`: a Docker Hub access token

Without these, images are published to GHCR only.

### Creating a Release

```bash
git tag v0.2.0
git push origin v0.2.0
```

## License

MIT. ROS 2 components retain their original licenses (Apache 2.0).
