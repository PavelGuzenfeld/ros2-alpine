# ROS 2 Jazzy on Alpine Linux

Minimal Docker image for ROS 2 Jazzy built from source on Alpine Linux (musl libc).
~200MB final image vs ~2GB for Ubuntu-based ROS 2 images.

## Quick Start

```bash
# Pull from GitHub Container Registry
docker pull ghcr.io/pavelguzenfeld/ros2-alpine:latest

# Run with host networking (for ROS 2 discovery)
docker run -it --network host ghcr.io/pavelguzenfeld/ros2-alpine:latest
```

### Build Locally

```bash
./pipeline.sh                  # Build and test
./pipeline.sh --build-only     # Build without testing
./pipeline.sh --test-only      # Test existing image
./pipeline.sh --clean          # Clean cache and rebuild
./build.sh                     # Build only (no pipeline)
./test.sh                      # Test existing image
```

## Image Details

| Property | Value |
|----------|-------|
| Base | Alpine Linux 3.21 |
| Architectures | amd64, arm64 |
| ROS Distribution | Jazzy (built from source) |
| Middleware | Fast-DDS 2.14.6 (rmw_fastrtps_cpp) |
| Python | 3.12 |
| Image size | ~200MB |
| Build time | ~3 hours (from source, 2 parallel jobs) |

## What's Included

- Core ROS 2: rcl, rclpy, rosidl, rmw
- Standard messages: std_msgs, builtin_interfaces, common_interfaces
- CLI tools: ros2 topic, ros2 node, ros2 service, etc.
- Fast-DDS 2.14.6 middleware (pinned for Jazzy compatibility)
- Python bindings (rclpy)
- colcon build system

## Usage

```bash
# Mount a workspace
docker run -it --network host -v $(pwd)/ws:/workspace \
    ghcr.io/pavelguzenfeld/ros2-alpine:latest

# Inside the container
source /opt/ros/jazzy/setup.bash
ros2 topic list

# Python node example
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

```bash
# Use more parallel jobs (needs more RAM)
docker build --build-arg PARALLEL_JOBS=4 -t ros2-jazzy-alpine .

# Pin a specific Alpine version
docker build --build-arg ALPINE_VERSION=3.21 -t ros2-jazzy-alpine .
```

## System Requirements

- Docker 20.10+ with BuildKit
- 8GB+ RAM (for building from source)
- 10GB+ disk space

## Project Structure

```
ros2-alpine/
├── Dockerfile           # Multi-stage build (8 colcon stages + runtime)
├── build.sh             # Docker build script
├── test.sh              # Image validation tests
├── pipeline.sh          # Build + test orchestration
├── .hadolint.yaml       # Hadolint configuration
└── .github/workflows/
    ├── build.yml        # CI: lint, build, test, publish
    └── release.yml      # GitHub Releases on version tags
```

## CI/CD

GitHub Actions automatically:
- **Push to main**: lint, build, test, publish to GHCR (+ Docker Hub if configured)
- **Tags (v\*)**: publish versioned images + GitHub Release
- **Weekly**: rebuild for Alpine security updates
- **PRs**: lint only (ShellCheck + Hadolint)

### Docker Hub Setup (Optional)

Add to GitHub repo settings:
- **Variable** `DOCKERHUB_USERNAME`: your Docker Hub username
- **Secret** `DOCKERHUB_TOKEN`: a Docker Hub access token

### Creating a Release

```bash
git tag v0.2.0
git push origin v0.2.0
```

## License

MIT. ROS 2 components retain their original licenses (Apache 2.0).
