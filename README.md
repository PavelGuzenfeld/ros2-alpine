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
| Build time | ~5.5 hours (from source, 2 parallel jobs) |

## What's Included

- **C++ client library**: rclcpp (with actions, components, lifecycle)
- **Python client library**: rclpy
- **Common interfaces**: std_msgs, geometry_msgs, sensor_msgs, nav_msgs, diagnostic_msgs, visualization_msgs, shape_msgs, trajectory_msgs, stereo_msgs
- **Transforms**: tf2, tf2_ros, tf2_geometry_msgs
- **Middleware**: Fast-DDS 2.14.6 with rmw_fastrtps_cpp
- **CLI tools**: ros2 topic, ros2 node, ros2 service, ros2 param, etc.
- **Build system**: colcon, ament_cmake, ament_lint
- **Utilities**: message_filters, unique_identifier_msgs, action_msgs, class_loader
- **Non-root**: runs as `ros` user by default

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
├── Dockerfile           # Multi-stage build (9 colcon stages + runtime)
├── build.sh             # Docker build script
├── test.sh              # Image validation tests (11 checks)
├── pipeline.sh          # Build + test orchestration
├── .hadolint.yaml       # Hadolint lint suppressions
├── .dockerignore
└── .github/workflows/
    ├── build.yml        # CI: lint, build, test, publish
    └── release.yml      # GitHub Releases on version tags
```

## CI/CD

GitHub Actions automatically:
- **Push to main**: lint (ShellCheck + Hadolint), build, test, publish to GHCR
- **Tags (v\*)**: publish versioned images + create GitHub Release
- **Weekly**: rebuild for Alpine security updates
- **PRs**: lint only

### Docker Hub (Optional)

Add to GitHub repo settings:
- **Variable** `DOCKERHUB_USERNAME`: your Docker Hub username
- **Secret** `DOCKERHUB_TOKEN`: a Docker Hub access token

### Creating a Release

```bash
git tag v0.4.0
git push origin v0.4.0
```

## License

MIT. ROS 2 components retain their original licenses (Apache 2.0).
