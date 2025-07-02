# ROS 2 Jazzy Alpine Build

A production-ready build system for creating minimal ROS 2 Jazzy containers on Alpine Linux from source.

## 📁 Project Structure

```
ros2-alpine-build/
├── Dockerfile.ros2-jazzy-alpine     # 🐳 Multi-stage ROS 2 build definition
├── build.sh                         # 🔨 Docker build orchestration
├── test.sh                          # 🧪 Comprehensive test suite
├── pipeline.sh                      # 🚀 Main build pipeline (ENTRY POINT)
├── README.md                        # 📖 Project documentation
└── .gitignore                       # 🚫 Git exclusion rules
```

## 🎯 File Responsibilities

| File | Purpose | Can Run Standalone |
|------|---------|-------------------|
| **`pipeline.sh`** | **Main entry point** - orchestrates build & test | ✅ **Recommended** |
| `build.sh` | Docker build execution and image management | ✅ |
| `test.sh` | Container functionality testing and validation | ✅ (requires image) |
| `Dockerfile.ros2-jazzy-alpine` | Multi-stage container build definition | ✅ (via Docker) |

## 🚀 Quick Start

### Option 1: Complete Pipeline (Recommended)
```bash
# Make scripts executable
chmod +x *.sh

# Run complete build and test pipeline
./pipeline.sh
```

### Option 2: Step-by-Step
```bash
# Build only
./build.sh

# Test only (after build)
./test.sh
```

## 📋 Pipeline Options

```bash
# Full pipeline (build + test)
./pipeline.sh

# Build only
./pipeline.sh --build-only

# Test existing image
./pipeline.sh --test-only

# Build without testing
./pipeline.sh --skip-tests

# Clean cache and build
./pipeline.sh --clean

# Interactive testing
./pipeline.sh --interactive

# Show system info
./pipeline.sh --info

# Help
./pipeline.sh --help
```

## 🧪 Test Suite

The test suite includes 6 comprehensive tests:

### Critical Tests (Must Pass)
1. **Container Startup** - Basic container functionality
2. **Python Imports** - ROS 2 Python packages (rclpy, ament_package)
3. **Node Creation** - ROS 2 node lifecycle

### Optional Tests (Nice to Have)
4. **CLI Tools** - ros2 command availability
5. **Environment Debug** - System information and paths
6. **Message Types** - ROS 2 message creation

## 🏗️ Build Process

### Multi-Stage Docker Build
1. **Builder Stage** (Alpine + Build Tools)
   - Foundation packages (gtest, ament)
   - External dependencies (Fast-DDS, Fast-CDR)
   - Core ROS packages (rcutils, rmw, rclpy)
   - Message interfaces and CLI tools

2. **Runtime Stage** (Clean Alpine)
   - Minimal runtime dependencies
   - Built ROS 2 packages
   - Environment configuration

### Build Stages
- **Stage 1-2**: Foundation and build system
- **Stage 3-4**: External dependencies and middleware
- **Stage 5-7**: Core ROS packages and Python bindings
- **Stage 8-10**: Message interfaces and CLI tools

## 📊 Specifications

- **Base Image**: Alpine Linux (latest)
- **ROS Distribution**: Jazzy (built from source)
- **Middleware**: FastRTPS (rmw_fastrtps_cpp)
- **Python Version**: 3.12
- **Build System**: colcon
- **Final Image Size**: ~200MB
- **Build Time**: 1-3 hours (hardware dependent)

## 🔧 System Requirements

- **Docker**: 20.10+ (with BuildKit support)
- **Memory**: 8GB+ recommended
- **Disk Space**: 10GB+ available
- **Network**: Internet connection for package downloads

## 🐛 Troubleshooting

### Build Issues
```bash
# Check system resources
./pipeline.sh --info

# Clean build cache
./pipeline.sh --clean

# Build only (skip tests)
./pipeline.sh --build-only
```

### Test Issues
```bash
# Run interactive tests
./pipeline.sh --interactive

# Test specific functionality
./test.sh --interactive

# Check container manually
docker run -it ros2-jazzy-alpine:latest bash
```

### Common Problems

1. **Out of Memory**: Increase Docker memory limit to 8GB+
2. **Disk Space**: Ensure 10GB+ available disk space
3. **Network Issues**: Check internet connection for git clones
4. **Permission Issues**: Ensure scripts are executable (`chmod +x *.sh`)

## 🚀 Usage Examples

### Run Container
```bash
# Basic usage
docker run -it ros2-jazzy-alpine:latest

# With network access
docker run -it --network host ros2-jazzy-alpine:latest

# Mount workspace
docker run -it -v $(pwd)/workspace:/workspace ros2-jazzy-alpine:latest
```

### Inside Container
```bash
# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Check ROS installation
ros2 pkg list | head -10

# Create a simple node
python3 -c "
import rclpy
from rclpy.node import Node

rclpy.init()
node = Node('test_node')
node.get_logger().info('Hello from Alpine ROS 2!')
rclpy.shutdown()
"
```

## 🤝 Contributing

1. Fork the repository
2. Create feature branch
3. Test your changes
4. Submit pull request

## 📄 License

This project is open source. See individual package licenses for ROS 2 components.

## 🏷️ Tags

`ros2` `alpine` `docker` `jazzy` `robotics` `minimal` `source-build`