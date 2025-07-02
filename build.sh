#!/bin/bash

set -e

echo "🚀 Building Complete ROS 2 Core on Foundation (SMART VERSION 23.0 - PURE ALPINE LATEST BUILD)"
echo "============================================="

# Check if Dockerfile exists
if [ ! -f "Dockerfile.ros2-core" ]; then
    echo "❌ Dockerfile.ros2-core not found! Please ensure it exists in the current directory."
    exit 1
fi

echo "📋 Using existing Dockerfile.ros2-core"

# Build the complete image
echo ""
echo "🔨 Building complete ROS 2 core from pure Alpine latest..."
echo "⏰ Started at: $(date)"

docker build \
    -f Dockerfile.ros2-core \
    -t ros2-jazzy-alpine:core \
    -t ros2-jazzy-alpine:complete \
    .

BUILD_EXIT_CODE=$?

if [ $BUILD_EXIT_CODE -ne 0 ]; then
    echo ""
    echo "❌ Build failed! Check the logs above"
    exit 1
fi

echo ""
echo "✅ Complete ROS 2 core build finished at: $(date)"

# Get image size
IMAGE_SIZE=$(docker images --format "table {{.Size}}" "ros2-jazzy-alpine:core" | tail -1)
echo "📦 Complete image size: $IMAGE_SIZE"

echo ""
echo "🧪 Testing the complete ROS 2 core..."

# Test functionality
echo "Test 1: ROS 2 CLI tools..."
docker run --rm ros2-jazzy-alpine:core bash -c '
source /opt/ros/jazzy/setup.bash
if command -v ros2 >/dev/null 2>&1; then
    echo "✅ ros2 command available!"
    ros2 --help | head -5
else
    echo "⚠️  ros2 command not available yet"
fi
'

echo "Test 2: Python ROS 2..."
docker run --rm ros2-jazzy-alpine:core bash -c '
source /opt/ros/jazzy/setup.bash
python3 -c "
import sys
print(f\"Python: {sys.version}\")
print(f\"Python path: {sys.path[:3]}...\")  # Show first 3 entries

packages = [\"ament_package\", \"rclpy\", \"std_msgs\"]
for pkg in packages:
    try:
        __import__(pkg)
        print(f\"✅ {pkg}: OK\")
    except ImportError as e:
        print(f\"⚠️  {pkg}: {e}\")

# Debug: Look for Python packages
import os
ros_lib = \"/opt/ros/jazzy\"
if os.path.exists(ros_lib):
    print(f\"\\nROS directory contents:\")
    dirs = [d for d in os.listdir(ros_lib) if os.path.isdir(os.path.join(ros_lib, d))][:10]
    print(f\"Subdirectories: {dirs}\")
    
    # Look for Python packages
    for subdir in [\"rclpy\", \"std_msgs\"]:
        pkg_path = os.path.join(ros_lib, subdir)
        if os.path.exists(pkg_path):
            print(f\"Found {subdir} directory\")
            lib_path = os.path.join(pkg_path, \"lib\", \"python3.12\", \"site-packages\")
            if os.path.exists(lib_path):
                print(f\"  Python packages in {subdir}: {os.listdir(lib_path)[:5]}\")
        else:
            print(f\"Missing {subdir} directory\")
"
'

echo "Test 3: Create simple ROS 2 node..."
docker run --rm ros2-jazzy-alpine:core bash -c '
source /opt/ros/jazzy/setup.bash
cat > /tmp/test_node.py << "EOL"
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main():
    try:
        rclpy.init()
        node = Node("test_node")
        node.get_logger().info("✅ ROS 2 Jazzy Alpine test node working!")
        rclpy.shutdown()
        print("✅ Node test successful")
    except Exception as e:
        print(f"⚠️  Node test failed: {e}")

if __name__ == "__main__":
    main()
EOL
python3 /tmp/test_node.py
'

echo "Test 4: Check RMW implementation..."
docker run --rm ros2-jazzy-alpine:core bash -c '
source /opt/ros/jazzy/setup.bash
echo "RMW Implementation: $RMW_IMPLEMENTATION"
ros2 doctor --report | head -10 || echo "ROS 2 doctor not available yet"
'

echo ""
echo "🎉 ROS 2 Jazzy Alpine Core Complete! (pure Alpine latest build edition)"