#!/bin/bash
set -uo pipefail

IMAGE_NAME="${1:-${ROS2_IMAGE:-ros2-jazzy-alpine:latest}}"

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

PASS=0
FAIL=0

run_test() {
    local name="$1"
    local cmd="$2"

    if docker run --rm "$IMAGE_NAME" bash -c "$cmd" >/dev/null 2>&1; then
        printf "  ${GREEN}PASS${NC}  %s\n" "$name"
        PASS=$((PASS + 1))
    else
        printf "  ${RED}FAIL${NC}  %s\n" "$name"
        FAIL=$((FAIL + 1))
    fi
}

echo "Testing image: $IMAGE_NAME"
echo ""

run_test "Container startup" \
    'source /opt/ros/jazzy/setup.bash'

run_test "Python rclpy import" \
    'source /opt/ros/jazzy/setup.bash && python3 -c "import rclpy"'

run_test "Python std_msgs import" \
    'source /opt/ros/jazzy/setup.bash && python3 -c "from std_msgs.msg import String"'

run_test "Python geometry_msgs import" \
    'source /opt/ros/jazzy/setup.bash && python3 -c "from geometry_msgs.msg import Twist, PoseStamped"'

run_test "Python sensor_msgs import" \
    'source /opt/ros/jazzy/setup.bash && python3 -c "from sensor_msgs.msg import Image, LaserScan, PointCloud2"'

run_test "Python nav_msgs import" \
    'source /opt/ros/jazzy/setup.bash && python3 -c "from nav_msgs.msg import Odometry, Path, OccupancyGrid"'

run_test "rclcpp headers present" \
    'source /opt/ros/jazzy/setup.bash && test -f /opt/ros/jazzy/rclcpp/include/rclcpp/rclcpp/node.hpp'

run_test "Node lifecycle" \
    'source /opt/ros/jazzy/setup.bash && timeout 5 python3 -c "
import rclpy
rclpy.init()
node = rclpy.create_node(\"test\")
node.get_logger().info(\"ok\")
rclpy.shutdown()"'

run_test "Message creation" \
    'source /opt/ros/jazzy/setup.bash && python3 -c "
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
msg = String(data=\"hello\")
twist = Twist()
t = Time(sec=1, nanosec=0)"'

run_test "ROS 2 CLI" \
    'source /opt/ros/jazzy/setup.bash && ros2 --help'

# shellcheck disable=SC2016 -- $(whoami) must expand inside the container, not on the host
run_test "Non-root user" \
    'test "$(whoami)" = "ros"'

TOTAL=$((PASS + FAIL))
echo ""
echo "Results: $PASS/$TOTAL passed"

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
