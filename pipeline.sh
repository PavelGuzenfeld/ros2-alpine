#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS2_IMAGE="${ROS2_IMAGE:-ros2-jazzy-alpine:latest}"

BLUE='\033[0;34m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

info()    { printf "${BLUE}[INFO]${NC} %s\n" "$1"; }
success() { printf "${GREEN}[OK]${NC}   %s\n" "$1"; }
error()   { printf "${RED}[ERR]${NC}  %s\n" "$1"; }

show_help() {
    cat <<EOF
Usage: $0 [OPTIONS]

ROS 2 Jazzy Alpine build pipeline.

Options:
  --build-only   Build without testing
  --test-only    Test existing image (skip build)
  --clean        Clean cache and rebuild
  --info         Show system information
  --help         Show this help

Environment:
  ROS2_IMAGE     Image name (default: ros2-jazzy-alpine:latest)
EOF
}

case "${1:-}" in
    --help|-h)
        show_help
        ;;
    --info)
        "$SCRIPT_DIR/build.sh" --info
        ;;
    --build-only)
        "$SCRIPT_DIR/build.sh"
        ;;
    --test-only)
        "$SCRIPT_DIR/test.sh" "$ROS2_IMAGE"
        ;;
    --clean)
        "$SCRIPT_DIR/build.sh" --clean
        "$SCRIPT_DIR/test.sh" "$ROS2_IMAGE"
        ;;
    *)
        info "Starting pipeline: build + test"
        start_time=$(date +%s)

        "$SCRIPT_DIR/build.sh"
        "$SCRIPT_DIR/test.sh" "$ROS2_IMAGE"

        duration=$(( $(date +%s) - start_time ))
        hours=$((duration / 3600))
        mins=$(((duration % 3600) / 60))
        success "Pipeline completed in ${hours}h ${mins}m"
        ;;
esac
