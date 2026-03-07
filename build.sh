#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="${ROS2_IMAGE:-ros2-jazzy-alpine:latest}"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

info()    { printf "${BLUE}[INFO]${NC} %s\n" "$1"; }
success() { printf "${GREEN}[OK]${NC}   %s\n" "$1"; }
warn()    { printf "${YELLOW}[WARN]${NC} %s\n" "$1"; }
error()   { printf "${RED}[ERR]${NC}  %s\n" "$1"; }

show_help() {
    cat <<EOF
Usage: $0 [OPTIONS]

Build the ROS 2 Jazzy Alpine Docker image.

Options:
  --no-cache    Build without Docker cache
  --clean       Remove existing images and cache, then build
  --info        Show system information
  --status      Check if image exists and works
  --help        Show this help

Environment:
  ROS2_IMAGE    Image name (default: ros2-jazzy-alpine:latest)
EOF
}

show_info() {
    info "System Information"
    echo "  OS:         $(uname -s) $(uname -r)"
    echo "  Arch:       $(uname -m)"
    echo "  Docker:     $(docker --version 2>/dev/null || echo 'not found')"
    echo "  Memory:     $(free -h 2>/dev/null | awk 'NR==2{print $7}' || echo 'N/A') available"
    echo "  Disk:       $(df -h . 2>/dev/null | awk 'NR==2{print $4}' || echo 'N/A') available"
    echo "  CPU cores:  $(nproc 2>/dev/null || echo 'N/A')"
}

check_status() {
    if docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
        local size
        size=$(docker images "$IMAGE_NAME" --format "{{.Size}}")
        success "Image exists: $IMAGE_NAME ($size)"
    else
        warn "Image not found: $IMAGE_NAME"
    fi
}

build() {
    local -a build_args=()
    if [ -n "${1:-}" ]; then
        build_args+=("$1")
    fi

    if [ ! -f "$DOCKERFILE" ]; then
        error "Dockerfile not found: $DOCKERFILE"
        exit 1
    fi

    info "Building $IMAGE_NAME"
    show_info
    echo ""

    local start_time
    start_time=$(date +%s)

    DOCKER_BUILDKIT=1 docker build \
        "${build_args[@]+"${build_args[@]}"}" \
        -f "$DOCKERFILE" \
        -t "$IMAGE_NAME" \
        "$SCRIPT_DIR"

    local duration=$(( $(date +%s) - start_time ))
    local hours=$((duration / 3600))
    local mins=$(((duration % 3600) / 60))

    echo ""
    success "Build completed in ${hours}h ${mins}m"
    check_status
}

case "${1:-}" in
    --help|-h)
        show_help
        ;;
    --info)
        show_info
        ;;
    --status)
        check_status
        ;;
    --clean)
        info "Cleaning Docker cache..."
        docker images --filter "reference=ros2-jazzy-alpine" -q | xargs -r docker rmi -f 2>/dev/null || true
        docker builder prune -f 2>/dev/null || true
        build --no-cache
        ;;
    --no-cache)
        build --no-cache
        ;;
    *)
        build
        ;;
esac
