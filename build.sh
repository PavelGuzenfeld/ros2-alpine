#!/bin/bash

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Configuration
IMAGE_NAME="ros2-jazzy-alpine:core"
DOCKERFILE="Dockerfile.ros2-core"

# Logging functions
log_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

log_header() {
    echo -e "${PURPLE}🚀 $1${NC}"
    echo "============================================="
}

# Help function
show_help() {
    echo "ROS 2 Jazzy Alpine Build Script"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "OPTIONS:"
    echo "  --help           Show this help message"
    echo "  --clean          Clean all Docker cache and rebuild from scratch"
    echo "  --force          Force rebuild without cache"
    echo "  --no-cache       Build without using Docker cache"
    echo "  --test-only      Skip build, only run tests on existing image"
    echo "  --build-only     Build without running tests"
    echo "  --fix-buildkit   Disable BuildKit (for systems without buildx)"
    echo "  --info           Show system information"
    echo "  --status         Check current build status"
    echo ""
    echo "EXAMPLES:"
    echo "  $0                    # Normal build with cache"
    echo "  $0 --clean            # Clean everything and rebuild"
    echo "  $0 --force            # Force rebuild without cache"
    echo "  $0 --test-only        # Test existing image"
    echo "  $0 --build-only       # Build but skip tests"
    echo "  $0 --fix-buildkit     # Build with legacy Docker (no BuildKit)"
    echo ""
    echo "TROUBLESHOOTING:"
    echo "  If build fails with BuildKit errors: use --fix-buildkit"
    echo "  If packages are missing: use --clean to rebuild completely"
    echo "  If build seems stuck: use --no-cache to avoid cache issues"
}

# System info function
show_system_info() {
    log_header "System Information"
    echo "  OS: $(uname -s) $(uname -r)"
    echo "  Architecture: $(uname -m)"
    echo "  Docker version: $(docker --version)"
    
    # Check Docker BuildKit support
    if docker buildx version >/dev/null 2>&1; then
        echo "  BuildKit support: ✅ Available"
    else
        echo "  BuildKit support: ❌ Missing (will use legacy builder)"
    fi
    
    # Memory and disk
    AVAILABLE_MEMORY=$(free -m | awk 'NR==2{printf "%.1f", $7/1024}' 2>/dev/null || echo "Unknown")
    AVAILABLE_DISK=$(df -BG . | awk 'NR==2{print $4}' | sed 's/G//' 2>/dev/null || echo "Unknown")
    echo "  Available memory: ${AVAILABLE_MEMORY}GB"
    echo "  Available disk: ${AVAILABLE_DISK}GB"
    
    # Docker images
    echo ""
    echo "Existing ROS 2 images:"
    docker images | grep -E "(ros2|jazzy|alpine)" || echo "  No ROS 2 images found"
}

# Status check function
check_build_status() {
    log_header "Build Status Check"
    
    # Check if image exists
    if docker images "$IMAGE_NAME" --format "{{.Repository}}:{{.Tag}}" | grep -q "$IMAGE_NAME"; then
        IMAGE_SIZE=$(docker images "$IMAGE_NAME" --format "{{.Size}}")
        IMAGE_DATE=$(docker images "$IMAGE_NAME" --format "{{.CreatedAt}}")
        log_success "Image exists: $IMAGE_NAME ($IMAGE_SIZE)"
        echo "  Created: $IMAGE_DATE"
        
        # Quick functionality test
        echo ""
        log_info "Testing basic functionality..."
        if docker run --rm "$IMAGE_NAME" bash -c 'source /opt/ros/jazzy/setup.bash && python3 -c "import rclpy; print(\"rclpy works\")" 2>/dev/null'; then
            log_success "rclpy import: Working"
        else
            log_warning "rclpy import: Failed (needs rebuild)"
        fi
        
        if docker run --rm "$IMAGE_NAME" bash -c 'source /opt/ros/jazzy/setup.bash && command -v ros2 >/dev/null && echo "ros2 CLI works" || echo "ros2 CLI missing"' 2>/dev/null; then
            log_success "ros2 CLI: Available"
        else
            log_warning "ros2 CLI: Missing"
        fi
    else
        log_warning "Image not found: $IMAGE_NAME"
        echo "  Run '$0' to build the image"
    fi
}

# Clean function
clean_docker() {
    log_info "Cleaning Docker cache and images..."
    
    # Remove existing ROS 2 images
    log_info "Removing existing ROS 2 Alpine images..."
    docker images | grep -E "ros2.*alpine" | awk '{print $3}' | xargs -r docker rmi -f || true
    
    # Clean build cache
    log_info "Cleaning Docker build cache..."
    docker builder prune -f || true
    docker system prune -f || true
    
    # Clean dangling images
    log_info "Removing dangling images..."
    docker image prune -f || true
    
    log_success "Docker cleanup completed"
}

# Build function
run_build() {
    local use_cache="$1"
    local use_buildkit="$2"
    local test_after="$3"
    
    log_header "Building ROS 2 Jazzy Alpine Container"
    
    # Check if Dockerfile exists
    if [ ! -f "$DOCKERFILE" ]; then
        log_error "$DOCKERFILE not found! Please ensure it exists in the current directory."
        return 1
    fi
    
    log_info "Using Dockerfile: $DOCKERFILE"
    
    # Check system resources
    log_info "Checking system resources..."
    AVAILABLE_MEMORY=$(free -m | awk 'NR==2{printf "%.1f", $7/1024}' 2>/dev/null || echo "0")
    AVAILABLE_DISK=$(df -BG . | awk 'NR==2{print $4}' | sed 's/G//' 2>/dev/null || echo "0")
    
    echo "  Available memory: ${AVAILABLE_MEMORY}GB"
    echo "  Available disk: ${AVAILABLE_DISK}GB"
    
    if (( $(echo "$AVAILABLE_MEMORY < 2.0" | bc -l 2>/dev/null || echo "0") )); then
        log_warning "Low memory (${AVAILABLE_MEMORY}GB). Build may fail or be very slow."
    fi
    
    if (( AVAILABLE_DISK < 10 )); then
        log_warning "Low disk space (${AVAILABLE_DISK}GB). Build may fail."
    fi
    
    # Configure Docker build
    local build_args=""
    
    if [ "$use_buildkit" = "false" ]; then
        log_info "Using legacy Docker builder (BuildKit disabled)"
        export DOCKER_BUILDKIT=0
    else
        log_info "Attempting to use BuildKit..."
        export DOCKER_BUILDKIT=1
    fi
    
    if [ "$use_cache" = "false" ]; then
        build_args="--no-cache"
        log_info "Building without cache"
    else
        log_info "Building with cache (faster but may miss updates)"
    fi
    
    # Create build log
    BUILD_LOG="/tmp/ros2_build_$(date +%s).log"
    log_info "Build log: $BUILD_LOG"
    log_info "Started at: $(date)"
    log_info "Expected duration: 1-3 hours"
    
    echo ""
    log_info "🔨 Starting Docker build..."
    
    # Run the build
    if docker build \
        $build_args \
        -f "$DOCKERFILE" \
        -t "$IMAGE_NAME" \
        -t "ros2-jazzy-alpine:complete" \
        . 2>&1 | tee "$BUILD_LOG"; then
        
        BUILD_SUCCESS=true
    else
        BUILD_SUCCESS=false
    fi
    
    # Analyze build results
    if [ "$BUILD_SUCCESS" = true ]; then
        echo ""
        log_success "Build completed at: $(date)"
        
        # Get image size
        if docker images "$IMAGE_NAME" --format "{{.Size}}" >/dev/null 2>&1; then
            IMAGE_SIZE=$(docker images "$IMAGE_NAME" --format "{{.Size}}")
            log_info "Final image size: $IMAGE_SIZE"
        fi
        
        # Analyze build log
        WARNING_COUNT=$(grep -c "⚠️\|WARNING\|WARN" "$BUILD_LOG" 2>/dev/null || echo "0")
        ERROR_COUNT=$(grep -c "❌\|ERROR\|FAILED" "$BUILD_LOG" 2>/dev/null || echo "0")
        
        echo ""
        log_info "Build analysis:"
        echo "  Warnings: $WARNING_COUNT"
        echo "  Errors: $ERROR_COUNT"
        
        if [ "$ERROR_COUNT" -gt 0 ]; then
            log_warning "Build completed but with errors. Recent errors:"
            grep -i "error\|failed" "$BUILD_LOG" | tail -3 || echo "  No specific error patterns found"
        fi
        
        # Run tests if requested
        if [ "$test_after" = "true" ]; then
            echo ""
            run_tests
        fi
        
        return 0
    else
        echo ""
        log_error "Build failed! Check logs above and $BUILD_LOG"
        
        # Analyze failure
        if grep -q "BuildKit.*missing\|buildx.*missing" "$BUILD_LOG"; then
            log_error "BuildKit/buildx issue detected"
            echo ""
            log_info "💡 Try rebuilding with: $0 --fix-buildkit"
        elif grep -q "No space left" "$BUILD_LOG"; then
            log_error "Insufficient disk space"
            echo ""
            log_info "💡 Try cleaning with: $0 --clean"
        elif grep -q "Cannot allocate memory" "$BUILD_LOG"; then
            log_error "Insufficient memory"
        else
            log_error "Unknown build failure"
            echo "Last 5 lines of build log:"
            tail -5 "$BUILD_LOG"
        fi
        
        return 1
    fi
}

# Test function
run_tests() {
    log_header "Testing ROS 2 Container"
    
    # Check if image exists
    if ! docker images "$IMAGE_NAME" --format "{{.Repository}}:{{.Tag}}" | grep -q "$IMAGE_NAME"; then
        log_error "Image not found: $IMAGE_NAME"
        log_info "Run '$0' to build the image first"
        return 1
    fi
    
    local tests_passed=0
    local tests_total=5
    
    # Test helper function
    run_test() {
        local test_name="$1"
        local test_command="$2"
        
        log_info "Running: $test_name"
        
        if docker run --rm "$IMAGE_NAME" bash -c "$test_command" >/dev/null 2>&1; then
            log_success "$test_name"
            return 0
        else
            log_warning "$test_name failed"
            # Show detailed output for debugging
            echo "Detailed output:"
            docker run --rm "$IMAGE_NAME" bash -c "$test_command" 2>&1 | head -10
            return 1
        fi
    }
    
    # Run tests
    echo ""
    
    # Test 1: Container startup
    if run_test "Container Startup" 'source /opt/ros/jazzy/setup.bash && echo "OK"'; then
        ((tests_passed++))
    fi
    
    # Test 2: Python packages
    if run_test "Python Package Import" 'source /opt/ros/jazzy/setup.bash && python3 -c "import rclpy; import std_msgs.msg; print(\"OK\")"'; then
        ((tests_passed++))
    fi
    
    # Test 3: Node creation
    if run_test "ROS 2 Node Creation" 'source /opt/ros/jazzy/setup.bash && timeout 5 python3 -c "
import rclpy
from rclpy.node import Node
rclpy.init()
node = Node(\"test\")
rclpy.shutdown()
print(\"OK\")"'; then
        ((tests_passed++))
    fi
    
    # Test 4: Message creation
    if run_test "Message Creation" 'source /opt/ros/jazzy/setup.bash && python3 -c "
from std_msgs.msg import String
from builtin_interfaces.msg import Time
msg = String()
msg.data = \"test\"
time_msg = Time()
print(\"OK\")"'; then
        ((tests_passed++))
    fi
    
    # Test 5: CLI tools
    if run_test "ROS 2 CLI Tools" 'source /opt/ros/jazzy/setup.bash && command -v ros2 && ros2 --help >/dev/null'; then
        ((tests_passed++))
    fi
    
    # Test summary
    echo ""
    log_header "Test Results"
    echo "Tests passed: $tests_passed/$tests_total"
    
    if [ "$tests_passed" -eq "$tests_total" ]; then
        log_success "🎉 All tests passed! ROS 2 container is fully functional."
    elif [ "$tests_passed" -ge 3 ]; then
        log_warning "⚠️ Core functionality works, but some features are missing."
        log_info "Container is usable for basic ROS 2 development."
    else
        log_error "💥 Critical tests failed! Container needs rebuilding."
        log_info "Try: $0 --clean"
    fi
    
    return 0
}

# Main execution
main() {
    case "${1:-}" in
        --help|-h)
            show_help
            ;;
        --info)
            show_system_info
            ;;
        --status)
            check_build_status
            ;;
        --clean)
            clean_docker
            log_info "Now run '$0' to rebuild from scratch"
            ;;
        --force)
            run_build false true true
            ;;
        --no-cache)
            run_build false true true
            ;;
        --test-only)
            run_tests
            ;;
        --build-only)
            run_build true true false
            ;;
        --fix-buildkit)
            log_info "Building with legacy Docker (BuildKit disabled)"
            run_build true false true
            ;;
        "")
            # Default build
            run_build true true true
            ;;
        *)
            log_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
}

# Execute main function
main "$@"