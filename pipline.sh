#!/bin/bash

# ROS 2 Jazzy Alpine Build Pipeline
# Orchestrates the complete build and test process

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_SCRIPT="$SCRIPT_DIR/build.sh"
TEST_SCRIPT="$SCRIPT_DIR/test.sh"
DOCKERFILE="$SCRIPT_DIR/Dockerfile.ros2-jazzy-alpine"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

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

log_stage() {
    echo ""
    echo -e "${BLUE}📋 Stage: $1${NC}"
    echo "============================================="
}

# Check prerequisites
check_prerequisites() {
    log_info "Checking pipeline prerequisites..."
    
    # Check if required files exist
    local missing_files=()
    
    if [ ! -f "$BUILD_SCRIPT" ]; then
        missing_files+=("build.sh")
    fi
    
    if [ ! -f "$TEST_SCRIPT" ]; then
        missing_files+=("test.sh")
    fi
    
    if [ ! -f "$DOCKERFILE" ]; then
        missing_files+=("Dockerfile.ros2-jazzy-alpine")
    fi
    
    if [ ${#missing_files[@]} -gt 0 ]; then
        log_error "Missing required files:"
        for file in "${missing_files[@]}"; do
            echo "  - $file"
        done
        return 1
    fi
    
    # Check if scripts are executable
    if [ ! -x "$BUILD_SCRIPT" ]; then
        log_info "Making build.sh executable..."
        chmod +x "$BUILD_SCRIPT"
    fi
    
    if [ ! -x "$TEST_SCRIPT" ]; then
        log_info "Making test.sh executable..."
        chmod +x "$TEST_SCRIPT"
    fi
    
    # Check Docker
    if ! command -v docker &> /dev/null; then
        log_error "Docker is required but not installed"
        return 1
    fi
    
    if ! docker info &> /dev/null; then
        log_error "Docker daemon is not running"
        return 1
    fi
    
    log_success "Prerequisites check passed"
    return 0
}

# Show system information
show_system_info() {
    log_info "System Information:"
    echo "  OS: $(uname -s) $(uname -r)"
    echo "  Architecture: $(uname -m)"
    echo "  Docker version: $(docker --version | cut -d' ' -f3 | cut -d',' -f1)"
    echo "  Available disk space: $(df -h . | awk 'NR==2 {print $4}')"
    echo "  Available memory: $(free -h | grep Mem | awk '{print $7}' 2>/dev/null || echo 'N/A')"
    echo "  CPU cores: $(nproc 2>/dev/null || echo 'N/A')"
}

# Run build stage
run_build_stage() {
    log_stage "Build ROS 2 Container"
    
    log_info "Starting build process..."
    log_info "Expected build time: 1-3 hours depending on hardware"
    
    if "$BUILD_SCRIPT"; then
        log_success "Build stage completed successfully"
        return 0
    else
        log_error "Build stage failed"
        return 1
    fi
}

# Run test stage
run_test_stage() {
    log_stage "Test ROS 2 Container"
    
    log_info "Starting test suite..."
    
    if "$TEST_SCRIPT"; then
        log_success "Test stage completed successfully"
        return 0
    else
        log_error "Test stage failed"
        return 1
    fi
}

# Clean up function
cleanup() {
    log_info "Cleaning up temporary files..."
    # Add any cleanup logic here if needed
    log_success "Cleanup completed"
}

# Show usage information
show_help() {
    echo "ROS 2 Jazzy Alpine Build Pipeline"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "OPTIONS:"
    echo "  --build-only     Run only the build stage"
    echo "  --test-only      Run only the test stage (requires existing image)"
    echo "  --skip-tests     Build but skip tests"
    echo "  --clean          Clean Docker build cache before building"
    echo "  --interactive    Run tests interactively after build"
    echo "  --info           Show system information and exit"
    echo "  --help           Show this help message"
    echo ""
    echo "EXAMPLES:"
    echo "  $0                    # Run complete pipeline (build + test)"
    echo "  $0 --build-only       # Build container only"
    echo "  $0 --test-only        # Test existing container"
    echo "  $0 --skip-tests       # Build without testing"
    echo "  $0 --clean            # Clean cache and build"
    echo ""
    echo "DESCRIPTION:"
    echo "  This pipeline builds ROS 2 Jazzy on Alpine Linux from source and"
    echo "  runs a comprehensive test suite to verify functionality."
    echo ""
    echo "  The build process includes:"
    echo "  - Multi-stage Docker build"
    echo "  - ROS 2 core packages compilation"
    echo "  - Runtime environment setup"
    echo ""
    echo "  The test suite includes:"
    echo "  - Container startup verification"
    echo "  - Python package imports"
    echo "  - ROS 2 node creation"
    echo "  - CLI tools availability"
    echo "  - Message type functionality"
}

# Main pipeline execution
run_full_pipeline() {
    local skip_tests=${1:-false}
    local interactive_tests=${2:-false}
    
    log_header "ROS 2 Jazzy Alpine Build Pipeline"
    
    local pipeline_start_time=$(date)
    local pipeline_start_seconds=$(date +%s)
    
    log_info "Pipeline started at: $pipeline_start_time"
    
    # Show system info
    show_system_info
    echo ""
    
    # Check prerequisites
    if ! check_prerequisites; then
        log_error "Prerequisites check failed"
        exit 1
    fi
    echo ""
    
    # Run build stage
    if ! run_build_stage; then
        log_error "Pipeline failed at build stage"
        exit 1
    fi
    
    # Run test stage (unless skipped)
    if [ "$skip_tests" = false ]; then
        echo ""
        if [ "$interactive_tests" = true ]; then
            if ! "$TEST_SCRIPT" --interactive; then
                log_warning "Interactive tests encountered issues"
            fi
        else
            if ! run_test_stage; then
                log_warning "Pipeline completed with test failures"
                log_info "Container was built successfully but some tests failed"
            fi
        fi
    else
        log_info "Tests skipped as requested"
    fi
    
    # Pipeline summary
    echo ""
    log_header "Pipeline Summary"
    
    local pipeline_end_seconds=$(date +%s)
    local pipeline_duration=$((pipeline_end_seconds - pipeline_start_seconds))
    local pipeline_duration_hours=$((pipeline_duration / 3600))
    local pipeline_duration_min=$(((pipeline_duration % 3600) / 60))
    local pipeline_duration_sec=$((pipeline_duration % 60))
    
    log_info "Pipeline duration: ${pipeline_duration_hours}h ${pipeline_duration_min}m ${pipeline_duration_sec}s"
    log_info "Started: $pipeline_start_time"
    log_info "Completed: $(date)"
    
    # Final image info
    if docker images ros2-jazzy-alpine:latest --format "table {{.Repository}}:{{.Tag}}" | grep -q "ros2-jazzy-alpine:latest"; then
        IMAGE_SIZE=$(docker images ros2-jazzy-alpine:latest --format "{{.Size}}")
        log_success "🎉 ROS 2 Jazzy Alpine container ready!"
        log_info "Image: ros2-jazzy-alpine:latest ($IMAGE_SIZE)"
        echo ""
        log_info "To run the container:"
        echo "  docker run -it ros2-jazzy-alpine:latest"
        echo ""
        log_info "To run with GUI support:"
        echo "  docker run -it --network host ros2-jazzy-alpine:latest"
    else
        log_error "Final image verification failed"
        exit 1
    fi
}

# Handle command line arguments
main() {
    case "${1:-}" in
        --build-only)
            log_header "Build Only Mode"
            check_prerequisites
            show_system_info
            echo ""
            run_build_stage
            ;;
        --test-only)
            log_header "Test Only Mode"
            check_prerequisites
            echo ""
            run_test_stage
            ;;
        --skip-tests)
            run_full_pipeline true false
            ;;
        --clean)
            log_header "Clean Build Mode"
            "$BUILD_SCRIPT" --clean
            run_full_pipeline false false
            ;;
        --interactive)
            run_full_pipeline false true
            ;;
        --info)
            log_header "System Information"
            show_system_info
            ;;
        --help|-h)
            show_help
            ;;
        "")
            run_full_pipeline false false
            ;;
        *)
            log_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
}

# Trap for cleanup on exit
trap cleanup EXIT

# Execute main function
main "$@"