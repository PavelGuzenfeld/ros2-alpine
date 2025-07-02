#!/bin/bash

# Quick fix script for ROS 2 Alpine Pipeline
# Fixes the file naming and compatibility issues

set -e

echo "🔧 Fixing ROS 2 Alpine Pipeline Issues..."
echo "========================================"

# Check if we're in the right directory
if [ ! -f "pipline.sh" ] && [ ! -f "pipeline.sh" ]; then
    echo "❌ Neither pipline.sh nor pipeline.sh found in current directory"
    echo "   Please run this script from your ROS 2 Alpine project directory"
    exit 1
fi

# 1. Fix the typo in pipeline script name
if [ -f "pipline.sh" ] && [ ! -f "pipeline.sh" ]; then
    echo "📝 Renaming pipline.sh to pipeline.sh (fixing typo)"
    mv pipline.sh pipeline.sh
    echo "✅ Fixed pipeline script name"
fi

# 2. Update pipeline.sh to use correct Dockerfile name
if [ -f "pipeline.sh" ]; then
    echo "📝 Updating pipeline.sh to use Dockerfile.ros2-core"
    sed -i.bak 's/Dockerfile\.ros2-jazzy-alpine/Dockerfile.ros2-core/g' pipeline.sh
    echo "✅ Updated Dockerfile reference in pipeline.sh"
fi

# 3. Update test.sh to use correct image name
if [ -f "test.sh" ]; then
    echo "📝 Updating test.sh to use correct image tag"
    sed -i.bak 's/ros2-jazzy-alpine:latest/ros2-jazzy-alpine:core/g' test.sh
    echo "✅ Updated image name in test.sh"
fi

# 4. Make scripts executable
echo "📝 Making scripts executable"
chmod +x *.sh
echo "✅ Scripts are now executable"

# 5. Verify fixes
echo ""
echo "🔍 Verifying fixes..."

# Check Dockerfile exists
if [ -f "Dockerfile.ros2-core" ]; then
    echo "✅ Dockerfile.ros2-core found"
else
    echo "❌ Dockerfile.ros2-core not found"
fi

# Check updated files
if grep -q "Dockerfile.ros2-core" pipeline.sh 2>/dev/null; then
    echo "✅ pipeline.sh uses correct Dockerfile name"
else
    echo "❌ pipeline.sh still has incorrect Dockerfile name"
fi

if grep -q "ros2-jazzy-alpine:core" test.sh 2>/dev/null; then
    echo "✅ test.sh uses correct image tag"
else
    echo "❌ test.sh still has incorrect image tag"
fi

echo ""
echo "🎉 Pipeline fixes completed!"
echo ""
echo "Now you can run:"
echo "  ./pipeline.sh              # Full build and test"
echo "  ./pipeline.sh --build-only # Build only"
echo "  ./pipeline.sh --help       # Show all options"
echo ""
echo "Note: Backup files (.bak) were created for modified scripts"