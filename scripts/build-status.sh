#!/bin/bash
# Check Docker build status

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_FILE="$PROJECT_DIR/logs/docker-build.log"

echo "Docker Build Status"
echo "=================="
echo ""

mkdir -p "$PROJECT_DIR/logs"

# Check if build is running
if pgrep -f "docker-compose build interceptor-dev" > /dev/null; then
    echo "✅ Build is RUNNING"
    echo ""
    echo "Recent progress:"
    tail -15 "$LOG_FILE" 2>/dev/null || echo "(Log not yet available)"
else
    echo "⏹️  Build is NOT running"
    echo ""
    # Check if image was created
    if docker images | grep -q "interceptor-drone"; then
        echo "✅ Docker image exists:"
        docker images | grep interceptor-drone
        echo ""
        echo "You can now run: make up"
    else
        echo "❌ No image found. Build may have failed."
        echo ""
        echo "Check the full log:"
        tail -50 "$LOG_FILE" 2>/dev/null || echo "(No log file)"
    fi
fi
