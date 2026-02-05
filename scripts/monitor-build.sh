#!/bin/bash
# Monitor Docker build progress

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_FILE="$PROJECT_DIR/logs/docker-build.log"

echo "Monitoring Docker build for interceptor-drone..."
echo ""

mkdir -p "$PROJECT_DIR/logs"

if [ ! -f "$LOG_FILE" ]; then
    echo "No build log found. Build may not have started yet."
    echo "Run: make build"
    exit 1
fi

# Show last 30 lines and follow
echo "Recent build output:"
echo "===================="
tail -30 "$LOG_FILE"
echo ""
echo "Build is still running... Press Ctrl+C to stop monitoring"
echo "(This will NOT stop the build, only this monitor)"
echo ""

# Check if build process is still running
while pgrep -f "docker-compose build interceptor-dev" > /dev/null; do
    sleep 5
    clear
    echo "Docker Build Monitor - $(date)"
    echo "================================"
    echo ""
    tail -40 "$LOG_FILE"
    echo ""
    echo "Status: Building..."
done

echo ""
echo "Build process completed!"
echo ""
echo "Final output:"
tail -50 "$LOG_FILE"

# Check if build succeeded
if docker images | grep -q "interceptor-drone"; then
    echo ""
    echo "✅ Build successful! You can now run: make up"
else
    echo ""
    echo "❌ Build may have failed. Check full log: $LOG_FILE"
fi
