#!/bin/bash
# Quick test checklist for stereo_sync_node
# Run this inside the Docker container after building

clear
echo "╔════════════════════════════════════════════════════════════╗"
echo "║     STEREO SYNC NODE - QUICK TEST CHECKLIST              ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

PASSED=0
FAILED=0

check_test() {
    local test_name="$1"
    local test_cmd="$2"
    local expected_pattern="$3"
    
    echo -n "⏳ $test_name... "
    
    if eval "$test_cmd" 2>&1 | grep -q "$expected_pattern"; then
        echo "✅ PASS"
        ((PASSED++))
        return 0
    else
        echo "❌ FAIL"
        ((FAILED++))
        return 1
    fi
}

# Prerequisites
echo "📋 PRE-TEST CHECKLIST"
echo "====================="
echo ""
echo "Before running this script, ensure:"
echo "  □ You're inside Docker container (make shell)"
echo "  □ Workspace is built (make build-ws)"
echo "  □ Simulation is running (make sim in another terminal)"
echo ""
read -p "Press ENTER when ready..."
echo ""

# Source workspace
source /workspace/interceptor_ws/install/setup.bash 2>/dev/null || {
    echo "❌ ERROR: Could not source workspace"
    echo "   Run: source install/setup.bash"
    exit 1
}

echo "🧪 RUNNING TESTS"
echo "================"
echo ""

# Test 1: Message type exists
check_test \
    "Message type registered" \
    "ros2 interface show interceptor_interfaces/msg/StereoImagePair" \
    "left_image"

# Test 2: Node executable exists
check_test \
    "Node executable exists" \
    "ls /workspace/interceptor_ws/install/interceptor_drone/lib/interceptor_drone/stereo_sync_node" \
    "stereo_sync_node"

# Test 3: Camera topics present
echo -n "⏳ Waiting for camera topics (up to 30s)... "
for i in {1..30}; do
    if ros2 topic list | grep -q "/stereo/left/image_raw" && \
       ros2 topic list | grep -q "/stereo/right/image_raw"; then
        echo "✅ PASS"
        ((PASSED++))
        ((PASSED++)) # Count both left and right
        TOPICS_FOUND=1
        break
    fi
    echo -n "."
    sleep 1
done

if [ -z "$TOPICS_FOUND" ]; then
    echo "❌ FAIL (Timeout)"
    ((FAILED++))
    ((FAILED++))
fi

# Test 4: Start perception pipeline and check output
echo ""
echo "🚀 Starting perception pipeline (sync + depth)..."
ros2 launch interceptor_drone perception.launch.py &
PERCEPTION_PID=$!
sleep 5

check_test \
    "Sync node is running" \
    "ros2 node list" \
    "stereo_sync_node"

check_test \
    "Depth node is running" \
    "ros2 node list" \
    "stereo_depth_processor"

check_test \
    "Depth topic exists" \
    "ros2 topic list" \
    "/stereo/depth"

# Test 5: Measure rate
echo ""
echo "⏱️  Measuring depth output rate (5 seconds)..."
timeout 6 ros2 topic hz /stereo/depth 2>&1 | grep -q "average rate"
if [ $? -eq 0 ]; then
    RATE=$(timeout 6 ros2 topic hz /stereo/depth 2>&1 | grep "average rate" | awk '{print $4}')
    echo "📊 Measured rate: ${RATE} Hz"
    
    if (( $(echo "$RATE > 1.0" | bc -l) )); then
        echo "✅ PASS: Rate > 1 Hz"
        ((PASSED++))
    else
        echo "❌ FAIL: Rate too low"
        ((FAILED++))
    fi
else
    echo "❌ FAIL: Could not measure rate"
    ((FAILED++))
fi

# Cleanup
kill $PERCEPTION_PID 2>/dev/null || true
pkill -f stereo_sync_node
pkill -f stereo_depth_processor
sleep 1

# Results
echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                      TEST RESULTS                          ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "✅ PASSED: $PASSED"
echo "❌ FAILED: $FAILED"
echo ""

if [ $FAILED -eq 0 ]; then
    echo "🎉 ALL TESTS PASSED!"
    echo ""
    echo "You can proceed to Phase 2.2: stereo_depth_processor"
    echo ""
    echo "To continue:"
    echo "  1. Run full integration test: ./scripts/integration_test_stereo.sh"
    echo "  2. See detailed docs: cat docs/TESTING_PROCEDURE.md"
    exit 0
else
    echo "⚠️  SOME TESTS FAILED"
    echo ""
    echo "Please check:"
    echo "  • Is Gazebo running? (ros2 launch interceptor_drone simulation.launch.py)"
    echo "  • Is workspace built? (colcon build)"
    echo "  • See troubleshooting: cat docs/TESTING_PROCEDURE.md"
    exit 1
fi
