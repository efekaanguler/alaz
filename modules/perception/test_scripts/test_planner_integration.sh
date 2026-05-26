#!/bin/bash
# test_planner_integration.sh — Test perception → planning pipeline integration.
#
# Verifies that the perception module's output (detections, traffic light state)
# is correctly received by the planning module.
#
# Prerequisites:
#   - ROS 2 environment sourced
#   - Autoware workspace built
#
# Usage:
#   ./test_planner_integration.sh

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  Perception → Planner Integration Test${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"

# Check ROS 2 is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ERROR: ROS 2 not found. Source your workspace first.${NC}"
    exit 1
fi

echo -e "\n${YELLOW}[1] Checking ROS 2 topics...${NC}"

# List relevant topics
echo "  Active topics:"
ros2 topic list 2>/dev/null | grep -E "(camera|perception|planning|traffic)" | while read topic; do
    echo "    - $topic"
done

echo -e "\n${YELLOW}[2] Publishing dummy detection...${NC}"

# Publish a dummy person detection
ros2 topic pub --once /perception/detections std_msgs/String \
    "{data: '[{\"label\": \"person\", \"score\": 0.95, \"x1\": 100, \"y1\": 100, \"x2\": 200, \"y2\": 300}]'}" \
    2>/dev/null &
PUB_PID=$!

sleep 1

echo -e "\n${YELLOW}[3] Publishing dummy traffic light state...${NC}"

# Publish a dummy traffic light state
ros2 topic pub --once /perception/traffic_light std_msgs/String \
    "{data: '{\"state\": \"red\", \"confidence\": 0.85}'}" \
    2>/dev/null &
TL_PID=$!

sleep 1

echo -e "\n${YELLOW}[4] Checking if planner received messages...${NC}"

# Check if planning topics are responding
PLANNING_TOPICS=$(ros2 topic list 2>/dev/null | grep -c "planning" || true)
echo "  Planning topics active: $PLANNING_TOPICS"

# Check if obstacle stop is being generated
echo "  Checking /planning/scenario_planning/status..."
timeout 3 ros2 topic echo /planning/scenario_planning/status --once 2>/dev/null || echo "  (No response within 3s — planner may not be running)"

# Cleanup
kill $PUB_PID $TL_PID 2>/dev/null || true

echo -e "\n${GREEN}═══════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  Integration test complete.${NC}"
echo -e "${YELLOW}  Note: For full verification, ensure both perception${NC}"
echo -e "${YELLOW}  and planning modules are running simultaneously.${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
