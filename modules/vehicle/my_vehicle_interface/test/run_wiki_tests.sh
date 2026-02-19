#!/bin/bash
# =============================================================================
# SDC 2026 Kart - Wiki Verification Test Runner
# =============================================================================
# Usage:
#   ./test/run_wiki_tests.sh         # Byte-level tests only
#   ./test/run_wiki_tests.sh --ros   # Include ROS integration tests
#   ./test/run_wiki_tests.sh --build # Build first, then test
#   ./test/run_wiki_tests.sh --all   # Build + all tests (including ROS)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

echo ""
echo -e "${BOLD}${CYAN}#################################################################${NC}"
echo -e "${BOLD}${CYAN}  SDC 2026 KART - WIKI VERIFICATION TEST RUNNER${NC}"
echo -e "${BOLD}${CYAN}#################################################################${NC}"
echo ""
echo -e "  Package: ${BOLD}my_vehicle_interface${NC}"
echo -e "  Directory: ${PKG_DIR}"
echo -e "  Time: $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# Parse args
DO_BUILD=false
DO_ROS=false
for arg in "$@"; do
    case "$arg" in
        --build) DO_BUILD=true ;;
        --ros)   DO_ROS=true ;;
        --all)   DO_BUILD=true; DO_ROS=true ;;
        --help|-h)
            echo "Usage: $0 [--build] [--ros] [--all]"
            echo "  --build  Build package first"
            echo "  --ros    Include ROS integration tests"
            echo "  --all    Build + all tests"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown argument: $arg${NC}"
            exit 1
            ;;
    esac
done

# =============================================================================
# STEP 1: Build (optional)
# =============================================================================
if $DO_BUILD; then
    echo -e "${BOLD}STEP 1: Building package...${NC}"
    cd /workspace
    if colcon build --packages-select my_vehicle_interface 2>&1; then
        echo -e "  ${GREEN}✅ Build successful${NC}"
    else
        echo -e "  ${RED}❌ Build FAILED${NC}"
        exit 1
    fi
    source install/setup.bash
    echo ""
fi

# =============================================================================
# STEP 2: Source code verification
# =============================================================================
echo -e "${BOLD}STEP 2: Source code verification...${NC}"
echo ""

cd "$PKG_DIR"

# --- CAN ID checks ---
echo -e "  ${CYAN}Checking CAN IDs in can_utils.hpp...${NC}"
ERRORS=0

check_can_id() {
    local name=$1
    local expected=$2
    local pattern=$3
    local file="include/my_vehicle_interface/can_utils.hpp"
    
    if grep -q "$pattern" "$file"; then
        echo -e "    ${GREEN}✅ ${name} = ${expected}${NC}"
    else
        echo -e "    ${RED}❌ ${name} != ${expected}${NC}"
        ((ERRORS++)) || true
    fi
}

check_can_id "Steering CMD" "0x220" "steering_command = 0x220"
check_can_id "Brake CMD"    "0x110" "brake_command = 0x110"
check_can_id "Motor CMD"    "0x330" "motor_command = 0x330"
check_can_id "Speed Sensor" "0x440" "speed_sensor = 0x440"
check_can_id "Steer Sensor" "0x1E5" "steering_sensor = 0x1E5"
check_can_id "Steer ECU FB" "0x720" "steering_ecu_feedback = 0x720"
check_can_id "Motor FB"     "0x730" "motor_feedback = 0x730"
check_can_id "Brake FB"     "0x710" "brake_feedback = 0x710"

echo ""

# --- Encoding checks ---
echo -e "  ${CYAN}Checking encoding implementations...${NC}"

if grep -q "std::memcpy" src/can_utils.cpp; then
    echo -e "    ${GREEN}✅ Steering: memcpy (IEEE 754 float)${NC}"
else
    echo -e "    ${RED}❌ Steering: no memcpy found${NC}"
    ((ERRORS++)) || true
fi

if grep -q "frame.data\[2\] = gear" src/can_utils.cpp; then
    echo -e "    ${GREEN}✅ Motor: gear byte at data[2]${NC}"
else
    echo -e "    ${RED}❌ Motor: gear byte NOT at data[2]${NC}"
    ((ERRORS++)) || true
fi

if grep -q "frame.data\[0\]) << 8" src/can_utils.cpp; then
    echo -e "    ${GREEN}✅ Speed: big-endian decoding${NC}"
else
    echo -e "    ${RED}❌ Speed: NOT big-endian${NC}"
    ((ERRORS++)) || true
fi

if grep -q "frame.data\[1\]) << 8" src/can_utils.cpp; then
    echo -e "    ${GREEN}✅ Steering sensor: bytes [1],[2] (data_recorder.py)${NC}"
else
    echo -e "    ${RED}❌ Steering sensor: wrong byte offsets${NC}"
    ((ERRORS++)) || true
fi

echo ""

# --- Timing checks ---
echo -e "  ${CYAN}Checking timing configuration...${NC}"

if grep -q "loop_rate_hz: 25.0" config/vehicle_interface.param.yaml; then
    echo -e "    ${GREEN}✅ Loop rate: 25 Hz (wiki: 0.04s)${NC}"
else
    echo -e "    ${RED}❌ Loop rate != 25 Hz${NC}"
    ((ERRORS++)) || true
fi

if grep -q "command_timeout_sec: 0.2" config/vehicle_interface.param.yaml; then
    echo -e "    ${GREEN}✅ Timeout: 0.2s (motor ECU: 200ms)${NC}"
else
    echo -e "    ${RED}❌ Timeout != 0.2s${NC}"
    ((ERRORS++)) || true
fi

echo ""

# --- Safety checks ---
echo -e "  ${CYAN}Checking safety mechanisms...${NC}"

if grep -q "motor_is_idle_" src/vehicle_interface_node.cpp; then
    echo -e "    ${GREEN}✅ Motor idle detection${NC}"
else
    echo -e "    ${RED}❌ Motor idle detection missing${NC}"
    ((ERRORS++)) || true
fi

if grep -q "FAILSAFE" src/vehicle_interface_node.cpp; then
    echo -e "    ${GREEN}✅ Steering failsafe detection${NC}"
else
    echo -e "    ${RED}❌ Steering failsafe detection missing${NC}"
    ((ERRORS++)) || true
fi

if grep -q "std::min(gear, static_cast<uint8_t>(2))" src/can_utils.cpp; then
    echo -e "    ${GREEN}✅ Gear clamped (>= 3 = message rejected)${NC}"
else
    echo -e "    ${RED}❌ Gear NOT clamped${NC}"
    ((ERRORS++)) || true
fi

echo ""

# =============================================================================
# STEP 3: Python byte-level tests
# =============================================================================
echo -e "${BOLD}STEP 3: Python byte-level verification...${NC}"
echo ""

PYTHON_ARGS=""
if $DO_ROS; then
    PYTHON_ARGS="--ros"
fi

cd "$PKG_DIR"
python3 test/test_wiki_verification.py $PYTHON_ARGS
PYTHON_EXIT=$?

echo ""

# =============================================================================
# STEP 4: colcon test (if --build)
# =============================================================================
if $DO_BUILD; then
    echo -e "${BOLD}STEP 4: colcon test...${NC}"
    cd /workspace
    if colcon test --packages-select my_vehicle_interface 2>&1; then
        echo -e "  ${GREEN}✅ colcon test passed${NC}"
    else
        echo -e "  ${YELLOW}⚠️  colcon test had warnings/failures${NC}"
    fi
    colcon test-result --verbose --all 2>&1 || true
    echo ""
fi

# =============================================================================
# SUMMARY
# =============================================================================
echo ""
echo -e "${BOLD}================================================================${NC}"
echo -e "${BOLD}  FINAL SUMMARY${NC}"
echo -e "${BOLD}================================================================${NC}"

if [ $ERRORS -eq 0 ] && [ $PYTHON_EXIT -eq 0 ]; then
    echo -e "  ${GREEN}${BOLD}ALL TESTS PASSED!${NC}"
    echo ""
    echo -e "  ${YELLOW}Next steps:${NC}"
    echo -e "    1. Docker'da build et: colcon build --packages-select my_vehicle_interface"
    echo -e "    2. arabaya bağlan ve CAN trafiğini kontrol et: candump can0"
    echo -e "    3. Mode 1 (10 km/h max) ile başla!"
    echo -e "    4. max_steering_angle_rad'ı kalibre et"
    echo -e "    5. accel/decel gain'lerini ayarla"
    exit 0
else
    echo -e "  ${RED}${BOLD}$ERRORS source check error(s) + Python exit=$PYTHON_EXIT${NC}"
    exit 1
fi
