#!/usr/bin/env bash
# shellcheck disable=SC2086,SC2124

set -e

# Define terminal colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/.."

# Determine ROS distro from environment or default to humble
ros_distro=${ROS_DISTRO:-humble}
if [ "$ros_distro" = "humble" ]; then
    source "$WORKSPACE_ROOT/amd64.env"
else
    source "$WORKSPACE_ROOT/amd64_jazzy.env"
fi
if [ "$(uname -m)" = "aarch64" ]; then
    source "$WORKSPACE_ROOT/arm64.env"
fi

# Default values
option_no_nvidia=false
option_devel=false
option_headless=false
option_pull_latest_image=false
option_perception_bootstrap=""
MAP_PATH=""
DATA_PATH=""
WORKSPACE_PATH=""
USER_ID=""
WORKSPACE=""
DEFAULT_LAUNCH_CMD="ros2 launch autoware_launch autoware.launch.xml map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit"

# Function to print help message
print_help() {
    echo -e "\n------------------------------------------------------------"
    echo -e "${RED}Note:${NC} The --map-path option is mandatory for the runtime. For development environment with shell access, use --devel option."
    echo -e "      Default launch command: ${GREEN}${DEFAULT_LAUNCH_CMD}${NC}"
    echo -e "------------------------------------------------------------"
    echo -e "${RED}Usage:${NC} run.sh [OPTIONS] [LAUNCH_CMD](optional)"
    echo -e "Options:"
    echo -e "  ${GREEN}--help/-h${NC}            Display this help message"
    echo -e "  ${GREEN}--map-path${NC}           Specify to mount map files into /autoware_map (mandatory for runtime)"
    echo -e "  ${GREEN}--data-path${NC}          Specify to mount data files into /autoware_data (mandatory for runtime)"
    echo -e "  ${GREEN}--devel${NC}              Launch the latest Autoware development environment with shell access"
    echo -e "  ${GREEN}--workspace${NC}          (--devel only)Specify the directory to mount into /workspace, by default it uses current directory (pwd)"
    echo -e "  ${GREEN}--no-nvidia${NC}          Disable NVIDIA GPU support"
    echo -e "  ${GREEN}--headless${NC}           Run Autoware in headless mode (default: false)"
    echo -e "  ${GREEN}--pull-latest-image${NC}  Pull the latest image before starting the container"
    echo -e "  ${GREEN}--perception-bootstrap${NC}  (devel only) Auto-install perception runtime deps (vision_msgs + onnxruntime) as root at container start"
    echo -e "  ${GREEN}--no-perception-bootstrap${NC}  Disable the auto perception bootstrap in devel mode"
    echo ""
}

# Parse arguments
parse_arguments() {
    while [ "$1" != "" ]; do
        case "$1" in
        --help | -h)
            print_help
            exit 1
            ;;
        --no-nvidia)
            option_no_nvidia=true
            ;;
        --devel)
            option_devel=true
            ;;
        --headless)
            option_headless=true
            ;;
        --pull-latest-image)
            option_pull_latest_image=true
            ;;
        --perception-bootstrap)
            option_perception_bootstrap=true
            ;;
        --no-perception-bootstrap)
            option_perception_bootstrap=false
            ;;
        --workspace)
            WORKSPACE_PATH="$2"
            shift
            ;;
        --map-path)
            MAP_PATH="$2"
            shift
            ;;
        --data-path)
            DATA_PATH="$2"
            shift
            ;;
        --*)
            echo "Unknown option: $1"
            print_help
            exit 1
            ;;
        -*)
            echo "Unknown option: $1"
            print_help
            exit 1
            ;;
        *)
            LAUNCH_CMD="$@"
            break
            ;;
        esac
        shift
    done
}

# Set the docker image and workspace variables
set_variables() {
    # Set user ID and group ID to match the local user
    USER_ID="-e LOCAL_UID=$(id -u) -e LOCAL_GID=$(id -g) -e LOCAL_USER=$(id -un) -e LOCAL_GROUP=$(id -gn)"

    # Set map path
    if [ "$MAP_PATH" != "" ]; then
        MAP="-v ${MAP_PATH}:/autoware_map:ro"
    fi

    # Set data path
    if [ "$DATA_PATH" != "" ]; then
        DATA="-v ${DATA_PATH}:/autoware_data:rw"
    fi

    if [ "$option_devel" = "true" ]; then
        # Set image based on option
        IMAGE="ghcr.io/autowarefoundation/autoware:universe-devel"

        # Set workspace path, if not provided use the current directory
        if [ "$WORKSPACE_PATH" = "" ]; then
            WORKSPACE_PATH=$(pwd)
        fi
        WORKSPACE="-v ${WORKSPACE_PATH}:/workspace"

        # Set launch command
        if [ "$LAUNCH_CMD" = "" ]; then
            LAUNCH_CMD="/bin/bash"
        fi

        # Auto-enable perception bootstrap when the mounted workspace looks like this project.
        if [ "$option_perception_bootstrap" = "" ]; then
            if [ -d "${WORKSPACE_PATH}/modules/perception" ]; then
                option_perception_bootstrap=true
            else
                option_perception_bootstrap=false
            fi
        fi
    else
        # Set image based on option
        IMAGE="ghcr.io/autowarefoundation/autoware:universe"

        # Set map path
        if [ "$MAP_PATH" = "" ] || [ "$DATA_PATH" = "" ]; then
            echo -e "\n------------------------------------------------------------"
            echo -e "${RED}Note:${NC} The --map-path and --data-path option is mandatory for the universe(runtime image). For development environment with shell access, use --devel option."
            echo -e "------------------------------------------------------------"
            exit 1
        fi

        # Set default launch command if not provided
        if [ "$LAUNCH_CMD" = "" ]; then
            LAUNCH_CMD=${DEFAULT_LAUNCH_CMD}
        fi
    fi
}

# Set GPU flag based on option
set_gpu_flag() {
    if [ "$option_no_nvidia" = "true" ]; then
        GPU_FLAG=""
    else
        GPU_FLAG="--gpus all"
        IMAGE=${IMAGE}-cuda
    fi
}

# Set X display variables
set_x_display() {
    MOUNT_X=""
    if [ "$option_headless" = "false" ]; then
        MOUNT_X="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix"
        xhost + >/dev/null
    fi
}

bootstrap_perception_deps() {
    local container_name="$1"
    local bootstrap_rc

    echo -e "${BLUE}[bootstrap]${NC} Preparing perception runtime deps inside container (root)..."
    set +e
    docker exec -u 0 "${container_name}" bash -lc "
set -e
export DEBIAN_FRONTEND=noninteractive
mkdir -p /var/lib/apt/lists/partial

need_apt=0
python3 -c 'import vision_msgs.msg' >/dev/null 2>&1 || need_apt=1
if ! python3 -m pip --version >/dev/null 2>&1; then
  need_apt=1
fi

if [ \"\$need_apt\" -eq 1 ]; then
  apt-get update
  apt-get install -y --no-install-recommends python3-pip ros-${ros_distro}-vision-msgs
fi

python3 -c 'import vision_msgs.msg' >/dev/null 2>&1 || true

need_python_fix=0
python3 -c 'import numpy, cv2, onnxruntime, sys; sys.exit(0 if int(str(numpy.__version__).split(\".\")[0]) < 2 else 1)' >/dev/null 2>&1 || need_python_fix=1
if [ \"\$need_python_fix\" -eq 1 ]; then
  python3 -m pip install --no-cache-dir --upgrade --force-reinstall \"numpy<2\" onnxruntime
fi
" >/dev/null
    bootstrap_rc=$?
    set -e

    if [ $bootstrap_rc -eq 0 ]; then
        echo -e "${GREEN}[bootstrap]${NC} perception deps ready (vision_msgs + onnxruntime + numpy<2 pin)"
    else
        echo -e "${RED}[bootstrap]${NC} perception bootstrap failed (continuing anyway)."
        echo -e "${BLUE}[bootstrap]${NC} You can retry manually with:"
        echo "  docker exec -u 0 -it ${container_name} bash"
    fi
}

launch_devel_with_bootstrap() {
    local tz_value container_name shell_uid shell_gid docker_run_cmd docker_exec_cmd
    tz_value="$(cat /etc/timezone 2>/dev/null || true)"
    container_name="autoware-devel-$(id -un)-$$"
    shell_uid="$(id -u)"
    shell_gid="$(id -g)"

    echo -e "${BLUE}[bootstrap]${NC} Starting container in background: ${container_name}"
    set -x
    docker run -d --rm --name "${container_name}" --net=host ${GPU_FLAG} ${USER_ID} ${MOUNT_X} \
        -e XAUTHORITY=${XAUTHORITY} -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e NVIDIA_DRIVER_CAPABILITIES=all -e TZ="${tz_value}" \
        ${WORKSPACE} ${MAP} ${DATA} ${IMAGE} \
        tail -f /dev/null
    set +x

    cleanup_bg_container() {
        docker rm -f "${container_name}" >/dev/null 2>&1 || true
    }
    trap cleanup_bg_container EXIT INT TERM

    bootstrap_perception_deps "${container_name}"

    echo -e "${BLUE}[bootstrap]${NC} Opening shell/command in running container..."
    if [ "$option_devel" = "true" ] && [ "$LAUNCH_CMD" = "/bin/bash" ]; then
        docker exec -it -u "${shell_uid}:${shell_gid}" -w /workspace "${container_name}" /bin/bash
    else
        docker exec -it -u "${shell_uid}:${shell_gid}" -w /workspace "${container_name}" bash -lc "${LAUNCH_CMD}"
    fi
}

# Main script execution
main() {
    # Parse arguments
    parse_arguments "$@"
    set_variables
    set_gpu_flag
    set_x_display

    if [ "$option_devel" = "true" ]; then
        echo -e "${GREEN}-----------------------------------------------------------------${NC}"
        echo -e "${BLUE}Launching Autoware development environment${NC}"
    else
        echo -e "${GREEN}-----------------------------------------------------------------${NC}"
        echo -e "${GREEN}Launching Autoware${NC}"
    fi
    echo -e "${GREEN}IMAGE:${NC} ${IMAGE}"
    if [ "$option_devel" = "true" ]; then
        echo -e "${GREEN}WORKSPACE PATH(mounted):${NC} ${WORKSPACE_PATH}:/workspace"
    fi
    if [ "$MAP_PATH" != "" ]; then
        echo -e "${GREEN}MAP PATH(mounted):${NC} ${MAP_PATH}:/autoware_map"
    fi
    echo -e "${GREEN}LAUNCH CMD:${NC} ${LAUNCH_CMD}"
    echo -e "${GREEN}-----------------------------------------------------------------${NC}"

    if [ "$option_pull_latest_image" = "true" ]; then
        docker pull ${IMAGE}
    fi

    if [ "$option_devel" = "true" ] && [ "$option_perception_bootstrap" = "true" ]; then
        echo -e "${GREEN}PERCEPTION BOOTSTRAP:${NC} enabled"
        launch_devel_with_bootstrap
        return 0
    fi

    # Launch the container (direct mode)
    TZ_VALUE="$(cat /etc/timezone 2>/dev/null || true)"
    set -x
    docker run -it  --rm --net=host ${GPU_FLAG} ${USER_ID} ${MOUNT_X} \
        -e XAUTHORITY=${XAUTHORITY} -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e NVIDIA_DRIVER_CAPABILITIES=all -e TZ="${TZ_VALUE}" \
        ${WORKSPACE} ${MAP} ${DATA} ${IMAGE} \
        ${LAUNCH_CMD}
}

# Execute the main script
main "$@"
