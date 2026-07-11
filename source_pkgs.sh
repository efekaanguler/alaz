#!/usr/bin/env bash

SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source /opt/autoware/setup.bash
source "${SOURCE_DIR}/install/setup.bash"
