#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
AUTOWARE_RUN="${SCRIPT_DIR}/../autoware/docker/run.sh"


WORKSPACE_PATH="$SCRIPT_DIR"

# for non-GPU: USE_GPU=0 ./dev_run.sh

if [[ "${USE_GPU:-0}" == "0" ]]; then
  exec "${AUTOWARE_RUN}" --devel --no-nvidia --workspace "${WORKSPACE_PATH}" /bin/bash
else
  exec "${AUTOWARE_RUN}" --devel --workspace "${WORKSPACE_PATH}" /bin/bash
fi
