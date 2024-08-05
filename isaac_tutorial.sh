#!/usr/bin/env bash
# get isaac tutorial directory
ISAAC_TUTORIAL_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
export ISAAC_TUTORIAL_PATH=${ISAAC_TUTORIAL_DIR}
echo "Setting ISAAC_TUTORIAL_PATH: ${ISAAC_TUTORIAL_PATH}"