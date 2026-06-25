#!/usr/bin/env bash
#
# Configure and build this MoveIt Pro workspace for ROS 2 Jazzy.
#
#   1. moveit_pro configure   — point the CLI at this workspace + config package
#   2. moveit_pro envfile      — (re)generate the .env file in the repo
#   3. ensure MOVEIT_ROS_DISTRO=jazzy is present in .env
#   4. moveit_pro build        — build the user image + workspace
#
set -euo pipefail

# Resolve the directory this script lives in (the workspace root).
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PACKAGE="franka_arm_hw"
ENV_FILE="${WORKSPACE_DIR}/.env"

cd "${WORKSPACE_DIR}"

echo ">>> Updating git submodules"
git submodule update --init --recursive -f

echo ">>> Configuring MoveIt Pro (config=${CONFIG_PACKAGE}, workspace=${WORKSPACE_DIR})"
moveit_pro configure -c "${CONFIG_PACKAGE}" -w "${WORKSPACE_DIR}"

echo ">>> Generating .env file"
moveit_pro envfile

echo ">>> Ensuring MOVEIT_ROS_DISTRO=jazzy in .env"
if [[ -f "${ENV_FILE}" ]] && grep -q '^MOVEIT_ROS_DISTRO=' "${ENV_FILE}"; then
  # Normalize any existing value to jazzy.
  sed -i 's/^MOVEIT_ROS_DISTRO=.*/MOVEIT_ROS_DISTRO=jazzy/' "${ENV_FILE}"
else
  echo "MOVEIT_ROS_DISTRO=jazzy" >> "${ENV_FILE}"
fi

echo ">>> Building MoveIt Pro (user image + workspace)"
moveit_pro build

echo ">>> Done."
