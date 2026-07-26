# Docker image for extending MoveIt Pro with a custom overlay.
#
# Example build command (with defaults):
#
# docker build -f ./Dockerfile .
#

# Specify the MoveIt Pro release to build on top of.
# Set MOVEIT_ROS_DISTRO=jazzy (in .env) to build on the ROS 2 Jazzy base image.
ARG MOVEIT_PRO_BASE_IMAGE=picknikciuser/moveit-studio:${MOVEIT_DOCKER_TAG:-main}-${MOVEIT_ROS_DISTRO:-jazzy}
ARG USERNAME=moveit-pro-user
ARG USER_UID=1000
ARG USER_GID=1000

##################################################
# Starting from the specified MoveIt Pro release #
##################################################
# The image tag is specified in the argument itself.
# hadolint ignore=DL3006
FROM ${MOVEIT_PRO_BASE_IMAGE} AS base

# Create a non-root user
ARG USERNAME
ARG USER_UID
ARG USER_GID

# Copy source code from the workspace's ROS 2 packages to a workspace inside the container
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Set real time limits
# Ensure the directory exists
RUN mkdir -p /etc/security

# Also mkdir with user permission directories which will be mounted later to avoid docker creating them as root
WORKDIR $USER_WS
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    if getent passwd $USER_UID >/dev/null; then userdel -f -r "$(getent passwd $USER_UID | cut -d: -f1)" 2>/dev/null || true; fi && \
    if getent group $USER_GID >/dev/null; then groupdel "$(getent group $USER_GID | cut -d: -f1)" 2>/dev/null || true; fi && \
    groupadd --gid $USER_GID ${USERNAME} && \
    useradd --uid $USER_UID --gid $USER_GID --shell /bin/bash --create-home ${USERNAME} && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    cp -r /etc/skel/. /home/${USERNAME} && \
    mkdir -p \
      /home/${USERNAME}/.ccache \
      /home/${USERNAME}/.config \
      /home/${USERNAME}/.ignition \
      /home/${USERNAME}/.colcon \
      /home/${USERNAME}/.ros && \
    chown -R $USER_UID:$USER_GID /home/${USERNAME} /opt/overlay_ws/

# Add user to dialout group to enable communication with serial USB devices (gripper, FTS, ...)
# Add user to video group to enable communication with cameras
RUN usermod -aG dialout,video ${USERNAME}

# Add user to the realtime group to enable RT limits
RUN groupadd realtime && \
    usermod -a -G realtime ${USERNAME}

# Install additional dependencies
# You can also add any necessary apt-get install, pip install, etc. commands at this point.
# NOTE: The /opt/overlay_ws folder contains MoveIt Pro binary packages and the source file.
# The skip-keys are dependencies declared by the vendored franka_ros2 packages
# (e.g. franka_mobile_sensors) that have no rosdep key on the Jazzy/Noble base image.
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=bind,target=${USER_WS}/src,source=./src \
    . /opt/overlay_ws/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths src \
      --ignore-src \
      --skip-keys "olv_module_descriptions sick_safetyscanners2 zed_wrapper zed_description robotiq_driver robotiq_controllers robotiq_description clearpath_mecanum_drive_controller"

# Work around a corrupt OpenCV core library in the humble base image.
# The shipped /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d has an
# off-by-one DT_RELACOUNT (declares 4878 relative relocations, has 4877).
# glibc 2.35's loader trusts RELACOUNT and fast-paths that many relocations
# as R_X86_64_RELATIVE without type-checking, so it aborts with
# "elf_machine_rela_relative: Assertion ... R_X86_64_RELATIVE failed" the
# moment any node loads OpenCV core (e.g. the required objective_server_node),
# which crashes the whole app. Reinstalling restores the pristine library.
#
# The runtime package name is version- and distro-specific (humble/jammy ships
# libopencv-core4.5d, jazzy/noble ships libopencv-core406t64), so resolve it
# dynamically from the installed package rather than hardcoding it. On jazzy the
# library is not corrupt, making this reinstall a harmless pristine-restore.
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    opencv_core_pkg="$(dpkg -S /usr/lib/*/libopencv_core.so.[0-9]* 2>/dev/null | awk -F: '{print $1}' | sort -u | grep -v -- '-dev' | head -n1)" && \
    if [ -z "${opencv_core_pkg}" ]; then echo "ERROR: no libopencv-core runtime package installed" >&2; exit 1; fi && \
    echo "Reinstalling OpenCV core package: ${opencv_core_pkg}" && \
    apt-get install -q -y --reinstall --no-install-recommends "${opencv_core_pkg}"

# Build a manage_overruns-patched ros2_control hardware_interface into the overlay.
#
# franka_ros2 ships patches/manage_overruns.patch, which adds a `manage_overruns`
# attribute to ros2_control's async hardware support so an <async manage_overruns=...>
# in the URDF is honored. It targets hardware_interface 4.44.0 (the version in the
# jazzy/noble base image) and needs realtime_tools 3.10.1 (vendored in the franka_ros2
# submodule), whose AsyncFunctionHandlerParams carries the manage_overruns field that the
# distro's stock 3.11.0 lacks. We replicate franka_ros2's "clone ros2_control + git apply"
# flow at image-build time: shallow-clone ros2_control @ 4.44.0, apply the patch at the
# repo root, build the patched hardware_interface (plus realtime_tools 3.10.1) in a temp
# workspace, and drop the resulting lib/include/share into the existing merge-install
# overlay so it shadows the stock /opt/ros library for both the user_workspace build and
# runtime. The existing overlay setup.sh already exports the prefix generically, so no
# setup regeneration is needed. Guarded to hardware_interface 4.44.0 -> a no-op elsewhere.
# NOTE: `set -u` is intentionally NOT used: sourcing ROS setup.sh references unbound
# shell vars (e.g. AMENT_TRACE_SETUP_FILES) and would abort under nounset.
# hadolint ignore=DL3003,SC1091
RUN --mount=type=bind,target=/tmp/ws_src,source=./src \
    set -ex; \
    hw_pkg_xml="$(ls /opt/ros/*/share/hardware_interface/package.xml 2>/dev/null | head -n1)"; \
    hw_ver="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' "${hw_pkg_xml}" 2>/dev/null || true)"; \
    ros_distro="$(basename "$(dirname "$(dirname "$(dirname "${hw_pkg_xml}")")")")"; \
    if [ "${hw_ver}" != "4.44.0" ]; then \
      echo "manage_overruns overlay: hardware_interface is '${hw_ver:-unknown}' (not 4.44.0) -> skipping"; \
    else \
      patch_file="/tmp/ws_src/external_dependencies/franka_ros2/patches/manage_overruns.patch"; \
      rt_src="/tmp/ws_src/external_dependencies/franka_ros2/realtime_tools"; \
      rm -rf /tmp/mo_ws; mkdir -p /tmp/mo_ws/src; \
      git clone --depth 1 --branch 4.44.0 https://github.com/ros-controls/ros2_control.git /tmp/mo_ws/src/ros2_control; \
      git -C /tmp/mo_ws/src/ros2_control apply --verbose "${patch_file}"; \
      cp -a "${rt_src}" /tmp/mo_ws/src/realtime_tools; \
      . "/opt/ros/${ros_distro}/setup.sh"; \
      . /opt/overlay_ws/install/setup.sh; \
      cd /tmp/mo_ws; \
      colcon build --install-base /tmp/mo_ws/install \
        --packages-select realtime_tools hardware_interface \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF; \
      for d in realtime_tools hardware_interface; do \
        if [ -d "/tmp/mo_ws/install/${d}" ]; then cp -a "/tmp/mo_ws/install/${d}/." "/opt/overlay_ws/install/${d}/"; fi; \
      done; \
      cd /; rm -rf /tmp/mo_ws; \
      echo "manage_overruns overlay: installed patched hardware_interface + realtime_tools 3.10.1"; \
    fi

# Set up colcon defaults for the new user
USER ${USERNAME}
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update
COPY colcon-defaults.yaml /home/${USERNAME}/.colcon/defaults.yaml

# Custom colcon mixins for this overlay (auto-loaded from the mixin search path).
COPY mixin/ /home/${USERNAME}/.colcon/mixin/user-overlay/

# Run ccache in depend mode. ccache's default preprocessor mode feeds a separate
# cpp pass to GCC 13 (the jazzy/noble toolchain), which segfaults inside the
# compiler's garbage collector ("internal compiler error: Segmentation fault" in
# gt_ggc_mx_lang_tree_node) while instantiating the deep Eigen/pinocchio template
# trees in libfranka's robot_model.cpp / rate_limiting.cpp. Depend mode invokes
# the compiler directly on the source (hashing via -MD output instead of a cpp
# pass), which compiles cleanly while keeping ccache's caching benefit.
ENV CCACHE_DEPEND=1

# hadolint ignore=DL3002
USER root

###################################################################
# Target for the developer build which does not compile any code. #
###################################################################
FROM base AS user-overlay-dev

ARG USERNAME
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Install any additional packages for development work
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        less \
        gdb \
        nano \
        tmux

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]

#########################################
# Target for compiled, deployable image #
#########################################
FROM base AS user-overlay

ARG USERNAME
ARG USER_WS=/home/${USERNAME}/user_ws
ENV USER_WS=${USER_WS}

# Compile the workspace
WORKDIR $USER_WS

# Set up the user's .bashrc file and shell.
CMD ["/usr/bin/bash"]
