#!/usr/bin/env bash
# Sources the ROS environment, then the workspace overlay if one has been
# built, and hands over to the container command.
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

exec "$@"
