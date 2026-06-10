# !/bin/bash

set -e
source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "DISPLAY=$DISPLAY"
echo "XAUTHORITY=$XAUTHORITY"
env | grep DISPLAY || true

echo "Provided arguments: $@"

exec "$@"
