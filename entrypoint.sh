# !/bin/bash

set -e
source /opt/ros/jazzy/setup.bash

echo "DISPLAY=$DISPLAY"
echo "XAUTHORITY=$XAUTHORITY"
env | grep DISPLAY || true

echo "Provided arguments: $@"

exec "$@"