# Here you can config commands when the container is inicialized

#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash

echo "Provided arguments: $@"

exec $@