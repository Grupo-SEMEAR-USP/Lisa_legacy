#!/bin/bash
# Script to easily clear the ~/.ros/log folder
./docker/exec.sh "$1" "$2" rm -rf ~/.ros/log