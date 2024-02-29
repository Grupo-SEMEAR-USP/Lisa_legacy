#!/bin/bash

# If you want to build all the packages in debug [NOT RECOMMENDED]
# catkin config -DCMAKE_BUILD_TYPE=Debug

# If you want to build all the packages in release [RECOMMENDED]
catkin config -DCMAKE_BUILD_TYPE=Release

# Build all the workspace
catkin build