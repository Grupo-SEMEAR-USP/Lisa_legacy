# Quickstart for Noetic
[ [Back to README.md](../README.md) ]

## Previous steps
Previous steps are available on the [README.md](../README.md) file.
NOTE: We have only the Go1 workspace that is fully compatible with ROS Noetic.

## Step 2 - Go1 robot workspace in Noetic

The following command will prepare all the needed repositories and build the needed docker image with noetic-legged image.
```bash
./scripts/prepare_legged_ws.sh noetic-legged
```

## Step 3 - ROS Noetic Full "LEGGED" version (Ubuntu 18.04)
**NOTE: Already built if you followed step 2 **
To easily start a ROS container:
```bash
./docker/run.sh noetic-legged
```

## Step 4 - Build the packages
Now, inside the docker container let's build all the needed packages.

```If you don't know whether you are inside a container, check if yout current folder and user look something like:``` **user@computer:~/catkin_ws$**

```bash
./scripts/build_legged_ws.sh
```
