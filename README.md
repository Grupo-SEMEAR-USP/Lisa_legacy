# SEMEAR: Lisa Workspace 💻

A simplified repository for Legged Robotics Workspace using ROS + Docker. 😎

What is *contained* in this repository?
* Dockerfiles for some of ROS distros with the needed build instructions.
* Scripts that makes docker a little bit easier.

# [Docker Images](docs/IMAGES.md)
You can checkout what Docker Images we already have and its level of support/reliability in [docs/IMAGES.md](docs/IMAGES.md).


# Quickstart 🚀

## Step 0 - Install GIT and Docker

### GIT 🌳
```bash
sudo apt install -y git
```

### Docker 🐳
We **strongly recommend** using the installation of docker from this repository:
[Linux Stuffs](https://github.com/lomcin/linux-stuffs).

***IMPORTANT FOR NVIDIA's GPU's USERS***: There's also a script for installation of NVIDIA CONTAINER TOOLKIT in the [Linux Stuffs](https://github.com/lomcin/linux-stuffs) repository.

## Step 1 - Clone this repository
To download this repository with the dependencies repositories use the following command:
```bash
git clone https://github.com/lomcin/legged_ws.git
```

**NOTE: If you just have cloned this repository you will need the following steps:**

## Steps for Noetic (Ubuntu 20.04) [recommended] 👈
Follow the steps in the [docs/QUICKSTART_NOETIC.md](docs/QUICKSTART_NOETIC.md).
## Steps for Melodic (Ubuntu 18.04)
Follow the steps in the [docs/QUICKSTART_MELODIC.md](docs/QUICKSTART_MELODIC.md).

# FAQ - Frequently Asked Questions ❓
[Click here to be redirected to docs/FAQ.md file.](docs/FAQ.md)






