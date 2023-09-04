# racecar 🏎️

A containerized toolkit for the [F1Tenth](https://f1tenth.org/) autonomous racing platform.

## Docker 🐳
Do the following in order:
Replacing `<scenario>` for your use case: `sim | real`

```bash
cd docker

bash build_docker.<scenario>.sh
bash run_docker.<scenario>.sh
bash exec_docker.<scenario>.sh
```

## Configuring Environment 🔨

You will need to modify these files:
```bash
# ROS domain, etc.
config/ros.env

# Foxglove data platform API
docker/logger/secrets/foxglove_api_key.txt 
```

## Sourcing Workspace 💡
Your first command after launching a new terminal in the container should be:
```bash
. /source_sim
```

## Racecar CLI 🤖
The `racecar` CLI provides many useful tools for you in the container 🤗
```bash
racecar build # build workspace
racecar bridge # start foxglove bridge
racecar bag # record rosbag
racecar teleop # run teleop
racecar sim # run simulator
```

## Submodules 📦

This repo is built up using submodules.
If you wish to add more modules to it, simply call:
```bash
git submodule add <repo_url> <path>
```