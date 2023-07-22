#!/bin/bash
mkdir -p ../car_ws/vehicle_logs
docker run -p 8765:8765 --name f1tenth-gym -it --rm \
    -v $(pwd)/../car_ws:/sim_ws \
    -v $(pwd)/../car_ws/vehicle_logs:/sim_ws/vehicle_logs \
    -v $(pwd)/../libf1tenth:/libf1tenth \
    f1tenth_gym_ros:latest bash