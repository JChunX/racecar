#!/bin/bash
mkdir -p ../../car_ws/vehicle_logs
docker run --name f1tenth_logger -it --rm \
    -v $(pwd)/../../car_ws/vehicle_logs:/car_ws/vehicle_logs \
    f1tenth_logger:latest bash