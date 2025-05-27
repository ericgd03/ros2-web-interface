#!/bin/bash

python3 ros2-object-detection.py &
python3 ros2-grpc-wrapper.py &
go run gateway.go &
python3 app.py &