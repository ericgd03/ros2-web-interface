# ROS2 Web Interface

Before running the system, you need to install

```bash
  pip install grpcio grpcio-tools
```

- grpc
- go

The commands used to generate the python and go files from the .proto file are

python3 -m grpc_tools.protoc -I./protos —-python_out=./protos —-grpc_python_out=./protos ./protos/rpc-demo.proto
protoc --go_out=./protos --go-grpc_out=./protos ./protos/rpc-demo.proto

Start the gRPC server that subscribes to the puzzlebot topics

phyton3 ros2-grpc-wrapper.py

Start the go gateway, it listens to the gRPC server and exposes the data as RESTful HTTP API

go run gateway.go

Start the flask app that genarates the dashboard on the web page

python3 app.py
