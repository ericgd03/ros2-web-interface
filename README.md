# ROS2 Web Interface

This project provides a web interface for ROS 2 using a gRPC server, a Go-based gateway, and a Flask dashboard.

## Prerequisites

Before running the system, ensure the following dependencies are installed:

*Python gRPC tools*

```bash
  pip install grpcio grpcio-tools
```

*Go and Protocol Buffers compiler*

```bash
sudo apt update
sudo apt install -y golang-go protobuf-compiler
```

*gRPC plugins for Go*

```bash
go install google.golang.org/protobuf/cmd/protoc-gen-go@latest
go install google.golang.org/grpc/cmd/protoc-gen-go-grpc@latest
```

Make sure $GOPATH/bin is in your system PATH:

```bash
export PATH="$PATH:$(go env GOPATH)/bin"
```

## Generate code from the .proto files (They are already created)

To generate the Python and Go bindings from the .proto file, run the following commands:

*Python*

```bash
python3 -m grpc_tools.protoc -I./protos --python_out=./protos --grpc_python_out=./protos ./protos/rpc-demo.proto
```

*go*

```bash
protoc --go_out=./protos --go-grpc_out=./protos ./protos/rpc-demo.proto
```

## Running the system

Start the gRPC server that subscribes to the puzzlebot topics
```bash
phyton3 ros2-grpc-wrapper.py
```

Start the go gateway, it listens to the gRPC server and exposes the data as RESTful HTTP API
```bash
go run gateway.go
```

Start the flask app that genarates the dashboard on the web page
```bash
python3 app.py
```
