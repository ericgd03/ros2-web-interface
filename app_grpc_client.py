from flask import Flask, Response, render_template, jsonify
import requests
import json
import grpc
import sys
sys.path.insert(1, './protos')
import rpc_demo_pb2
import rpc_demo_pb2_grpc

app = Flask(__name__)

# Set up gRPC channel
channel = grpc.insecure_channel('localhost:7042')  # Use same port as ROS gRPC server
stub = rpc_demo_pb2_grpc.RPCDemoStub(channel)

@app.route('/image')
def get_image():
    try:
        response = stub.GetImage(rpc_demo_pb2.google_dot_protobuf_dot_empty__pb2.Empty())
        return Response(response.image_data, mimetype='image/jpeg')  # Change to 'image/png' if needed
    except grpc.RpcError as e:
        return f"Failed to get image: {e}", 500

@app.route('/trailer')
def get_trailer():
    response = stub.GetTrailer(rpc_demo_pb2.google_dot_protobuf_dot_empty__pb2.Empty())
    return jsonify({"trailer_number": response.trailer_number})

@app.route('/velocity')
def get_velocity():
    response = stub.GetVelocity(rpc_demo_pb2.google_dot_protobuf_dot_empty__pb2.Empty())
    return jsonify({"linear_velocity": response.linear_velocity,
                    "angular_velocity": response.angular_velocity
    })

@app.route('/')
def index():
    return render_template('result.html')

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=8000, debug=True)
