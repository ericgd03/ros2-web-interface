import grpc
import cv2
import numpy as np
import sys

# Add path to generated gRPC modules
sys.path.insert(1, './protos')
import rpc_demo_pb2
import rpc_demo_pb2_grpc

def main():
    channel = grpc.insecure_channel('localhost:7042')
    stub = rpc_demo_pb2_grpc.RPCDemoStub(channel)

    try:
        response = stub.GetImage(rpc_demo_pb2.ImageRequest())

        if not response.image_data:
            print("No image data received.")
            return

        print(f"Received image of format: {response.format}")

        # Convert byte data to numpy array
        nparr = np.frombuffer(response.image_data, np.uint8)

        # Decode JPEG byte stream to OpenCV image
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if img is None:
            print("Failed to decode image")
            return

        # Display the image
        cv2.imshow('Received Image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except grpc.RpcError as e:
        print(f"gRPC Error: {e.code()} - {e.details()}")

if __name__ == '__main__':
    main()