import signal
import threading
from concurrent import futures

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

import grpc
import sys
sys.path.insert(1, './protos')
import rpc_demo_pb2
import rpc_demo_pb2_grpc

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage # Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Image
import cv2 # Image

class RPCDemoImpl(rpc_demo_pb2_grpc.RPCDemoServicer):

    def __init__(self, node):
        self.node = node
        
        self.latest_frame = None # Image
        self.bridge = CvBridge() # Image
        self.node.create_subscription(Image, '/image_raw', self.image_callback, 10) # Image

        self.current_trailer = ""
        self.node.create_subscription(String, '/trailer', self.trailer_callback, 10)

        self.linaer_velocity = 0
        self.angular_velocity = 0
        self.node.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)

        print("Initialized gRPC Server")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)#, desired_encoding='bgr8')
            self.latest_frame = cv_image
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {e}")

    def trailer_callback(self, msg):
        self.current_trailer = msg.data

    def vel_callback(self, msg):
        self.linaer_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def GetImage(self, request, context):
        print("Got image request from:", context.peer())
        if self.latest_frame is None:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details('No image available yet')
            return rpc_demo_pb2.ImageResponse()

        success, encoded = cv2.imencode('.jpg', self.latest_frame)
        if not success:
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details('Failed to encode image')
            return rpc_demo_pb2.ImageResponse()

        return rpc_demo_pb2.ImageResponse(
            image_data=encoded.tobytes(),
            format='jpeg'
        )

    def GetTrailer(self, request, context):
        print("Got trailer status request from", context.peer())
        return rpc_demo_pb2.TrailerStatus(
            trailer_number = self.current_trailer
        )

    def GetVelocity(self, request, context):
        print("Got velocity request from", context.peer())
        return rpc_demo_pb2.Velocities(
            linear_velocity = self.linaer_velocity,
            angular_velocity = self.angular_velocity
        )

terminate = threading.Event()
def terminate_server(signum, frame):
    print("Got signal {}, {}".format(signum, frame))
    rclpy.shutdown()
    terminate.set()

def main(args=None):
    print("----ROS-gRPC-Wrapper----")
    signal.signal(signal.SIGINT, terminate_server)

    print("Starting ROS Node")
    rclpy.init(args=args)
    node = rclpy.create_node('object_position_wrapper')

    print("Starting gRPC Server")
    server_addr = "[::]:7042"
    service = RPCDemoImpl(node)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    rpc_demo_pb2_grpc.add_RPCDemoServicer_to_server(service, server)
    server.add_insecure_port(server_addr)
    server.start()
    print("gRPC Server listening on " + server_addr)

    print("Running ROS Node")
    executor = futures.ThreadPoolExecutor()
    executor.submit(lambda: rclpy.spin(node))
    
    terminate.wait()
    print("Stopping gRPC Server")
    server.stop(1).wait()
    print("Exited")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()