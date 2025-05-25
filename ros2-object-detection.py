import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image, CompressedImage # Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np
from cv_bridge import CvBridge # Image

import asyncio
import threading
import websockets
import base64

class ObjectDetectionNode(Node):

    def __init__(self):

        super().__init__('object_detection')
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)  # 100 Hz

        self.publisher_image = self.create_publisher(Image, '/image_raw', 10) # Image
        self.publisher_trailer = self.create_publisher(String, '/trailer', 10)
        self.publisher_velocity = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()
        self.msg_trailer = String()
        self.msg_velocity = Twist()

        self.loop = asyncio.new_event_loop()
        t = threading.Thread(target=self.run_async_loop, daemon=True)
        t.start()

        self.msg_trailer.data = "trailer_1"
        self.time = 0
        self.frame = None

        self.get_logger().info('ROS2 node initialized')

    def timer_callback(self):

        # img = cv2.imread("isorepublic-red-green-apples-1.jpg", cv2.IMREAD_COLOR)
        # self.frame = cv2.resize(self.frame, (640,480))
        # self.frame = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        # self.publisher_image.publish(self.frame)
        # self.get_logger().info('Publishing image')

        self.publisher_trailer.publish(self.msg_trailer)

        self.msg_velocity.linear.x = np.sin(self.time)
        self.msg_velocity.angular.z = np.cos(self.time)
        self.publisher_velocity.publish(self.msg_velocity)
        if (self.time > 360):
            self.time = 0
        else:
            self.time += self.timer_period

    def run_async_loop(self):

        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.receive_frames())

    async def receive_frames(self):

        uri = "ws://10.22.142.196:8765"  # Change this to host IP if not using Docker Desktop
        self.get_logger().info(f"Connecting to {uri}")

        try:
            async with websockets.connect(uri) as websocket:
                self.get_logger().info("Connected to WebSocket server")
                while rclpy.ok():
                    try:
                        data = await websocket.recv()
                        self.frame = self.decode_frame(data)
                        self.frame = cv2.resize(self.frame, (640,480))
                        self.frame = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
                        self.publisher_image.publish(self.frame)
                    except websockets.exceptions.ConnectionClosed:
                        self.get_logger().warn("WebSocket closed")
                        break

        except Exception as e:
            self.get_logger().error(f"Failed to connect or receive: {e}")

    def decode_frame(self, base64_string):
        jpg_bytes = base64.b64decode(base64_string)
        np_arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return frame

def main(args=None):

    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()