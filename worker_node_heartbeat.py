#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import time

class WorkerHeartbeatNode(Node):
    def __init__(self):
        super().__init__('worker1_heartbeat')
        # 파라미터를 통해 노드 이름을 지정 (기본값 "worker-node1")
        self.declare_parameter("node_name", "worker-node1")
        self.node_name = self.get_parameter("node_name").get_parameter_value().string_value

        # raw_image 토픽 콜백에서 최근 수신 시간을 업데이트함
        self.last_image_time = 0.0
        self.subscription = self.create_subscription(
            Image,
            '/camera/raw_image',
            self.raw_image_callback,
            10
        )
        # heartbeat 메시지 발행을 위한 publisher 생성
        self.heartbeat_publisher = self.create_publisher(Int32, '/worker1_heartbeat', 10)
        self.get_logger().info(f"Worker Heartbeat Node started for {self.node_name}")

        # 30Hz 주기로 heartbeat를 발행하는 타이머 (1/30초 ≈ 0.0333초 주기)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def raw_image_callback(self, msg):
        # raw_image 메시지가 도착하면 현재 시간을 기록
        self.last_image_time = time.time()
        self.get_logger().debug(f"Received raw_image at {self.last_image_time}")

    def timer_callback(self):
        # 타이머 콜백이 30Hz로 호출되며, 최근 raw_image 수신 여부를 확인한 후 heartbeat를 발행
        current_time = time.time()
        # 예: 최근 1초 이내에 raw_image가 수신되었으면 heartbeat 발행
        if current_time - self.last_image_time < 0.15:
            heartbeat = Int32()
            heartbeat.data = 1
            self.heartbeat_publisher.publish(heartbeat)
            self.get_logger().info(f"Published heartbeat: {heartbeat.data}")
        else:
            self.get_logger().warn("No recent raw_image received, skipping heartbeat.")

def main(args=None):
    rclpy.init(args=args)
    node = WorkerHeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Worker Heartbeat Node stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import time

class WorkerHeartbeatNode(Node):
    def __init__(self):
        super().__init__('worker1_heartbeat')
        # 파라미터를 통해 노드 이름을 지정 (기본값 "worker-node1")
        self.declare_parameter("node_name", "worker-node1")
        self.node_name = self.get_parameter("node_name").get_parameter_value().string_value

        # raw_image 토픽 콜백에서 최근 수신 시간을 업데이트함
        self.last_image_time = 0.0
        self.subscription = self.create_subscription(
            Image,
            '/camera/raw_image',
            self.raw_image_callback,
            10
        )
        # heartbeat 메시지 발행을 위한 publisher 생성
        self.heartbeat_publisher = self.create_publisher(Int32, '/worker1_heartbeat', 10)
        self.get_logger().info(f"Worker Heartbeat Node started for {self.node_name}")

        # 30Hz 주기로 heartbeat를 발행하는 타이머 (1/30초 ≈ 0.0333초 주기)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def raw_image_callback(self, msg):
        # raw_image 메시지가 도착하면 현재 시간을 기록
        self.last_image_time = time.time()
        self.get_logger().debug(f"Received raw_image at {self.last_image_time}")

    def timer_callback(self):
        # 타이머 콜백이 30Hz로 호출되며, 최근 raw_image 수신 여부를 확인한 후 heartbeat를 발행
        current_time = time.time()
        # 예: 최근 1초 이내에 raw_image가 수신되었으면 heartbeat 발행
        if current_time - self.last_image_time < 0.15:
            heartbeat = Int32()
            heartbeat.data = 1
            self.heartbeat_publisher.publish(heartbeat)
            self.get_logger().info(f"Published heartbeat: {heartbeat.data}")
        else:
            self.get_logger().warn("No recent raw_image received, skipping heartbeat.")

def main(args=None):
    rclpy.init(args=args)
    node = WorkerHeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Worker Heartbeat Node stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
