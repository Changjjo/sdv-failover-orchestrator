#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from kubernetes import client, config
from kubernetes.client import V1DeleteOptions
from kubernetes.client.exceptions import ApiException

class PodEvictor(Node):
    def __init__(self):
        super().__init__('pod_evictor')
        # Load kubeconfig file (adjust path for K3s installation)
        try:
            config.load_kube_config(config_file="/etc/rancher/k3s/k3s.yaml")
            self.get_logger().info('Kubeconfig loaded.')
        except Exception as e:
            self.get_logger().error(f"[ERROR] Kubeconfig load failed: {e}")
            sys.exit(1)

        # Create a single CoreV1Api instance to reuse
        self.core_api = client.CoreV1Api()

        # Record start time for grace period
        self.start_time = time.time()

        # Dictionary to store the last heartbeat time for each worker node
        self.heartbeat_times = {
            'worker-node1': self.start_time,
            'worker-node2': self.start_time,
            'worker-node3': self.start_time,
        }

        # Dictionary to track if a deletion request has been sent for each node
        self.evicted = {
            'worker-node1': False,
            'worker-node2': False,
            'worker-node3': False,
        }

        # Subscribe to heartbeat topics for each node (Int32 messages)
        self.create_subscription(Int32, '/worker1_heartbeat', self.worker1_callback, 10)
        self.create_subscription(Int32, '/worker2_heartbeat', self.worker2_callback, 10)
        self.create_subscription(Int32, '/worker3_heartbeat', self.worker3_callback, 10)
        self.get_logger().info("Subscribed to heartbeat topics.")

        # Timer callback running at 30Hz (~0.033 sec interval)
        self.timer = self.create_timer(1.0/60.0, self.timer_callback)

    def worker1_callback(self, msg: Int32):
        if self.evicted['worker-node1']:
            self.get_logger().info("worker-node1 recovered.")
        self.heartbeat_times['worker-node1'] = time.time()
        self.evicted['worker-node1'] = False

    def worker2_callback(self, msg: Int32):
        if self.evicted['worker-node2']:
            self.get_logger().info("worker-node2 recovered.")
        self.heartbeat_times['worker-node2'] = time.time()
        self.evicted['worker-node2'] = False

    def worker3_callback(self, msg: Int32):
        if self.evicted['worker-node3']:
            self.get_logger().info("worker-node3 recovered.")
        self.heartbeat_times['worker-node3'] = time.time()
        self.evicted['worker-node3'] = False

    def find_pods_by_node(self, namespace: str, node_name: str):
        """
        Find all pods in the given namespace that are on the specified node.
        """
        pods = self.core_api.list_namespaced_pod(
            namespace=namespace,
            field_selector=f"spec.nodeName={node_name}"
        )
        return pods.items

    def delete_pod(self, namespace: str, pod_name: str):
        """
        Send a deletion request for a pod.
        """
        delete_options = V1DeleteOptions()
        try:
            self.core_api.delete_namespaced_pod(
                name=pod_name,
                namespace=namespace,
                body=delete_options
            )
            self.get_logger().info(f"Deleted: {pod_name}")
        except ApiException as e:
            self.get_logger().error(f"[ERROR] Delete failed {pod_name}: {e}")

    def delete_pods_by_node(self, namespace: str, node_name: str):
        """
        Delete all pods on the specified node.
        """
        pods = self.find_pods_by_node(namespace, node_name)
        if not pods:
            self.get_logger().info(f"No pods on {node_name}.")
            return
        for pod in pods:
            self.get_logger().info(f"Deleting {pod.metadata.name}...")
            self.delete_pod(namespace, pod.metadata.name)

    def timer_callback(self):
        """
        Called at 30Hz to check the last heartbeat for each node.
        A 1-second grace period is applied after start.
        When the elapsed time since the last heartbeat exceeds the threshold,
        measure and display both:
          - The pure API call overhead (from entering the condition until API calls finish)
          - The total failover duration (from the last heartbeat to API call completion)
        """
        current_time = time.time()
        grace_period = 3.0  # 1 second grace period
        if current_time - self.start_time < grace_period:
            self.get_logger().debug("In grace period.")
            return

        for node, last in self.heartbeat_times.items():
            if current_time - last > 0.3:
                if not self.evicted[node]:
                    self.get_logger().warn(f"No heartbeat from {node}; threshold exceeded. Deleting pods.")
                    # Measure pure API overhead
                    overhead_start = time.time()
                    self.delete_pods_by_node("default", node)
                    overhead_time = time.time() - overhead_start
                    # Measure total failover duration from last heartbeat until API call completes
                    total_duration = time.time() - last
                    self.get_logger().info(
                        f"{node}: API overhead: {overhead_time:.3f}s, Total failover duration: {total_duration:.3f}s."
                    )
                    self.evicted[node] = True
                else:
                    self.get_logger().debug(f"{node} already evicted.")
            else:
                self.get_logger().debug(f"{node} ok.")

def main(args=None):
    rclpy.init(args=args)
    pod_evictor = PodEvictor()
    try:
        rclpy.spin(pod_evictor)
    except KeyboardInterrupt:
        pod_evictor.get_logger().info("Shutting down.")
    finally:
        pod_evictor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

