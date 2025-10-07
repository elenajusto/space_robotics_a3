# ...existing code...
import time
import rclpy
from rclpy.node import Node
from ultralytics import YOLO


class ModelRunnerNode(Node):
    def __init__(self):
        super().__init__('model_runner')
        self.get_logger().info('ModelRunner started')


def main(args=None):
    rclpy.init(args=args)
    node = ModelRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# ...existing code...