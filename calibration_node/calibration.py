import rclpy
from rclpy.node import Node
from vision_msgs.msg import DetectedCropArray, DetectedCrop
from std_msgs.msg import Header
import numpy as np


class CropTransformerNode(Node):
    def __init__(self):
        super().__init__('crop_transformer_node')

        self.subscription = self.create_subscription(
            DetectedCropArray,
            '/detected_crops',
            self.callback,
            10)

        self.publisher_ = self.create_publisher(DetectedCropArray, '/transformed_crops', 10)

        # 예시로 4x4 행렬 (hand-eye calibration matrix)
        self.X = np.array([
            [1.0, 0.0, 0.0, 0.1],  # 10cm x offset
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.2],  # 20cm z offset
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.get_logger().info("CropTransformerNode started.")

    def callback(self, msg: DetectedCropArray):
        transformed_msg = DetectedCropArray()
        transformed_msg.header = Header()
        transformed_msg.header.stamp = self.get_clock().now().to_msg()
        transformed_msg.total_objects = msg.total_objects

        for crop in msg.objects:
            # cm → m 변환
            position = np.array([crop.x / 100.0, crop.y / 100.0, crop.z / 100.0, 1.0])
            transformed = self.X @ position

            new_crop = DetectedCrop()
            new_crop.id = crop.id
            new_crop.x = transformed[0] * 100.0  # 다시 m → cm
            new_crop.y = transformed[1] * 100.0
            new_crop.z = transformed[2] * 100.0

            transformed_msg.objects.append(new_crop)
            self.get_logger().info(
                f"[Transform] ID={new_crop.id}, X={new_crop.x:.2f}, Y={new_crop.y:.2f}, Z={new_crop.z:.2f}"
            )

        self.publisher_.publish(transformed_msg)
        self.get_logger().info("Transformed crops published.")


def main(args=None):
    rclpy.init(args=args)
    node = CropTransformerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
