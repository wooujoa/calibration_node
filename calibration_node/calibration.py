import rclpy
from rclpy.node import Node
from vision_msgs.msg import CropPose, DetectedCrop, DetectedCropArray
from std_msgs.msg import Header
import numpy as np


class CropTransformerNode(Node):
    def __init__(self):
        super().__init__('crop_transformer_node')

        # 구독: 단일 객체 포즈
        self.create_subscription(CropPose, '/CropPose/obj', self.crop_pose_callback, 10)
        
        # 구독: 다중 객체 감지 결과
        self.create_subscription(DetectedCropArray, '/detected_crops', self.detected_crops_callback, 10)

        # 퍼블리시: 변환된 결과
        self.crop_pose_pub = self.create_publisher(CropPose, '/transformed_crop_pose', 10)
        self.detected_crops_pub = self.create_publisher(DetectedCropArray, '/transformed_crops', 10)

        # 핸드-아이 캘리브레이션 행렬
        self.X = np.array([
            [1.0, 0.0, 0.0, 0.1],  # x축 10cm offset
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.2],  # z축 20cm offset
            [0.0, 0.0, 0.0, 1.0]
        ])

        self.get_logger().info("CropTransformerNode (master calibration node) started.")

    def apply_transform(self, x_cm, y_cm, z_cm):
        """cm 단위 입력을 m로 변환하여 행렬 적용 후 다시 cm로 반환"""
        point = np.array([x_cm / 100.0, y_cm / 100.0, z_cm / 100.0, 1.0])
        transformed = self.X @ point
        return transformed[:3] * 100.0  # 다시 cm로 변환

    def crop_pose_callback(self, msg: CropPose):
        tx, ty, tz = self.apply_transform(msg.x, msg.y, msg.z)

        transformed_msg = CropPose()
        transformed_msg.x = tx
        transformed_msg.y = ty
        transformed_msg.z = tz

        self.crop_pose_pub.publish(transformed_msg)

        self.get_logger().info(
            f"[CropPose] Transformed: X={tx:.2f}, Y={ty:.2f}, Z={tz:.2f}"
        )

    def detected_crops_callback(self, msg: DetectedCropArray):
        transformed_array = DetectedCropArray()
        transformed_array.header = Header()
        transformed_array.header.stamp = self.get_clock().now().to_msg()
        transformed_array.total_objects = msg.total_objects

        for crop in msg.objects:
            tx, ty, tz = self.apply_transform(crop.x, crop.y, crop.z)

            new_crop = DetectedCrop()
            new_crop.id = crop.id
            new_crop.x = tx
            new_crop.y = ty
            new_crop.z = tz

            transformed_array.objects.append(new_crop)

            self.get_logger().info(
                f"[DetectedCrop] ID={new_crop.id}, Transformed: X={tx:.2f}, Y={ty:.2f}, Z={tz:.2f}"
            )

        self.detected_crops_pub.publish(transformed_array)
        self.get_logger().info("Transformed DetectedCropArray published.")


def main(args=None):
    rclpy.init(args=args)
    node = CropTransformerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
