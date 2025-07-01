import rclpy
from rclpy.node import Node
from vision_msgs.msg import CropPose, DetectedCrop, DetectedCropArray
from std_msgs.msg import Header
import numpy as np


class CropTransformerNode(Node):
    def __init__(self):
        super().__init__('crop_transformer_node')

        # 구독: 단일 객체 포즈 (m 단위)
        self.create_subscription(CropPose, '/CropPose/obj', self.crop_pose_callback, 10)

        # 구독: 다중 객체 감지 결과 (m 단위)
        self.create_subscription(DetectedCropArray, '/detected_crops', self.detected_crops_callback, 10)

        # 퍼블리시: 변환된 결과 (m 단위)
        self.crop_pose_pub = self.create_publisher(CropPose, '/CropPose/obj/result', 10)
        self.detected_crops_pub = self.create_publisher(DetectedCropArray, '/detected_crops/result', 10)

        # 이동 벡터 (카메라 기준에서 그리퍼 원점 위치)
        self.t = np.array([0.03, 0.055, 0.075])

        # 회전 행렬 (카메라 기준에서 그리퍼 축 방향)
        self.R = np.array([
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0],
            [1.0,  0.0,  0.0]
        ])

        """
        # 핸드-아이 캘리브레이션 행렬
        self.X = np.array([
            [0.0, -1.0, 0.0, 0.03],
            [0.0, 0.0, -1.0, 0.055],
            [1.0, 0.0, 0.0, 0.075],
            [0.0, 0.0, 0.0, 1.0]
        ])
        """

        self.get_logger().info("CropTransformerNode (master calibration node) started.")

    def apply_transform(self, x, y, z):
        # 입력과 출력 모두 m 단위
        point_cam = np.array([x, y, z])
        translated = point_cam - self.t
        transformed = self.R.T @ translated
        return transformed

    def crop_pose_callback(self, msg: CropPose):
        tx, ty, tz = self.apply_transform(msg.x, msg.y, msg.z)

        transformed_msg = CropPose()
        transformed_msg.x = tx
        transformed_msg.y = ty
        transformed_msg.z = tz

        self.crop_pose_pub.publish(transformed_msg)

        self.get_logger().info(
            f"[CropPose] Transformed: X={tx:.3f} m, Y={ty:.3f} m, Z={tz:.3f} m"
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
                f"[DetectedCrop] ID={new_crop.id}, Transformed: X={tx:.3f} m, Y={ty:.3f} m, Z={tz:.3f} m"
            )

        self.detected_crops_pub.publish(transformed_array)
        self.get_logger().info("Transformed DetectedCropArray published.")


def main(args=None):
    rclpy.init(args=args)
    node = CropTransformerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
