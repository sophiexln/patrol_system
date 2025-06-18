import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import yaml

class ReferenceImageCapture(Node):
    def __init__(self):
        super().__init__('reference_image_capture')

        # 카메라 토픽 subscribe
        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',  # 사용 중인 카메라 토픽 이름 (본인 환경에 맞게 확인)
            self.image_callback,
            10
        )

        # AMCL pose 토픽 subscribe
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.bridge = CvBridge()
        self.current_pose = None  # pose 최신 값 저장

        # 저장 경로
        self.image_dir = os.path.expanduser('~/ros2_ws/src/patrol_system/logs/images/reference')
        os.makedirs(self.image_dir, exist_ok=True)

        self.pose_dir = os.path.expanduser('~/ros2_ws/src/patrol_system/logs/poses/reference')
        os.makedirs(self.pose_dir, exist_ok=True)

        self.get_logger().info('📸 Reference Image Capture Node Started')

    def pose_callback(self, msg):
        # Pose 최신 값 저장
        self.current_pose = msg.pose.pose

    def image_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().warn('Pose not yet received, skipping image save.')
            return

        # 현재 시간
        now = datetime.now().strftime("%Y%m%d-%H%M%S")

        # 이미지 저장
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_filename = f'reference_image_{now}.png'
        image_path = os.path.join(self.image_dir, image_filename)
        cv2.imwrite(image_path, cv_image)
        self.get_logger().info(f'✅ Saved reference image: {image_path}')

        # Pose 저장 (yaml 형식)
        pose_data = {
            'position': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'z': self.current_pose.position.z
            },
            'orientation': {
                'x': self.current_pose.orientation.x,
                'y': self.current_pose.orientation.y,
                'z': self.current_pose.orientation.z,
                'w': self.current_pose.orientation.w
            }
        }

        pose_filename = f'reference_pose_{now}.yaml'
        pose_path = os.path.join(self.pose_dir, pose_filename)
        with open(pose_path, 'w') as f:
            yaml.dump(pose_data, f)

        self.get_logger().info(f'✅ Saved reference pose: {pose_path}')

def main(args=None):
    rclpy.init(args=args)
    node = ReferenceImageCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
