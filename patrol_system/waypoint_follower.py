# patrol_system/waypoint_follower.py (최종본 - 촬영지점 접근 실패 처리 + 덮어쓰기 방지 적용 + 메타데이터 기록 추가)

import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf_transformations
import yaml
import os
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
from datetime import datetime
from patrol_system.metadata_manager import MetadataManager


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Action Client 준비
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # 카메라 토픽 구독
        self.bridge = CvBridge()
        self.current_image = None
        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)

        # 현재 Pose 구독
        self.current_pose = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Waypoints 로드
        self.waypoints = self.load_waypoints(os.path.expanduser('~/ros2_ws/src/patrol_system/config/waypoint.yaml'))
        self.get_logger().info(f'{len(self.waypoints)}개의 Waypoints 로드됨')

        # logs/images 디렉토리 준비
        self.image_save_path = os.path.expanduser('~/ros2_ws/src/patrol_system/logs/images/current')
        os.makedirs(self.image_save_path, exist_ok=True)

        self.failure_image_save_path = os.path.expanduser('~/ros2_ws/src/patrol_system/logs/images/failure')
        os.makedirs(self.failure_image_save_path, exist_ok=True)

        # 메타데이터 매니저 준비 (여기 추가!!)
        self.metadata_manager = MetadataManager(
            os.path.expanduser('~/ros2_ws/src/patrol_system/logs/metadata/capture_metadata.yaml')
        )

        # Waypoint 별 상태 관리
        self.saved_flags = {}          # 정상 도착 후 이미지 저장 여부
        self.retry_counts = {}         # Waypoint 별 retry count
        self.timeout_counters = {}     # Waypoint 별 timeout counter (feedback 기준)

        # 파라미터 설정
        self.max_retry = 2             # 최대 retry 횟수
        self.timeout_threshold = 50    # feedback callback 호출 횟수로 timeout 판단 (~50 * 0.1초 = 약 5초 정도 유효)

    def load_waypoints(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        waypoints_data = data['waypoints']
        waypoints = []
        for wp_entry in waypoints_data:
            pose_data = wp_entry['pose']
            waypoint = {
                'x': pose_data['x'],
                'y': pose_data['y'],
                'yaw': pose_data['theta']
            }
            waypoints.append(waypoint)

        return waypoints

    def yaw_to_quaternion(self, yaw):
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        from geometry_msgs.msg import Quaternion
        quat = Quaternion()
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]
        return quat

    def send_goal(self):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = []

        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.orientation = self.yaw_to_quaternion(wp['yaw'])

            goal_msg.poses.append(pose)

        self._action_client.wait_for_server()
        self.get_logger().info('Sending FollowWaypoints goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('FollowWaypoints goal rejected')
            return

        self.get_logger().info('FollowWaypoints goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'현재 Waypoint {current_waypoint} 주행 중')

        # Waypoint별 상태 초기화 (처음 접근 시)
        if current_waypoint not in self.saved_flags:
            self.saved_flags[current_waypoint] = False
            self.retry_counts[current_waypoint] = 0
            self.timeout_counters[current_waypoint] = 0

        # 현재 pose 기반으로 Waypoint 도착 여부 판단
        if self.current_pose is not None:
            target_wp = self.waypoints[current_waypoint]
            dx = self.current_pose.position.x - target_wp['x']
            dy = self.current_pose.position.y - target_wp['y']
            distance = math.sqrt(dx * dx + dy * dy)

            self.get_logger().info(f'Distance to Waypoint {current_waypoint}: {distance:.2f} m')

            # 도착 시 → 정상 current image 저장
            if distance < 0.3 and not self.saved_flags[current_waypoint]:
                if self.current_image is not None:
                    now = datetime.now().strftime("%Y%m%d-%H%M%S")
                    image_filename = os.path.join(self.image_save_path, f'current_{current_waypoint}_{now}.jpg')
                    cv2.imwrite(image_filename, self.current_image)
                    self.get_logger().info(f'✅ 이미지 저장됨 (정상 도착): {image_filename}')
                    
                    # 메타데이터 기록 (추가!!)
                    self.metadata_manager.add_record(
                        waypoint_id=current_waypoint,
                        timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        position=self.current_pose.position,
                        orientation=self.current_pose.orientation,
                        capture_status='success',
                        image_filename=image_filename.split('/')[-1]
                    )
                    
                    self.saved_flags[current_waypoint] = True
                else:
                    self.get_logger().warn('⚠️ 이미지 없음 - 저장 실패')

            # Timeout 처리 → 일정 시간 이상 거리 유지 시 retry 시도
            else:
                self.timeout_counters[current_waypoint] += 1

                if self.timeout_counters[current_waypoint] > self.timeout_threshold:
                    if self.retry_counts[current_waypoint] < self.max_retry:
                        self.retry_counts[current_waypoint] += 1
                        self.timeout_counters[current_waypoint] = 0
                        self.get_logger().warn(f'⚠️ Waypoint {current_waypoint} 접근 실패 - 재시도 {self.retry_counts[current_waypoint]}/{self.max_retry}')
                        # 재시도 시 → FollowWaypoints 자체가 내부적으로 waypoint 유지하므로 추가 조치는 필요 없음 (계속 feedback 받음)
                    else:
                        # 최대 재시도 초과 시 → failure image 저장 후 다음 waypoint로 넘어감
                        if not self.saved_flags[current_waypoint]:
                            if self.current_image is not None:
                                now = datetime.now().strftime("%Y%m%d-%H%M%S")
                                failure_image_filename = os.path.join(self.failure_image_save_path, f'failure_{current_waypoint}_{now}.jpg')
                                cv2.imwrite(failure_image_filename, self.current_image)
                                self.get_logger().warn(f'❌ Waypoint {current_waypoint} 접근 실패 - Failure 이미지 저장됨: {failure_image_filename}')
                                
                                # 메타데이터 기록 (추가!!)
                                self.metadata_manager.add_record(
                                    waypoint_id=current_waypoint,
                                    timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                                    position=self.current_pose.position,
                                    orientation=self.current_pose.orientation,
                                    capture_status='failure',
                                    image_filename=failure_image_filename.split('/')[-1]
                                )
                            else:
                                self.get_logger().warn('⚠️ 이미지 없음 - Failure 저장 실패')

                            self.saved_flags[current_waypoint] = True  # 실패 처리 완료 표시 → 다음 waypoint 진행 유도

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('✅ FollowWaypoints 주행 완료!')
        rclpy.shutdown()

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    node.send_goal()
    rclpy.spin(node)
    node.destroy_node()
    # rclpy.shutdown() → get_result_callback에서 이미 shutdown 처리

if __name__ == '__main__':
    main()
