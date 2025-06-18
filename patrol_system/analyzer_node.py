import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np
import json
import os

# SIFT 생성
sift = cv2.SIFT_create()
bf = cv2.BFMatcher()

# 경로 설정
REFERENCE_DIR = 'logs/images/reference'
CURRENT_DIR = 'logs/images/current'
FAILURE_DIR = 'logs/images/failure'
RESULT_JSON_PATH = 'logs/result.json'

class AnalyzerNode(Node):
    def __init__(self):
        super().__init__('analyzer_node')
        self.get_logger().info('✅ Analyzer Node Started!')
        
        # Subscribe to image_analysis_request topic
        self.subscription = self.create_subscription(
            String,
            '/image_analysis_request',
            self.image_analysis_callback,
            10
        )
        
        # 결과 저장용 dict 초기화
        self.result_data = {}

    def image_analysis_callback(self, msg):
        # 메시지 예시: '{"point_id": 3, "reference_image": "reference_image_3.png", "target_image": "current_image_3.png", "is_failure": false}'
        data = json.loads(msg.data)
        
        point_id = data['point_id']
        reference_image_path = os.path.join(REFERENCE_DIR, data['reference_image'])
        
        if data['is_failure']:
            target_image_path = os.path.join(FAILURE_DIR, data['target_image'])
        else:
            target_image_path = os.path.join(CURRENT_DIR, data['target_image'])
        
        # 분석 실행
        anomaly, good_match_count = self.analyze_image_pair(reference_image_path, target_image_path, point_id)
        
        # 결과 업데이트
        self.result_data[f'point_{point_id}'] = {
            'anomaly': anomaly,
            'good_matches': good_match_count,
            'result_image_path': f'logs/images/result/point_{point_id}_result.png',
            'bbox_image_path': f'logs/images/result/point_{point_id}_bbox.png' if not anomaly else 'N/A'
        }
        
        # result.json 저장
        with open(RESULT_JSON_PATH, 'w') as f:
            json.dump(self.result_data, f, indent=4)
        
        self.get_logger().info(f'✅ Point {point_id} 분석 완료 → result.json 업데이트 완료')

    def analyze_image_pair(self, ref_path, target_path, point_id, threshold=10):
        self.get_logger().info(f'\n[Point {point_id}] 분석 시작...')
        
        ref_img = cv2.imread(ref_path, cv2.IMREAD_GRAYSCALE)
        target_img = cv2.imread(target_path, cv2.IMREAD_GRAYSCALE)
        
        kp1, des1 = sift.detectAndCompute(ref_img, None)
        kp2, des2 = sift.detectAndCompute(target_img, None)
        
        matches = bf.knnMatch(des1, des2, k=2)
        
        good_matches = []
        pts_target = []
        
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)
                pts_target.append(kp2[m.trainIdx].pt)
        
        self.get_logger().info(f'Good matches: {len(good_matches)}')
        
        # 이상 여부 판단
        if len(good_matches) < threshold:
            anomaly = True
            self.get_logger().warn(f'🚨 Anomaly detected at Point {point_id}!')
        else:
            anomaly = False
            self.get_logger().info(f'✅ Normal at Point {point_id}')
        
        # 결과 이미지 시각화 저장 (drawMatches)
        img_matches = cv2.drawMatches(ref_img, kp1, target_img, kp2, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        save_path_matches = f'logs/images/result/point_{point_id}_result.png'
        cv2.imwrite(save_path_matches, img_matches)
        self.get_logger().info(f'→ 결과 이미지 저장: {save_path_matches}')
        
        # Bounding Box 표시 (target 이미지에)
        if pts_target:
            pts_np = np.array(pts_target)
            x_min, y_min = np.min(pts_np, axis=0).astype(int)
            x_max, y_max = np.max(pts_np, axis=0).astype(int)
            
            target_img_color = cv2.cvtColor(target_img, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(target_img_color, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            
            save_path_bbox = f'logs/images/result/point_{point_id}_bbox.png'
            cv2.imwrite(save_path_bbox, target_img_color)
            self.get_logger().info(f'→ Bounding Box 이미지 저장: {save_path_bbox}')
        else:
            self.get_logger().warn(f'→ Good matches가 없어 Bounding Box 없음.')
        
        return anomaly, len(good_matches)

def main(args=None):
    rclpy.init(args=args)
    analyzer_node = AnalyzerNode()
    rclpy.spin(analyzer_node)
    analyzer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
