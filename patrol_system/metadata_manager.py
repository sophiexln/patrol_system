# patrol_system/metadata_manager.py

import os
import yaml

class MetadataManager:
    def __init__(self, metadata_file_path):
        self.metadata_file = metadata_file_path
        os.makedirs(os.path.dirname(self.metadata_file), exist_ok=True)
        
        # 기존 기록이 있으면 로드, 없으면 빈 리스트
        if os.path.exists(self.metadata_file):
            with open(self.metadata_file, 'r') as f:
                try:
                    data = yaml.safe_load(f)
                    self.records = data.get('captures', [])
                except Exception:
                    self.records = []
        else:
            self.records = []

    def add_record(self, waypoint_id, timestamp, position, orientation, capture_status, image_filename):
        record = {
            'waypoint_id': waypoint_id,
            'timestamp': timestamp,
            'position': {
                'x': position.x,
                'y': position.y,
                'z': position.z
            },
            'orientation': {
                'x': orientation.x,
                'y': orientation.y,
                'z': orientation.z,
                'w': orientation.w
            },
            'capture_status': capture_status,
            'image_filename': image_filename
        }
        self.records.append(record)
        self._save()

    def _save(self):
        with open(self.metadata_file, 'w') as f:
            yaml.dump({'captures': self.records}, f)
