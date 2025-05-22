########## YoloModel ##########
import os
import json
import time
from collections import Counter

import rclpy
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np


PACKAGE_NAME = 'pick_and_place_text'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

YOLO_MODEL_FILENAME = 'yolov8n_tools_0122.pt'
YOLO_CLASS_NAME_JSON = "class_name_tool.json"

YOLO_MODEL_PATH = os.path.join(PACKAGE_PATH, 'resource', YOLO_MODEL_FILENAME)
YOLO_JSON_PATH = os.path.join(PACKAGE_PATH, 'resource', YOLO_CLASS_NAME_JSON)

class YoloModel():
    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)
        with open(YOLO_JSON_PATH, "r", encoding="utf-8") as file:
            class_dict = json.load(file)
            self.reversed_class_dict = {v: int(k) for k, v in class_dict.items()}

    def get_frames(self, img_node, duration=1.0):
        """get frames while target_time"""
        end_time = time.time() + duration
        frames = {}

        while time.time() < end_time:
            rclpy.spin_once(img_node)
            frame = img_node.get_color_frame()
            stamp = img_node.get_color_frame_stamp()
            if frame is not None:
                frames[stamp] = frame
            time.sleep(0.01)

        if not frames:
            print("No frames captured in %.2f seconds", duration)

        print("%d frames captured", len(frames))
        return list(frames.values())

    def get_best_detection(self, img_node, target):
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:  # Check if frames are empty
            return None

        results = self.model(frames, verbose=False)
        detections = self._aggregate_detections(results)
        label_id = self.reversed_class_dict[target]

        matches = [d for d in detections if d["label"] == label_id]
        if not matches:
            print("No matches found for the target label.")
            return None, None
        best_det = max(matches, key=lambda x: x["score"])
        return best_det["box"], best_det["score"]

    def _aggregate_detections(self, results, confidence_threshold=0.5, iou_threshold=0.5):
        """
        Fuse raw detection boxes across frames using IoU-based grouping
        and majority voting for robust final detections.
        """
        raw = []
        for res in results:
            for box, score, label in zip(
                res.boxes.xyxy.tolist(),
                res.boxes.conf.tolist(),
                res.boxes.cls.tolist(),
            ):
                if score >= confidence_threshold:
                    raw.append({"box": box, "score": score, "label": int(label)})

        final = []
        used = [False] * len(raw)

        for i, det in enumerate(raw):
            if used[i]:
                continue
            group = [det]
            used[i] = True
            for j, other in enumerate(raw):
                if not used[j] and other["label"] == det["label"]:
                    if self._iou(det["box"], other["box"]) >= iou_threshold:
                        group.append(other)
                        used[j] = True

            boxes = np.array([g["box"] for g in group])
            scores = np.array([g["score"] for g in group])
            labels = [g["label"] for g in group]

            final.append({
                "box": boxes.mean(axis=0).tolist(),
                "score": float(scores.mean()),
                "label": Counter(labels).most_common(1)[0][0],
            })

        return final

    
    def _iou(self, box1, box2):
        """
        Compute Intersection over Union (IoU) between two boxes [x1, y1, x2, y2].
        """
        x1, y1 = max(box1[0], box2[0]), max(box1[1], box2[1])
        x2, y2 = min(box1[2], box2[2]), min(box1[3], box2[3])
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0.0