#!/usr/bin/env python3

from dataclasses import dataclass
from threading import Lock
from typing import Dict, List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


@dataclass
class CandidateDetection:
    bbox: np.ndarray  # [x1, y1, x2, y2]
    score: float
    class_id: str


@dataclass
class Track:
    track_id: int
    bbox: np.ndarray
    score: float
    class_id: str
    age: int = 1
    hits: int = 1
    time_since_update: int = 0


def _bbox_iou(a: np.ndarray, b: np.ndarray) -> float:
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])

    iw = max(0.0, x2 - x1)
    ih = max(0.0, y2 - y1)
    inter = iw * ih

    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])

    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def _bbox_from_detection(det: Detection2D) -> np.ndarray:
    if hasattr(det.bbox.center, 'position'):
        cx = float(det.bbox.center.position.x)
        cy = float(det.bbox.center.position.y)
    else:
        cx = float(det.bbox.center.x)
        cy = float(det.bbox.center.y)

    w = float(det.bbox.size_x)
    h = float(det.bbox.size_y)

    x1 = cx - w * 0.5
    y1 = cy - h * 0.5
    x2 = cx + w * 0.5
    y2 = cy + h * 0.5

    return np.array([x1, y1, x2, y2], dtype=np.float32)


def _extract_score_and_class(det: Detection2D) -> Tuple[float, str]:
    if len(det.results) == 0:
        return 0.0, '0'

    result = det.results[0]
    if hasattr(result, 'hypothesis'):
        hyp = result.hypothesis
        score = float(getattr(hyp, 'score', 0.0))
        if hasattr(hyp, 'class_id'):
            class_id = str(hyp.class_id)
        elif hasattr(hyp, 'id'):
            class_id = str(hyp.id)
        else:
            class_id = '0'
        return score, class_id

    score = float(getattr(result, 'score', 0.0))
    if hasattr(result, 'class_id'):
        class_id = str(result.class_id)
    elif hasattr(result, 'id'):
        class_id = str(result.id)
    else:
        class_id = '0'
    return score, class_id


def _set_bbox(det: Detection2D, bbox: np.ndarray) -> None:
    x1, y1, x2, y2 = bbox.tolist()
    cx = (x1 + x2) * 0.5
    cy = (y1 + y2) * 0.5
    w = max(0.0, x2 - x1)
    h = max(0.0, y2 - y1)

    det.bbox.size_x = float(w)
    det.bbox.size_y = float(h)

    if hasattr(det.bbox.center, 'position'):
        det.bbox.center.position.x = float(cx)
        det.bbox.center.position.y = float(cy)
    else:
        det.bbox.center.x = float(cx)
        det.bbox.center.y = float(cy)

    if hasattr(det.bbox.center, 'theta'):
        det.bbox.center.theta = 0.0


def _set_hypothesis(result: ObjectHypothesisWithPose, class_id: str, score: float) -> None:
    if hasattr(result, 'hypothesis'):
        if hasattr(result.hypothesis, 'class_id'):
            result.hypothesis.class_id = str(class_id)
        elif hasattr(result.hypothesis, 'id'):
            try:
                result.hypothesis.id = int(class_id)
            except ValueError:
                result.hypothesis.id = 0
        result.hypothesis.score = float(score)
    else:
        if hasattr(result, 'class_id'):
            result.class_id = str(class_id)
        elif hasattr(result, 'id'):
            try:
                result.id = int(class_id)
            except ValueError:
                result.id = 0
        if hasattr(result, 'score'):
            result.score = float(score)


class ByteTrackNode(Node):
    def __init__(self) -> None:
        super().__init__('bytetrack_node')

        self.declare_parameter('track_high_thresh', 0.5)
        self.declare_parameter('track_low_thresh', 0.1)
        self.declare_parameter('new_track_thresh', 0.6)
        self.declare_parameter('match_thresh', 0.7)
        self.declare_parameter('track_buffer', 30)
        self.declare_parameter('min_hits', 1)
        self.declare_parameter('enable_visualizer', False)

        self.track_high_thresh = float(self.get_parameter('track_high_thresh').get_parameter_value().double_value)
        self.track_low_thresh = float(self.get_parameter('track_low_thresh').get_parameter_value().double_value)
        self.new_track_thresh = float(self.get_parameter('new_track_thresh').get_parameter_value().double_value)
        self.match_thresh = float(self.get_parameter('match_thresh').get_parameter_value().double_value)
        self.track_buffer = int(self.get_parameter('track_buffer').get_parameter_value().integer_value)
        self.min_hits = int(self.get_parameter('min_hits').get_parameter_value().integer_value)
        self.enable_visualizer = bool(self.get_parameter('enable_visualizer').get_parameter_value().bool_value)

        self.bridge = CvBridge()
        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1

        self.latest_image = None
        self.image_lock = Lock()

        self.pub = self.create_publisher(Detection2DArray, '~/output/tracked_rect', 10)
        self.sub_det = self.create_subscription(
            Detection2DArray,
            '~/input/detection_rect',
            self._on_detections,
            10,
        )
        self.sub_img = self.create_subscription(
            Image, '~/input/image', self._on_image, qos_profile_sensor_data
        )

        self.pub_debug = None
        if self.enable_visualizer:
            self.pub_debug = self.create_publisher(Image, '~/debug/image', 10)

        self.get_logger().info(
            f'ByteTrack-like tracker started. high={self.track_high_thresh}, low={self.track_low_thresh}, '
            f'new={self.new_track_thresh}, match={self.match_thresh}, buffer={self.track_buffer}, '
            f'min_hits={self.min_hits}, visualizer={self.enable_visualizer}'
        )

    def _on_image(self, msg: Image) -> None:
        if not self.enable_visualizer:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:  # pylint: disable=broad-except
            return

        with self.image_lock:
            self.latest_image = (msg.header, image)

    def _on_detections(self, msg: Detection2DArray) -> None:
        dets = self._parse_detections(msg)
        self._step_tracks(dets)
        tracked = self._to_detection_array(msg)
        self.pub.publish(tracked)

        if self.enable_visualizer and self.pub_debug is not None:
            self._publish_debug_image(msg.header)

    def _parse_detections(self, msg: Detection2DArray) -> List[CandidateDetection]:
        out: List[CandidateDetection] = []

        for det in msg.detections:
            bbox = _bbox_from_detection(det)
            score, class_id = _extract_score_and_class(det)
            out.append(CandidateDetection(bbox=bbox, score=score, class_id=class_id))

        return out

    def _associate(self, track_ids: List[int], detections: List[CandidateDetection], match_thresh: float) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        if len(track_ids) == 0 or len(detections) == 0:
            return [], track_ids.copy(), list(range(len(detections)))

        pairs = []
        for tid in track_ids:
            track = self.tracks[tid]
            for det_idx, det in enumerate(detections):
                iou = _bbox_iou(track.bbox, det.bbox)
                if iou >= match_thresh:
                    pairs.append((iou, tid, det_idx))

        pairs.sort(key=lambda x: x[0], reverse=True)

        used_tracks = set()
        used_dets = set()
        matches: List[Tuple[int, int]] = []

        for _, tid, didx in pairs:
            if tid in used_tracks or didx in used_dets:
                continue
            used_tracks.add(tid)
            used_dets.add(didx)
            matches.append((tid, didx))

        unmatched_tracks = [tid for tid in track_ids if tid not in used_tracks]
        unmatched_dets = [idx for idx in range(len(detections)) if idx not in used_dets]

        return matches, unmatched_tracks, unmatched_dets

    def _step_tracks(self, detections: List[CandidateDetection]) -> None:
        for track in self.tracks.values():
            track.age += 1
            track.time_since_update += 1

        high_dets = [d for d in detections if d.score >= self.track_high_thresh]
        low_dets = [d for d in detections if self.track_low_thresh <= d.score < self.track_high_thresh]

        all_track_ids = list(self.tracks.keys())

        high_matches, unmatched_track_ids, unmatched_high_idxs = self._associate(
            all_track_ids,
            high_dets,
            self.match_thresh,
        )

        for tid, didx in high_matches:
            self._update_track(tid, high_dets[didx])

        # Second-stage association with low-score detections (ByteTrack core idea).
        low_matches, still_unmatched_track_ids, _ = self._associate(
            unmatched_track_ids,
            low_dets,
            self.match_thresh,
        )

        for tid, didx in low_matches:
            self._update_track(tid, low_dets[didx])

        for didx in unmatched_high_idxs:
            cand = high_dets[didx]
            if cand.score >= self.new_track_thresh:
                self._create_track(cand)

        stale_ids = [tid for tid, t in self.tracks.items() if t.time_since_update > self.track_buffer]
        for tid in stale_ids:
            del self.tracks[tid]

        # Keep logic explicit so future policy changes can use this list if needed.
        _ = still_unmatched_track_ids

    def _update_track(self, tid: int, det: CandidateDetection) -> None:
        track = self.tracks[tid]
        track.bbox = det.bbox.copy()
        track.score = det.score
        track.class_id = det.class_id
        track.hits += 1
        track.time_since_update = 0

    def _create_track(self, det: CandidateDetection) -> None:
        tid = self.next_track_id
        self.next_track_id += 1

        self.tracks[tid] = Track(
            track_id=tid,
            bbox=det.bbox.copy(),
            score=det.score,
            class_id=det.class_id,
        )

    def _to_detection_array(self, source: Detection2DArray) -> Detection2DArray:
        out = Detection2DArray()
        out.header = source.header

        for tid in sorted(self.tracks.keys()):
            track = self.tracks[tid]
            if track.time_since_update != 0:
                continue
            if track.hits < self.min_hits:
                continue

            det = Detection2D()
            det.header = source.header
            _set_bbox(det, track.bbox)

            result = ObjectHypothesisWithPose()
            _set_hypothesis(result, track.class_id, track.score)
            det.results.append(result)

            if hasattr(det, 'id'):
                det.id = str(track.track_id)

            out.detections.append(det)

        return out

    def _publish_debug_image(self, header) -> None:
        with self.image_lock:
            if self.latest_image is None:
                return
            _, image = self.latest_image
            debug = image.copy()

        for tid in sorted(self.tracks.keys()):
            track = self.tracks[tid]
            if track.time_since_update != 0:
                continue
            if track.hits < self.min_hits:
                continue

            x1, y1, x2, y2 = track.bbox.astype(int).tolist()
            color = (0, 255, 0)
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            label = f'id={tid} cls={track.class_id} score={track.score:.2f}'
            cv2.putText(debug, label, (x1, max(0, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        msg.header = header
        self.pub_debug.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ByteTrackNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
