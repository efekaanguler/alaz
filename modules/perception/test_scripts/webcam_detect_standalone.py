#!/usr/bin/env python3
"""Standalone webcam + YOLOv8 + HSV Traffic Light — Apple Silicon optimised.

Pipeline:
  YOLOv8n  → Detect: person, car, bus, truck, motorbike, bicycle
  HSV+Contrast → Detect & classify 2-light pedestrian traffic lights (model-free)

Runs on Mac with NO Docker, NO ROS 2.
Uses CoreML (Apple GPU/Neural Engine) for fast inference.

Usage:
    pip install opencv-python onnxruntime numpy
    python3 webcam_detect_standalone.py

Press 'q' to quit.
"""

import argparse
import collections
import json
import math
import queue
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import cv2
import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    print("ERROR: pip install onnxruntime")
    raise SystemExit(1)

# ── Globals ──────────────────────────────────────────────────────────────────
_stop = threading.Event()
_TL_COLORS = {"red": (0, 0, 255), "green": (0, 220, 0)}

# ── Helpers ──────────────────────────────────────────────────────────────────

def make_session(path: str, threads: int = 4) -> ort.InferenceSession:
    opts = ort.SessionOptions()
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    opts.intra_op_num_threads = threads
    opts.inter_op_num_threads = 2
    providers = ort.get_available_providers()
    use = []
    if "CoreMLExecutionProvider" in providers:
        use.append("CoreMLExecutionProvider")
    use.append("CPUExecutionProvider")
    return ort.InferenceSession(str(path), sess_options=opts, providers=use)


def load_lines(p: str) -> List[str]:
    f = Path(p)
    return [l.strip() for l in f.read_text().splitlines() if l.strip()] if f.is_file() else []


def load_cmap(p: str) -> Dict[int, Tuple]:
    f = Path(p)
    if not f.is_file():
        return {}
    try:
        return {int(k): tuple(v) for k, v in json.loads(f.read_text()).items()}
    except Exception:
        return {}


# ── YOLOv8 ───────────────────────────────────────────────────────────────────

def _letterbox(img, sz):
    h, w = img.shape[:2]
    r = min(sz / h, sz / w)
    rw, rh = int(round(w * r)), int(round(h * r))
    pw, ph = (sz - rw) / 2.0, (sz - rh) / 2.0
    if (w, h) != (rw, rh):
        img = cv2.resize(img, (rw, rh), interpolation=cv2.INTER_LINEAR)
    t, b = int(round(ph - 0.1)), int(round(ph + 0.1))
    l2, r2 = int(round(pw - 0.1)), int(round(pw + 0.1))
    return cv2.copyMakeBorder(img, t, b, l2, r2,
                              cv2.BORDER_CONSTANT, value=(114, 114, 114)), r, pw, ph


def yolo_detect(sess, inp_name, frame, sz, sthr, nthr, allowed):
    """Run YOLOv8 detection (no traffic light — handled separately)."""
    img, ratio, pw, ph = _letterbox(frame, sz)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    blob = (rgb.astype(np.float32) / 255.0).transpose(2, 0, 1)[np.newaxis]

    pred = sess.run(None, {inp_name: blob})[0].squeeze()
    if pred.ndim == 1:
        pred = pred[np.newaxis, :]
    if pred.shape[0] == 84 and pred.shape[1] > 100:
        pred = pred.T

    boxes = pred[:, :4]
    cls_p = pred[:, 4:]
    cids = cls_p.argmax(1)
    scores = cls_p[np.arange(len(cids)), cids]

    mask = scores >= sthr
    if allowed:
        mask &= np.isin(cids, list(allowed))
    if not mask.any():
        return []

    boxes, scores, cids = boxes[mask], scores[mask], cids[mask]
    H, W = frame.shape[:2]
    cx, cy, bw, bh = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    x1 = np.clip((cx - bw/2 - pw) / ratio, 0, W-1)
    y1 = np.clip((cy - bh/2 - ph) / ratio, 0, H-1)
    x2 = np.clip((cx + bw/2 - pw) / ratio, 0, W-1)
    y2 = np.clip((cy + bh/2 - ph) / ratio, 0, H-1)

    nms_in = [[float(x1[i]), float(y1[i]),
               max(0, float(x2[i]-x1[i])), max(0, float(y2[i]-y1[i]))]
              for i in range(len(scores))]
    idxs = cv2.dnn.NMSBoxes(nms_in, scores.tolist(), sthr, nthr)
    if len(idxs) == 0:
        return []
    idxs = np.array(idxs).reshape(-1)
    return [(float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]),
             float(scores[i]), int(cids[i])) for i in idxs]


# ══════════════════════════════════════════════════════════════════════════════
# MODEL-FREE TRAFFIC LIGHT DETECTION (contrast-based)
#
# Key insight: A traffic light LED is a BRIGHT circle inside a DARK housing.
# This creates HIGH brightness contrast. Room walls, ceilings, and skin tones
# are uniformly lit → LOW contrast. Measuring this contrast is the trick.
# ══════════════════════════════════════════════════════════════════════════════

def _find_color_blobs(hsv, v_ch, s_ch, color_name, frame_h, frame_w):
    """Find circular blobs of a specific color with contrast measurement."""
    h_ch = hsv[:, :, 0]

    if color_name == "red":
        # Tight thresholds: real TL LEDs are very saturated and bright
        # Loose S/V thresholds catch skin, warm walls, orange objects
        mask = ((h_ch < 10) | (h_ch > 160)) & (s_ch > 120) & (v_ch > 140)
    elif color_name == "green":
        mask = (h_ch > 35) & (h_ch < 85) & (s_ch > 60) & (v_ch > 100)
    else:
        return []

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    cleaned = cv2.morphologyEx(mask.astype(np.uint8) * 255, cv2.MORPH_CLOSE, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blobs = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 300:  # TL blobs should be substantial, not tiny noise
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter < 1:
            continue
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        if circularity < 0.35:
            continue

        x, y, w, h_box = cv2.boundingRect(cnt)
        aspect = w / max(h_box, 1)
        if aspect < 0.3 or aspect > 3.0:
            continue
        if y + h_box / 2 > frame_h * 0.75:  # TLs are in upper portion of frame
            continue

        radius = max(w, h_box) / 2.0
        bcx = x + w / 2.0
        bcy = y + h_box / 2.0

        blob_v = v_ch[y:y+h_box, x:x+w]
        blob_s = s_ch[y:y+h_box, x:x+w]
        mean_b = float(blob_v.mean()) if blob_v.size > 0 else 0
        mean_s = float(blob_s.mean()) if blob_s.size > 0 else 0

        # ── CONTRAST: measure ring brightness around the blob ──
        expand = max(int(radius * 0.8), 5)
        ox1, oy1 = max(0, x - expand), max(0, y - expand)
        ox2, oy2 = min(frame_w, x + w + expand), min(frame_h, y + h_box + expand)

        outer = v_ch[oy1:oy2, ox1:ox2].copy().astype(np.float32)
        iy1, ix1 = y - oy1, x - ox1
        outer[iy1:iy1+h_box, ix1:ix1+w] = 0

        ring_mask = outer > 0
        ring_count = int(np.count_nonzero(ring_mask))
        ring_mean = float(outer[ring_mask].mean()) if ring_count > 5 else mean_b
        contrast = mean_b - ring_mean

        blobs.append({
            "cx": bcx, "cy": bcy, "radius": radius,
            "area": area, "circularity": circularity,
            "brightness": mean_b, "saturation": mean_s,
            "contrast": contrast,
            "bbox": (x, y, w, h_box), "color": color_name,
        })

    return blobs


def detect_traffic_lights(frame):
    """Detect 2-light pedestrian traffic lights — fully model-free.

    Uses contrast-based validation to eliminate false positives:
    real TL LEDs have HIGH contrast (bright in dark housing),
    room walls/ceilings have LOW contrast (uniform brightness).

    Phase 1: PAIR MATCHING (red above green) → highest confidence
    Phase 2: SINGLE BLOB + HIGH CONTRAST  → medium confidence

    Returns: list of (x1, y1, x2, y2, confidence, color_label) tuples.
    """
    H, W = frame.shape[:2]

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    v_ch = hsv[:, :, 2]
    s_ch = hsv[:, :, 1]

    red_blobs = _find_color_blobs(hsv, v_ch, s_ch, "red", H, W)
    green_blobs = _find_color_blobs(hsv, v_ch, s_ch, "green", H, W)

    results = []
    used_red = set()
    used_green = set()

    # ══════════════════════════════════════════════════════════════════
    # PHASE 1: PAIR MATCHING (highest confidence)
    # ══════════════════════════════════════════════════════════════════
    for ri, rb in enumerate(red_blobs):
        best_gi = -1
        best_score = 0

        for gi, gb in enumerate(green_blobs):
            if gi in used_green:
                continue
            if rb["cy"] >= gb["cy"]:
                continue

            avg_r = (rb["radius"] + gb["radius"]) / 2
            if avg_r < 1:
                continue

            h_dist = abs(rb["cx"] - gb["cx"])
            if h_dist > avg_r * 2.0:
                continue

            v_dist = gb["cy"] - rb["cy"]
            if v_dist < avg_r * 0.5 or v_dist > avg_r * 7.0:
                continue

            r_ratio = max(rb["radius"], gb["radius"]) / max(min(rb["radius"], gb["radius"]), 0.1)
            if r_ratio > 3.0:
                continue

            score = (1.0 - h_dist / (avg_r * 2.0)) * 0.5 + \
                    (1.0 - min((r_ratio - 1.0) / 2.0, 1.0)) * 0.5
            if score > best_score:
                best_score = score
                best_gi = gi

        if best_gi >= 0:
            gb = green_blobs[best_gi]
            avg_r = (rb["radius"] + gb["radius"]) / 2
            pad = avg_r * 0.6

            bx1 = int(max(0, min(rb["bbox"][0], gb["bbox"][0]) - pad))
            by1 = int(max(0, rb["bbox"][1] - pad))
            bx2 = int(min(W, max(rb["bbox"][0]+rb["bbox"][2], gb["bbox"][0]+gb["bbox"][2]) + pad))
            by2 = int(min(H, gb["bbox"][1] + gb["bbox"][3] + pad))

            color = "red" if rb["brightness"] > gb["brightness"] else "green"
            brighter = rb if color == "red" else gb
            conf = 0.75 + 0.10 * min(brighter["brightness"] / 200.0, 1.0) \
                        + 0.10 * min(brighter["circularity"], 1.0) \
                        + 0.05 * best_score

            results.append((bx1, by1, bx2, by2, min(round(conf, 2), 0.95), color))
            used_red.add(ri)
            used_green.add(best_gi)

    # ══════════════════════════════════════════════════════════════════
    # PHASE 2: SINGLE BLOB + CONTRAST (medium confidence)
    # ══════════════════════════════════════════════════════════════════
    MIN_CONTRAST = 50  # Must be clearly brighter than surroundings

    if not results:
        all_blobs = [b for i, b in enumerate(red_blobs) if i not in used_red] + \
                    [b for i, b in enumerate(green_blobs) if i not in used_green]

        for blob in all_blobs:
            if blob["contrast"] < MIN_CONTRAST:
                continue
            if blob["brightness"] < 150:  # LED must be genuinely bright
                continue
            if blob["saturation"] < 80:  # Must be colorful, not grayish
                continue

            pad = blob["radius"] * 1.5
            bx1 = int(max(0, blob["cx"] - pad))
            by1 = int(max(0, blob["cy"] - pad))
            bx2 = int(min(W, blob["cx"] + pad))
            by2 = int(min(H, blob["cy"] + pad))

            contrast_score = min(blob["contrast"] / 80.0, 1.0)
            conf = 0.40 + 0.25 * contrast_score \
                        + 0.10 * min(blob["brightness"] / 200.0, 1.0) \
                        + 0.10 * min(blob["circularity"], 1.0)

            results.append((bx1, by1, bx2, by2,
                            min(round(conf, 2), 0.80), blob["color"]))

    # NMS
    if len(results) > 1:
        boxes = [[r[0], r[1], r[2]-r[0], r[3]-r[1]] for r in results]
        scores = [r[4] for r in results]
        idxs = cv2.dnn.NMSBoxes(boxes, scores, 0.20, 0.3)
        if len(idxs) > 0:
            idxs = np.array(idxs).reshape(-1)
            results = [results[i] for i in idxs]
        else:
            results = []

    return results


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    root = Path(__file__).resolve().parent.parent

    ap = argparse.ArgumentParser()
    ap.add_argument("--device",    type=int,   default=0)
    ap.add_argument("--model",     default=str(root/"models"/"yolov8n.onnx"))
    ap.add_argument("--labels",    default=str(root/"models"/"labels.txt"))
    ap.add_argument("--color-map", default=str(root/"models"/"color_map.json"))
    ap.add_argument("--score-thr", type=float, default=0.35)
    ap.add_argument("--nms-thr",   type=float, default=0.40)
    ap.add_argument("--classes",   default="0,1,2,3,5,7",
                    help="0=person 1=bicycle 2=car 3=motorbike 5=bus 7=truck")
    a = ap.parse_args()

    allowed = {int(c) for c in a.classes.split(",") if c.strip()} or None
    yolo_labels = load_lines(a.labels)
    cmap = load_cmap(a.color_map) or {0:(56,56,255),2:(31,112,255)}

    yolo = make_session(a.model, 4)
    yn = yolo.get_inputs()[0].name
    print(f"[YOLO] {Path(a.model).name}  providers={yolo.get_providers()}")
    print(f"[YOLO] Classes: {sorted(allowed) if allowed else 'all'}")
    print("[TL]   Contrast-based detector (model-free, no YOLO)")

    cap = cv2.VideoCapture(a.device)
    if not cap.isOpened():
        print(f"ERROR: Can't open camera {a.device}"); raise SystemExit(1)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print(f"[CAM]  {int(cap.get(3))}x{int(cap.get(4))}")
    print("Press 'q' to quit.\n")

    lock = threading.Lock()
    shared_dets: List[Tuple] = []
    shared_raw: Optional[np.ndarray] = None
    det_fps = 0.0

    def worker():
        nonlocal shared_dets, shared_raw, det_fps
        q: queue.Queue = queue.Queue(maxsize=1)
        worker.q = q
        t0 = time.time()
        cnt = 0
        while not _stop.is_set():
            try:
                frame = q.get(timeout=0.05)
            except queue.Empty:
                continue
            dets = yolo_detect(yolo, yn, frame, 640, a.score_thr, a.nms_thr, allowed)
            with lock:
                shared_dets = dets
                shared_raw = frame
            cnt += 1
            now = time.time()
            if now - t0 >= 1.0:
                det_fps = cnt / (now - t0)
                cnt = 0
                t0 = now

    w = threading.Thread(target=worker, daemon=True)
    w.start()

    print("[YOLO] Warming up CoreML (first run may be slow)...")
    dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    yolo_detect(yolo, yn, dummy, 640, a.score_thr, a.nms_thr, allowed)
    print("[YOLO] Warmup done.\n")

    # ── Display loop ─────────────────────────────────────────────────────────
    disp_t = time.time()
    disp_cnt = 0
    disp_fps = 0.0

    TL_WINDOW = 1.5
    tl_history = collections.deque()
    tl_confirmed = None

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        try:
            worker.q.put_nowait(frame)
        except queue.Full:
            pass

        canvas = frame.copy()

        # ── 1. YOLO detections (person, car, etc.) ──
        with lock:
            yolo_dets = list(shared_dets)

        for x1, y1, x2, y2, score, cid in yolo_dets:
            ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)
            nm = yolo_labels[cid] if cid < len(yolo_labels) else f"c{cid}"
            color = cmap.get(cid, (56, 200, 56))
            txt = f"{nm} {score:.0%}"
            cv2.rectangle(canvas, (ix1, iy1), (ix2, iy2), color, 2)
            (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(canvas, (ix1, iy1-th-6), (ix1+tw+4, iy1), color, -1)
            cv2.putText(canvas, txt, (ix1+2, iy1-3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # ── 2. Traffic light detection (model-free, every frame) ──
        tl_dets = detect_traffic_lights(canvas)

        for tx1, ty1, tx2, ty2, tl_conf, raw_color in tl_dets:
            itx1, ity1, itx2, ity2 = int(tx1), int(ty1), int(tx2), int(ty2)

            # Temporal smoothing (asymmetric)
            now_t = time.time()
            tl_history.append((now_t, raw_color))
            while tl_history and tl_history[0][0] < now_t - TL_WINDOW:
                tl_history.popleft()

            if tl_confirmed is None:
                tl_confirmed = raw_color
            else:
                votes_r = sum(1 for _, lb in tl_history if lb == "red")
                votes_g = sum(1 for _, lb in tl_history if lb == "green")
                total_v = votes_r + votes_g
                if total_v > 0:
                    dominant = "red" if votes_r > votes_g else "green"
                    dom_ratio = max(votes_r, votes_g) / total_v
                    if dominant != tl_confirmed:
                        first_dom = next((t for t, lb in tl_history if lb == dominant), now_t)
                        elapsed = now_t - first_dom
                        switch_time = 0.5 if dominant == "red" else 1.0
                        if elapsed >= switch_time and dom_ratio > 0.6:
                            tl_confirmed = dominant

            draw_color = _TL_COLORS.get(tl_confirmed, (140, 140, 140))
            txt = f"TL {tl_confirmed} {tl_conf:.0%}"
            cv2.rectangle(canvas, (itx1, ity1), (itx2, ity2), draw_color, 3)
            (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(canvas, (itx1, ity1-th-8), (itx1+tw+6, ity1), draw_color, -1)
            cv2.putText(canvas, txt, (itx1+3, ity1-4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        # ── HUD ──
        n_total = len(yolo_dets) + len(tl_dets)
        disp_cnt += 1
        now = time.time()
        if now - disp_t >= 0.5:
            disp_fps = disp_cnt / (now - disp_t)
            disp_cnt = 0
            disp_t = now

        fH = canvas.shape[0]
        cv2.putText(canvas, f"Dets: {n_total}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 230), 2, cv2.LINE_AA)
        tl_status = tl_confirmed if tl_confirmed else "---"
        cv2.putText(canvas, f"TL: {tl_status}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    _TL_COLORS.get(tl_status, (200, 200, 200)), 2, cv2.LINE_AA)
        cv2.putText(canvas, f"Cam: {disp_fps:.0f} fps", (10, fH-12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"Det: {det_fps:.1f} fps", (10, fH-32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)

        cv2.imshow("Detection", canvas)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    _stop.set()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
