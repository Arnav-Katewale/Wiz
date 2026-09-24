import os

import cv2
import numpy as np

VOC_CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle",
    "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant", "sheep", "sofa", "train",
    "tvmonitor",
]

DEFAULT_MODEL_DIR = os.path.expanduser("~/Wiz/models/mobilenet_ssd")
DNN_INPUT_SIZE = (300, 300)
DNN_SCALE = 0.007843
DNN_MEAN = 127.5

_ORB = cv2.ORB_create(nfeatures=200)


def preprocess_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(gray, (5, 5), 0)


def detect_features(gray_frame):
    return _ORB.detect(gray_frame, None)


def load_detector(model_dir=None):
    model_dir = model_dir or DEFAULT_MODEL_DIR
    prototxt = os.path.join(model_dir, "deploy.prototxt")
    weights = os.path.join(model_dir, "mobilenet_iter_73000.caffemodel")
    if not (os.path.isfile(prototxt) and os.path.isfile(weights)):
        raise FileNotFoundError(
            f"MobileNet-SSD model files not found in {model_dir}. "
            "Run scripts/download_vision_models.sh first."
        )
    return cv2.dnn.readNetFromCaffe(prototxt, weights)


def detect_objects(net, frame, confidence_threshold=0.4):
    h, w = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(
        cv2.resize(frame, DNN_INPUT_SIZE), DNN_SCALE, DNN_INPUT_SIZE, DNN_MEAN
    )
    net.setInput(blob)
    raw = net.forward()

    results = []
    for i in range(raw.shape[2]):
        confidence = float(raw[0, 0, i, 2])
        if confidence < confidence_threshold:
            continue
        class_id = int(raw[0, 0, i, 1])
        label = VOC_CLASSES[class_id] if 0 <= class_id < len(VOC_CLASSES) else "unknown"
        box = raw[0, 0, i, 3:7] * np.array([w, h, w, h])
        x1, y1, x2, y2 = box.astype(int)
        x1, y1 = max(0, int(x1)), max(0, int(y1))
        x2, y2 = min(w - 1, int(x2)), min(h - 1, int(y2))
        if x2 <= x1 or y2 <= y1:
            continue
        results.append({
            "label": label,
            "confidence": confidence,
            "bbox": (x1, y1, x2 - x1, y2 - y1),
        })
    return results
