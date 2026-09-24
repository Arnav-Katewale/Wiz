import os

import numpy as np
import pytest

from wiz_vision.processing import (
    preprocess_frame, detect_features, load_detector, detect_objects, DEFAULT_MODEL_DIR,
)

FIXTURE_DIR = os.path.join(os.path.dirname(__file__), 'fixtures')
SAMPLE_IMAGE = os.path.join(FIXTURE_DIR, 'sample_person.jpg')
MODEL_AVAILABLE = os.path.isdir(DEFAULT_MODEL_DIR) and os.path.isfile(
    os.path.join(DEFAULT_MODEL_DIR, 'mobilenet_iter_73000.caffemodel')
)


def _checkerboard(size=200, squares=8):
    img = np.zeros((size, size, 3), dtype=np.uint8)
    step = size // squares
    for i in range(squares):
        for j in range(squares):
            if (i + j) % 2 == 0:
                img[i * step:(i + 1) * step, j * step:(j + 1) * step] = 255
    return img


def test_preprocess_frame_shape_and_dtype():
    frame = _checkerboard()
    result = preprocess_frame(frame)
    assert result.shape == frame.shape[:2]
    assert result.dtype == np.uint8


def test_detect_features_finds_corners_on_checkerboard():
    frame = _checkerboard()
    gray = preprocess_frame(frame)
    keypoints = detect_features(gray)
    assert len(keypoints) > 0


def test_detect_features_finds_nothing_on_blank_frame():
    blank = np.full((200, 200, 3), 128, dtype=np.uint8)
    gray = preprocess_frame(blank)
    keypoints = detect_features(gray)
    assert len(keypoints) == 0


def test_load_detector_raises_if_model_missing(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_detector(str(tmp_path))


@pytest.mark.skipif(not MODEL_AVAILABLE, reason='MobileNet-SSD model not downloaded (run scripts/download_vision_models.sh)')
def test_detect_objects_finds_person_in_sample_image():
    import cv2
    net = load_detector()
    frame = cv2.imread(SAMPLE_IMAGE)
    assert frame is not None
    results = detect_objects(net, frame, confidence_threshold=0.4)
    labels = [r['label'] for r in results]
    assert 'person' in labels


@pytest.mark.skipif(not MODEL_AVAILABLE, reason='MobileNet-SSD model not downloaded (run scripts/download_vision_models.sh)')
def test_detect_objects_returns_well_formed_results_on_synthetic_frame():
    net = load_detector()
    frame = _checkerboard()
    results = detect_objects(net, frame, confidence_threshold=0.4)
    for r in results:
        assert isinstance(r['label'], str)
        assert 0.0 <= r['confidence'] <= 1.0
        x, y, w, h = r['bbox']
        assert w > 0 and h > 0
