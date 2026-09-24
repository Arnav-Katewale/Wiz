from wiz_vision.tracking import CentroidTracker


def _det(x, y, w=20, h=20, label='object', confidence=0.9):
    return {'label': label, 'confidence': confidence, 'bbox': (x, y, w, h)}


def test_new_detection_gets_registered():
    tracker = CentroidTracker()
    objects = tracker.update([_det(10, 10)])
    assert len(objects) == 1


def test_track_id_persists_as_object_moves_smoothly():
    tracker = CentroidTracker()
    tracker.update([_det(10, 10)])
    (first_id,) = tracker.objects.keys()

    for step in range(1, 6):
        tracker.update([_det(10 + step * 5, 10)])

    assert list(tracker.objects.keys()) == [first_id]
    cx, _cy = tracker.objects[first_id][:2]
    assert cx == 10 + 5 * 5 + 10


def test_track_is_dropped_after_max_disappeared():
    tracker = CentroidTracker(max_disappeared=2)
    tracker.update([_det(10, 10)])
    tracker.update([])
    tracker.update([])
    tracker.update([])
    assert len(tracker.objects) == 0


def test_two_distinct_objects_get_separate_ids():
    tracker = CentroidTracker()
    objects = tracker.update([_det(10, 10), _det(200, 200)])
    assert len(objects) == 2
