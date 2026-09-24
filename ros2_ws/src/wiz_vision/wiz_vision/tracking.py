import itertools

import numpy as np


class CentroidTracker:

    def __init__(self, max_disappeared=10):
        self._next_id = itertools.count(1)
        self.max_disappeared = max_disappeared
        self.objects = {}
        self.disappeared = {}

    def update(self, detections):
        if not detections:
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self._deregister(object_id)
            return self.objects

        input_centroids = []
        for det in detections:
            x, y, w, h = det["bbox"]
            input_centroids.append((x + w // 2, y + h // 2))

        if not self.objects:
            for centroid, det in zip(input_centroids, detections):
                self._register(centroid, det)
            return self.objects

        object_ids = list(self.objects.keys())
        object_centroids = [(v[0], v[1]) for v in self.objects.values()]

        d = self._distance_matrix(object_centroids, input_centroids)
        rows = d.min(axis=1).argsort()
        cols = d.argmin(axis=1)[rows]

        used_rows, used_cols = set(), set()
        for row, col in zip(rows, cols):
            if row in used_rows or col in used_cols:
                continue
            object_id = object_ids[row]
            cx, cy = input_centroids[col]
            det = detections[col]
            self.objects[object_id] = (cx, cy, det["bbox"], det["label"], det["confidence"])
            self.disappeared[object_id] = 0
            used_rows.add(row)
            used_cols.add(col)

        unused_rows = set(range(len(object_centroids))) - used_rows
        for row in unused_rows:
            object_id = object_ids[row]
            self.disappeared[object_id] += 1
            if self.disappeared[object_id] > self.max_disappeared:
                self._deregister(object_id)

        unused_cols = set(range(len(input_centroids))) - used_cols
        for col in unused_cols:
            self._register(input_centroids[col], detections[col])

        return self.objects

    def _register(self, centroid, det):
        object_id = next(self._next_id)
        self.objects[object_id] = (centroid[0], centroid[1], det["bbox"], det["label"], det["confidence"])
        self.disappeared[object_id] = 0

    def _deregister(self, object_id):
        del self.objects[object_id]
        del self.disappeared[object_id]

    @staticmethod
    def _distance_matrix(a, b):
        a = np.array(a, dtype=float)
        b = np.array(b, dtype=float)
        return np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
