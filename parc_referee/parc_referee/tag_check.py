"""Checks whether a visit image contains the claimed AprilTag (36h11).

The result is only shown to the judges; it never rejects a visit by itself.
"""

try:
    import cv2
    import numpy as np
    _DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        _DETECTOR = cv2.aruco.ArucoDetector(_DICT, cv2.aruco.DetectorParameters())

        def _detect(gray):
            return _DETECTOR.detectMarkers(gray)[1]
    else:                                             # older OpenCV (e.g. JetPack builds)
        _PARAMS = cv2.aruco.DetectorParameters_create()

        def _detect(gray):
            return cv2.aruco.detectMarkers(gray, _DICT, parameters=_PARAMS)[1]
    AVAILABLE = True
except Exception:  # OpenCV or its aruco module missing
    AVAILABLE = False


def check_tag(image_bytes, tag_id):
    """Return (result, detected_ids).

    result is 'match', 'mismatch', 'not_detected', 'no_image' or 'unchecked'.
    """
    if not image_bytes:
        return 'no_image', []
    if not AVAILABLE:
        return 'unchecked', []
    try:
        img = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None:
            return 'unchecked', []
        ids = _detect(img)
        detected = sorted(int(i) for i in ids.flatten()) if ids is not None else []
    except Exception:
        return 'unchecked', []
    if tag_id in detected:
        return 'match', detected
    return ('mismatch' if detected else 'not_detected'), detected
