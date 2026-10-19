"""USB camera relay served by auto_dem_http_server.py.

The relay is deliberately thin: the controlling PC keeps every validation and
verification rule, and only the capture object (cv2.VideoCapture) lives here,
next to the hardware. ``handle(method, path, body)`` answers ``(status, payload)``,
where the payload is a JSON-able dict or ``(bytes, headers)`` for a raw frame.

Python 3.8 standard library, plus OpenCV/numpy from the system (python3-opencv).
"""

import threading


class RelayError(Exception):
    def __init__(self, status, message):
        Exception.__init__(self, message)
        self.status = status


def _require(body, key, kind):
    value = body.get(key) if isinstance(body, dict) else None
    if not isinstance(value, kind) or isinstance(value, bool) and kind is not bool:
        raise RelayError(400, "%s is required (%s)" % (key, getattr(kind, "__name__", kind)))
    return value


def load_cv2():
    import cv2
    import numpy  # noqa: F401 - frames are numpy arrays; fail at startup, not on the first read

    return cv2


class CameraRelay:
    """One cv2.VideoCapture, driven property by property by the controlling PC."""

    def __init__(self, cv2_module):
        self._cv2 = cv2_module
        self._lock = threading.Lock()
        self._cap = None

    def handle(self, method, path, body):
        if method != "POST":
            raise RelayError(404, "not found: %s %s" % (method, path))
        with self._lock:
            if path == "/camera/open":
                return self._open(body)
            if path == "/camera/release":
                self._release()
                return 200, {"released": True}
            cap = self._cap
            if cap is None:
                raise RelayError(409, "camera is not open; POST /camera/open first")
            if path == "/camera/set":
                prop = _require(body, "prop", int)
                value = _require(body, "value", (int, float))
                return 200, {"ok": bool(cap.set(prop, float(value)))}
            if path == "/camera/get":
                return 200, {"value": float(cap.get(_require(body, "prop", int)))}
            if path == "/camera/grab":
                return 200, {"ok": bool(cap.grab())}
            if path == "/camera/read":
                ok, frame = cap.read()
                if not ok or frame is None:
                    return 200, {"ok": False}
                headers = {
                    "X-Frame-Shape": ",".join(str(n) for n in frame.shape),
                    "X-Frame-Dtype": frame.dtype.str,
                }
                return 200, (frame.tobytes(), headers)
        raise RelayError(404, "not found: %s %s" % (method, path))

    def _open(self, body):
        source = body.get("source") if isinstance(body, dict) else None
        if isinstance(source, bool) or not isinstance(source, (int, str)):
            raise RelayError(400, "source is required (int index or device path)")
        api = _require(body, "api", int)
        self._release()
        cap = self._cv2.VideoCapture(source, api)
        if not cap.isOpened():
            cap.release()
            raise RelayError(502, "could not open camera source %r" % (source,))
        self._cap = cap
        return 200, {"opened": True}

    def _release(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
