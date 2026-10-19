#!/usr/bin/env python3
"""HTTP server exposing auto_dem's measurement sequence and the FR3 for remote control.

auto_dem's manipulation lives in euslisp (dielectrometer-manip.l) and is started by a
ROS topic without any completion signal, so an external system (a lab workflow server
on another PC) cannot run it and watch it finish. This node adds a JSON/HTTP entry
point on top of it: the measurement sequence as a run with a status, and the FR3's
basic motions (pose, joint and straight-line moves, gripper) for the tray handover
that the embrio side choreographs itself. The robot is reached through the roseus
node next to this file (auto_dem_embrio_node.l, service /embrio/auto_dem/call); no
upstream file is modified, and nothing here runs unless it is started explicitly.

The same server also relays the USB camera wired to this PC (embrio_camera.py).
--devices picks which of dem, fr3, camera to serve (default: all).

Usage (catkin workspace sourced, roscore and `roslaunch auto_dem fr3_dielec.launch`
up on the robot machine, the euslisp node from auto_dem_http_server.launch running):

    rosrun auto_dem auto_dem_http_server.py [--port 8302] [--devices dem,fr3,camera]
    roslaunch auto_dem auto_dem_http_server.launch [port:=8302] [devices:=dem,fr3,camera]

Without dem or fr3 in --devices, rospy is not imported, so the camera relay alone
runs as plain `python3 auto_dem_http_server.py --devices camera`.

API (JSON over HTTP):

    GET  /health            {"devices": [...]}
    GET  /status            {"state": "idle|running|finished|failed", "running": bool,
                             "params": {...}|null, "log": [str], "last_error": str|null,
                             "started_at": float|null, "finished_at": float|null}
    POST /start             body: {"thickness": float, "sample_name": str, "file_path": str,
                             "do_measure": bool, "insert_until_contact": bool, "measure_times": 1..10}
                            (all optional; defaults match the euslisp functions)
                            202 {"status": "accepted", "params": {...}}
                            400 on invalid parameters, 409 while the arm is busy.
    POST /abort             cancels the running motion and unwinds the running handler
                            200 {"aborted": true}
    GET  /fr3/status        {"connected", "enabled", "has_errors", "busy", "last_error"}
    GET  /fr3/pose          {"joints": [7 deg], "position": [x y z roll pitch yaw], "gripper_mm"}
    POST /fr3/move-joints   {"joints": [7], "speed": deg/s}
    POST /fr3/move-position {"position": [6], "speed": mm/s, "step_mm": mm}
    POST /fr3/gripper/move  {"position": mm, "speed": mm/s, "force": N} -> {"achieved": mm}
    GET  /fr3/gripper/position
    POST /fr3/abort         same as /abort
    POST /camera/*          see embrio_camera.py

One arm slot: a run and the fr3 calls share it, so a call that arrives while the
arm is busy answers 409 instead of queueing behind a motion. Abort bypasses the slot.
The euslisp node answering {"ok": false} is a 500 with its message; the node not
answering at all is a 503.

Python 3.8 standard library, plus rospy and the roseus srv package for dem/fr3,
python3-opencv for camera.
"""

import argparse
import json
import logging
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import embrio_camera

DEFAULT_PORT = 8302
DEVICES = ("dem", "fr3", "camera")
SERVICE = "/embrio/auto_dem/call"
ABORT_TOPIC = "/embrio/auto_dem/abort"
LOG_MAX = 200
JOINT_COUNT = 7
POSITION_LENGTH = 6
# Wall-clock bound on one service call; a measurement run has no bound of its own.
MOVE_TIMEOUT_S = 600.0
RUN_TIMEOUT_S = None

DEFAULT_PARAMS = {
    "thickness": 0.3,
    "sample_name": "sample",
    "file_path": "C:/Users/test/Desktop/asano/test_file.csv",
    "do_measure": True,
    "insert_until_contact": True,
    "measure_times": 1,
}
MEASURE_TIMES_MAX = 10

# Path prefix -> relay, for the devices being served.
RELAYS = {}
# Bound in main() only when dem or fr3 is served: importing it needs ROS.
rospy = None


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_params(body):
    """Return validated run params merged over defaults; raise ValueError."""
    if not isinstance(body, dict):
        raise ValueError("request body must be a JSON object")
    unknown = set(body) - set(DEFAULT_PARAMS)
    if unknown:
        raise ValueError("unknown parameter(s): %s" % ", ".join(sorted(unknown)))
    params = dict(DEFAULT_PARAMS)
    params.update(body)
    if not _is_number(params["thickness"]) or params["thickness"] <= 0:
        raise ValueError("thickness must be a number > 0")
    for key in ("sample_name", "file_path"):
        if not isinstance(params[key], str) or not params[key]:
            raise ValueError("%s must be a non-empty string" % key)
    for key in ("do_measure", "insert_until_contact"):
        if not isinstance(params[key], bool):
            raise ValueError("%s must be a boolean" % key)
    times = params["measure_times"]
    if not isinstance(times, int) or isinstance(times, bool):
        raise ValueError("measure_times must be an integer")
    if not 1 <= times <= MEASURE_TIMES_MAX:
        raise ValueError("measure_times must be between 1 and %d" % MEASURE_TIMES_MAX)
    params["thickness"] = float(params["thickness"])
    return params


def validate_joints(joints):
    if not isinstance(joints, list) or len(joints) != JOINT_COUNT or not all(_is_number(v) for v in joints):
        raise ValueError("joints must be a list of %d numbers" % JOINT_COUNT)
    return [float(v) for v in joints]


def validate_position(position):
    if (not isinstance(position, list) or len(position) != POSITION_LENGTH
            or not all(_is_number(v) for v in position)):
        raise ValueError("position must be a list of %d numbers (x, y, z, roll, pitch, yaw)" % POSITION_LENGTH)
    return [float(v) for v in position]


def validate_positive(body, key, default=None):
    value = body.get(key, default)
    if value is None:
        raise ValueError("%s is required" % key)
    if not _is_number(value) or value <= 0:
        raise ValueError("%s must be a number > 0" % key)
    return float(value)


# --- S-expression request encoding ---------------------------------------------------------------

def sexp(value):
    """A euslisp-readable literal: data only, nothing the reader could evaluate."""
    if value is True:
        return "t"
    if value is False or value is None:
        return "nil"
    if isinstance(value, int):
        return "%d" % value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("numbers must be finite")
        return repr(value)
    if isinstance(value, str):
        if any(ord(c) < 32 for c in value):
            raise ValueError("strings must not contain control characters")
        return '"%s"' % value.replace("\\", "\\\\").replace('"', '\\"')
    if isinstance(value, (list, tuple)):
        return "(%s)" % " ".join(sexp(v) for v in value)
    raise ValueError("unsupported value %r" % (value,))


def request_sexp(name, kwargs):
    parts = [name]
    for key, value in kwargs.items():
        parts.append(":%s %s" % (key, sexp(value)))
    return "(%s)" % " ".join(parts)


class EusUnavailable(Exception):
    """The euslisp node did not answer: not started, or died with the robot interface."""


class EusError(Exception):
    """The euslisp node answered {"ok": false}."""


class EusClient:
    """rospy client of the roseus node's service and abort topic."""

    def __init__(self):
        from roseus.srv import StringString
        from std_msgs.msg import Empty

        self._proxy = rospy.ServiceProxy(SERVICE, StringString, persistent=False)
        self._abort = rospy.Publisher(ABORT_TOPIC, Empty, queue_size=1)
        self._empty = Empty

    def call(self, name, timeout, **kwargs):
        text = request_sexp(name, kwargs)
        try:
            reply = self._proxy(text).str
        except rospy.ServiceException as e:
            raise EusUnavailable("%s did not answer: %s" % (SERVICE, e))
        try:
            payload = json.loads(reply)
        except ValueError:
            raise EusError("malformed reply from %s: %r" % (SERVICE, reply[:200]))
        if not isinstance(payload, dict) or "ok" not in payload:
            raise EusError("malformed reply from %s: %r" % (SERVICE, reply[:200]))
        if not payload["ok"]:
            raise EusError(str(payload.get("error", "unknown error")))
        return payload.get("result")

    def abort(self):
        self._abort.publish(self._empty())


class ArmSlot:
    """The one caller allowed to move the arm at a time; a busy arm refuses, never queues."""

    def __init__(self):
        self._lock = threading.Lock()
        self.holder = None

    def acquire(self, holder):
        if not self._lock.acquire(blocking=False):
            return False
        self.holder = holder
        return True

    def release(self):
        self.holder = None
        self._lock.release()

    @property
    def busy(self):
        return self.holder is not None


class RunState:
    """State of the single measurement run slot, shared across request threads."""

    def __init__(self):
        self._lock = threading.Lock()
        self._thread = None
        self.state = "idle"
        self.params = None
        self.log = []
        self.last_error = None
        self.started_at = None
        self.finished_at = None

    def snapshot(self):
        with self._lock:
            return {
                "state": self.state,
                "running": self.state == "running",
                "params": dict(self.params) if self.params else None,
                "log": list(self.log[-LOG_MAX:]),
                "last_error": self.last_error,
                "started_at": self.started_at,
                "finished_at": self.finished_at,
            }

    def append_log(self, message):
        with self._lock:
            self.log.append(str(message))
            del self.log[:-LOG_MAX]

    def start(self, params, client, slot):
        """Launch the sequence in a worker thread holding the arm slot; raise RuntimeError if busy."""
        with self._lock:
            if self.state == "running":
                raise RuntimeError("a measurement run is already in progress")
            if not slot.acquire("run"):
                raise RuntimeError("the arm is busy (%s)" % slot.holder)
            self.state = "running"
            self.params = params
            self.log = ["measurement started"]
            self.last_error = None
            self.started_at = time.time()
            self.finished_at = None
            self._thread = threading.Thread(
                target=self._run, args=(params, client, slot), name="auto-dem-run", daemon=True
            )
            self._thread.start()

    def _run(self, params, client, slot):
        try:
            client.call("run_measurement", RUN_TIMEOUT_S, **params)
        # BaseException: a vanished run must surface as failed, never as a silently dead thread.
        except BaseException as e:
            log_error("measurement run failed: %r" % (e,))
            with self._lock:
                self.state = "failed"
                self.last_error = str(e) if isinstance(e, (EusError, EusUnavailable)) else repr(e)
                self.finished_at = time.time()
        else:
            with self._lock:
                if self.state == "running":
                    self.state = "finished"
                    self.log.append("measurement finished")
                    self.finished_at = time.time()
        finally:
            slot.release()

    def mark_aborted(self):
        """An abort reached the robot: the run is over on this side whatever the sequence does next."""
        with self._lock:
            if self.state == "running":
                self.state = "failed"
                self.last_error = "aborted"
                self.finished_at = time.time()
                self.log.append("aborted")


RUN_STATE = RunState()
ARM_SLOT = ArmSlot()
CLIENT = None


def log_info(message):
    if rospy is not None:
        rospy.loginfo(message)
    else:
        logging.info(message)


def log_error(message):
    if rospy is not None:
        rospy.logerr(message)
    else:
        logging.error(message)


def parse_devices(text):
    names = [name.strip() for name in text.split(",") if name.strip()]
    unknown = sorted(set(names) - set(DEVICES))
    if unknown or not names:
        raise argparse.ArgumentTypeError(
            "devices must be a comma-separated subset of %s, got %r" % (",".join(DEVICES), text)
        )
    return [name for name in DEVICES if name in names]


class Handler(BaseHTTPRequestHandler):
    # Set by main(): the devices this process serves.
    devices = ()

    def _reply(self, code, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _reply_raw(self, code, body, headers):
        self.send_response(code)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        for key, value in headers.items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        return json.loads(raw.decode("utf-8")) if raw.strip() else {}

    def _disabled(self, device):
        self._reply(404, {"error": "%s is disabled on this server (--devices %s)" % (device, ",".join(self.devices))})

    def _prefix(self):
        return self.path.split("/")[1] if self.path.startswith("/") else ""

    def _relay(self, method):
        """Serve /camera/*; return False when the path is not a relay path."""
        prefix = self._prefix()
        if prefix != "camera":
            return False
        relay = RELAYS.get(prefix)
        if relay is None:
            self._disabled(prefix)
            return True
        try:
            body = self._read_json() if method == "POST" else {}
            code, payload = relay.handle(method, self.path, body)
        except ValueError as e:
            self._reply(400, {"error": "invalid JSON body: %s" % e})
            return True
        except embrio_camera.RelayError as e:
            if e.status >= 500:
                log_error("%s %s failed: %s" % (method, self.path, e))
            self._reply(e.status, {"error": str(e)})
            return True
        except Exception as e:
            log_error("%s %s failed: %r" % (method, self.path, e))
            self._reply(500, {"error": "%s: %s" % (type(e).__name__, e)})
            return True
        if isinstance(payload, tuple):
            self._reply_raw(code, payload[0], payload[1])
        else:
            self._reply(code, payload)
        return True

    def _arm_call(self, name, timeout, **kwargs):
        """One euslisp call under the arm slot; the reply is sent here."""
        if not ARM_SLOT.acquire(name):
            self._reply(409, {"error": "the arm is busy (%s)" % ARM_SLOT.holder})
            return
        try:
            result = CLIENT.call(name, timeout, **kwargs)
        except EusUnavailable as e:
            log_error("%s: %s" % (name, e))
            self._reply(503, {"error": str(e)})
            return
        except EusError as e:
            log_error("%s failed: %s" % (name, e))
            self._reply(500, {"error": str(e)})
            return
        finally:
            ARM_SLOT.release()
        self._reply(200, result if isinstance(result, dict) else {"result": result})

    def _abort(self):
        try:
            CLIENT.abort()
        except Exception as e:
            log_error("abort failed: %r" % (e,))
            self._reply(500, {"error": "%s: %s" % (type(e).__name__, e)})
            return
        RUN_STATE.mark_aborted()
        log_info("abort published to %s" % ABORT_TOPIC)
        self._reply(200, {"aborted": True})

    def _fr3_status(self):
        connected, error = True, None
        try:
            CLIENT.call("ping", 5.0)
        except (EusUnavailable, EusError) as e:
            connected, error = False, str(e)
        self._reply(200, {
            "connected": connected,
            "enabled": connected,
            "has_errors": error is not None,
            "busy": ARM_SLOT.busy,
            "last_error": error,
        })

    def do_GET(self):
        if self.path == "/health":
            self._reply(200, {"devices": list(self.devices)})
        elif self._relay("GET"):
            pass
        elif self.path == "/status":
            if "dem" in self.devices:
                self._reply(200, RUN_STATE.snapshot())
            else:
                self._disabled("dem")
        elif self._prefix() == "fr3":
            if "fr3" not in self.devices:
                self._disabled("fr3")
            elif self.path == "/fr3/status":
                self._fr3_status()
            elif self.path == "/fr3/pose":
                self._arm_call("pose", 5.0)
            elif self.path == "/fr3/gripper/position":
                self._arm_call("gripper_position", 5.0)
            else:
                self._reply(404, {"error": "not found: %s" % self.path})
        else:
            self._reply(404, {"error": "not found: %s" % self.path})

    def do_POST(self):
        if self._relay("POST"):
            return
        try:
            body = self._read_json()
        except ValueError as e:
            self._reply(400, {"error": "invalid JSON body: %s" % e})
            return
        if self.path in ("/abort", "/fr3/abort"):
            device = "dem" if self.path == "/abort" else "fr3"
            if device not in self.devices:
                self._disabled(device)
            else:
                self._abort()
            return
        if self.path == "/start":
            self._start(body)
            return
        if self._prefix() == "fr3":
            if "fr3" not in self.devices:
                self._disabled("fr3")
                return
            try:
                if self.path == "/fr3/move-joints":
                    self._arm_call("move_joints", MOVE_TIMEOUT_S,
                                   joints=validate_joints(body.get("joints")),
                                   speed=validate_positive(body, "speed"))
                elif self.path == "/fr3/move-position":
                    self._arm_call("move_position", MOVE_TIMEOUT_S,
                                   position=validate_position(body.get("position")),
                                   speed=validate_positive(body, "speed"),
                                   step_mm=validate_positive(body, "step_mm", 5.0))
                elif self.path == "/fr3/gripper/move":
                    self._arm_call("gripper_move", MOVE_TIMEOUT_S,
                                   position=validate_positive(body, "position"),
                                   speed=validate_positive(body, "speed"),
                                   force=validate_positive(body, "force"))
                else:
                    self._reply(404, {"error": "not found: %s" % self.path})
            except ValueError as e:
                self._reply(400, {"error": str(e)})
            return
        self._reply(404, {"error": "not found: %s" % self.path})

    def _start(self, body):
        if "dem" not in self.devices:
            self._disabled("dem")
            return
        try:
            params = validate_params(body)
        except ValueError as e:
            self._reply(400, {"error": str(e)})
            return
        try:
            RUN_STATE.start(params, CLIENT, ARM_SLOT)
        except RuntimeError as e:
            self._reply(409, {"error": str(e)})
            return
        self._reply(202, {"status": "accepted", "params": params})

    def log_message(self, fmt, *args):
        log_info("auto_dem_http: " + fmt % args)


def main():
    global rospy, CLIENT
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--devices", type=parse_devices, default=list(DEVICES))
    # roslaunch appends the remapping arguments (__name:=, __log:=) to every
    # node's argv; drop them (as rospy.myargv() does) so argparse only sees our own flags.
    args = parser.parse_args([arg for arg in sys.argv[1:] if ":=" not in arg])

    # A served device whose dependency is missing fails here, not on its first request.
    if "dem" in args.devices or "fr3" in args.devices:
        import rospy as _rospy

        rospy = _rospy
        rospy.init_node("auto_dem_http", disable_signals=True)
        CLIENT = EusClient()
    else:
        logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if "camera" in args.devices:
        RELAYS["camera"] = embrio_camera.CameraRelay(embrio_camera.load_cv2())
    Handler.devices = tuple(args.devices)

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    log_info("auto_dem_http listening on 0.0.0.0:%d (devices: %s)" % (args.port, ",".join(args.devices)))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
