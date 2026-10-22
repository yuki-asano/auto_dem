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
                             "do_measure": bool, "insert_until_contact": bool, "measure_times": 1..10,
                             "measure_40g": bool, "measure_28g": bool, "measure_10g": bool}
                            (all optional; thickness, sample_name and file_path are left
                             to the euslisp function's defaults when omitted. The chosen
                             frequencies run in the fixed order 40, 28, 10; at least one)
                            202 {"status": "accepted", "params": {...}}
                            400 on invalid parameters, 409 while the arm is busy.
    POST /abort             cancels the running motion and unwinds the running handler
                            200 {"aborted": true}
    GET  /fr3/status        {"connected", "enabled", "has_errors", "robot_mode", "busy", "last_error"}
    GET  /fr3/pose          {"joints": [7 deg], "position": [x y z roll pitch yaw], "gripper_mm"}
    POST /fr3/move-joints   {"joints": [7], "speed": deg/s, "linear_speed": mm/s (optional: the TCP
                             runs a straight line to those joints' pose, ending on the joints themselves)}
    POST /fr3/move-position {"position": [6], "speed": mm/s, "step_mm": mm}
    POST /fr3/gripper/move  {"position": mm, "speed": mm/s} -> {"achieved": mm}
    POST /fr3/gripper/grasp {"width": mm, "speed": mm/s, "force": N,
                             "epsilon_inner": mm, "epsilon_outer": mm} -> {"achieved": mm}
    GET  /fr3/gripper/position
    POST /fr3/abort         same as /abort
    POST /fr3/errors/clear  franka_control's error recovery (a reflex after a collision)
                            -> {"robot_mode", "errors"}; 409 while the arm is busy
    POST /camera/*          see embrio_camera.py

One arm slot: a run and the fr3 calls share it, so a call that arrives while the
arm is busy answers 409 instead of queueing behind a motion. Abort bypasses the slot.
The euslisp node answering {"ok": false} is a 500 with its message; the node not
answering at all is a 503.

Python 3.8 standard library, plus rospy and the roseus srv package for dem/fr3,
actionlib and franka_msgs for fr3, python3-opencv for camera.
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
STATE_TOPIC = "/franka_state_controller/franka_states"
RECOVERY_ACTION = "/franka_control/error_recovery"
# franka_msgs/FrankaState.robot_mode; the arm takes no motion in the last three until recovered.
ROBOT_MODES = ("other", "idle", "move", "guiding", "reflex", "user_stopped", "automatic_error_recovery")
FAULT_MODES = ("reflex", "user_stopped", "automatic_error_recovery")
# franka_state_controller publishes at 30 Hz; older than this means franka_control is gone.
STATE_MAX_AGE_S = 2.0
RECOVERY_TIMEOUT_S = 15.0
LOG_MAX = 200
JOINT_COUNT = 7
POSITION_LENGTH = 6
# Wall-clock bound on one service call; a measurement run has no bound of its own.
MOVE_TIMEOUT_S = 600.0
RUN_TIMEOUT_S = None

DEFAULT_PARAMS = {
    "do_measure": True,
    "insert_until_contact": True,
    "measure_times": 1,
    "measure_40g": True,
    "measure_28g": True,
    "measure_10g": True,
}
FREQUENCY_PARAMS = ("measure_40g", "measure_28g", "measure_10g")
# Left out unless given, so the euslisp function's own keyword default applies.
OPTIONAL_PARAMS = ("thickness", "sample_name", "file_path")
MEASURE_TIMES_MAX = 10
# libfranka's own default for both sides of the grasp window.
GRASP_EPSILON_MM = 5.0

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
    unknown = set(body) - set(DEFAULT_PARAMS) - set(OPTIONAL_PARAMS)
    if unknown:
        raise ValueError("unknown parameter(s): %s" % ", ".join(sorted(unknown)))
    params = dict(DEFAULT_PARAMS)
    params.update(body)
    if "thickness" in params:
        if not _is_number(params["thickness"]) or params["thickness"] <= 0:
            raise ValueError("thickness must be a number > 0")
        params["thickness"] = float(params["thickness"])
    for key in ("sample_name", "file_path"):
        if key in params and (not isinstance(params[key], str) or not params[key]):
            raise ValueError("%s must be a non-empty string" % key)
    for key in ("do_measure", "insert_until_contact") + FREQUENCY_PARAMS:
        if not isinstance(params[key], bool):
            raise ValueError("%s must be a boolean" % key)
    if not any(params[key] for key in FREQUENCY_PARAMS):
        raise ValueError("select at least one of %s" % ", ".join(FREQUENCY_PARAMS))
    times = params["measure_times"]
    if not isinstance(times, int) or isinstance(times, bool):
        raise ValueError("measure_times must be an integer")
    if not 1 <= times <= MEASURE_TIMES_MAX:
        raise ValueError("measure_times must be between 1 and %d" % MEASURE_TIMES_MAX)
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


def validate_non_negative(body, key):
    value = body.get(key)
    if value is None:
        raise ValueError("%s is required" % key)
    if not _is_number(value) or value < 0:
        raise ValueError("%s must be a number >= 0" % key)
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


class FrankaUnavailable(Exception):
    """No fresh robot state, or franka_control's recovery action is not served."""


class FrankaError(Exception):
    """franka_control's error recovery did not succeed."""


class FrankaMonitor:
    """The robot's own mode and errors from franka_control, which the roseus node does not see."""

    def __init__(self):
        import actionlib
        from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryGoal, FrankaState

        self._lock = threading.Lock()
        self._latest = None
        self._goal = ErrorRecoveryGoal
        self._recovery = actionlib.SimpleActionClient(RECOVERY_ACTION, ErrorRecoveryAction)
        self._succeeded = actionlib.GoalStatus.SUCCEEDED
        rospy.Subscriber(STATE_TOPIC, FrankaState, self._on_state, queue_size=1)

    def _on_state(self, msg):
        errors = [name for name in msg.current_errors.__slots__ if getattr(msg.current_errors, name)]
        mode = ROBOT_MODES[msg.robot_mode] if 0 <= msg.robot_mode < len(ROBOT_MODES) else str(msg.robot_mode)
        with self._lock:
            self._latest = (time.monotonic(), mode, errors)

    def state(self, newer_than=None):
        with self._lock:
            latest = self._latest
        if latest is None or time.monotonic() - latest[0] > STATE_MAX_AGE_S:
            raise FrankaUnavailable("no %s within %.0f s: is franka_control up?" % (STATE_TOPIC, STATE_MAX_AGE_S))
        if newer_than is not None and latest[0] <= newer_than:
            return None
        return {"robot_mode": latest[1], "errors": latest[2]}

    @staticmethod
    def fault(state):
        if state["robot_mode"] in FAULT_MODES or state["errors"]:
            return "robot_mode %s, errors %s" % (state["robot_mode"], state["errors"])
        return None

    def recover(self):
        """Send the recovery goal, then wait for a robot state taken after it; raise unless that state is clear."""
        if not self._recovery.wait_for_server(rospy.Duration(5.0)):
            raise FrankaUnavailable("%s is not served: is franka_control up?" % RECOVERY_ACTION)
        self._recovery.send_goal(self._goal())
        if not self._recovery.wait_for_result(rospy.Duration(RECOVERY_TIMEOUT_S)):
            self._recovery.cancel_goal()
            raise FrankaError("%s did not finish within %.0f s" % (RECOVERY_ACTION, RECOVERY_TIMEOUT_S))
        if self._recovery.get_state() != self._succeeded:
            raise FrankaError("%s ended in state %d: %s" % (
                RECOVERY_ACTION, self._recovery.get_state(), self._recovery.get_goal_status_text()))
        done = time.monotonic()
        state = None
        while state is None:
            if time.monotonic() - done > STATE_MAX_AGE_S:
                raise FrankaUnavailable("no %s after the recovery" % STATE_TOPIC)
            time.sleep(0.05)
            state = self.state(newer_than=done)
        fault = self.fault(state)
        if fault:
            raise FrankaError("the recovery succeeded but the robot still reports %s" % fault)
        return state


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
FRANKA = None


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
        connected, error, robot_mode = True, None, None
        try:
            CLIENT.call("ping", 5.0)
        except (EusUnavailable, EusError) as e:
            connected, error = False, str(e)
        try:
            state = FRANKA.state()
            robot_mode = state["robot_mode"]
            fault = FRANKA.fault(state)
        except FrankaUnavailable as e:
            fault = str(e)
        if fault:
            error = fault if error is None else "%s; %s" % (error, fault)
        self._reply(200, {
            "connected": connected,
            "enabled": connected,
            "has_errors": error is not None,
            "robot_mode": robot_mode,
            "busy": ARM_SLOT.busy,
            "last_error": error,
        })

    def _fr3_clear_errors(self):
        if not ARM_SLOT.acquire("errors_clear"):
            self._reply(409, {"error": "the arm is busy (%s)" % ARM_SLOT.holder})
            return
        try:
            state = FRANKA.recover()
        except FrankaUnavailable as e:
            log_error("error recovery: %s" % e)
            self._reply(503, {"error": str(e)})
            return
        except FrankaError as e:
            log_error("error recovery failed: %s" % e)
            self._reply(500, {"error": str(e)})
            return
        finally:
            ARM_SLOT.release()
        log_info("FR3 errors cleared (robot_mode %s)" % state["robot_mode"])
        self._reply(200, state)

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
            if self.path == "/fr3/errors/clear":
                self._fr3_clear_errors()
                return
            try:
                if self.path == "/fr3/move-joints":
                    extra = {}
                    if "linear_speed" in body:
                        extra["linear_speed"] = validate_positive(body, "linear_speed")
                    self._arm_call("move_joints", MOVE_TIMEOUT_S,
                                   joints=validate_joints(body.get("joints")),
                                   speed=validate_positive(body, "speed"), **extra)
                elif self.path == "/fr3/move-position":
                    self._arm_call("move_position", MOVE_TIMEOUT_S,
                                   position=validate_position(body.get("position")),
                                   speed=validate_positive(body, "speed"),
                                   step_mm=validate_positive(body, "step_mm", 5.0))
                elif self.path == "/fr3/gripper/move":
                    self._arm_call("gripper_move", MOVE_TIMEOUT_S,
                                   position=validate_non_negative(body, "position"),
                                   speed=validate_positive(body, "speed"))
                elif self.path == "/fr3/gripper/grasp":
                    self._arm_call("gripper_grasp", MOVE_TIMEOUT_S,
                                   width=validate_non_negative(body, "width"),
                                   speed=validate_positive(body, "speed"),
                                   force=validate_positive(body, "force"),
                                   epsilon_inner=validate_positive(body, "epsilon_inner", GRASP_EPSILON_MM),
                                   epsilon_outer=validate_positive(body, "epsilon_outer", GRASP_EPSILON_MM))
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
    global rospy, CLIENT, FRANKA
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
        if "fr3" in args.devices:
            FRANKA = FrankaMonitor()
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
