"""
app/app.py — AGV Web Monitoring Dashboard
==========================================
Flask server running in a daemon thread alongside the asyncio main loop.
Shares the AMRState object directly (GIL-safe for reads; single-attribute
write for plc_sequence_request).

Access from any device on 192.168.2.x: http://192.168.2.100:5000
"""

import os
import re
import subprocess
import time
import threading
import io
import zipfile
from flask import Flask, render_template, jsonify, request, send_from_directory, send_file
from logger import get_error_log
import keyboard_control
import logging

logger = logging.getLogger(__name__)

# Flask looks for templates relative to the app.py file location
_here = os.path.dirname(os.path.abspath(__file__))
app   = Flask(__name__, template_folder=os.path.join(_here, "templates"))

_state   = None   # set once by run_server()
_engine  = None   # set once by run_server()
_manager = None   # set once by run_server() — toggles RFID/SLMP drivers live

_NUM_SEQ_BITS = 9

_Y_LABELS = ["Y0","Y1","Y2","Y3","Y4","Y5","Y6","Y7",
             "Y10","Y11","Y12","Y13","Y14","Y15","Y16","Y17","Y20"]
_X_LABELS = ["X0","X1","X2","X3","X4","X5","X6","X7",
             "X10","X11","X12","X13"]

def _build_di_names():
    import config
    names = ["[SPARE]"] * config.NUM_DI
    mapping = {
        config.DI_START:       "START",
        config.DI_RESET:       "RESET",
        config.DI_MODE_SWITCH: "MODE SWITCH",
        config.DI_EMERGENCY:   "EMERGENCY",
        config.DI_FWD:         "FWD",
        config.DI_REV:         "REV",
        config.DI_LEFT:        "LEFT",
        config.DI_RIGHT:       "RIGHT",
    }
    for idx, label in mapping.items():
        if 0 <= idx < config.NUM_DI:
            names[idx] = label
    return names

def _build_do_names():
    import config
    names = ["[SPARE]"] * config.NUM_DO
    mc = config.MOTOR_CHANNELS
    do_map = {
        mc["left"]["do_fwd"]:    "LEFT FWD",
        mc["left"]["do_rev"]:    "LEFT REV",
        mc["left"]["do_brake"]:  "LEFT BRAKE",
        mc["right"]["do_fwd"]:   "RIGHT FWD",
        mc["right"]["do_rev"]:   "RIGHT REV",
        mc["right"]["do_brake"]: "RIGHT BRAKE",
    }
    if config.DO_HORN >= 0:
        do_map[config.DO_HORN] = "HORN"
    if config.DO_LIGHT >= 0:
        do_map[config.DO_LIGHT] = "LIGHT" 
    for idx, label in do_map.items():
        if 0 <= idx < config.NUM_DO:
            names[idx] = label
    return names


_TROLLEY_WRITABLE = {
    "M2104","M2105","M2106","M2107","M2109",
    "M2124","M2125","M2126","M2127","M2129",
    "M2212","M2213",
}


def _build_state_snapshot():
    """Derive the full JSON payload from the shared state object."""
    import config
    s = _state
    mode = s.current_mode

    # ── AGV summary ───────────────────────────────────────────────────────────
    agv = {
        "mode":                 mode,
        "emergency":            s.emergency_active,
        "system_error":         s.system_error,
        "system_error_detail":  s.system_error_detail,
        "speed_mode":           s.speed_mode,
        "track_follow":         s.track_follow,
        "selected_fork_target": s.selected_fork_target,
        "sequence_stop":        s.sequence_stop,
        "sequence_request":     s.plc_sequence_request,
        "seq_pulse_active":     s.plc_sequence_request is not None,
        "latest_rfid_tag":      s.latest_rfid_tag,
        "web_button_mode":      s.web_button_mode,
        "web_btn_manual":       s.web_btn_manual,
        "web_btn_emergency":    s.web_btn_emergency,
        "kb_estop":             s.kb_estop,
        "kb_mode_override":     s.kb_mode_override,
        "keypad_devices":       list(s.keypad_devices),
        "rotate_status":        dict(s.rotate_status or {}),
        "track_noise":          dict(s.track_noise or {}),
        "sensor_hold_active":   s.sensor_hold_active,
    }

    # ── What the AGV is writing to the PLC right now ──────────────────────────
    req = s.plc_sequence_request
    plc_writes = {
        "M2000_READY":             mode not in (None, "emergency"),
        "M2001_EMERGENCY":         s.emergency_active,
        "M2002_TROLLEY_EMERGENCY": s.emergency_active,
        "M2003_MANUAL":            mode == "manual",
        "M2004_AUTO":              mode in ("armed", "running"),
        "M2005_RUNNING":           mode == "running",
    }
    for i in range(_NUM_SEQ_BITS):
        plc_writes[f"M{2010 + i}_SEQ{i}"] = (req == i)

    # ── What the AGV is reading from the PLC ─────────────────────────────────
    plc_reads = dict(s.plc_inputs)   # M0–M4
    complete  = s.plc_sequence_complete
    for i in range(_NUM_SEQ_BITS):
        plc_reads[f"M{2040 + i}_SEQ{i}_COMPLETE"] = complete[i] if i < len(complete) else False

    # ── Magnetic sensor (MGS1600 via CAN) ────────────────────────────────────
    raw = s.latest_sensor
    sensor = {
        "left_mm":        raw["left_mm"]        if raw else None,
        "tape_detected":  raw["tape_detected"]   if raw else False,
        "left_marker":    raw["left_marker"]     if raw else False,
        "right_marker":   raw["right_marker"]    if raw else False,
        "sensor_failure": raw["sensor_failure"]  if raw else False,
        "left_rpm":       round(s.left_rpm,  1),
        "right_rpm":      round(s.right_rpm, 1),
        # Actual per-wheel linear velocity (m/s) from motor RPM:
        #   wheel_rpm = motor_rpm / GEAR_RATIO; v = wheel_rpm * circ / 60
        "v_left":         round(s.left_rpm  / config.GEAR_RATIO * config.WHEEL_CIRCUMFERENCE / 60.0, 3),
        "v_right":        round(s.right_rpm / config.GEAR_RATIO * config.WHEEL_CIRCUMFERENCE / 60.0, 3),
        "pid_output":     round(s.pid_output, 1),
        "target_speed":   round(s.target_speed, 3),
        "speed_mode":     s.speed_mode,
    }

    # ── Sequence engine status ────────────────────────────────────────────────
    sequences = _engine.status() if _engine is not None else {
        "active": None, "armed": None, "cooldowns": {}, "last_event": None
    }

    # ── Driver frame rates (DI / CAN / RFID) ──────────────────────────────────
    # The CAN rate is the one that matters for PID tuning: the control loop only
    # advances when a sensor frame lands, so this IS the effective loop rate.
    drivers = _manager.driver_health() if _manager is not None else []

    return {
        "agv":        agv,
        "plc_writes": plc_writes,
        "plc_reads":  plc_reads,
        "sensor":     sensor,
        "sequences":  sequences,
        "drivers":    drivers,
    }


_VALID_COMMANDS = {"forward", "fwd_left", "fwd_right", "reverse",
                   "rvs_left", "rvs_right", "left", "right"}  # manual pendant jog

@app.route("/")
def index():
    import config
    return render_template("index.html", cabang_names=config.CABANG_NAMES)

@app.route("/manual")
def manual():
    return render_template("manual.html")

@app.route("/api/manual/command", methods=["POST"])
def api_manual_command():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if _state.current_mode != "manual":
        return jsonify({"error": f"AGV not in manual mode (current: {_state.current_mode})"}), 403

    data = request.get_json(silent=True) or {}
    cmd  = data.get("command")   # None = stop

    if cmd is not None and cmd not in _VALID_COMMANDS:
        return jsonify({"error": f"unknown command: {cmd}"}), 400

    _state.web_manual_command = cmd
    _state.web_manual_expire  = time.time() + 0.4   # 400 ms watchdog
    return jsonify({"ok": True, "command": cmd})


@app.route("/api/keyboard", methods=["POST"])
def api_keyboard():
    """One-press keyboard action (laptop keyboard / scripts):
    {"key": "1".."9" | "0" | "q" | "e" | "k" | "l" | "enter" | ...}.
    Movement keys (WASD) use /api/manual/command like the jog buttons.
    Same key map as the USB keypad on the AGV PC — see keyboard_control.py."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    data = request.get_json(silent=True) or {}
    key = str(data.get("key", "")).lower()
    if key not in keyboard_control.ACTION_KEYS:
        return jsonify({"error": f"unknown key '{key}'"}), 400
    ok, msg = keyboard_control.handle_key(_state, key, source="web keyboard")
    return jsonify({"ok": ok, "message": msg}), (200 if ok else 409)


@app.route("/api/rotate", methods=["POST"])
def api_rotate():
    """180° spin-turn on the tape from the MANUAL page (same as keys T / Y):
    {"direction": "left" | "right"} — MANUAL or ARMED, tape under the front
    sensor. {"direction": "stop"} — stop a running turn."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    data = request.get_json(silent=True) or {}
    direction = data.get("direction")
    if direction not in ("left", "right", "stop"):
        return jsonify({"error": "direction must be 'left', 'right' or 'stop'"}), 400
    ok, msg = keyboard_control.request_rotate(_state, direction, source="web")
    if not ok:
        return jsonify({"error": msg}), 409
    return jsonify({"ok": True, "message": msg})


# CABANG 1..11 (point_1..point_11) plus any requires_target used in the profile.
import config as _cfg
_VALID_FORK_TARGETS = ({f"point_{i}" for i in range(1, 12)}
                       | {s.get("trigger", {}).get("requires_target")
                          for s in _cfg.SEQUENCES} - {None})

@app.route("/api/fork_target", methods=["POST"])
def api_fork_target():
    """Operator selects which fork destination the AGV should take next.

    Filtering happens in rfid_processor.py: every fork tag is physically read
    on each loop pass, but only the one matching state.selected_fork_target
    is forwarded to the sequence engine. Intentionally not mode-gated — the
    operator picks this at homepost before the AGV reaches RUNNING.
    """
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503

    data   = request.get_json(silent=True) or {}
    target = data.get("target")   # None = clear selection

    if target is not None and target not in _VALID_FORK_TARGETS:
        return jsonify({"error": f"unknown target: {target}"}), 400

    _state.selected_fork_target = target
    logger.info("[HMI] Fork target set to: %s", target)
    return jsonify({"ok": True, "target": target})


@app.route("/api/state")
def api_state():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    return jsonify(_build_state_snapshot())


MANUAL_SEQ_ENABLED = False


@app.route("/api/sequence", methods=["POST"])
def api_sequence():
    """Manual PLC sequence request (M2010-M2018).

    seq0-seq4 additionally double as the fork/cabang selector: picking one of
    these ALSO sets state.selected_fork_target (point_1..point_5), on top of
    the usual PLC bit write. seq5-seq8 behave exactly as before — PLC only,
    no fork target change.
    """
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503

    inputs      = _state.plc_inputs
    master_on   = inputs.get("M4_MASTER_ON",  False)
    manual_mode = inputs.get("M1_MODE_MAN",   False)

    data = request.get_json(silent=True) or {}
    seq  = data.get("sequence")

    if seq is None:
        # CLEAR — PLC MANUAL MODE (M1) check temporarily disabled for testing.
        # To re-enable: uncomment the block below.
        # if not manual_mode:
        #     return jsonify({"error": "CLEAR requires PLC MANUAL MODE (M1)"}), 403
        _state.plc_sequence_request = None
        return jsonify({"ok": True, "sequence": None})

    # SET — all PLC guards (MASTER ON / MANUAL MODE) temporarily disabled for
    # testing. To re-enable: uncomment the block below.
    # if not master_on:
    #     return jsonify({"error": "SET requires MASTER ON (M4)"}), 403
    # if not manual_mode:
    #     return jsonify({"error": "SET requires PLC MANUAL MODE (M1)"}), 403
    if not isinstance(seq, int) or not (0 <= seq < _NUM_SEQ_BITS):
        return jsonify({"error": f"sequence must be int 0–{_NUM_SEQ_BITS - 1} or null"}), 400

    # Manual sequence requests are switched off (2026-10-29, the HOME page
    # panel was removed): they made the PLC run the trolley wherever the AGV
    # was. Only the station sequences request PLC sequences. CLEAR (None,
    # above) still works. Set MANUAL_SEQ_ENABLED = True to allow it again
    # (then only when the AGV is not driving).
    if not MANUAL_SEQ_ENABLED:
        logger.warning("[HMI] Manual PLC SEQ %d refused — manual sequence requests are off", seq)
        return jsonify({"error": "manual sequence requests are disabled — "
                                 "use Pilih Cabang / F1–F11 to choose a branch"}), 403
    if _state.current_mode == "running" and not _state.sequence_stop:
        logger.warning("[HMI] Manual PLC SEQ %d refused — AGV is driving in AUTO", seq)
        return jsonify({"error": "AGV is driving — a manual SEQ would move the trolley now. "
                                 "To choose a branch use Pilih Cabang (or F1–F11)."}), 409
    logger.warning("[HMI] Manual PLC SEQ %d sent from the HOME page (AGV %s)",
                   seq, _state.current_mode)
    _state.plc.last_request = {"seq": seq, "t": time.time(), "source": "HOME page SEQ (manual)"}

    _state.plc_sequence_request      = seq
    _state.plc_sequence_pulse_expire = time.time() + 1.0   # 1-second pulse

    # seq0-seq4 double as fork/cabang selection (point_1..point_5).
    fork_target = None
    if 0 <= seq <= 4:
        fork_target = f"point_{seq + 1}"
        _state.selected_fork_target = fork_target
        logger.info("[HMI] seq%d also set fork target to %s", seq, fork_target)

    return jsonify({"ok": True, "sequence": seq, "fork_target": fork_target})



@app.route("/errors")
def errors():
    return render_template("errors.html")


@app.route("/api/errors")
def api_errors():
    return jsonify(get_error_log())


# ── Run logs: download from the ERRORS page ───────────────────────────────────
# Every AUTO run is recorded by debugging/plotter.py to analysis_plot/
# <YYYYMMDD_HHMMSS>_data.csv (+ _analysisplot.png); the controller's text log
# is logs/agv_<YYYYMMDD_HHMMSS>.log. Only these names can be downloaded.
_REPO = os.path.dirname(_here)
_RUN_RE = re.compile(r"^(\d{8}_\d{6})_(data\.csv|analysisplot\.png)$")
_LOG_RE = re.compile(r"^agv_\d{8}_\d{6}\.log(\.\d+)?$")


def _plot_dir():
    from debugging.plotter import PLOT_DIR
    return PLOT_DIR


def _log_dir():
    for d in (os.path.join(_REPO, "logs"), os.path.abspath("logs")):
        if os.path.isdir(d):
            return d
    return os.path.join(_REPO, "logs")


def _runs():
    """Recorded runs, newest first: [{run, time, csv, png, size}]."""
    d = _plot_dir()
    runs = {}
    try:
        names = os.listdir(d)
    except OSError:
        names = []
    for n in names:
        m = _RUN_RE.match(n)
        if not m:
            continue
        p = os.path.join(d, n)
        r = runs.setdefault(m.group(1), {"run": m.group(1), "time": 0, "csv": None,
                                         "png": None, "size": 0})
        r["csv" if n.endswith(".csv") else "png"] = n
        r["time"] = max(r["time"], os.path.getmtime(p))
        r["size"] += os.path.getsize(p)
    return sorted(runs.values(), key=lambda r: r["time"], reverse=True)


def _logs():
    d = _log_dir()
    try:
        names = [n for n in os.listdir(d) if _LOG_RE.match(n)]
    except OSError:
        names = []
    out = [{"name": n, "time": os.path.getmtime(os.path.join(d, n)),
            "size": os.path.getsize(os.path.join(d, n))} for n in names]
    return sorted(out, key=lambda r: r["time"], reverse=True)


@app.route("/api/runlogs")
def api_runlogs():
    return jsonify({"runs": _runs()[:30], "logs": _logs()[:10],
                    "recording": bool(_state and _state.current_mode == "running")})


@app.route("/api/runlogs/file/<name>")
def api_runlogs_file(name):
    if _RUN_RE.match(name):
        return send_from_directory(_plot_dir(), name, as_attachment=True)
    if _LOG_RE.match(name):
        return send_from_directory(_log_dir(), name, as_attachment=True)
    return jsonify({"error": "unknown file"}), 404


@app.route("/api/runlogs/zip")
def api_runlogs_zip():
    """The newest n runs (CSV + PNG) and the newest controller log, one zip."""
    n = max(1, min(int(request.args.get("n", 5)), 30))
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for r in _runs()[:n]:
            for f in (r["csv"], r["png"]):
                if f:
                    z.write(os.path.join(_plot_dir(), f), f"analysis_plot/{f}")
        for lg in _logs()[:1]:
            z.write(os.path.join(_log_dir(), lg["name"]), f"logs/{lg['name']}")
    buf.seek(0)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return send_file(buf, mimetype="application/zip", as_attachment=True,
                     download_name=f"agv_runlogs_{stamp}.zip")


@app.route("/api/runlogs/save", methods=["POST"])
def api_runlogs_save():
    """Save the AUTO run recorded so far without stopping the AGV."""
    if _state is None or _state.current_mode != "running":
        return jsonify({"ok": False, "error": "no AUTO run is being recorded "
                        "(a run is saved by itself when AUTO ends)"}), 409
    _state.runlog_save_request = True
    return jsonify({"ok": True})


@app.route("/io")
def io_monitor():
    return render_template("io_monitor.html")


@app.route("/params")
def params():
    return render_template("params.html")


@app.route("/api/io")
def api_io():
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    di = _state.latest_di or [False] * config.NUM_DI
    do_raw = _state.latest_do
    return jsonify({
        "di":       list(di[:config.NUM_DI]),
        "di_names": _build_di_names(),
        "do":       [do_raw.get(i, False) for i in range(config.NUM_DO)],
        "do_names": _build_do_names(),
        "num_di":   config.NUM_DI,
        "num_do":   config.NUM_DO,
    })


@app.route("/api/params")
def api_params():
    import config, math
    return jsonify({
        "Networking": {
            "AGV ID":        config.AGV_ID,
            "Device ID":     config.DEVICE_ID,
            "Local IP":      config.LOCAL_IP,
            "DIO IP":        config.DIO_IP,
            "AO IP":         config.AO_IP,
            "Modbus Port":   config.MODBUS_PORT,
            "RFID IP":       config.RFID_IP,
            "RFID Port":     config.RFID_PORT,
            "SLMP IP":       config.SLMP_IP,
            "SLMP Port":     config.SLMP_PORT,
        },
        "IO Mapping": {
            "DI Base":        config.DI_BASE,
            "DO Base":        config.DO_BASE,
            "Num DI":         config.NUM_DI,
            "Num DO":         config.NUM_DO,
            "AO Base":        config.AO_BASE,
            "Num AO":         config.NUM_AO,
            "DI Flipped":     config.DI_FLIPPED,
            "DI Emergency":   config.DI_EMERGENCY,
            "DI Fwd":         config.DI_FWD,
            "DI Rev":         config.DI_REV,
            "DI Left":        config.DI_LEFT,
            "DI Right":       config.DI_RIGHT,
            "DI Mode Switch": config.DI_MODE_SWITCH,
            "DI Start":       config.DI_START,
            "DI Reset":       config.DI_RESET,
            "DI Prox":        config.DI_PROX,
        },
        "Motor Channels": {
            "Left Fwd (DO)":    config.MOTOR_CHANNELS["left"]["do_fwd"],
            "Left Rev (DO)":    config.MOTOR_CHANNELS["left"]["do_rev"],
            "Left Brake (DO)":  config.MOTOR_CHANNELS["left"]["do_brake"],
            "Left Speed (AO)":  config.MOTOR_CHANNELS["left"]["ao_speed"],
            "Right Fwd (DO)":   config.MOTOR_CHANNELS["right"]["do_fwd"],
            "Right Rev (DO)":   config.MOTOR_CHANNELS["right"]["do_rev"],
            "Right Brake (DO)": config.MOTOR_CHANNELS["right"]["do_brake"],
            "Right Speed (AO)": config.MOTOR_CHANNELS["right"]["ao_speed"],
        },
        "Kinematics": {
            "Wheel Diameter (m)":      config.WHEEL_DIAMETER,
            "Gear Ratio":              config.GEAR_RATIO,
            "Wheel Circumference (m)": round(config.WHEEL_CIRCUMFERENCE, 4),
        },
        "Speeds": {
            "Manual High (m/s)":          config.MANUAL_TARGET_HIGH_SPEED,
            "Manual Slow (m/s)":          config.MANUAL_TARGET_SLOW_SPEED,
            "Auto High (m/s)":            config.AUTO_TARGET_HIGH_SPEED,
            "Auto Slow (m/s)":            config.AUTO_TARGET_SLOW_SPEED,
            "Auto Approach (m/s)":        config.AUTO_TARGET_APPROACH_SPEED,
            "Accel Rate (m/s²)":          config.ACCEL_RATE,
            "Accel Medium / High / Max":  f"{config.ACCEL_RATE_MEDIUM} / {config.ACCEL_RATE_HIGH} / {config.ACCEL_RATE_MAX}",
            "Decel Rate (m/s²)":          config.DECEL_RATE,
            "Approach Decel Rate (m/s²)": config.APPROACH_DECEL_RATE,
        },
        "PID Tuning": {
            "KP (High)":                        config.KP,
            "TD (High)":                        config.TD,
            "N (High)":                         config.N,
            "KP (Slow)":                        config.KP_SLOW,
            "TD (Slow)":                        config.TD_SLOW,
            "N (Slow)":                         config.N_SLOW,
            "KP (Approach)":                    config.KP_APPROACH,
            "TD (Approach)":                    config.TD_APPROACH,
            "N (Approach)":                     config.N_APPROACH,
            "TI":                               config.TI,
            "TI Deadband (mm)":                 config.TI_DEADBAND,
            "TI Max":                           config.TI_MAX,
            "DT (s)":                           config.DT,
            "V Red Coef (High)":                config.V_RED_COEF,
            "V Red Coef (Slow)":                config.V_RED_COEF_SLOW,
            "V Red Coef (Approach)":            config.V_RED_COEF_APPROACH,
            "Output Clamp (RPM)":               config.OUTPUT_CLAMP_RPM,
            "Output Clamp Into-Bar (Approach)": config.OUTPUT_CLAMP_INTO_BAR_APPROACH,
            "Output Clamp Away (Approach)":     config.OUTPUT_CLAMP_AWAY_APPROACH,
            "SR Alpha":                         config.SR_ALPHA,
            "SR Cap":                           config.SR_CAP,
        },
        "CAN Sensor": {
            "COB ID":               config.SENSOR_COB_ID,
            "Timeout (s)":          config.CAN_TIMEOUT,
            "Node ID":              config.CAN_NODE_ID,
            "Flag Tape Detect":     config.FLAG_TAPE_DETECT,
            "Flag Left Marker":     config.FLAG_LEFT_MARKER,
            "Flag Right Marker":    config.FLAG_RIGHT_MARKER,
            "Flag Sensor Fail":     config.FLAG_SENSOR_FAIL,
            "Sensor Max (mm)":      config.SENSOR_MAX_MM,
            "Sensor Max Step (mm)": config.SENSOR_MAX_STEP_MM,
        },
        "RFID": {
            "Sequence Stop Delay (s)": config.SEQUENCE_STOP_DELAY,
            "Init Cmd (hex)":          config.RFID_INIT_CMD.hex().upper(),
        },
        "Feedforward": {
            "Enabled":           config.FF_ENABLED,
            "Track Width (m)":   config.TRACK_WIDTH,
            "Curve Radius (m)":  config.CURVE_RADIUS,
            "Direction Sign":    config.FF_DIRECTION_SIGN,
            "Curve Speed Modes": ", ".join(sorted(config.FF_CURVE_MODES)),
            "FF Alpha":          config.FF_ALPHA,
            "FF Exit Alpha":     config.FF_EXIT_ALPHA,
            "FF Scale":          config.FF_SCALE,
        },
        "Timing": {
            "DI Poll Interval (s)": config.DI_POLL_INTERVAL,
        },
        "Watchdog": {
            "DI Timeout (s)":   config.WATCHDOG_DI_TIMEOUT_S,
            "CAN Timeout (s)":  config.WATCHDOG_CAN_TIMEOUT_S,
            "RFID Timeout (s)": config.WATCHDOG_RFID_TIMEOUT_S,
        },
        "Encoder": {
            "Enabled":            config.ENCODER_ENABLED,
            "Node ID":            config.ENCODER_NODE_ID,
            "Bitrate":            config.ENCODER_BITRATE,
            "Counts / Rev":       config.ENCODER_COUNTS_PER_REV,
            "Total Range":        config.ENCODER_TOTAL_RANGE,
            "Wheel Diameter (m)": config.ENCODER_WHEEL_DIAMETER_M,
            "TPDO COB-ID Base":   config.ENCODER_TPDO_COB_ID_BASE,
        },
        "Calibration Ramp": {
            "V Start (m/s)": config.CAL_V_START,
            "V Step (m/s)":  config.CAL_V_STEP,
            "V Max (m/s)":   config.CAL_V_MAX,
            "Dwell (s)":     config.CAL_DWELL_S,
        },
        "Features": {
            "DIO Enabled":        config.DIO_ENABLED,
            "CAN Enabled":        config.CAN_ENABLED,
            "RFID Navigation":    config.NAV_ENABLED,
            "RFID Sequence":      config.SEQ_ENABLED,
            "SLMP Enabled":       config.SLMP_ENABLED,
        },
        "Hardware": {
            "V Range":            config.V_RANGE,
            "DAC Res":            config.DAC_RES,
            "RPM/Volt (Left)":    config.RPM_PER_VOLT_LEFT,
            "RPM Offset (Left)":  config.RPM_VOLT_OFFSET_LEFT,
            "RPM/Volt (Right)":   config.RPM_PER_VOLT_RIGHT,
            "RPM Offset (Right)": config.RPM_VOLT_OFFSET_RIGHT,
        },
    })


@app.route("/api/params/features", methods=["POST"])
def api_set_features():
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503

    mode = _state.current_mode
    if mode in ("running", "emergency"):
        return jsonify({"error": f"Cannot change settings while AGV is {mode}"}), 403

    if _manager is None:
        return jsonify({"error": "driver manager unavailable"}), 503

    data    = request.get_json(silent=True) or {}
    allowed = {"CAN_ENABLED", "NAV_ENABLED", "SEQ_ENABLED", "SLMP_ENABLED"}
    updates = {k: v for k, v in data.items() if k in allowed}
    if not updates:
        return jsonify({"error": "no valid feature flags provided"}), 400
    for k, v in updates.items():
        if not isinstance(v, bool):
            return jsonify({"error": f"{k} must be true or false"}), 400

    # The manager enforces the NAV -> SEQ -> SLMP hierarchy: disabling a parent
    # cascades to children; enabling a child whose parent is off is rejected.
    # CAN is independent (the magnetic sensor — can be disabled when disconnected).
    try:
        for k, v in updates.items():
            _manager.set_enabled(k, v)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    except Exception as e:
        return jsonify({"error": f"failed to apply: {e}"}), 500

    return jsonify({
        "ok": True,
        "CAN_ENABLED":  config.CAN_ENABLED,
        "NAV_ENABLED":  config.NAV_ENABLED,
        "SEQ_ENABLED":  config.SEQ_ENABLED,
        "SLMP_ENABLED": config.SLMP_ENABLED,
    })


def _tuning_guard():
    """Shared guard for the live-tuning endpoints.

    Returns an error (body, status) tuple, or None when the request may proceed.
    Tuning is allowed only from idle ARMED / idle MANUAL — never while the AGV is
    driving a line or sitting in a fault. That restriction is what lets the values
    be applied with a plain setattr: auto_mode is rebuilt from scratch on the next
    START, so it re-reads every one of them."""
    if _state is None:
        return {"error": "state not initialised"}, 503
    mode = _state.current_mode
    if mode in ("running", "emergency", "autotune", "rotating"):
        return {"error": f"Cannot change tuning while AGV is {mode}"}, 403
    return None


def _tuning_values(config):
    return {k: getattr(config, k) for k in config.TUNABLE_SPECS}


def _save_tuning(config):
    """Persist every tunable that differs from the profile to
    config.TUNING_SAVE_FILE (atomic write), so it is the default after a
    restart. Nothing differs -> the file is removed. Returns (ok, error)."""
    import json
    path = config.TUNING_SAVE_FILE
    values = {k: getattr(config, k) for k in config.TUNABLE_SPECS
              if getattr(config, k) != config.TUNABLE_DEFAULTS[k]}
    try:
        if not values:
            if os.path.exists(path):
                os.remove(path)
        else:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump({"saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                           "map_version": config.TUNING_MAP_VERSION,
                           "values": values}, f, indent=2, sort_keys=True)
            os.replace(tmp, path)
        config.TUNING_SAVED = values
        return True, None
    except OSError as e:
        logger.error("TUNING: could not save to %s — %s", path, e)
        return False, str(e)


@app.route("/api/params/tuning", methods=["GET", "POST"])
def api_tuning():
    import config, math

    if request.method == "GET":
        mode = _state.current_mode if _state is not None else None
        return jsonify({
            "values":   _tuning_values(config),
            "specs":    {k: list(v) for k, v in config.TUNABLE_SPECS.items()},
            "defaults": config.TUNABLE_DEFAULTS,
            "saved":    sorted(config.TUNING_SAVED),
            "locked":   mode in ("running", "emergency", "autotune", "rotating"),
        })

    err = _tuning_guard()
    if err:
        return jsonify(err[0]), err[1]

    data = request.get_json(silent=True) or {}
    if not isinstance(data, dict) or not data:
        return jsonify({"error": "no parameters provided"}), 400

    # Validate the WHOLE batch before mutating anything, so a single rejected
    # field cannot leave a gain set half-applied.
    updates = {}
    for k, raw in data.items():
        if k not in config.TUNABLE_SPECS:
            return jsonify({"error": f"'{k}' is not a tunable parameter"}), 400
        try:
            v = float(raw)
        except (TypeError, ValueError):
            return jsonify({"error": f"{k} must be a number (got {raw!r})"}), 400
        if not math.isfinite(v):
            return jsonify({"error": f"{k} must be finite (got {raw!r})"}), 400
        lo, hi = config.TUNABLE_SPECS[k]
        if not (lo <= v <= hi):
            return jsonify({"error": f"{k} must be in [{lo}, {hi}] (got {v})"}), 400
        updates[k] = v

    for k, v in updates.items():
        setattr(config, k, v)

    saved, save_err = _save_tuning(config)
    logger.info("TUNING: applied%s %s", " and saved" if saved else " (NOT saved)",
                ", ".join(f"{k}={v}" for k, v in sorted(updates.items())))
    return jsonify({"ok": True, "applied": sorted(updates),
                    "saved": saved, "save_error": save_err,
                    "values": _tuning_values(config)})


@app.route("/api/params/tuning/reset", methods=["POST"])
def api_tuning_reset():
    import config

    err = _tuning_guard()
    if err:
        return jsonify(err[0]), err[1]

    for k, v in config.TUNABLE_DEFAULTS.items():
        setattr(config, k, v)

    saved, save_err = _save_tuning(config)      # nothing differs -> file removed
    logger.info("TUNING: reset all tunable parameters to profile defaults")
    return jsonify({"ok": True, "saved": saved, "save_error": save_err,
                    "values": _tuning_values(config)})


_RESTART_CMD = ["/usr/bin/systemctl", "restart", "agv-controller.service"]

@app.route("/api/web_button/mode", methods=["POST"])
def api_web_button_mode():
    """Enable or disable web button bypass mode."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    data = request.get_json(silent=True) or {}
    enable = data.get("enable")
    if not isinstance(enable, bool):
        return jsonify({"error": "enable must be true or false"}), 400
    _state.web_button_mode = enable
    if not enable:
        _state.web_btn_start     = False
        _state.web_btn_reset     = False
        _state.web_btn_manual    = False
        _state.web_btn_emergency = False
    logger.info("[WebBtn] bypass mode %s", "ON" if enable else "OFF")
    return jsonify({"ok": True, "web_button_mode": enable})


@app.route("/api/web_button/press", methods=["POST"])
def api_web_button_press():
    """Send a virtual button press. Requires web_button_mode active."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if not _state.web_button_mode:
        return jsonify({"error": "web button mode not active"}), 403

    data   = request.get_json(silent=True) or {}
    button = data.get("button")   # "start" | "reset" | "manual" | "emergency"
    value  = data.get("value", True)

    if button == "start":
        _state.web_btn_start = bool(value)
    elif button == "reset":
        _state.web_btn_reset = bool(value)
    elif button == "manual":
        _state.web_btn_manual = bool(value)
    elif button == "emergency":
        _state.web_btn_emergency = bool(value)
    else:
        return jsonify({"error": f"unknown button: {button}"}), 400

    return jsonify({"ok": True, "button": button, "value": bool(value)})


@app.route("/api/restart", methods=["POST"])
def api_restart():
    """Restart the controller systemd service (operator button on /errors).

    Allowed only when the AGV is not actively moving under auto and not in
    emergency — i.e. mode must be manual / armed / idle. Requires a passwordless
    sudoers rule for `systemctl restart agv-controller.service`."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503

    mode = _state.current_mode
    if mode in ("running", "emergency"):
        return jsonify({"error": f"Cannot restart while AGV is {mode}"}), 403

    # Verify passwordless sudo is configured for exactly this command before
    # firing, so we can return a clear error instead of silently doing nothing.
    check = subprocess.run(["sudo", "-n", "-l"] + _RESTART_CMD,
                           capture_output=True, text=True)
    if check.returncode != 0:
        return jsonify({"error": "sudo not configured for systemctl restart — "
                                 "add a /etc/sudoers.d rule"}), 500

    logger.warning("[Restart] Operator requested controller restart via HMI (mode=%s)", mode)
    # Fire and return: systemctl hands the restart job to PID 1, which performs
    # it even after this process is terminated. Popen so we don't block/wait.
    subprocess.Popen(["sudo", "-n"] + _RESTART_CMD)
    return jsonify({"ok": True})


# ── Wheel-speed calibration (open-loop encoder ramp) ──────────────────────────

@app.route("/api/calibration/status")
def api_calibration_status():
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    s = _state
    enc_age = (time.time() - s.encoder_last_rx) if s.encoder_last_rx else None
    return jsonify({
        "available":   config.ENCODER_ENABLED,
        "mode":        s.current_mode,
        "wheel":       s.calibration_wheel,
        "requested":   s.calibration_request,
        "calibration": dict(s.calibration_status),
        "encoder": {
            "connected": s.encoder_connected,
            "rpm":       round(s.encoder_rpm, 2),
            "v":         round(s.encoder_v, 4),
            "count":     s.encoder_count,
            "age_s":     round(enc_age, 2) if enc_age is not None else None,
        },
    })


# ── PID auto-tune (relay test, see autotune.py) ─────────────────────────────

@app.route("/api/autotune", methods=["GET"])
def api_autotune_status():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    return jsonify({"mode": _state.current_mode, "running": _state.current_mode == "autotune",
                    **_state.autotune_status})


@app.route("/api/autotune/start", methods=["POST"])
def api_autotune_start():
    import autotune, math
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if _state.current_mode != "armed":
        return jsonify({"error": f"Auto-tune starts only from ARMED (now {_state.current_mode})"}), 403
    raw = _state.latest_sensor
    if not (raw and raw.get("tape_detected")):
        return jsonify({"error": "no tape under the front sensor"}), 409
    data = request.get_json(silent=True) or {}
    mode = str(data.get("speed_mode", "HIGH")).upper()
    if mode not in autotune.MODE_INFO:
        return jsonify({"error": f"speed_mode must be one of {sorted(autotune.MODE_INFO)}"}), 400
    limits = {"relay_rpm": (5, 400), "err_limit_mm": (10, 90), "max_s": (5, 120), "cycles": (2, 10)}
    params = {"speed_mode": mode}
    for k, (lo, hi) in limits.items():
        try:
            v = float(data.get(k, autotune.DEFAULTS[k]))
        except (TypeError, ValueError):
            return jsonify({"error": f"{k} must be a number"}), 400
        if not (math.isfinite(v) and lo <= v <= hi):
            return jsonify({"error": f"{k} must be in [{lo}, {hi}]"}), 400
        params[k] = v
    _state.autotune_params = params
    _state.autotune_status.update(autotune.new_status())
    _state.autotune_status["phase"] = "starting"
    _state.autotune_request = True
    logger.warning("[AUTOTUNE] start requested via HMI: %s", params)
    return jsonify({"ok": True, "params": params})


@app.route("/api/autotune/stop", methods=["POST"])
def api_autotune_stop():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    _state.autotune_request = False
    return jsonify({"ok": True})


@app.route("/api/calibration/start", methods=["POST"])
def api_calibration_start():
    """Start the open-loop calibration ramp. Allowed only from ARMED (AUTO
    selector, safe, not running/manual/emergency)."""
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if not config.ENCODER_ENABLED:
        return jsonify({"error": "encoder disabled in profile"}), 403

    mode = _state.current_mode
    if mode != "armed":
        return jsonify({"error": f"Calibration starts only from ARMED (current: {mode}). "
                                 "Select AUTO and release START first."}), 403

    data  = request.get_json(silent=True) or {}
    wheel = data.get("wheel", _state.calibration_wheel)
    if wheel not in ("left", "right"):
        return jsonify({"error": "wheel must be 'left' or 'right'"}), 400

    _state.calibration_wheel   = wheel
    _state.calibration_request = True
    logger.warning("[Calib] start requested via HMI (wheel=%s)", wheel)
    return jsonify({"ok": True, "wheel": wheel})


@app.route("/api/calibration/stop", methods=["POST"])
def api_calibration_stop():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    _state.calibration_request = False
    logger.info("[Calib] stop requested via HMI")
    return jsonify({"ok": True})


@app.route("/trolley")
def trolley():
    return render_template("trolley.html")


@app.route("/api/trolley/state")
def api_trolley_state():
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503

    t = _state.trolley
    top_labels = ["M2104","M2105","M2106","M2107","M2108","M2109"]
    bot_labels = ["M2124","M2125","M2126","M2127","M2128","M2129"]

    return jsonify({
        "slmp_enabled": config.SLMP_ENABLED,
        "agv_mode":     _state.current_mode,
        "y_bits":       dict(zip(_Y_LABELS, t.y_bits)),
        "x_bits":       dict(zip(_X_LABELS, t.x_bits)),
        "top_m_bits":   dict(zip(top_labels, t.top_m_bits)),
        "bot_m_bits":   dict(zip(bot_labels, t.bot_m_bits)),
        "write_bits":   dict(t.write_bits),
    })


@app.route("/api/trolley/write", methods=["POST"])
def api_trolley_write():
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if not config.SLMP_ENABLED:
        return jsonify({"error": "SLMP disabled"}), 403
    if _state.current_mode != "manual":
        return jsonify({"error": f"AGV not in manual mode (current: {_state.current_mode})"}), 403

    data    = request.get_json(silent=True) or {}
    address = data.get("address")
    value   = data.get("value")

    if address not in _TROLLEY_WRITABLE:
        return jsonify({"error": f"address not writable: {address}"}), 400
    if value not in (0, 1):
        return jsonify({"error": "value must be 0 or 1"}), 400

    _state.trolley.write_bits[address] = value
    return jsonify({"ok": True, "address": address, "value": value})


@app.route("/api/wifi")
def api_wifi():
    try:
        out = subprocess.check_output(
            ["nmcli", "-f", "IN-USE,SIGNAL,SSID", "dev", "wifi"],
            timeout=2, text=True, stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            if line.strip().startswith("*"):
                parts = line.split()
                signal = int(parts[1])
                ssid   = " ".join(parts[2:])
                return jsonify({"ok": True, "signal": signal, "ssid": ssid})
        return jsonify({"ok": False, "signal": None, "ssid": None})
    except Exception as e:
        return jsonify({"ok": False, "signal": None, "ssid": None, "error": str(e)})


def run_server(state, engine=None, manager=None):
    global _state, _engine, _manager
    import config
    _state   = state
    _engine  = engine
    _manager = manager
    logger.info("[Flask] Dashboard at http://%s:5000", config.LOCAL_IP)
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)