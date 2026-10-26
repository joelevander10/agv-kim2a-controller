"""
Configuration loader.

Profile selection
-----------------
Set the AGV_ID environment variable before starting:
    AGV_ID=agv1_kim python3 main.py

If AGV_ID is not set, defaults to "agv1_kim".
Looks for: profiles/{AGV_ID}.json
Falls back to: parameters.json  (backwards-compatibility)
"""

import json
import os

_dir = os.path.dirname(os.path.abspath(__file__))

# ── Profile selection ─────────────────────────────────────────────────────────

AGV_ID = os.environ.get("AGV_ID", "agv1_kim")
print(AGV_ID)
_profile_path   = os.path.join(_dir, "profiles", f"{AGV_ID}.json")
_fallback_path  = os.path.join(_dir, "parameters.json")

if os.path.exists(_profile_path):
    _param_path = _profile_path
elif os.path.exists(_fallback_path):
    _param_path = _fallback_path
else:
    raise FileNotFoundError(
        f"No profile found for AGV_ID='{AGV_ID}'. "
        f"Expected: {_profile_path} or {_fallback_path}"
    )

try:
    with open(_param_path, "r") as f:
        _params = json.load(f)
except json.JSONDecodeError as e:
    raise ValueError(f"CRITICAL: {_param_path} is malformed: {e}")

# ── Networking ────────────────────────────────────────────────────────────────
LOCAL_IP    = _params["networking"].get("LOCAL_IP", "0.0.0.0")
DIO_IP      = _params["networking"]["DIO_IP"]
AO_IP       = _params["networking"]["AO_IP"]
MODBUS_PORT = _params["networking"]["MODBUS_PORT"]
DEVICE_ID   = _params["networking"]["DEVICE_ID"]
RFID_IP     = _params["networking"]["RFID_IP"]
RFID_PORT   = _params["networking"]["RFID_PORT"]
SLMP_IP     = _params["networking"]["SLMP_IP"]
SLMP_PORT   = _params["networking"]["SLMP_PORT"]

# ── IO Mapping ────────────────────────────────────────────────────────────────
DI_FLIPPED     = bool(_params["io_mapping"].get("DI_FLIPPED", 0))
DI_BASE        = _params["io_mapping"]["DI_BASE"]
DO_BASE        = _params["io_mapping"]["DO_BASE"]
NUM_DI         = _params["io_mapping"]["NUM_DI"]
NUM_DO         = _params["io_mapping"]["NUM_DO"]
AO_BASE        = _params["io_mapping"]["AO_BASE"]
NUM_AO         = _params["io_mapping"]["NUM_AO"]
DI_EMERGENCY   = _params["io_mapping"]["DI_EMERGENCY"]
DI_FWD         = _params["io_mapping"]["DI_FWD"]
DI_REV         = _params["io_mapping"]["DI_REV"]
DI_LEFT        = _params["io_mapping"]["DI_LEFT"]
DI_RIGHT       = _params["io_mapping"]["DI_RIGHT"]
# Physical jog pendant on DI_FWD/REV/LEFT/RIGHT. false = those inputs are
# ignored in MANUAL (drive with the USB keypad or the web MANUAL page only).
PENDANT_ENABLED = bool(_params["io_mapping"].get("PENDANT_ENABLED", True))
DI_MODE_SWITCH = _params["io_mapping"]["DI_MODE_SWITCH"]
DI_START       = _params["io_mapping"]["DI_START"]
DI_RESET       = _params["io_mapping"]["DI_RESET"]
# Magnetic proximity sensor used as a station marker for sequences (-1 = none).
DI_PROX        = _params["io_mapping"].get("DI_PROX", -1)

# ── Motor Channels (parameterized — avoids hardcoded DO 0-5 in motion.py) ────
# Falls back to the hardcoded mapping if the profile doesn't have this section
# (backwards-compat with old parameters.json).
MOTOR_CHANNELS = _params.get("motor_channels", {
    "left":  {"do_fwd": 0, "do_rev": 1, "do_brake": 2, "ao_speed": 0},
    "right": {"do_fwd": 3, "do_rev": 4, "do_brake": 5, "ao_speed": 1},
})

# ── Hardware Specs ────────────────────────────────────────────────────────────
V_RANGE = _params["hardware"]["V_RANGE"]
DAC_RES = _params["hardware"]["DAC_RES"]

# Motor speed calibration: rpm = RPM_PER_VOLT * volts + RPM_VOLT_OFFSET.
# Per-wheel (left/right differ ~6%); each falls back to the shared value if a
# profile only provides the single pair.
RPM_PER_VOLT    = _params["hardware"].get("RPM_PER_VOLT", 646.59)
RPM_VOLT_OFFSET = _params["hardware"].get("RPM_VOLT_OFFSET", -101.2)
RPM_PER_VOLT_LEFT     = _params["hardware"].get("RPM_PER_VOLT_LEFT",     RPM_PER_VOLT)
RPM_VOLT_OFFSET_LEFT  = _params["hardware"].get("RPM_VOLT_OFFSET_LEFT",  RPM_VOLT_OFFSET)
RPM_PER_VOLT_RIGHT    = _params["hardware"].get("RPM_PER_VOLT_RIGHT",    RPM_PER_VOLT)
RPM_VOLT_OFFSET_RIGHT = _params["hardware"].get("RPM_VOLT_OFFSET_RIGHT", RPM_VOLT_OFFSET)

# ── Kinematics ────────────────────────────────────────────────────────────────
WHEEL_DIAMETER      = _params["kinematics"]["WHEEL_DIAMETER"]
GEAR_RATIO          = _params["kinematics"]["GEAR_RATIO"]
WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER
# Distance from the drive-wheel axle forward to the front MGS1600 (m). Used by
# the PID auto-tune (autotune.py) to separate sensor look-ahead from drive lag.
SENSOR_LOOKAHEAD_M  = float(_params["kinematics"].get("SENSOR_LOOKAHEAD_M", 0.076))

# ── Speeds ────────────────────────────────────────────────────────────────────
MANUAL_TARGET_HIGH_SPEED     = _params["speeds"]["MANUAL_TARGET_HIGH_SPEED"]
MANUAL_TARGET_SLOW_SPEED     = _params["speeds"]["MANUAL_TARGET_SLOW_SPEED"]
AUTO_TARGET_HIGH_SPEED       = _params["speeds"]["AUTO_TARGET_HIGH_SPEED"]
AUTO_TARGET_SLOW_SPEED       = _params["speeds"]["AUTO_TARGET_SLOW_SPEED"]
AUTO_TARGET_APPROACH_SPEED   = _params["speeds"].get("AUTO_TARGET_APPROACH_SPEED", 0.1)
AUTO_TARGET_MEDIUM_SPEED     = _params["speeds"].get("AUTO_TARGET_MEDIUM_SPEED", 0.5)
AUTO_TARGET_MAX_SPEED        = _params["speeds"].get("AUTO_TARGET_MAX_SPEED", 1.0)
ACCEL_RATE                   = _params["speeds"]["ACCEL_RATE"]
DECEL_RATE                   = _params["speeds"]["DECEL_RATE"]
APPROACH_DECEL_RATE          = _params["speeds"]["APPROACH_DECEL_RATE"]

# ── PID Tuning ────────────────────────────────────────────────────────────────
KP = _params["pid_tuning"]["KP"]
TD = _params["pid_tuning"]["TD"]
N  = _params["pid_tuning"]["N"]

KP_SLOW = _params["pid_tuning"]["KP_SLOW"]
TD_SLOW = _params["pid_tuning"]["TD_SLOW"]
N_SLOW  = _params["pid_tuning"]["N_SLOW"]

KP_APPROACH         = _params["pid_tuning"].get("KP_APPROACH",         1.6)
TD_APPROACH         = _params["pid_tuning"].get("TD_APPROACH",         0.12)
N_APPROACH          = _params["pid_tuning"].get("N_APPROACH",          12)

# MEDIUM / MAX gains — fall back to the HIGH set when a profile has none.
KP_MEDIUM = _params["pid_tuning"].get("KP_MEDIUM", KP)
TD_MEDIUM = _params["pid_tuning"].get("TD_MEDIUM", TD)
N_MEDIUM  = _params["pid_tuning"].get("N_MEDIUM",  N)

KP_MAX = _params["pid_tuning"].get("KP_MAX", KP)
TD_MAX = _params["pid_tuning"].get("TD_MAX", TD)
N_MAX  = _params["pid_tuning"].get("N_MAX",  N)

TI          = _params["pid_tuning"]["TI"]
TI_DEADBAND = _params["pid_tuning"]["TI_DEADBAND"]
TI_MAX      = _params["pid_tuning"]["TI_MAX"]
DT          = _params["pid_tuning"]["DT"]

V_RED_COEF            = _params["pid_tuning"]["V_RED_COEF"]
V_RED_COEF_SLOW       = _params["pid_tuning"]["V_RED_COEF_SLOW"]
V_RED_COEF_APPROACH   = _params["pid_tuning"].get("V_RED_COEF_APPROACH", 0)
V_RED_COEF_MEDIUM     = _params["pid_tuning"].get("V_RED_COEF_MEDIUM", V_RED_COEF)
V_RED_COEF_MAX        = _params["pid_tuning"].get("V_RED_COEF_MAX", V_RED_COEF)
OUTPUT_CLAMP_RPM = _params["pid_tuning"]["OUTPUT_CLAMP_RPM"]
# Asymmetric APPROACH clamp (magnitudes, rpm). The station approach is guided by a
# one-sided physical bar on the AGV's right, so steering INTO the bar is held tight
# (the bar absorbs that correction mechanically) while steering off the bar keeps
# more authority. Default to the symmetric clamp so older profiles are unchanged.
OUTPUT_CLAMP_INTO_BAR_APPROACH = _params["pid_tuning"].get("OUTPUT_CLAMP_INTO_BAR_APPROACH", OUTPUT_CLAMP_RPM)
OUTPUT_CLAMP_AWAY_APPROACH     = _params["pid_tuning"].get("OUTPUT_CLAMP_AWAY_APPROACH",     OUTPUT_CLAMP_RPM)
SR_ALPHA         = _params["pid_tuning"]["SR_ALPHA"]
SR_CAP           = _params["pid_tuning"]["SR_CAP"]

# ── CAN Sensor ────────────────────────────────────────────────────────────────
SENSOR_COB_ID    = _params["can_sensor"]["SENSOR_COB_ID"]
FLAG_TAPE_DETECT  = _params["can_sensor"]["FLAG_TAPE_DETECT"]
FLAG_LEFT_MARKER  = _params["can_sensor"]["FLAG_LEFT_MARKER"]
FLAG_RIGHT_MARKER = _params["can_sensor"]["FLAG_RIGHT_MARKER"]
FLAG_SENSOR_FAIL  = _params["can_sensor"]["FLAG_SENSOR_FAIL"]
CAN_TIMEOUT      = _params["can_sensor"]["CAN_TIMEOUT"]
CAN_NODE_ID      = _params["can_sensor"]["CAN_NODE_ID"]

# Sensor sanity / slew limits for the lateral offset (left_mm) fed to the PID.
# Reject readings beyond ±SENSOR_MAX_MM (sensor half-width is ~80 mm); clamp
# per-cycle jumps larger than SENSOR_MAX_STEP_MM to reject glitch frames.
SENSOR_MAX_MM      = _params["can_sensor"].get("SENSOR_MAX_MM", 100.0)
SENSOR_MAX_STEP_MM = _params["can_sensor"].get("SENSOR_MAX_STEP_MM", 40.0)

# ── RFID ──────────────────────────────────────────────────────────────────────
RFID_INIT_CMD       = bytes.fromhex(_params["rfid"]["RFID_INIT_CMD_HEX"])
# RFID_COMMANDS kept for backwards-compat if old parameters.json is used
RFID_COMMANDS       = _params.get("rfid_commands", {})
SEQUENCE_STOP_DELAY = _params["rfid"]["SEQUENCE_STOP_DELAY"]

# ── Sequences (Phase 3 — sequence engine) ────────────────────────────────────
SEQUENCES = _params.get("sequences", [])

# ── Feature flags ────────────────────────────────────────────────────────────
# RFID is split into two categories with a dependency hierarchy:
#   NAV  (navigation) — corner speed-zone tags (set_speed). Independent base.
#   SEQ  (sequence)   — stop/run/PLC-handshake sequences. Requires NAV.
#   SLMP (PLC link)   — requires SEQ (and therefore NAV).
# Cascade: disabling NAV disables SEQ disables SLMP.
# Legacy profiles with a single RFID_ENABLED flag map onto both NAV and SEQ.
_feat = _params.get("features", {})
_legacy_rfid = _feat.get("RFID_ENABLED", 1)
DIO_ENABLED  = bool(_feat.get("DIO_ENABLED", 1))
CAN_ENABLED  = bool(_feat.get("CAN_ENABLED", 1))
NAV_ENABLED  = bool(_feat.get("NAV_ENABLED", _legacy_rfid))
SEQ_ENABLED  = bool(_feat.get("SEQ_ENABLED", _legacy_rfid))
SLMP_ENABLED = bool(_feat.get("SLMP_ENABLED", 0))

# Enforce the dependency hierarchy at load (a child can't outlive its parent).
if not NAV_ENABLED:
    SEQ_ENABLED = False
if not SEQ_ENABLED:
    SLMP_ENABLED = False

# ── Watchdog timeouts (Phase 4) ───────────────────────────────────────────────
_wd = _params.get("watchdog", {})
WATCHDOG_DI_TIMEOUT_S   = _wd.get("DI_TIMEOUT_S",   1.0)
WATCHDOG_CAN_TIMEOUT_S  = _wd.get("CAN_TIMEOUT_S",  1.0)
WATCHDOG_RFID_TIMEOUT_S = _wd.get("RFID_TIMEOUT_S", 5.0)

# ── Hardware poll intervals ────────────────────────────────────────────────────
_timing = _params.get("timing", {})
DI_POLL_INTERVAL = _timing.get("DI_POLL_INTERVAL", 0.02)

# ── Curvature feedforward (corner steering) ───────────────────────────────────
# In a sustained turn a pure-feedback controller settles with a standing
# cross-track error (it needs error to generate the turn). Feeding forward the
# geometric differential removes that lag and lets corners be taken faster.
#   output_ff = FF_DIRECTION_SIGN * 0.5 * base_rpm * (TRACK_WIDTH / CURVE_RADIUS)
# applied while speed_mode is one of FF_CURVE_MODES (the corner zones).
_ff = _params.get("feedforward", {})
FF_ENABLED        = bool(_ff.get("ENABLED", 0))
TRACK_WIDTH       = _ff.get("TRACK_WIDTH_M", 0.46)    # centre-to-centre wheel track
CURVE_RADIUS      = _ff.get("CURVE_RADIUS_M", 1.5)
_ff_dir           = str(_ff.get("CURVE_DIRECTION", "left")).lower()
FF_DIRECTION_SIGN = -1.0 if _ff_dir == "left" else 1.0   # left turn => output < 0
FF_CURVE_MODES    = set(_ff.get("CURVE_SPEED_MODES", ["SLOW"]))
FF_ALPHA          = _ff.get("FF_ALPHA", 0.15)            # smooths corner entry/exit
FF_EXIT_ALPHA     = _ff.get("FF_EXIT_ALPHA", FF_ALPHA)   # fast collapse on corner exit (else FF_ALPHA)
FF_SCALE          = _ff.get("FF_SCALE", 1.0)             # overall multiplier on the corner compensation


# ── Encoder (wheel-speed calibration) ─────────────────────────────────────────
# CANopen friction-wheel encoder used to measure ACTUAL wheel ground speed.
# Multi-turn: TPDO1 maps object 0x6004 (24-bit), so the count wraps at
# TOTAL_RANGE, not every revolution. Rolling contact => v_ground = omega * RADIUS.
_enc = _params.get("encoder", {})
ENCODER_ENABLED          = bool(_enc.get("ENABLED", 0))
ENCODER_NODE_ID          = int(_enc.get("NODE_ID", 1))
ENCODER_BITRATE          = int(_enc.get("BITRATE", 125000))
ENCODER_COUNTS_PER_REV   = int(_enc.get("COUNTS_PER_REV", 4096))
ENCODER_TOTAL_RANGE      = int(_enc.get("TOTAL_RANGE", 16777216))
ENCODER_WHEEL_DIAMETER_M = float(_enc.get("WHEEL_DIAMETER_M", 0.060))
ENCODER_RADIUS_M         = ENCODER_WHEEL_DIAMETER_M / 2.0
ENCODER_TPDO_COB_ID_BASE = int(_enc.get("TPDO_COB_ID_BASE", 0x180))

# ── Wheel-speed calibration ramp (open-loop straight) ─────────────────────────
_cal = _params.get("calibration", {})
CAL_V_START = float(_cal.get("V_START", 0.0))
CAL_V_STEP  = float(_cal.get("V_STEP", 0.02))
CAL_V_MAX   = float(_cal.get("V_MAX", 0.80))
CAL_DWELL_S = float(_cal.get("DWELL_S", 4.0))

# ── 180° spin-turn on the tape (rotate.py) ──────────────────────────────────
# Keys T (left/CCW) and Y (right/CW), or the ROTATE buttons on the manual page,
# in MANUAL or ARMED. Spins at SPEED_MPS (wheel speed), creeps at CREEP_MPS once
# the front sensor finds the tape again, brakes when |position| <= CENTER_TOL_MM.
# MIN/MAX_ANGLE_DEG: estimated-angle window in which the tape is accepted /
# after which the turn is aborted.
_rot = _params.get("rotate", {})
ROTATE_ENABLED            = bool(_rot.get("ENABLED", True))
ROTATE_SPEED_MPS          = float(_rot.get("SPEED_MPS", 0.10))
ROTATE_CREEP_MPS          = float(_rot.get("CREEP_MPS", 0.04))
ROTATE_ACCEL_MPS2         = float(_rot.get("ACCEL_MPS2", 0.2))
ROTATE_CENTER_TOL_MM      = float(_rot.get("CENTER_TOL_MM", 5.0))
ROTATE_MIN_ANGLE_DEG      = float(_rot.get("MIN_ANGLE_DEG", 100.0))
ROTATE_MAX_ANGLE_DEG      = float(_rot.get("MAX_ANGLE_DEG", 260.0))
ROTATE_TIMEOUT_S          = float(_rot.get("TIMEOUT_S", 40.0))
ROTATE_SETTLE_S           = float(_rot.get("SETTLE_S", 0.5))

# ── USB keypad on the AGV PC (drivers/keypad_evdev.py) ───────────────────────
# Keys: see keyboard_control.py (same key map as AGV Kayaba). Any keyboard-type
# USB device is used unless NAME_CONTAINS is set (then only devices whose name
# contains that text). EXCLUDE_NAME_CONTAINS skips built-in keyboards. GRAB =
# keys go only to the AGV (not to the desktop/terminal of the AGV PC).
_keypad = _params.get("keypad", {})
KEYPAD_ENABLED          = bool(_keypad.get("ENABLED", False))
KEYPAD_NAME_CONTAINS    = str(_keypad.get("NAME_CONTAINS", ""))
KEYPAD_EXCLUDE_CONTAINS = list(_keypad.get("EXCLUDE_NAME_CONTAINS", ["AT Translated Set 2"]))
KEYPAD_GRAB             = bool(_keypad.get("GRAB", True))


# ── Config validation (fail-fast at boot) ─────────────────────────────────────
class ConfigError(ValueError):
    """Raised at import time when a profile parameter is out of safe range."""


_MAX_SPEED_MPS = 5.0   # sanity ceiling for any commanded speed


def _validate():
    """Range-check critical parameters so the controller refuses to start on a
    bad profile rather than dividing by zero or running away mid-motion."""
    errors = []

    def check(cond, msg):
        if not cond:
            errors.append(msg)

    # Division-by-zero / runaway guards
    check(WHEEL_DIAMETER > 0, f"WHEEL_DIAMETER must be > 0 (got {WHEEL_DIAMETER})")
    check(GEAR_RATIO    > 0, f"GEAR_RATIO must be > 0 (got {GEAR_RATIO})")
    check(DT            > 0, f"DT must be > 0 (got {DT})")
    check(TI is None or TI > 0, f"TI must be > 0 (used as 1/TI) (got {TI})")
    check(V_RANGE       > 0, f"V_RANGE must be > 0 (got {V_RANGE})")
    check(DAC_RES       > 0, f"DAC_RES must be > 0 (got {DAC_RES})")
    check(RPM_PER_VOLT       != 0, f"RPM_PER_VOLT must be != 0 (got {RPM_PER_VOLT})")
    check(RPM_PER_VOLT_LEFT  != 0, f"RPM_PER_VOLT_LEFT must be != 0 (got {RPM_PER_VOLT_LEFT})")
    check(RPM_PER_VOLT_RIGHT != 0, f"RPM_PER_VOLT_RIGHT must be != 0 (got {RPM_PER_VOLT_RIGHT})")
    check(OUTPUT_CLAMP_RPM > 0, f"OUTPUT_CLAMP_RPM must be > 0 (got {OUTPUT_CLAMP_RPM})")
    check(OUTPUT_CLAMP_INTO_BAR_APPROACH > 0, f"OUTPUT_CLAMP_INTO_BAR_APPROACH must be > 0 (got {OUTPUT_CLAMP_INTO_BAR_APPROACH})")
    check(OUTPUT_CLAMP_AWAY_APPROACH     > 0, f"OUTPUT_CLAMP_AWAY_APPROACH must be > 0 (got {OUTPUT_CLAMP_AWAY_APPROACH})")
    check(N      > 0, f"N must be > 0 (got {N})")
    check(N_SLOW > 0, f"N_SLOW must be > 0 (got {N_SLOW})")
    check(N_APPROACH > 0, f"N_APPROACH must be > 0 (got {N_APPROACH})")

    # Speeds: non-negative and within a sane ceiling
    for name, val in [
        ("MANUAL_TARGET_HIGH_SPEED", MANUAL_TARGET_HIGH_SPEED),
        ("MANUAL_TARGET_SLOW_SPEED", MANUAL_TARGET_SLOW_SPEED),
        ("AUTO_TARGET_HIGH_SPEED",   AUTO_TARGET_HIGH_SPEED),
        ("AUTO_TARGET_SLOW_SPEED",   AUTO_TARGET_SLOW_SPEED),
        ("AUTO_TARGET_APPROACH_SPEED",   AUTO_TARGET_APPROACH_SPEED),
        ("AUTO_TARGET_MEDIUM_SPEED", AUTO_TARGET_MEDIUM_SPEED),
        ("AUTO_TARGET_MAX_SPEED",    AUTO_TARGET_MAX_SPEED),
    ]:
        check(0 <= val <= _MAX_SPEED_MPS,
              f"{name} must be in [0, {_MAX_SPEED_MPS}] m/s (got {val})")
    check(ACCEL_RATE > 0, f"ACCEL_RATE must be > 0 (got {ACCEL_RATE})")

    # IO mapping sanity
    check(NUM_DI >= 0, f"NUM_DI must be >= 0 (got {NUM_DI})")
    check(NUM_DO >= 0, f"NUM_DO must be >= 0 (got {NUM_DO})")
    check(NUM_AO >= 0, f"NUM_AO must be >= 0 (got {NUM_AO})")

    # Network ports
    for name, val in [("MODBUS_PORT", MODBUS_PORT), ("RFID_PORT", RFID_PORT),
                      ("SLMP_PORT", SLMP_PORT)]:
        check(1 <= val <= 65535, f"{name} must be in 1..65535 (got {val})")

    # Sensor limits
    check(SENSOR_MAX_MM      > 0, f"SENSOR_MAX_MM must be > 0 (got {SENSOR_MAX_MM})")
    check(SENSOR_MAX_STEP_MM > 0, f"SENSOR_MAX_STEP_MM must be > 0 (got {SENSOR_MAX_STEP_MM})")

    # Feedforward geometry (only matters when enabled)
    if FF_ENABLED:
        check(TRACK_WIDTH  > 0, f"TRACK_WIDTH_M must be > 0 (got {TRACK_WIDTH})")
        check(CURVE_RADIUS > 0, f"CURVE_RADIUS_M must be > 0 (got {CURVE_RADIUS})")
        check(0.0 < FF_ALPHA <= 1.0, f"FF_ALPHA must be in (0,1] (got {FF_ALPHA})")
        check(0.0 < FF_EXIT_ALPHA <= 1.0, f"FF_EXIT_ALPHA must be in (0,1] (got {FF_EXIT_ALPHA})")

    # Encoder / calibration (only matters when the encoder is enabled)
    if ENCODER_ENABLED:
        check(ENCODER_COUNTS_PER_REV > 0,
              f"encoder COUNTS_PER_REV must be > 0 (got {ENCODER_COUNTS_PER_REV})")
        check(ENCODER_TOTAL_RANGE   > 0,
              f"encoder TOTAL_RANGE must be > 0 (got {ENCODER_TOTAL_RANGE})")
        check(ENCODER_RADIUS_M      > 0,
              f"encoder WHEEL_DIAMETER_M must be > 0 (got {ENCODER_WHEEL_DIAMETER_M})")
        check(CAL_V_STEP > 0, f"calibration V_STEP must be > 0 (got {CAL_V_STEP})")
        check(0 < CAL_V_MAX <= _MAX_SPEED_MPS,
              f"calibration V_MAX must be in (0, {_MAX_SPEED_MPS}] (got {CAL_V_MAX})")
        check(CAL_DWELL_S > 0, f"calibration DWELL_S must be > 0 (got {CAL_DWELL_S})")

    if errors:
        raise ConfigError(
            f"Invalid profile '{_param_path}':\n  - " + "\n  - ".join(errors))


_validate()


# ══════════════════════════════════════════════════════════════════════════════
#  LIVE-TUNABLE PARAMETERS  (params page → /api/params/tuning)
# ══════════════════════════════════════════════════════════════════════════════
# RAM only: the endpoint mutates these module globals in place. Nothing is ever
# written back to the profile JSON, so a restart restores the profile values.
#
# The endpoint is gated to idle ARMED / idle MANUAL. That matters: auto_mode is
# built fresh on every START (modes.py), so it re-reads all of these — including
# the PIDController constructor args — with no extra plumbing. manual_mode is
# the exception, and re-reads MANUAL_TARGET_* every cycle for the same reason.
#
# _validate() cannot be reused here (no arguments, validates the whole module,
# and it does not bound KP/TD/V_RED_COEF at all), so this table carries its own
# bounds. They are deliberately wider than any profile value — a guard against
# typos and runaway input, not a tuning opinion.
TUNABLE_SPECS = {   # key: (min, max)
    # Auto speeds and ramp rates
    "AUTO_TARGET_HIGH_SPEED":         (0.0,   _MAX_SPEED_MPS),
    "AUTO_TARGET_SLOW_SPEED":         (0.0,   _MAX_SPEED_MPS),
    "AUTO_TARGET_APPROACH_SPEED":     (0.0,   _MAX_SPEED_MPS),
    "AUTO_TARGET_MEDIUM_SPEED":       (0.0,   _MAX_SPEED_MPS),
    "AUTO_TARGET_MAX_SPEED":          (0.0,   _MAX_SPEED_MPS),
    "ACCEL_RATE":                     (0.01,  10.0),
    "DECEL_RATE":                     (0.01,  10.0),
    "APPROACH_DECEL_RATE":            (0.01,  10.0),
    # Manual speeds
    "MANUAL_TARGET_HIGH_SPEED":       (0.0,   _MAX_SPEED_MPS),
    "MANUAL_TARGET_SLOW_SPEED":       (0.0,   _MAX_SPEED_MPS),
    # PID gains — HIGH.  N >= 1: the derivative filter coefficient
    # alpha = dt / ((TD / N) + dt) degenerates as N -> 0.
    "KP":                             (0.0,   50.0),
    "TD":                             (0.0,   5.0),
    "N":                              (1.0,   100.0),
    "V_RED_COEF":                     (0.0,   50.0),
    # PID gains — MEDIUM
    "KP_MEDIUM":                      (0.0,   50.0),
    "TD_MEDIUM":                      (0.0,   5.0),
    "N_MEDIUM":                       (1.0,   100.0),
    "V_RED_COEF_MEDIUM":              (0.0,   50.0),
    # PID gains — MAX
    "KP_MAX":                         (0.0,   50.0),
    "TD_MAX":                         (0.0,   5.0),
    "N_MAX":                          (1.0,   100.0),
    "V_RED_COEF_MAX":                 (0.0,   50.0),
    # PID gains — SLOW
    "KP_SLOW":                        (0.0,   50.0),
    "TD_SLOW":                        (0.0,   5.0),
    "N_SLOW":                         (1.0,   100.0),
    "V_RED_COEF_SLOW":                (0.0,   50.0),
    # PID gains — APPROACH
    "KP_APPROACH":                    (0.0,   50.0),
    "TD_APPROACH":                    (0.0,   5.0),
    "N_APPROACH":                     (1.0,   100.0),
    "V_RED_COEF_APPROACH":            (0.0,   50.0),
    # Output clamps
    "OUTPUT_CLAMP_RPM":               (1.0,   5000.0),
    "OUTPUT_CLAMP_INTO_BAR_APPROACH": (1.0,   5000.0),
    "OUTPUT_CLAMP_AWAY_APPROACH":     (1.0,   5000.0),
    # Curvature feedforward
    "FF_SCALE":                       (0.0,   5.0),
    "FF_ALPHA":                       (0.001, 1.0),
    "FF_EXIT_ALPHA":                  (0.001, 1.0),
}

# Snapshot for "reset to profile". Captured after _validate(), so it reflects the
# profile exactly, including every .get() fallback default.
TUNABLE_DEFAULTS = {k: globals()[k] for k in TUNABLE_SPECS}

# ── Saved tuning (PARAMS page APPLY / auto-tune APPLY) ────────────────────────
# Every tunable that differs from the profile is written to this file by the
# web server, and loaded here on boot so it survives a restart. RESET TO
# PROFILE on the PARAMS page deletes it. Lives in data/ (not tracked by git).
TUNING_SAVE_FILE = os.environ.get(
    "AGV_TUNING_FILE", os.path.join(_dir, "data", f"tuning_{AGV_ID}.json"))
TUNING_SAVED = {}
try:
    with open(TUNING_SAVE_FILE, "r") as _f:
        _saved = json.load(_f)
    for _k, _v in (_saved.get("values", {}) if isinstance(_saved, dict) else {}).items():
        if _k in TUNABLE_SPECS:
            _lo, _hi = TUNABLE_SPECS[_k]
            try:
                _v = float(_v)
            except (TypeError, ValueError):
                continue
            if _lo <= _v <= _hi:
                globals()[_k] = _v
                TUNING_SAVED[_k] = _v
    if TUNING_SAVED:
        print(f"Saved tuning loaded from {TUNING_SAVE_FILE}: "
              + ", ".join(f"{k}={v}" for k, v in sorted(TUNING_SAVED.items())))
except FileNotFoundError:
    pass
except (OSError, ValueError) as _e:
    print(f"WARNING: saved tuning {TUNING_SAVE_FILE} ignored ({_e})")
