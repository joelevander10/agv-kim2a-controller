"""
keyboard_control.py — keyboard / keypad commands for the AGV (KIM2A)
=====================================================================
One place that turns a key into an action, used by BOTH:
  * a USB keypad plugged into the AGV PC        (drivers/keypad_evdev.py)
  * the laptop keyboard (POST /api/keyboard)

Same key map as the AGV Kayaba keypad, so operators use one layout on both
AGVs. Keys for features this AGV does not have (auto reverse, 180° rotate,
MEDIUM/MAX speed) are recognised but refused with a message.

Movement (hold-to-drive, MANUAL mode only)
    W forward   S reverse   A spin left   D spin right
    (arrow keys / remote D-pad do the same)
    W+A / W+D / S+A / S+D = curve. Releasing the keys stops.
PLC trolley controls (hold-to-run, MANUAL only, needs SLMP enabled)
    Same bits as the buttons on the TROLLEY page; 1 while held, 0 on release.
    Z pusher EXTEND (M2212)        X pusher RETRACT (M2213)
    U top conveyor FWD (M2104)     I top RVS (M2105)     O top CLR ERR (M2106)
    B bottom conveyor FWD (M2124)  N bottom RVS (M2125)  M bottom CLR ERR (M2126)
    Opposite keys held together (Z+X, U+I, B+N) cancel each other.
    Brake / brake release stay on the TROLLEY page.
Actions (one press)
    1 APPROACH  2 SLOW  3 MEDIUM  4 HIGH  5 MAX   (AUTO speed mode; ARMED/RUNNING)
    G START AUTO from ARMED (same as the START button; needs tape)
    Q switch MANUAL <-> AUTO (overrides the selector until it is moved)
    K / L select forward track LEFT / RIGHT by hand (ARMED/RUNNING).
          A later RFID set_track sequence can change it again.
    E EMERGENCY STOP — latched, released only by RESET
    ENTER RESET (same as the RESET button: releases E-stop / emergency,
          stops AUTO back to ARMED)
    T / Y rotate 180° on the tape LEFT (CCW) / RIGHT (CW)   (MANUAL/ARMED;
          press T or Y again during the turn to stop it)
    R  auto reverse — not on this AGV
    7 / 8 / 9 / 0  cabang — not used on this AGV's keypad (pick it on the HMI)
"""
import logging
import time

import config

logger = logging.getLogger(__name__)

MOVE_KEYS = {"w", "a", "s", "d"}
SPEED_KEYS = {"1": "APPROACH", "2": "SLOW", "3": "MEDIUM", "4": "HIGH", "5": "MAX"}
TRACK_KEYS = {"k": "left", "l": "right"}
ROTATE_KEYS = {"t": "left", "y": "right"}
# PLC trolley manual bits (hold-to-run, MANUAL only) — see apply_plc().
PLC_HOLD_KEYS = {
    "z": "M2212",   # pusher EXTEND
    "x": "M2213",   # pusher RETRACT
    "u": "M2104",   # top conveyor FWD
    "i": "M2105",   # top conveyor RVS
    "o": "M2106",   # top conveyor CLR ERR
    "b": "M2124",   # bottom conveyor FWD
    "n": "M2125",   # bottom conveyor RVS
    "m": "M2126",   # bottom conveyor CLR ERR
}
PLC_KEY_NAMES = {
    "z": "pusher EXTEND", "x": "pusher RETRACT",
    "u": "top conv FWD", "i": "top conv RVS", "o": "top conv CLR ERR",
    "b": "bottom conv FWD", "n": "bottom conv RVS", "m": "bottom conv CLR ERR",
}
PLC_OPPOSITES = (("z", "x"), ("u", "i"), ("b", "n"))
# Kayaba keys with no matching hardware/feature on KIM2A.
UNSUPPORTED_KEYS = {
    "r": "this AGV has no AUTO REVERSE",
    "7": "cabang is selected on the HMI, not the keypad",
    "8": "cabang is selected on the HMI, not the keypad",
    "9": "cabang is selected on the HMI, not the keypad",
    "0": "cabang is selected on the HMI, not the keypad",
}
ACTION_KEYS = (set(SPEED_KEYS) | set(TRACK_KEYS) | set(ROTATE_KEYS)
               | set(UNSUPPORTED_KEYS) | {"q", "e", "g", "enter"})
MOVE_WATCHDOG_S = 0.4      # same 400 ms watchdog as the web jog buttons


def move_command(held):
    """Manual jog command for the set of movement keys held, or None."""
    up, down, left, right = "w" in held, "s" in held, "a" in held, "d" in held
    if up and down:
        up = down = False          # contradictory — ignore both
    if left and right:
        left = right = False
    if up and left:    return "fwd_left"
    if up and right:   return "fwd_right"
    if down and left:  return "rvs_left"
    if down and right: return "rvs_right"
    if up:             return "forward"
    if down:           return "reverse"
    if left:           return "left"
    if right:          return "right"
    return None


def apply_move(state, cmd):
    """Drive (or stop with cmd=None) through the same channel as the web jog
    buttons. Only takes effect in MANUAL; the watchdog in manual_mode stops
    the AGV if the caller stops refreshing (lost connection, unplugged keypad)."""
    if cmd is not None and state.current_mode != "manual":
        return False
    state.web_manual_command = cmd
    state.web_manual_expire = time.time() + MOVE_WATCHDOG_S
    return True


def plc_block_reason(state):
    """Why PLC hold keys can't run right now, or None if they can."""
    if not config.SLMP_ENABLED:
        return "PLC link (SLMP) is disabled"
    if state.current_mode != "manual":
        return f"PLC keys work in MANUAL only (now {state.current_mode})"
    return None


def apply_plc(state, held, prev):
    """Drive the PLC trolley bits for the PLC keys currently `held`.

    `prev` is the set of addresses this caller turned on last time; only those
    are ever turned off again, so a button held on the TROLLEY web page is not
    cancelled by the keypad. Outside MANUAL (or with SLMP off) every bit the
    caller owns is released. Returns the new set of addresses held on."""
    keys = set(held) & set(PLC_HOLD_KEYS)
    for a, b in PLC_OPPOSITES:
        if a in keys and b in keys:
            keys -= {a, b}                 # contradictory — run neither
    if plc_block_reason(state) is not None:
        keys = set()
    want = {PLC_HOLD_KEYS[k] for k in keys}
    wb = state.trolley.write_bits
    for addr in prev - want:
        wb[addr] = 0
    for addr in want:
        wb[addr] = 1                       # re-assert while held
    return want


def handle_key(state, key, source="keyboard"):
    """Run the one-press action for `key`. Returns (ok, message)."""
    key = (key or "").lower()
    mode = state.current_mode

    if key == "e":
        if not state.kb_estop:
            state.kb_estop = True
            logger.critical("[%s] E — EMERGENCY STOP (press RESET to release)", source)
        return True, "EMERGENCY STOP — press RESET to release"

    if key == "enter":
        state.kb_reset_request = True        # one RESET pulse, read by mode_manager
        logger.info("[%s] ENTER — RESET", source)
        return True, "RESET"

    if key in UNSUPPORTED_KEYS:
        return False, UNSUPPORTED_KEYS[key]

    if key == "g":
        if mode != "armed":
            return False, f"G starts AUTO only from ARMED (now {mode})"
        raw = state.latest_sensor
        if not (raw and raw.get("tape_detected")):
            return False, "no tape under the front sensor"
        state.kb_start_request = True        # one START pulse, read by mode_manager
        logger.info("[%s] G — START AUTO", source)
        return True, "START AUTO"

    if key in SPEED_KEYS:
        speed = SPEED_KEYS[key]
        if mode not in ("armed", "running"):
            return False, f"speed keys work in ARMED/RUNNING (now {mode})"
        state.speed_mode = speed
        logger.info("[%s] %s — AUTO speed mode %s", source, key, speed)
        return True, f"speed {speed}"

    if key == "q":
        want_manual = mode != "manual"
        if state.web_button_mode:
            state.web_btn_manual = want_manual
        else:
            state.kb_mode_override = want_manual
        logger.info("[%s] Q — switch to %s", source, "MANUAL" if want_manual else "AUTO")
        return True, "MANUAL" if want_manual else "AUTO (ARMED)"

    if key in ROTATE_KEYS:
        return request_rotate(state, ROTATE_KEYS[key], source)

    if key in TRACK_KEYS:
        side = TRACK_KEYS[key]
        if mode in ("armed", "running"):
            state.track_follow = side        # read every cycle by auto_mode
            logger.info("[%s] %s — forward track %s (manual)", source, key.upper(), side.upper())
            return True, f"forward track {side.upper()}"
        return False, f"track keys work in ARMED/RUNNING (now {mode})"

    return False, f"unknown key '{key}'"


def request_rotate(state, direction, source="keyboard"):
    """Ask mode_manager for a 180° turn ("left" / "right"), or "stop".
    While a turn is running, any rotate request stops it."""
    import rotate
    if state.current_mode == "rotating" or direction == "stop":
        if state.current_mode != "rotating":
            return False, "no turn running"
        state.rotate_request = "stop"
        logger.info("[%s] rotate — STOP", source)
        return True, "rotate STOP"
    ok, why = rotate.check_start(state)
    if not ok:
        return False, why
    state.rotate_request = direction
    logger.info("[%s] rotate 180° %s", source, direction.upper())
    return True, f"rotate 180° {direction.upper()}"
