"""
keyboard_control.py — keyboard / keypad commands for the AGV (KIM2A)
=====================================================================
One place that turns a key into an action, used by BOTH:
  * a USB keypad plugged into the AGV PC        (drivers/keypad_evdev.py)
  * the laptop keyboard (POST /api/keyboard)

Same key map as the AGV Kayaba keypad, so operators use one layout on both
AGVs. Keys for hardware this AGV does not have (pusher, auto reverse, 180°
rotate, MEDIUM/MAX speed) are recognised but refused with a message.

Movement (hold-to-drive, MANUAL mode only)
    W forward   S reverse   A spin left   D spin right
    (arrow keys / remote D-pad do the same)
    W+A / W+D / S+A / S+D = curve. Releasing the keys stops.
Actions (one press)
    1 APPROACH  2 SLOW  4 HIGH          (AUTO speed mode; ARMED/RUNNING)
    3 / 5       MEDIUM / MAX            — not on this AGV
    Q switch MANUAL <-> AUTO (overrides the selector until it is moved)
    K / L select forward track LEFT / RIGHT by hand (ARMED/RUNNING).
          A later RFID set_track sequence can change it again.
    E EMERGENCY STOP — latched, released only by RESET
    ENTER RESET (same as the RESET button: releases E-stop / emergency,
          stops AUTO back to ARMED)
    Z / X  pusher, R  auto reverse, T / Y  rotate 180° — not on this AGV
    7 / 8 / 9 / 0  cabang — not used on this AGV's keypad (pick it on the HMI)
"""
import logging
import time

logger = logging.getLogger(__name__)

MOVE_KEYS = {"w", "a", "s", "d"}
SPEED_KEYS = {"1": "APPROACH", "2": "SLOW", "4": "HIGH"}
TRACK_KEYS = {"k": "left", "l": "right"}
# Kayaba keys with no matching hardware/feature on KIM2A.
UNSUPPORTED_KEYS = {
    "3": "speed MEDIUM is not available on this AGV (APPROACH/SLOW/HIGH only)",
    "5": "speed MAX is not available on this AGV (APPROACH/SLOW/HIGH only)",
    "z": "this AGV has no pusher",
    "x": "this AGV has no pusher",
    "r": "this AGV has no AUTO REVERSE",
    "t": "this AGV has no 180° rotate",
    "y": "this AGV has no 180° rotate",
    "7": "cabang is selected on the HMI, not the keypad",
    "8": "cabang is selected on the HMI, not the keypad",
    "9": "cabang is selected on the HMI, not the keypad",
    "0": "cabang is selected on the HMI, not the keypad",
}
ACTION_KEYS = (set(SPEED_KEYS) | set(TRACK_KEYS)
               | set(UNSUPPORTED_KEYS) | {"q", "e", "enter"})
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

    if key in TRACK_KEYS:
        side = TRACK_KEYS[key]
        if mode in ("armed", "running"):
            state.track_follow = side        # read every cycle by auto_mode
            logger.info("[%s] %s — forward track %s (manual)", source, key.upper(), side.upper())
            return True, f"forward track {side.upper()}"
        return False, f"track keys work in ARMED/RUNNING (now {mode})"

    return False, f"unknown key '{key}'"
