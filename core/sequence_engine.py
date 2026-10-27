"""
Sequence Engine — declarative, config-driven sequence runner.

Sequences are defined in the profile JSON under "sequences". Each sequence has:
  - a trigger  (rfid | marker | rfid_then_marker)
  - an actions list (ordered steps executed sequentially)
  - a cooldown  (seconds to ignore the same trigger after completion)
  - requires_mode (mode that must be active for the trigger to fire)

Adding a new sequence:  edit the profile JSON — zero Python changes needed.
Adding a new action type: add one method + register it in __init__.

Trigger types
-------------
rfid              : fires immediately when the matching RFID tag is read
marker            : fires when the matching tape marker is detected by auto_mode
rfid_then_marker  : RFID arms the approach (slows down), marker executes the rest
"""

import asyncio
import logging
import time

import config

logger = logging.getLogger(__name__)


class SequenceTimeout(Exception):
    pass


class SequencePLCFault(Exception):
    """Raised when a PLC handshake never completes within its done_timeout.
    Unlike SequenceTimeout, this holds the AGV stopped (fault) until the
    operator presses RESET — it must NOT auto-resume."""
    pass


class SequenceEngine:

    def __init__(self, state, sequence_defs):
        """
        Args:
            state:          AMRState — shared state object
            sequence_defs:  list of sequence dicts loaded from config.SEQUENCES
        """
        self._state     = state
        self._sequences = sequence_defs

        # name → epoch at which the cooldown expires (0.0 = never triggered yet)
        self._cooldowns: dict = {}

        # rfid_then_marker: name → seq_def for sequences that have been RFID-armed
        # and are waiting for their marker
        self._armed: dict = {}

        # name of currently executing sequence (one at a time)
        self._active_sequence: str | None = None
        self._current_seq: dict | None = None

        # asyncio Task for the running sequence — cancelled on mode transitions
        self._active_task: asyncio.Task | None = None

        # Most recent notable sequence event (timeout / completion / PLC fault),
        # surfaced on the dashboard. None until the first event.
        # {name, kind, message, ts}
        self._last_event: dict | None = None

        # ── Built-in action registry ──────────────────────────────────────────
        self._actions: dict = {
            "set_speed":         self._act_set_speed,
            "set_track":         self._act_set_track,
            "sensor_hold":       self._act_sensor_hold,
            "wait_marker":       self._act_wait_marker,
            "stop_agv":          self._act_stop_agv,
            "resume":            self._act_resume,
            "sequence_stop":     self._act_sequence_stop,
            "wait_seconds":      self._act_wait_seconds,
            "plc_request":       self._act_plc_request,
            "wait_plc_complete": self._act_wait_plc_complete,
        }

    # ── Plugin point ─────────────────────────────────────────────────────────

    def register_action(self, name: str, handler):
        """Register a custom action handler so new action types can be added
        without modifying this file.

        handler signature:  async def handler(params: dict) -> None
        """
        self._actions[name] = handler

    # ── Feature category gating (navigation vs sequence) ─────────────────────

    @staticmethod
    def _category(seq: dict) -> str:
        """Classify a sequence as 'navigation' or 'sequence'.

        Explicit "category" in the JSON wins. Otherwise infer: a sequence whose
        actions are purely speed changes (corner slow-downs) is navigation;
        anything that stops the AGV, waits, or talks to the PLC is sequence."""
        cat = seq.get("category")
        if cat in ("navigation", "sequence"):
            return cat
        actions = seq.get("actions", [])
        if actions and all(a.get("type") == "set_speed" for a in actions):
            return "navigation"
        return "sequence"

    def _category_enabled(self, seq: dict) -> bool:
        """True if the feature flag for this sequence's category is on."""
        if self._category(seq) == "navigation":
            return config.NAV_ENABLED
        return config.SEQ_ENABLED

    # ── Public trigger interface (called by rfid_processor and auto_mode) ────

    async def on_rfid_tag(self, tag_hex: str):
        """Called by rfid_processor whenever a tag is read."""
        now = time.time()
        for seq in self._sequences:
            trig = seq["trigger"]

            if trig["type"] == "rfid" and trig["rfid_tag"] == tag_hex:
                if not self._category_enabled(seq):
                    continue
                if not self._check_preconditions(seq, now):
                    continue
                self._active_task = asyncio.create_task(self._run_sequence(seq))

            elif trig["type"] == "rfid_then_marker" and trig["rfid_tag"] == tag_hex:
                if not self._category_enabled(seq):
                    continue
                if not self._check_preconditions(seq, now):
                    continue
                # Arm: slow down for approach, record armed state. The approach
                # speed is configurable per trigger ("approach_speed"); e.g. SLOW
                # for a curved approach (with feedforward), APPROACH for a precise
                # 0.1 m/s station crawl. Defaults to APPROACH.
                approach_speed = trig.get("approach_speed", "APPROACH")
                self._armed[seq["name"]] = seq
                self._state.speed_mode       = approach_speed
                self._state.pending_sequence = seq["name"]
                # A SEQ approach arm is not a corner — clear feedforward so a
                # station approach on a straight doesn't get the corner bias.
                self._state.nav_in_corner = False
                logger.info("[SEQ] Armed '%s' — approach at %s for marker", seq["name"], approach_speed)

                # Watchdog: if the marker never shows up, don't sit armed at
                # APPROACH crawl indefinitely — fault out and resume HIGH.
                approach_timeout = trig.get("approach_timeout")
                if approach_timeout:
                    asyncio.create_task(
                        self._approach_timeout_watch(seq["name"], approach_timeout))

    async def on_marker(self, side: str):
        """Called by auto_mode whenever a left/right marker is detected in the
        sensor frame.  Fires both armed rfid_then_marker sequences and pure
        marker-triggered sequences."""
        now = time.time()

        # Check armed rfid_then_marker sequences first
        for name, seq in list(self._armed.items()):
            trig = seq["trigger"]
            if trig.get("marker_side") == side:
                if not self._category_enabled(seq):
                    continue
                del self._armed[name]
                self._state.pending_sequence = None
                logger.info("[SEQ] Marker '%s' detected — launching '%s'", side, name)
                # skip set_speed + wait_marker steps (already done during approach)
                self._active_task = asyncio.create_task(self._run_sequence(seq, skip_approach=True))

        # Check pure marker-triggered sequences
        for seq in self._sequences:
            trig = seq["trigger"]
            if trig["type"] == "marker" and trig.get("marker_side") == side:
                if not self._category_enabled(seq):
                    continue
                if not self._check_preconditions(seq, now):
                    continue
                self._active_task = asyncio.create_task(self._run_sequence(seq))

    async def cancel_active(self):
        """Cancel the currently running sequence task, if any.
        Called by mode_manager on emergency, manual switch, or stop."""
        if self._active_task and not self._active_task.done():
            self._active_task.cancel()
            try:
                await self._active_task
            except (asyncio.CancelledError, Exception):
                pass
        self._active_task = None

    async def _approach_timeout_watch(self, name: str, timeout: float):
        """If `name` is still armed `timeout` seconds after its RFID arm-trigger
        (i.e. the marker never arrived), disarm it, log a fault, and resume
        HIGH so the AGV doesn't sit crawling at APPROACH speed forever."""
        await asyncio.sleep(timeout)
        if name in self._armed:
            del self._armed[name]
            self._state.pending_sequence = None
            self._state.speed_mode       = "HIGH"
            logger.error("[SEQ] '%s' approach timed out after %.0fs — no marker, "
                         "resuming HIGH", name, timeout)
            self._record_event(name, "timeout",
                               f"approach timed out after {timeout:.0f}s — no marker, resumed HIGH")

    def cancel_armed(self):
        """Disarm all pending rfid_then_marker sequences.
        Called by mode_manager on any mode transition (emergency, manual switch,
        reset) so stale armed states don't survive the mode change."""
        if self._armed:
            logger.info("[SEQ] Disarming %d pending sequence(s)", len(self._armed))
        self._armed.clear()
        self._state.pending_sequence = None

    # ── Status (for dashboard) ────────────────────────────────────────────────

    def _record_event(self, name: str, kind: str, message: str) -> None:
        """Stash the most recent notable sequence outcome for the dashboard.
        kind is one of 'completed' | 'timeout' | 'fault'."""
        self._last_event = {
            "name": name, "kind": kind, "message": message, "ts": time.time(),
        }

    def status(self) -> dict:
        """Returns a snapshot for the Flask /api/state response."""
        now = time.time()
        last_event = None
        if self._last_event is not None:
            ev = self._last_event
            last_event = {
                "name":    ev["name"],
                "kind":    ev["kind"],
                "message": ev["message"],
                "age_s":   round(now - ev["ts"], 1),
            }
        return {
            "active":    self._active_sequence,
            "armed":     list(self._armed.keys()) or None,
            "cooldowns": {
                name: round(exp - now, 1)
                for name, exp in self._cooldowns.items()
                if exp > now
            },
            "last_event": last_event,
        }

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _check_preconditions(self, seq: dict, now: float) -> bool:
        """Returns True if the sequence may fire (mode + cooldown checks)."""
        required_mode = seq.get("requires_mode")
        if required_mode and self._state.current_mode != required_mode:
            logger.debug("[SEQ] '%s' ignored — mode is '%s', need '%s'",
                         seq["name"], self._state.current_mode, required_mode)
            return False

        cooldown_exp = self._cooldowns.get(seq["name"], 0.0)
        if now < cooldown_exp:
            logger.debug("[SEQ] '%s' ignored — cooldown %.1fs remaining",
                         seq["name"], cooldown_exp - now)
            return False

        if self._active_sequence is not None:
            logger.debug("[SEQ] '%s' ignored — '%s' already running",
                         seq["name"], self._active_sequence)
            return False

        return True

    async def _run_sequence(self, seq: dict, skip_approach: bool = False):
        """Execute the action list for a sequence.

        skip_approach=True skips the wait_marker step (the marker that triggered
        this sequence already satisfied it). set_speed steps still run, so a
        sequence can switch speed zones after the marker.
        """
        name = seq["name"]
        logger.info("[SEQ] Running '%s'", name)
        self._active_sequence = name
        self._current_seq = seq          # available to action handlers

        try:
            for action in seq["actions"]:
                atype = action["type"]

                if skip_approach and atype == "wait_marker":
                    continue

                handler = self._actions.get(atype)
                if handler is None:
                    logger.warning("[SEQ] '%s' — unknown action type '%s', skipping",
                                   name, atype)
                    continue

                await handler(action)

            logger.info("[SEQ] Completed '%s'", name)
            self._record_event(name, "completed", "completed")

        except asyncio.CancelledError:
            logger.info("[SEQ] Cancelled '%s'", name)
            self._state.sequence_stop = False
            self._state.nav_in_corner = False  # cancel always clears (mode reset follows)
            raise

        except SequenceTimeout as exc:
            logger.warning("[SEQ] '%s' timed out (%s) — resuming", name, exc)
            self._state.sequence_stop = False
            self._state.speed_mode    = "HIGH"
            self._record_event(name, "timeout", f"timed out ({exc}) — resumed HIGH")

        except SequencePLCFault as exc:
            # Hold the AGV stopped (fault). Do NOT clear sequence_stop — the
            # operator must press RESET, which returns to ARMED and clears it.
            logger.error("[SEQ] '%s' PLC FAULT (%s) — holding, operator RESET required",
                         name, exc)
            self._state.sequence_stop = True
            self._record_event(name, "fault", f"PLC fault ({exc}) — RESET required")

        finally:
            self._active_sequence = None
            self._current_seq     = None
            # Only clear nav_in_corner for SEQ sequences on completion.
            # NAV sequences leave it set (they established a speed zone that
            # persists until the next speed change or a reset).
            if self._category(seq) != "navigation":
                self._state.nav_in_corner = False
            self._cooldowns[name] = time.time() + seq.get("cooldown_s", 0.0)

    # ── Built-in action handlers ──────────────────────────────────────────────
    # Each handler receives the full action dict (params) from the JSON.

    async def _act_set_speed(self, p: dict):
        self._state.speed_mode = p["speed"]
        # Only a NAV sequence setting SLOW means "we are in a corner" — seq
        # approaches may also use SLOW but must not get curvature feedforward.
        # "corner": false (e.g. branch / fork tags) slows down WITHOUT the
        # corner feedforward, which is tuned for the main loop's left curves.
        if self._category(self._current_seq) == "navigation":
            self._state.nav_in_corner = (p["speed"] == "SLOW" and p.get("corner", True))
        else:
            self._state.nav_in_corner = False
        logger.info("[SEQ] Speed → %s", p["speed"])

    async def _act_set_track(self, p: dict):
        # Fork selection: switch which MGS1600 track the steering PID follows.
        # The sensor always reports both tracks (superimposed on a single line),
        # so this is purely a host-side choice — no command is sent to the sensor.
        side = str(p.get("side", "")).lower()
        if side not in ("left", "right"):
            logger.warning("[SEQ] set_track: invalid side %r — ignored", p.get("side"))
            return
        self._state.track_follow = side
        self._state.track_set_seq += 1          # starts a fork area (core/track_select.py)
        logger.info("[SEQ] Track follow → %s", side.upper())

    async def _act_sensor_hold(self, p: dict):
        # Floor spot that shifts the tape reading: cross the next distance_m
        # without the sensor, steering held as it was (auto_mode).
        dist = float(p.get("distance_m", 0.3))
        self._state.sensor_hold_request = dist
        logger.info("[SEQ] Sensor hold → %.2f m", dist)

    async def _act_wait_marker(self, p: dict):
        side     = p["side"]
        deadline = time.time() + p.get("timeout", 15.0)
        logger.info("[SEQ] Waiting for %s marker (timeout %.0fs)", side, p.get("timeout", 15.0))
        while time.time() < deadline:
            if self._state.current_mode != "running":
                raise asyncio.CancelledError()
            if side == "prox":
                # Magnetic proximity sensor on DI_PROX (not the CAN tape frame).
                di = self._state.latest_di
                if config.DI_PROX >= 0 and di and len(di) > config.DI_PROX \
                        and di[config.DI_PROX]:
                    return
            else:
                sen = self._state.latest_sensor
                if sen and sen.get(f"{side}_marker"):
                    return
            await asyncio.sleep(0.02)
        raise SequenceTimeout(f"wait_marker side={side}")

    async def _act_stop_agv(self, p: dict):
        logger.info("[SEQ] Stopping AGV")
        self._state.sequence_stop = True

    async def _act_resume(self, p: dict):
        self._state.sequence_stop = False
        speed = p.get("speed")
        if speed is not None:
            self._state.speed_mode = speed
        logger.info("[SEQ] Resuming — speed=%s", speed or "(unchanged)")

    async def _act_sequence_stop(self, p: dict):
        """Timed stop: stop the AGV for `duration` seconds then resume."""
        duration = p["duration"]
        logger.info("[SEQ] Timed stop %.1fs", duration)
        self._state.sequence_stop = True
        await asyncio.sleep(duration)
        self._state.sequence_stop = False
        logger.info("[SEQ] Timed stop ended")

    async def _act_wait_seconds(self, p: dict):
        await asyncio.sleep(p["duration"])

    async def _act_plc_request(self, p: dict):
        seq_num      = p["seq_num"]
        pulse        = p.get("pulse_duration", 1.0)
        self._state.plc_sequence_request      = seq_num
        self._state.plc_sequence_pulse_expire = time.time() + pulse
        logger.info("[SEQ] PLC request seq=%d (pulse %.1fs)", seq_num, pulse)

    async def _act_wait_plc_complete(self, p: dict):
        seq_num       = p["seq_num"]
        clear_timeout = p.get("clear_timeout", 5.0)
        done_timeout  = p.get("done_timeout", 30.0)

        # Phase A: wait for complete flag to go LOW (PLC acknowledged and started)
        logger.info("[SEQ] Waiting PLC seq=%d clear (flag LOW)", seq_num)
        deadline = time.time() + clear_timeout
        while self._state.plc_sequence_complete[seq_num]:
            if time.time() > deadline:
                logger.warning("[SEQ] PLC seq=%d clear timeout — proceeding", seq_num)
                return
            if self._state.current_mode != "running":
                raise asyncio.CancelledError()
            await asyncio.sleep(0.1)

        # Phase B: wait for the PLC complete flag to go HIGH, bounded by
        # done_timeout. On timeout the AGV faults (stops and holds) — it must
        # not silently resume with a possibly-unfinished load.
        logger.info("[SEQ] Waiting PLC seq=%d done (flag HIGH) — timeout %.0fs",
                    seq_num, done_timeout)
        deadline = time.time() + done_timeout
        while not self._state.plc_sequence_complete[seq_num]:
            if time.time() > deadline:
                raise SequencePLCFault(f"wait_plc_complete done seq={seq_num}")
            if self._state.current_mode != "running":
                raise asyncio.CancelledError()
            await asyncio.sleep(0.1)

        logger.info("[SEQ] PLC seq=%d complete", seq_num)
