import asyncio
import logging
import time

import config
import motion
import calibration
import autotune
import rotate

from core.pid import PIDController
from debugging.plotter import RunRecorder

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════════════════
#  AUTO MODE  (forward tape-following only)
# ══════════════════════════════════════════════════════════════════════════════

async def auto_mode(state, engine=None):
    """Forward tape-following PID loop.

    Sensor at front, speed modes active, sequence stops active, RFID + proximity
    marker hooks active. The AGV only ever runs auto forward.

    Args:
        engine: SequenceEngine instance. The marker-hook calls are guarded with
                `if engine`.
    """
    await motion.idle(state)
    await motion.set_forward(state, 0.0)
    error_sign = -1.0            # e = -pv  (positive pv → steer left)

    # Initialise PID with SLOW gains; forward mode will switch to HIGH on first
    # cycle if speed_mode == "HIGH".
    pid = PIDController(
        kp           = config.KP_SLOW,
        td           = config.TD_SLOW,
        n            = config.N_SLOW,
        ti           = config.TI,
        ti_deadband  = config.TI_DEADBAND,
        ti_max       = config.TI_MAX,
        dt           = config.DT,
        output_clamp = config.OUTPUT_CLAMP_RPM,
        v_red_coef   = config.V_RED_COEF_SLOW,
        sr_alpha     = config.SR_ALPHA,
        sr_cap       = config.SR_CAP,
    )

    current_target_speed = 0.0
    tape_was_lost        = False
    was_sequence_stopped = False
    last_speed_mode      = None   # tracks when gain set must change
    last_valid_pv        = None   # last accepted lateral offset (mm) for slew limit
    _dbg_count           = 0      # throttles the per-cycle PID debug log
    ff_filtered          = 0.0    # low-pass filtered curvature feedforward (rpm)
    prox_was_active      = False  # edge-detect for the DI proximity station marker

    recorder = RunRecorder()
    recorder.start()

    try:
        while True:

            # ── Emergency guard ───────────────────────────────────────────────
            # Use idle (not brake) so the wheel brakes are released and the AGV
            # can be pushed free by hand during an emergency.
            if state.emergency_active:
                await motion.idle(state)
                current_target_speed = 0.0
                pid.reset()
                last_valid_pv = None
                ff_filtered   = 0.0
                await asyncio.sleep(config.DT)
                continue

            # ── Sequence stop ─────────────────────────────────────────────────
            if state.sequence_stop:
                if not was_sequence_stopped:
                    logger.info("[AUTO] Sequence stop — holding.")
                    await motion.set_brake(state)
                    current_target_speed = 0.0
                    pid.reset()
                    last_valid_pv = None
                    ff_filtered   = 0.0
                    was_sequence_stopped = True
                await asyncio.sleep(config.DT)
                continue

            if was_sequence_stopped:
                logger.info("[AUTO] Sequence stop ended — resuming.")
                await motion.set_forward(state, 0.0)
                was_sequence_stopped = False

            # ── Target speed and PID gain selection ───────────────────────────
            if state.speed_mode == "APPROACH":
                target_speed = config.AUTO_TARGET_APPROACH_SPEED
            elif state.speed_mode == "SLOW":
                target_speed = config.AUTO_TARGET_SLOW_SPEED
            else:
                target_speed = config.AUTO_TARGET_HIGH_SPEED

            # Only switch gains when speed_mode actually changes — avoids
            # redundant recomputation of the derivative filter coefficient.
            if state.speed_mode != last_speed_mode:
                if state.speed_mode == "HIGH":
                    pid.update_gains(config.KP, config.TD, config.N,
                                     config.V_RED_COEF,
                                     clamp_hi=config.OUTPUT_CLAMP_RPM,
                                     clamp_lo=-config.OUTPUT_CLAMP_RPM)
                elif state.speed_mode == "APPROACH":
                    # Asymmetric clamp: barely steer into the one-sided guide bar
                    # (let it center the AGV mechanically), keep authority to steer
                    # off the bar. Positive output = yaw right = into the bar.
                    pid.update_gains(config.KP_APPROACH, config.TD_APPROACH,
                                     config.N_APPROACH, config.V_RED_COEF_APPROACH,
                                     clamp_hi=config.OUTPUT_CLAMP_INTO_BAR_APPROACH,
                                     clamp_lo=-config.OUTPUT_CLAMP_AWAY_APPROACH)
                else:
                    pid.update_gains(config.KP_SLOW, config.TD_SLOW,
                                     config.N_SLOW, config.V_RED_COEF_SLOW,
                                     clamp_hi=config.OUTPUT_CLAMP_RPM,
                                     clamp_lo=-config.OUTPUT_CLAMP_RPM)
                last_speed_mode = state.speed_mode

            # ── Drain sensor queue — keep only the latest frame ───────────────
            sensor = None
            while not state.sensor_queue.empty():
                sensor = await state.sensor_queue.get()

            if sensor is not None:

                logger.debug("[AUTO] left_marker=%s", sensor["left_marker"])

                # ── Tape-loss guard ───────────────────────────────────────────
                if not sensor["tape_detected"]:
                    logger.warning("[AUTO] LOST TAPE — stopping")
                    await motion.set_brake(state)
                    current_target_speed = 0.0
                    pid.reset()
                    last_valid_pv = None
                    ff_filtered   = 0.0
                    tape_was_lost = True
                    await asyncio.sleep(0.01)
                    continue

                if tape_was_lost:
                    logger.info("[AUTO] TAPE REACQUIRED — resuming")
                    await motion.set_forward(state, 0.0)
                    tape_was_lost = False

                # ── Marker hook (sequence engine) ──────────────────────────────
                if engine is not None:
                    if sensor["left_marker"]:
                        await engine.on_marker("left")
                    if sensor["right_marker"]:
                        await engine.on_marker("right")

                # ── Acceleration / deceleration ramp ──────────────────────────
                if current_target_speed < target_speed:
                    current_target_speed += config.ACCEL_RATE * config.DT
                    if current_target_speed > target_speed:
                        current_target_speed = target_speed
                elif current_target_speed > target_speed:
                    # APPROACH crawl uses a gentler decel so the RFID-armed
                    # slow-down is smooth before the prox marker hard-stops;
                    # corner/HIGH->SLOW slowdowns use the normal decel rate.
                    decel_rate = (config.APPROACH_DECEL_RATE
                                  if state.speed_mode == "APPROACH"
                                  else config.DECEL_RATE)
                    current_target_speed -= decel_rate * config.DT
                    if current_target_speed < target_speed:
                        current_target_speed = target_speed

                base_rpm = motion.mps_to_rpm(current_target_speed)

                # ── Sensor sanity + slew limit ────────────────────────────────
                # Reject physically impossible readings (a glitch frame would
                # otherwise jerk the steering). Out-of-range → skip this frame
                # and hold the last command; a large jump → clamp toward the
                # last accepted value.
                # Fork selection: follow whichever track a set_track sequence
                # chose (default left). On a single line both readings are
                # superimposed, so switching there is seamless; mid-fork the
                # slew clamp below bounds any residual step.
                pv = (sensor["right_mm"] if state.track_follow == "right"
                      else sensor["left_mm"])
                if pv is None or abs(pv) > config.SENSOR_MAX_MM:
                    _dbg_count += 1
                    if _dbg_count % 25 == 0:
                        logger.warning("[AUTO] sensor pv out of range (%s mm) — skipping frame", pv)
                    await asyncio.sleep(config.DT)
                    continue
                if last_valid_pv is not None:
                    step = pv - last_valid_pv
                    if abs(step) > config.SENSOR_MAX_STEP_MM:
                        pv = last_valid_pv + (config.SENSOR_MAX_STEP_MM
                                              if step > 0 else -config.SENSOR_MAX_STEP_MM)
                last_valid_pv = pv

                # ── Curvature feedforward (forward auto, corner zones only) ───
                # Supply the geometric turn so the PID doesn't need a standing
                # error to hold the curve. Low-pass filtered to smooth the
                # step at corner entry/exit. ff = sign*0.5*base*(W/R).
                ff_target = 0.0
                if config.FF_ENABLED and state.nav_in_corner:
                    ff_target = (config.FF_SCALE * config.FF_DIRECTION_SIGN
                                 * 0.5 * base_rpm
                                 * (config.TRACK_WIDTH / config.CURVE_RADIUS))
                # Gentle ramp when entering/deepening a corner; fast collapse on
                # exit so the residual turn command doesn't bleed into the straight
                # and over-rotate the AGV. ff_target is 0 outside corners (always
                # same sign as ff_filtered), so abs(ff_target) > abs(ff_filtered)
                # is true only while ramping up.
                _ff_alpha = (config.FF_ALPHA if abs(ff_target) > abs(ff_filtered)
                             else config.FF_EXIT_ALPHA)
                ff_filtered += _ff_alpha * (ff_target - ff_filtered)

                # ── PID compute ───────────────────────────────────────────────
                left_rpm, right_rpm, dbg = pid.compute(pv, base_rpm, error_sign,
                                                       feedforward=ff_filtered)

                left_v  = motion.rpm_to_voltage(left_rpm,  "left")
                right_v = motion.rpm_to_voltage(right_rpm, "right")

                state.set_ao(0, left_v)
                state.set_ao(1, right_v)

                state.left_rpm     = left_rpm
                state.right_rpm    = right_rpm
                state.pid_output   = dbg["output"]
                state.target_speed = current_target_speed

                recorder.record(error_mm=dbg["e"], left_rpm=left_rpm,
                                right_rpm=right_rpm, pid_output=dbg["output"],
                                d_term=dbg["d"],
                                target_speed_ms=target_speed)

                # Throttle the per-cycle PID debug line (~every 25 cycles) so an
                # accidental console DEBUG level can't flood at 100 Hz.
                _dbg_count += 1
                if _dbg_count % 25 == 0:
                    logger.debug(
                        "[%s] Target=%.2fm/s e=%+.1fmm P=%+.1f I=%+.1f D=%+.1f "
                        "out=%+.1f L=%.1f R=%.1frpm Vred=%.1frpm",
                        state.speed_mode, current_target_speed, dbg["e"],
                        dbg["p"], dbg["i"], dbg["d"], dbg["output"],
                        left_rpm, right_rpm, dbg["speed_reduction"],
                    )

            # ── DI proximity station marker ───────────────────────────────────
            # A magnetic proximity sensor on DI_PROX acts as a station marker for
            # sequences with marker_side "prox" (e.g. trolley_load_00). Fire on
            # the rising edge so one pass triggers one marker event. Independent
            # of the CAN tape frame, so it works even between sensor frames.
            # Only active when both SEQ and SLMP are enabled (the prox marker only
            # drives PLC-handshake sequences, which need SLMP).
            if (engine is not None and config.DI_PROX >= 0
                    and config.SEQ_ENABLED and config.SLMP_ENABLED):
                di = state.latest_di
                prox_active = bool(di and len(di) > config.DI_PROX and di[config.DI_PROX])
                if prox_active and not prox_was_active:
                    await engine.on_marker("prox")
                prox_was_active = prox_active

            # ── CAN timeout ───────────────────────────────────────────────────
            if time.time() - state.can_last_rx > config.CAN_TIMEOUT:
                if not tape_was_lost:
                    logger.warning("[AUTO] CAN TIMEOUT — sensor lost, stopping")
                    await motion.set_brake(state)
                    current_target_speed = 0.0
                    pid.reset()
                    last_valid_pv = None
                    ff_filtered   = 0.0
                    tape_was_lost = True

                await asyncio.sleep(config.DT)
                continue

            await asyncio.sleep(config.DT)

    finally:
        recorder.stop()


# ══════════════════════════════════════════════════════════════════════════════
#  MANUAL MODE
# ══════════════════════════════════════════════════════════════════════════════

async def manual_mode(state):
    """Handles pendant jogging, web remote and USB keypad.
    Web remote (state.web_manual_command) takes priority over physical DI buttons.
    While nothing is jogging the wheel brakes are ON (set_brake), not released.
    Emergency is fully owned by mode_manager — this task does not check it."""
    current_motion = None
    last_rpms      = None   # (rpm_high, rpm_slow) actually commanded

    while True:
        # Target motor RPM for each speed; motion helpers convert to per-wheel
        # volts. Re-read every cycle so live speed edits from the params page
        # apply immediately — this task keeps running for the whole time the
        # selector is in MANUAL, so caching these before the loop would pin them
        # to whatever the profile held at task start.
        rpm_high = motion.mps_to_rpm(config.MANUAL_TARGET_HIGH_SPEED)
        rpm_slow = motion.mps_to_rpm(config.MANUAL_TARGET_SLOW_SPEED)

        # ── Drain DI queue — keep only the latest frame ───────────────────────
        di = None
        while not state.di_queue.empty():
            di = await state.di_queue.get()

        # ── Web remote takes priority; fall back to physical DI ───────────────
        # Auto-expire stale web command (network lag / dropped connection safety)
        if state.web_manual_command is not None and time.time() > state.web_manual_expire:
            state.web_manual_command = None

        web_cmd = state.web_manual_command
        if web_cmd is not None:
            motion_state = web_cmd
        elif di is not None:
            pb_fwd   = di[config.DI_FWD]
            pb_rvs   = di[config.DI_REV]
            pb_left  = di[config.DI_LEFT]
            pb_right = di[config.DI_RIGHT]

            if pb_fwd and pb_left:       motion_state = "fwd_left"
            elif pb_fwd and pb_right:    motion_state = "fwd_right"
            elif pb_rvs and pb_left:     motion_state = "rvs_left"
            elif pb_rvs and pb_right:    motion_state = "rvs_right"
            elif pb_fwd:                 motion_state = "forward"
            elif pb_rvs:                 motion_state = "reverse"
            elif pb_left:                motion_state = "left"
            elif pb_right:               motion_state = "right"
            else:                        motion_state = "idle"
        else:
            await asyncio.sleep(0.01)
            continue

        # Re-issue on a speed change too, otherwise an edit made while a jog
        # button is held would not take effect until it is released and pressed
        # again.
        rpms = (rpm_high, rpm_slow)
        if motion_state != current_motion or rpms != last_rpms:
            if motion_state != current_motion:
                logger.debug("Manual: %s", motion_state.upper())
            if   motion_state == "fwd_left":  await motion.set_forward_left(state, rpm_high, rpm_slow)
            elif motion_state == "fwd_right": await motion.set_forward_right(state, rpm_high, rpm_slow)
            elif motion_state == "rvs_left":  await motion.set_reverse_left(state, rpm_high, rpm_slow)
            elif motion_state == "rvs_right": await motion.set_reverse_right(state, rpm_high, rpm_slow)
            elif motion_state == "forward":   await motion.set_forward(state, rpm_high)
            elif motion_state == "reverse":   await motion.set_reverse(state, rpm_high)
            elif motion_state == "left":      await motion.set_left(state, rpm_slow)
            elif motion_state == "right":     await motion.set_right(state, rpm_slow)
            elif motion_state == "idle":
                # Not jogging -> wheel brakes ON, so the AGV holds still in
                # MANUAL (released again by _drive on the next jog command).
                await motion.set_brake(state)
            current_motion = motion_state
            last_rpms      = rpms

        await asyncio.sleep(0.01)


# ══════════════════════════════════════════════════════════════════════════════
#  MODE MANAGER
# ══════════════════════════════════════════════════════════════════════════════

async def mode_manager(state, engine=None, manager=None):
    """Central state machine.

    States: None | "manual" | "armed" | "running" | "calibrate" | "emergency"

    DI conventions (from parameters.json / profile)
    ------------------------------------------------
    DI_MODE_SWITCH : HIGH = MANUAL, LOW = AUTO
    DI_EMERGENCY   : NO contact — True = triggered, False = safe
    DI_START       : momentary NO — rising edge = start
    DI_RESET       : momentary NO — rising edge = reset

    engine: SequenceEngine (Phase 3). Passed through to auto_mode and used for
            cancel_armed() on mode transitions. None until Phase 3 is wired up.
    """

    current_mode = None
    active_task  = None
    last_start   = False
    last_reset   = False
    last_phys_manual = None   # physical MANUAL/AUTO selector, for the Q-key override
    rotate_from_manual = False   # mode a 180° turn started from (back to it after)

    async def _cancel_active():
        nonlocal active_task
        if engine is not None:
            await engine.cancel_active()
        if active_task:
            active_task.cancel()
            try:
                await active_task
            except asyncio.CancelledError:
                pass
            active_task = None

    async def _flush_and_idle():
        # Setpoint tables are latest-wins, so asserting idle overwrites any
        # prior command — no queue to drain.
        await motion.idle(state)

    async def _flush_and_brake():
        await motion.set_brake(state)

    def _reset_sequence_state():
        state.speed_mode          = "HIGH"
        state.sequence_stop       = False
        state.nav_in_corner       = False
        state.track_follow        = "left"
        state.pending_sequence    = None
        state.calibration_request = False
        state.autotune_request    = False
        state.rotate_request      = None    # a T/Y press before this transition is dropped
        # selected_fork_target is intentionally NOT reset here — it's the
        # operator's chosen destination from the homepost HMI and should
        # survive a RESET/EMERGENCY/mode-switch so the operator doesn't have
        # to re-pick it every time the AGV briefly stops. It only changes when
        # the operator explicitly sets a new one (or clears it) from the HMI.
        if engine is not None:
            engine.cancel_armed()

    while True:
        if state.latest_di is None:
            await asyncio.sleep(0.01)
            continue

        di = state.latest_di

        # Physical emergency is always active regardless of web_button_mode.
        # Web emergency adds on top: either source can trigger emergency.
        phys_emergency = di[config.DI_EMERGENCY]
        if state.web_button_mode:
            emergency_safe = not phys_emergency and not state.web_btn_emergency
            switch_manual  = state.web_btn_manual
            btn_start      = state.web_btn_start
            btn_reset      = state.web_btn_reset
        else:
            emergency_safe = not phys_emergency
            phys_manual    = bool(di[config.DI_MODE_SWITCH])
            # Q key (keyboard/keypad) overrides the selector until the
            # selector itself is moved — then the physical switch rules again.
            if (last_phys_manual is not None and phys_manual != last_phys_manual
                    and state.kb_mode_override is not None):
                state.kb_mode_override = None
                logger.info("Mode selector moved — keyboard MANUAL/AUTO override cleared")
            last_phys_manual = phys_manual
            switch_manual  = (phys_manual if state.kb_mode_override is None
                              else state.kb_mode_override)
            btn_start      = di[config.DI_START]
            btn_reset      = di[config.DI_RESET]

        start_rising = btn_start and not last_start
        reset_rising = btn_reset and not last_reset

        last_start = btn_start
        last_reset = btn_reset

        # ENTER key (keyboard/keypad) = one RESET press, in any mode.
        if state.kb_reset_request:
            state.kb_reset_request = False
            reset_rising = True

        # G key (keyboard/keypad) = one START press. Only ARMED acts on it
        # (same tape check as the START button); anywhere else it is dropped.
        if state.kb_start_request:
            state.kb_start_request = False
            start_rising = True

        # E key emergency stop: latched until RESET. The same RESET press then
        # also performs the normal emergency recovery below.
        if state.kb_estop and reset_rising:
            state.kb_estop = False
            logger.info("Keyboard EMERGENCY STOP released by RESET.")
        if state.kb_estop:
            emergency_safe = False

        # Clear momentary web buttons after reading so they act as single pulses
        if state.web_button_mode:
            if state.web_btn_start: state.web_btn_start = False
            if state.web_btn_reset: state.web_btn_reset = False

        # ── System error guard (watchdog — Phase 4) ───────────────────────────
        if state.system_error and current_mode not in ("emergency", None):
            logger.error("SYSTEM ERROR — hardware driver lost, stopping")
            await _cancel_active()
            await _flush_and_brake()
            _reset_sequence_state()
            # Hold here until watchdog clears system_error
            await asyncio.sleep(0.01)
            continue

        # ══════════════════════════════════════════════════════════════════════
        #  EMERGENCY
        # ══════════════════════════════════════════════════════════════════════

        if not emergency_safe:
            if current_mode != "emergency":
                logger.critical("!! EMERGENCY — all motion stopped")
                await _cancel_active()
                # Always idle (not brake) on emergency — brakes released so the
                # AGV can be pushed free by hand.
                await _flush_and_idle()
                _reset_sequence_state()
                state.emergency_active = True
                current_mode           = "emergency"
                state.current_mode     = current_mode
            await asyncio.sleep(0.01)
            continue

        # ══════════════════════════════════════════════════════════════════════
        #  EMERGENCY RECOVERY
        # ══════════════════════════════════════════════════════════════════════

        if current_mode == "emergency":
            if not reset_rising:
                await asyncio.sleep(0.01)
                continue

            logger.info("Emergency cleared by operator RESET.")
            state.emergency_active = False

            if switch_manual:
                logger.info("Entering MANUAL.")
                await _flush_and_idle()
                active_task        = asyncio.create_task(manual_mode(state))
                current_mode       = "manual"
                state.current_mode = current_mode
            else:
                logger.info("Entering ARMED. Press START to run AUTO.")
                await _flush_and_idle()
                current_mode       = "armed"
                state.current_mode = current_mode

            await asyncio.sleep(0.01)
            continue

        # ══════════════════════════════════════════════════════════════════════
        #  NORMAL TRANSITIONS
        # ══════════════════════════════════════════════════════════════════════

        if current_mode is None:
            await _flush_and_idle()   # trigger DO/AO writer connections; they connect async
            # Always enter ARMED first so the DO/AO Modbus writers have time to
            # establish their TCP connections before manual_mode accepts pendant
            # input. The switch_manual check below fires on the very next loop
            # iteration and immediately transitions to MANUAL if the selector is
            # already in that position.
            logger.info("Startup: ARMED (initializing outputs)")
            current_mode       = "armed"
            state.current_mode = current_mode

        # ── 180° spin-turn (rotate.py) ────────────────────────────────────
        # Started by T / Y (keyboard + AGV keypad) or the manual page, from
        # MANUAL or ARMED. Ends centred on the tape, stopped, back in the mode
        # it came from (stop and wait). RESET, T/Y again, the web STOP or
        # moving the selector stops it; emergency is handled above.
        elif current_mode == "rotating":
            done = active_task is None or active_task.done()
            stop_req = state.rotate_request == "stop"
            if reset_rising or stop_req or done or bool(switch_manual) != rotate_from_manual:
                if done:
                    if active_task is not None and not active_task.cancelled():
                        exc = active_task.exception()
                        if exc is not None:
                            logger.error("[ROTATE] task error: %r", exc)
                            state.rotate_status = {"phase": "failed", "message": f"error: {exc}",
                                                   "angle": 0, "t": time.time()}
                    active_task = None
                else:
                    logger.info("[ROTATE] %s — stopping the turn.",
                                "RESET" if reset_rising else
                                "STOP" if stop_req else "mode selector moved")
                    await _cancel_active()
                await _flush_and_idle()
                state.rotate_request = None
                if switch_manual:
                    logger.info("[ROTATE] finished — back to MANUAL.")
                    active_task        = asyncio.create_task(manual_mode(state))
                    current_mode       = "manual"
                else:
                    logger.info("[ROTATE] finished — back to ARMED.")
                    current_mode       = "armed"
                state.current_mode = current_mode

        elif (current_mode in ("manual", "armed") and state.rotate_request
              and (current_mode == "manual") == bool(switch_manual)):
            direction = state.rotate_request
            state.rotate_request = None
            ok, why = (False, "nothing to stop") if direction == "stop" else rotate.check_start(state)
            if not ok:
                if direction != "stop":
                    logger.warning("[ROTATE] %s ignored — %s", direction.upper(), why)
                    state.rotate_status = {"phase": "refused", "message": why,
                                           "angle": 0, "t": time.time()}
            else:
                rotate_from_manual = current_mode == "manual"
                await _cancel_active()          # stops manual_mode's jog loop
                await _flush_and_idle()
                state.web_manual_command = None
                active_task        = asyncio.create_task(rotate.rotate_mode(state, direction))
                current_mode       = "rotating"
                state.current_mode = current_mode

        elif switch_manual and current_mode != "manual":
            logger.info("Mode switch: MANUAL")
            await _cancel_active()
            await _flush_and_idle()
            _reset_sequence_state()
            active_task        = asyncio.create_task(manual_mode(state))
            current_mode       = "manual"
            state.current_mode = current_mode

        elif not switch_manual and current_mode == "manual":
            logger.info("Mode switch: AUTO — ARMED. Place AGV on tape and press START.")
            await _cancel_active()
            await _flush_and_idle()
            current_mode       = "armed"
            state.current_mode = current_mode

        elif current_mode == "armed" and start_rising:
            tape_present  = False
            latest_sensor = None
            while not state.sensor_queue.empty():
                latest_sensor = state.sensor_queue.get_nowait()
            if latest_sensor is not None:
                tape_present = latest_sensor.get("tape_detected", False)
                await state.sensor_queue.put(latest_sensor)

            if not tape_present:
                logger.warning("START ignored — tape not detected. Place AGV on tape first.")
            else:
                logger.info("START — launching AUTO mode.")
                active_task        = asyncio.create_task(auto_mode(state, engine=engine))
                current_mode       = "running"
                state.current_mode = current_mode

        elif current_mode == "armed" and state.calibration_request:
            # Open-loop wheel-speed calibration ramp. No tape needed (no PID /
            # tape-following): the runner drives both wheels straight and logs
            # the encoder. Allowed only from ARMED (AUTO selector, safe).
            logger.info("CALIBRATION START — launching open-loop wheel-speed ramp.")
            await _cancel_active()
            await _flush_and_idle()
            active_task        = asyncio.create_task(
                calibration.calibrate_mode(state, manager))
            current_mode       = "calibrate"
            state.current_mode = current_mode

        elif current_mode == "armed" and state.autotune_request:
            # PID auto-tune (relay test) on a straight tape — see autotune.py.
            # Tape check is done by the web endpoint; RFID tags are ignored.
            logger.info("AUTOTUNE START — launching relay test.")
            await _cancel_active()
            await _flush_and_idle()
            active_task        = asyncio.create_task(autotune.autotune_mode(state))
            current_mode       = "autotune"
            state.current_mode = current_mode

        elif current_mode == "autotune" and (reset_rising or not state.autotune_request):
            logger.info("AUTOTUNE END — returning to ARMED.")
            state.autotune_request = False
            await _cancel_active()
            await _flush_and_brake()
            current_mode       = "armed"
            state.current_mode = current_mode

        elif current_mode == "calibrate" and (reset_rising or not state.calibration_request):
            logger.info("CALIBRATION STOP — returning to ARMED.")
            await _cancel_active()
            await _flush_and_idle()
            state.calibration_request = False
            current_mode       = "armed"
            state.current_mode = current_mode

        elif current_mode == "running" and reset_rising:
            logger.info("RESET — stopping AUTO, returning to ARMED.")
            await _cancel_active()
            await _flush_and_idle()
            _reset_sequence_state()
            current_mode       = "armed"
            state.current_mode = current_mode
            logger.info("ARMED. Press START to run again.")

        await asyncio.sleep(0.01)