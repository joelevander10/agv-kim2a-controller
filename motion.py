import config

# ── Kinematic Math ────────────────────────────────────────────────────────────
# Motor map is per-wheel: rpm = RPM_PER_VOLT[side] * V + RPM_VOLT_OFFSET[side].
# Left and right differ ~6% (measured), so the inverse used to command voltage
# is side-aware — this corrects both the speed scale and the L/R drift.

def _cal(side):
    if side == "right":
        return config.RPM_PER_VOLT_RIGHT, config.RPM_VOLT_OFFSET_RIGHT
    return config.RPM_PER_VOLT_LEFT, config.RPM_VOLT_OFFSET_LEFT

def voltage_to_rpm(voltage, side="left"):
    k, o = _cal(side)
    return k * voltage + o

def rpm_to_voltage(rpm, side="left"):
    """Inverse motor map for one wheel, clamped to [0, 5] V. rpm <= 0 → 0 V so
    a zero/negative command is a true stop (no creep from the affine offset, and
    we never command reverse through the speed AO here)."""
    if rpm <= 0:
        return 0.0
    k, o = _cal(side)
    return max(0.0, min(5.0, (rpm - o) / k))

def max_rpm(side="left"):
    """Highest motor rpm one wheel can be commanded (5 V on its speed AO)."""
    return voltage_to_rpm(5.0, side)

def steer_limit(base_rpm):
    """Largest steering difference (rpm per wheel) allowed at this speed:
    STEER_LIMIT_PCT % of the base speed, never below STEER_LIMIT_MIN_RPM.
    At 60 % the outer wheel runs at most 1.6x and the inner at least 0.4x
    the set speed. 0 switches the limit off (only the rpm clamp applies)."""
    if config.STEER_LIMIT_PCT <= 0:
        return None
    return max(config.STEER_LIMIT_MIN_RPM, config.STEER_LIMIT_PCT / 100.0 * max(0.0, base_rpm))

def keep_steering(left_rpm, right_rpm):
    """If either wheel is asked for more than it can do (5 V), slow BOTH
    wheels by the excess so the left/right difference — the steering — is
    kept. Without this, near top speed the faster wheel is clipped at 5 V and
    small steering corrections are lost. Used by autotune.py."""
    excess = max(left_rpm - max_rpm("left"), right_rpm - max_rpm("right"), 0.0)
    return left_rpm - excess, right_rpm - excess

def balance(left_rpm, right_rpm):
    """Wheel balance (config.WHEEL_BALANCE_PCT, all driving modes): scales the
    left / right wheel so a wheel that runs slightly fast is evened out.
    > 0 slows the left wheel and speeds the right one (fixes a pull to the
    RIGHT); < 0 the opposite. Not used for spins on the spot."""
    b = config.WHEEL_BALANCE_PCT
    return left_rpm * (1.0 - b / 200.0), right_rpm * (1.0 + b / 200.0)

def mps_to_rpm(v_meter_per_second):
    v_meter_per_minute = v_meter_per_second * 60
    wheel_rpm = v_meter_per_minute / config.WHEEL_CIRCUMFERENCE
    motor_rpm = wheel_rpm * config.GEAR_RATIO
    return motor_rpm

# ── Motion Commands ───────────────────────────────────────────────────────────

async def _drive(state, left_fwd: bool, left_v: float, right_fwd: bool, right_v: float):
    """
    Core drive primitive. Sets direction DO and speed AO for both wheels.
    Channel mapping is read from config.MOTOR_CHANNELS (set in profile JSON).
    """
    lch = config.MOTOR_CHANNELS["left"]
    rch = config.MOTOR_CHANNELS["right"]

    state.set_do(lch["do_fwd"],   left_fwd)
    state.set_do(lch["do_rev"],   not left_fwd)
    state.set_do(lch["do_brake"], False)

    state.set_do(rch["do_fwd"],   right_fwd)
    state.set_do(rch["do_rev"],   not right_fwd)
    state.set_do(rch["do_brake"], False)

    state.set_ao(lch["ao_speed"], left_v)
    state.set_ao(rch["ao_speed"], right_v)

# High-level helpers take target motor RPM (not voltage) and convert per-wheel.
# A target of 0 → 0 V (true stop). Callers pass mps_to_rpm(speed) or 0.0.

async def set_forward(state, rpm):
    await _drive(state, True, rpm_to_voltage(rpm, "left"),
                        True, rpm_to_voltage(rpm, "right"))

async def set_wheels(state, left_rpm, right_rpm):
    """Signed per-wheel command: positive rpm = forward, negative = reverse.
    Used by rotate.py (spin on the spot at a ramped speed)."""
    await _drive(state, left_rpm >= 0,  rpm_to_voltage(abs(left_rpm),  "left"),
                        right_rpm >= 0, rpm_to_voltage(abs(right_rpm), "right"))

async def set_reverse(state, rpm):
    await _drive(state, False, rpm_to_voltage(rpm, "left"),
                        False, rpm_to_voltage(rpm, "right"))

async def set_left(state, rpm):
    # spin left: left wheel reverse, right wheel forward
    await _drive(state, False, rpm_to_voltage(rpm, "left"),
                        True,  rpm_to_voltage(rpm, "right"))

async def set_right(state, rpm):
    # spin right: left wheel forward, right wheel reverse
    await _drive(state, True,  rpm_to_voltage(rpm, "left"),
                        False, rpm_to_voltage(rpm, "right"))

async def set_forward_left(state, rpm_fast, rpm_slow):
    # both wheels forward, right (outer) faster than left (inner)
    await _drive(state, True, rpm_to_voltage(rpm_slow, "left"),
                        True, rpm_to_voltage(rpm_fast, "right"))

async def set_forward_right(state, rpm_fast, rpm_slow):
    # both wheels forward, left (outer) faster than right (inner)
    await _drive(state, True, rpm_to_voltage(rpm_fast, "left"),
                        True, rpm_to_voltage(rpm_slow, "right"))

async def set_reverse_left(state, rpm_fast, rpm_slow):
    # both wheels reverse, right (outer) faster than left (inner)
    await _drive(state, False, rpm_to_voltage(rpm_slow, "left"),
                        False, rpm_to_voltage(rpm_fast, "right"))

async def set_reverse_right(state, rpm_fast, rpm_slow):
    # both wheels reverse, left (outer) faster than right (inner)
    await _drive(state, False, rpm_to_voltage(rpm_fast, "left"),
                        False, rpm_to_voltage(rpm_slow, "right"))

async def set_brake(state):
    lch = config.MOTOR_CHANNELS["left"]
    rch = config.MOTOR_CHANNELS["right"]
    state.set_ao(lch["ao_speed"], 0.0)
    state.set_ao(rch["ao_speed"], 0.0)
    state.set_do(lch["do_fwd"],   False)
    state.set_do(lch["do_rev"],   False)
    state.set_do(lch["do_brake"], True)
    state.set_do(rch["do_fwd"],   False)
    state.set_do(rch["do_rev"],   False)
    state.set_do(rch["do_brake"], True)

async def idle(state):
    lch = config.MOTOR_CHANNELS["left"]
    rch = config.MOTOR_CHANNELS["right"]
    state.set_ao(lch["ao_speed"], 0.0)
    state.set_ao(rch["ao_speed"], 0.0)
    state.set_do(lch["do_fwd"],   False)
    state.set_do(lch["do_rev"],   False)
    state.set_do(lch["do_brake"], False)
    state.set_do(rch["do_fwd"],   False)
    state.set_do(rch["do_rev"],   False)
    state.set_do(rch["do_brake"], False)
