import asyncio


# ── Typed state domains ───────────────────────────────────────────────────────

class SystemState:
    """Owned by mode_manager and safety_watchdog."""
    def __init__(self):
        self.current_mode     = None   # None | "manual" | "armed" | "running" | "calibrate" | "emergency"
        self.emergency_active = False
        self.system_error     = False  # set by safety_watchdog on driver comms loss
        self.system_error_detail = None  # name of the stale driver, or None


class KinematicState:
    """Owned by sequence_engine and mode_manager."""
    def __init__(self):
        self.speed_mode           = "HIGH"  # "HIGH" | "SLOW"
        self.sequence_stop        = False   # True = AGV should brake and wait
        self.pending_sequence     = None    # name of armed rfid_then_marker sequence
        self.web_manual_command   = None    # set by Flask remote; "forward"|"reverse"|"left"|"right"|"fwd_left"|"fwd_right"|"rvs_left"|"rvs_right"|None
        self.web_manual_expire    = 0.0     # epoch after which web_manual_command is treated as None
        self.nav_in_corner        = False   # True only while a NAV sequence holds SLOW; gates feedforward
        self.track_follow         = "left"  # "left" | "right" — which MGS1600 track the PID follows (fork selection)
        self.calibration_request  = False   # set by Flask to start the wheel-speed calibration ramp
        self.calibration_wheel    = "right" # "right" | "left" — which wheel the encoder is mounted on
        # Web button bypass — when True, web_btn_* override the physical DI signals
        # for START/RESET/MODE_SWITCH/EMERGENCY in mode_manager.
        self.web_button_mode      = False
        self.web_btn_start        = False   # momentary; mode_manager clears after detecting rising edge
        self.web_btn_reset        = False   # momentary; mode_manager clears after detecting rising edge
        self.web_btn_manual       = False   # level signal: True = MANUAL, False = AUTO
        self.web_btn_emergency    = False   # level signal: True = emergency active
        # Operator-selected fork destination (HMI at homepost). Tags scanned at
        # every fork point are matched against this before their action runs —
        # see rfid_processor.py. None = no target chosen; all fork_* sequences
        # with a requires_target are ignored until the operator picks one.
        self.selected_fork_target = None
        # Keyboard / keypad control (keyboard_control.py, same keys as Kayaba):
        #   kb_mode_override  None = follow the MANUAL/AUTO selector DI;
        #                     True/False = Q key forced MANUAL/AUTO (cleared
        #                     as soon as the physical selector is moved)
        #   kb_estop          E key emergency stop, latched until RESET
        #   kb_reset_request  ENTER key: one RESET pulse
        #   keypad_devices    names of USB keypads currently read on the AGV PC
        self.kb_mode_override  = None
        self.kb_estop          = False
        self.kb_reset_request  = False
        self.kb_start_request  = False   # G key: one START pulse (AUTO from ARMED)
        # 180° spin-turn (rotate.py): "left" | "right" | "stop" request from the
        # T / Y keys or the manual page; rotate_status is shown on the manual page.
        self.rotate_request    = None
        self.rotate_status     = {"phase": "idle", "message": "", "angle": 0}
        self.keypad_devices    = []
        # Track selection / floor disturbances (core/track_select.py):
        #   track_set_seq        +1 on every set_track action or K / L key —
        #                        starts a fork area (follow the chosen side)
        #   track_noise          selector status for the HMI
        #   sensor_hold_request  metres to cross without the sensor (RFID
        #                        "sensor_hold"), consumed by auto_mode
        self.track_set_seq       = 0
        self.track_noise         = {"locked": False, "events": 0, "last": None, "fork": False}
        self.sensor_hold_request = None
        self.sensor_hold_active  = False
        # Web ERRORS page "SAVE CURRENT RUN": auto_mode saves the run
        # recording so far (CSV + PNG) and starts a new one.
        self.runlog_save_request = False


class PerceptionState:
    """Written by hardware drivers; read by auto_mode and sequence_engine."""
    def __init__(self):
        self.latest_di     = None   # latest DI bits from Modbus
        self.latest_do     = {}     # {channel_no: bool} commanded DO state, updated by DOWriter
        self.latest_sensor = None   # last CAN frame dict from MGS1600
        self.can_last_rx   = 0.0    # epoch of last CAN message received
        # Motion telemetry — written by auto_mode each PID cycle
        self.left_rpm      = 0.0
        self.right_rpm     = 0.0
        self.pid_output    = 0.0
        self.target_speed  = 0.0    # current ramp target in m/s
        # Encoder telemetry — written by EncoderReader during calibration only
        self.encoder_count     = 0
        self.encoder_rpm       = 0.0   # encoder shaft rpm (signed)
        self.encoder_v         = 0.0   # AGV ground speed from encoder rim (m/s)
        self.encoder_last_rx   = 0.0   # epoch of last encoder frame (0.0 = never)
        self.encoder_connected = False


class PLCState:
    """Owned by slmp_handler; read by sequence_engine and dashboard."""
    def __init__(self):
        self.plc_inputs                = {}
        self.plc_sequence_request      = None   # int 0-16 → sets M201x; None → all LOW
        self.plc_sequence_pulse_expire = 0.0    # epoch after which request bit is cleared
        self.plc_sequence_complete     = [False] * 17  # M2040–M2056


class TrolleyState:
    """Owned by slmp_handler; read by Flask trolley dashboard."""
    def __init__(self):
        self.y_bits     = [False] * 17  # Y0–Y20 (octal), indices 0–16
        self.x_bits     = [False] * 12  # X0–X13 (octal), indices 0–11
        self.top_m_bits = [False] * 6   # M2104–M2109
        self.bot_m_bits = [False] * 6   # M2124–M2129
        self.write_bits = {             # set by Flask; written to PLC each cycle
            "M2104": 0, "M2105": 0, "M2106": 0, "M2107": 0, "M2109": 0,
            "M2124": 0, "M2125": 0, "M2126": 0, "M2127": 0, "M2129": 0,
            "M2212": 0, "M2213": 0,
        }


# ── Shared state object ───────────────────────────────────────────────────────

class AMRState:
    """
    Holds all shared queues and state domains for the AMR.
    Must be instantiated inside an active asyncio event loop.

    All fields are also accessible as flat attributes via property shims
    so existing code continues to work without changes.
    """

    def __init__(self):
        # ── Queues for cross-task communication ───────────────────────────────
        self.di_queue     = asyncio.Queue()  # DI readings
        self.rfid_queue    = asyncio.Queue()  # RFID tag reads
        self.latest_rfid_tag: str | None = None  # last tag seen (hex string)
        self.sensor_queue = asyncio.Queue()  # CAN magnetic sensor readings

        # ── Output setpoint tables (latest-wins, non-blocking) ────────────────
        # Producers (motion, auto_mode, watchdog) call set_ao/set_do; the AO/DO
        # writer tasks wake on the dirty Event and assert the *latest* values.
        # This bounds memory and prevents replay of stale commands after a
        # comms reconnect (a FIFO queue would replay the backlog).
        self.ao_setpoints = {}            # {channel_no: voltage}
        self.do_setpoints = {}            # {channel_no: bool}
        self.ao_dirty     = asyncio.Event()
        self.do_dirty     = asyncio.Event()

        # ── Wheel-speed calibration live status (read by Flask dashboard) ─────
        self.calibration_status = {
            "active":   False,
            "v_cmd":    0.0,
            "voltage":  0.0,
            "elapsed":  0.0,
            "wheel":    "right",
            "csv_path": None,
        }

        # ── PID auto-tune (autotune.py), started from the PARAMS page ─────────
        self.autotune_request = False   # set by Flask; cleared when the test ends
        self.autotune_params  = {}      # speed_mode, relay_rpm, err_limit_mm, max_s, cycles
        self.autotune_status  = {"phase": "idle", "message": "", "cycles": 0,
                                 "elapsed_s": 0.0, "params": {}, "result": None}

        # ── Typed domains ──────────────────────────────────────────────────────
        self.system     = SystemState()
        self.kinematic  = KinematicState()
        self.perception = PerceptionState()
        self.plc        = PLCState()
        self.trolley    = TrolleyState()

    # ── Output setpoint setters (synchronous: a group of calls with no await
    #    between them is applied atomically before a writer can wake) ──────────

    def set_ao(self, channel_no, voltage):
        self.ao_setpoints[channel_no] = voltage
        self.ao_dirty.set()

    def set_do(self, channel_no, active_state):
        self.do_setpoints[channel_no] = bool(active_state)
        self.do_dirty.set()

    # ── SystemState shims ─────────────────────────────────────────────────────

    @property
    def current_mode(self): return self.system.current_mode
    @current_mode.setter
    def current_mode(self, v): self.system.current_mode = v

    @property
    def emergency_active(self): return self.system.emergency_active
    @emergency_active.setter
    def emergency_active(self, v): self.system.emergency_active = v

    @property
    def system_error(self): return self.system.system_error
    @system_error.setter
    def system_error(self, v): self.system.system_error = v

    @property
    def system_error_detail(self): return self.system.system_error_detail
    @system_error_detail.setter
    def system_error_detail(self, v): self.system.system_error_detail = v

    # ── KinematicState shims ──────────────────────────────────────────────────

    @property
    def speed_mode(self): return self.kinematic.speed_mode
    @speed_mode.setter
    def speed_mode(self, v): self.kinematic.speed_mode = v

    @property
    def sequence_stop(self): return self.kinematic.sequence_stop
    @sequence_stop.setter
    def sequence_stop(self, v): self.kinematic.sequence_stop = v

    @property
    def pending_sequence(self): return self.kinematic.pending_sequence
    @pending_sequence.setter
    def pending_sequence(self, v): self.kinematic.pending_sequence = v

    @property
    def nav_in_corner(self): return self.kinematic.nav_in_corner
    @nav_in_corner.setter
    def nav_in_corner(self, v): self.kinematic.nav_in_corner = v

    @property
    def track_follow(self): return self.kinematic.track_follow
    @track_follow.setter
    def track_follow(self, v): self.kinematic.track_follow = v

    @property
    def calibration_request(self): return self.kinematic.calibration_request
    @calibration_request.setter
    def calibration_request(self, v): self.kinematic.calibration_request = v

    @property
    def calibration_wheel(self): return self.kinematic.calibration_wheel
    @calibration_wheel.setter
    def calibration_wheel(self, v): self.kinematic.calibration_wheel = v

    @property
    def web_manual_command(self): return self.kinematic.web_manual_command
    @web_manual_command.setter
    def web_manual_command(self, v): self.kinematic.web_manual_command = v

    @property
    def web_manual_expire(self): return self.kinematic.web_manual_expire
    @web_manual_expire.setter
    def web_manual_expire(self, v): self.kinematic.web_manual_expire = v

    @property
    def web_button_mode(self): return self.kinematic.web_button_mode
    @web_button_mode.setter
    def web_button_mode(self, v): self.kinematic.web_button_mode = v

    @property
    def web_btn_start(self): return self.kinematic.web_btn_start
    @web_btn_start.setter
    def web_btn_start(self, v): self.kinematic.web_btn_start = v

    @property
    def web_btn_reset(self): return self.kinematic.web_btn_reset
    @web_btn_reset.setter
    def web_btn_reset(self, v): self.kinematic.web_btn_reset = v

    @property
    def web_btn_manual(self): return self.kinematic.web_btn_manual
    @web_btn_manual.setter
    def web_btn_manual(self, v): self.kinematic.web_btn_manual = v

    @property
    def web_btn_emergency(self): return self.kinematic.web_btn_emergency
    @web_btn_emergency.setter
    def web_btn_emergency(self, v): self.kinematic.web_btn_emergency = v

    @property
    def selected_fork_target(self): return self.kinematic.selected_fork_target
    @selected_fork_target.setter
    def selected_fork_target(self, v): self.kinematic.selected_fork_target = v

    @property
    def kb_mode_override(self): return self.kinematic.kb_mode_override
    @kb_mode_override.setter
    def kb_mode_override(self, v): self.kinematic.kb_mode_override = v

    @property
    def kb_estop(self): return self.kinematic.kb_estop
    @kb_estop.setter
    def kb_estop(self, v): self.kinematic.kb_estop = v

    @property
    def kb_reset_request(self): return self.kinematic.kb_reset_request
    @kb_reset_request.setter
    def kb_reset_request(self, v): self.kinematic.kb_reset_request = v

    @property
    def kb_start_request(self): return self.kinematic.kb_start_request
    @kb_start_request.setter
    def kb_start_request(self, v): self.kinematic.kb_start_request = v

    @property
    def rotate_request(self): return self.kinematic.rotate_request
    @rotate_request.setter
    def rotate_request(self, v): self.kinematic.rotate_request = v

    @property
    def rotate_status(self): return self.kinematic.rotate_status
    @rotate_status.setter
    def rotate_status(self, v): self.kinematic.rotate_status = v

    @property
    def keypad_devices(self): return self.kinematic.keypad_devices
    @keypad_devices.setter
    def keypad_devices(self, v): self.kinematic.keypad_devices = v

    @property
    def track_set_seq(self): return self.kinematic.track_set_seq
    @track_set_seq.setter
    def track_set_seq(self, v): self.kinematic.track_set_seq = v

    @property
    def track_noise(self): return self.kinematic.track_noise
    @track_noise.setter
    def track_noise(self, v): self.kinematic.track_noise = v

    @property
    def sensor_hold_request(self): return self.kinematic.sensor_hold_request
    @sensor_hold_request.setter
    def sensor_hold_request(self, v): self.kinematic.sensor_hold_request = v

    @property
    def runlog_save_request(self): return self.kinematic.runlog_save_request
    @runlog_save_request.setter
    def runlog_save_request(self, v): self.kinematic.runlog_save_request = v

    @property
    def sensor_hold_active(self): return self.kinematic.sensor_hold_active
    @sensor_hold_active.setter
    def sensor_hold_active(self, v): self.kinematic.sensor_hold_active = v

    # ── PerceptionState shims ─────────────────────────────────────────────────

    @property
    def latest_di(self): return self.perception.latest_di
    @latest_di.setter
    def latest_di(self, v): self.perception.latest_di = v

    @property
    def latest_do(self): return self.perception.latest_do
    @latest_do.setter
    def latest_do(self, v): self.perception.latest_do = v

    @property
    def latest_sensor(self): return self.perception.latest_sensor
    @latest_sensor.setter
    def latest_sensor(self, v): self.perception.latest_sensor = v

    @property
    def can_last_rx(self): return self.perception.can_last_rx
    @can_last_rx.setter
    def can_last_rx(self, v): self.perception.can_last_rx = v

    @property
    def left_rpm(self): return self.perception.left_rpm
    @left_rpm.setter
    def left_rpm(self, v): self.perception.left_rpm = v

    @property
    def right_rpm(self): return self.perception.right_rpm
    @right_rpm.setter
    def right_rpm(self, v): self.perception.right_rpm = v

    @property
    def pid_output(self): return self.perception.pid_output
    @pid_output.setter
    def pid_output(self, v): self.perception.pid_output = v

    @property
    def target_speed(self): return self.perception.target_speed
    @target_speed.setter
    def target_speed(self, v): self.perception.target_speed = v

    @property
    def encoder_count(self): return self.perception.encoder_count
    @encoder_count.setter
    def encoder_count(self, v): self.perception.encoder_count = v

    @property
    def encoder_rpm(self): return self.perception.encoder_rpm
    @encoder_rpm.setter
    def encoder_rpm(self, v): self.perception.encoder_rpm = v

    @property
    def encoder_v(self): return self.perception.encoder_v
    @encoder_v.setter
    def encoder_v(self, v): self.perception.encoder_v = v

    @property
    def encoder_last_rx(self): return self.perception.encoder_last_rx
    @encoder_last_rx.setter
    def encoder_last_rx(self, v): self.perception.encoder_last_rx = v

    @property
    def encoder_connected(self): return self.perception.encoder_connected
    @encoder_connected.setter
    def encoder_connected(self, v): self.perception.encoder_connected = v

    # ── PLCState shims ────────────────────────────────────────────────────────

    @property
    def plc_inputs(self): return self.plc.plc_inputs
    @plc_inputs.setter
    def plc_inputs(self, v): self.plc.plc_inputs = v

    @property
    def plc_sequence_request(self): return self.plc.plc_sequence_request
    @plc_sequence_request.setter
    def plc_sequence_request(self, v): self.plc.plc_sequence_request = v

    @property
    def plc_sequence_pulse_expire(self): return self.plc.plc_sequence_pulse_expire
    @plc_sequence_pulse_expire.setter
    def plc_sequence_pulse_expire(self, v): self.plc.plc_sequence_pulse_expire = v

    @property
    def plc_sequence_complete(self): return self.plc.plc_sequence_complete
    @plc_sequence_complete.setter
    def plc_sequence_complete(self, v): self.plc.plc_sequence_complete = v