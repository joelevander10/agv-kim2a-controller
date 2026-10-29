import asyncio
import logging
import time
import pymcprotocol

import config
from drivers.base import ActuatorDriver

logger = logging.getLogger(__name__)

_NUM_SEQ_BITS = 9

_Y_LABELS = ["Y0","Y1","Y2","Y3","Y4","Y5","Y6","Y7",
             "Y10","Y11","Y12","Y13","Y14","Y15","Y16","Y17","Y20"]
_X_LABELS = ["X0","X1","X2","X3","X4","X5","X6","X7",
             "X10","X11","X12","X13"]


def _plc_sync_cycle(plc, state) -> None:
    """
    Single synchronous read/write cycle executed in a thread-pool worker so
    the asyncio event loop is never blocked.
    """
    mode = state.current_mode

    # ── Build M2000–M2005 status vector ──────────────────────────────────────
    status = [
        int(mode not in (None, "emergency")),          # M2000  AGV.READY
        int(state.emergency_active),                   # M2001  AGV.EMERGENCY
        int(state.emergency_active),                   # M2002  AGV.REQ TROLLEY EMERGENCY
        int(mode == "manual"),                         # M2003  AGV.MANUAL
        int(mode in ("armed", "running")),             # M2004  AGV.AUTO
        int(mode == "running"),                         # M2005  AGV.RUNNING
    ]
    plc.batchwrite_bitunits(headdevice="M2000", values=status)

    # ── Build M2010–M2026 one-hot sequence vector ─────────────────────────────
    seq = [0] * _NUM_SEQ_BITS
    req = state.plc_sequence_request
    if req is not None and 0 <= req < _NUM_SEQ_BITS:
        seq[req] = 1
    plc.batchwrite_bitunits(headdevice="M2010", values=seq)

    # ── Read M0–M4 PLC input bits ─────────────────────────────────────────────
    values = plc.batchread_bitunits(headdevice="M0", readsize=5)
    state.plc_inputs = {
        "M0_PB_START_AUTO": bool(values[0]),
        "M1_MODE_MAN":      bool(values[1]),
        "M2_MODE_AUTO":     bool(values[2]),
        "M3_EMERGENCY":     bool(values[3]),
        "M4_MASTER_ON":     bool(values[4]),
    }

    # ── Read M2040–M2056 sequence complete flags ──────────────────────────────
    complete = plc.batchread_bitunits(headdevice="M2040", readsize=_NUM_SEQ_BITS)
    state.plc_sequence_complete = [bool(v) for v in complete]

    # ── Auto-clear sequence request pulse ────────────────────────────────────
    req = state.plc_sequence_request
    if req is not None:
        if time.time() >= state.plc_sequence_pulse_expire:
            state.plc_sequence_request = None

    # ── Trolley manual control — writes ──────────────────────────────────────
    wb = state.trolley.write_bits
    # M2104–M2109: top conv (M2108 is R-only → always write 0)
    plc.batchwrite_bitunits(headdevice="M2104", values=[
        wb["M2104"], wb["M2105"], wb["M2106"], wb["M2107"], 0, wb["M2109"]
    ])
    # M2124–M2129: bottom conv (M2128 is R-only → always write 0)
    plc.batchwrite_bitunits(headdevice="M2124", values=[
        wb["M2124"], wb["M2125"], wb["M2126"], wb["M2127"], 0, wb["M2129"]
    ])
    # M2212–M2213: pusher
    plc.batchwrite_bitunits(headdevice="M2212", values=[wb["M2212"], wb["M2213"]])

    # ── Trolley manual control — reads ───────────────────────────────────────
    state.trolley.top_m_bits = [bool(v) for v in
        plc.batchread_bitunits(headdevice="M2104", readsize=6)]
    state.trolley.bot_m_bits = [bool(v) for v in
        plc.batchread_bitunits(headdevice="M2124", readsize=6)]
    state.trolley.y_bits = [bool(v) for v in
        plc.batchread_bitunits(headdevice="Y0", readsize=17)]
    state.trolley.x_bits = [bool(v) for v in
        plc.batchread_bitunits(headdevice="X0", readsize=12)]


# PLC outputs that move the trolley (Y index -> name), see the TROLLEY page.
_CONVEYOR_Y = {0: "top conveyor FWD", 1: "top conveyor RVS",
               4: "bottom conveyor FWD", 5: "bottom conveyor RVS"}


def _log_conveyor_starts(state, prev_y):
    """Log every time the PLC switches a conveyor on, with what the AGV was
    doing and who last asked the PLC for a sequence — so a trolley that moves
    at the wrong time can be traced (AGV station sequence, HOME page SEQ,
    keypad / TROLLEY page in MANUAL, or the PLC on its own)."""
    y = list(state.trolley.y_bits)
    if prev_y is not None:
        for idx, name in _CONVEYOR_Y.items():
            if idx < len(y) and y[idx] and not prev_y[idx]:
                lr = state.plc.last_request
                if lr and time.time() - lr["t"] < 120:
                    who = f"last AGV request: SEQ {lr['seq']} from {lr['source']} {time.time() - lr['t']:.0f} s ago"
                else:
                    who = "NO AGV request in the last 2 min — started by the PLC itself"
                manual = [k for k, v in state.trolley.write_bits.items() if v]
                if manual:
                    who += f"; manual trolley bits on: {', '.join(sorted(manual))}"
                logger.warning("[PLC] %s started — AGV %s%s, tag %s; %s", name,
                               state.current_mode,
                               " (stopped)" if state.sequence_stop else "",
                               state.latest_rfid_tag, who)
    return y


class SLMPDriver(ActuatorDriver):

    async def run(self, state) -> None:
        loop = asyncio.get_running_loop()

        while True:
            plc = pymcprotocol.Type3E(plctype="Q")
            plc.setaccessopt(commtype="binary")

            try:
                await loop.run_in_executor(
                    None, lambda: plc.connect(config.SLMP_IP, config.SLMP_PORT)
                )
                logger.info("[SLMP] Connected to PLC at %s:%s", config.SLMP_IP, config.SLMP_PORT)

                prev_y = None
                while True:
                    await loop.run_in_executor(None, _plc_sync_cycle, plc, state)
                    prev_y = _log_conveyor_starts(state, prev_y)
                    await asyncio.sleep(0.02)

            except Exception as e:
                logger.error("[SLMP] Error: %s. Retrying in 2s...", e)

            finally:
                try:
                    plc.close()
                except Exception:
                    pass

            await asyncio.sleep(2)


# ── Module-level shim ────────────────────────────────────────────────────────

_instance = SLMPDriver()

async def slmp_handler(state):
    await _instance.run(state)
