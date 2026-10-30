"""
rfid_processor.py — RFID tag dispatcher
========================================
Consumes tags from state.rfid_queue and forwards them to the SequenceEngine.

Fork-target (CABANG) gating
---------------------------
Branch tags are read on EVERY pass, whichever branch the operator picked. The
gating is done per sequence in core/sequence_engine.py (_check_preconditions):
  trigger "requires_target": point_n  -> only when CABANG n is selected
  trigger "unless_target":   point_n  -> only when CABANG n is NOT selected
so one tag can do different things, e.g. 00C1 = MEDIUM + left track when SCB
(CABANG 1) is selected, HIGH when another branch (or none) is selected.
All tags are forwarded to the engine.

All sequence behaviour is still defined in the profile JSON under "sequences".
To add a new action type, add one method to core/sequence_engine.py.
"""

import logging
import config

logger = logging.getLogger(__name__)


def _build_target_map():
    """tag -> required_target, built once from the profile's sequences list.
    Only rfid triggers with a 'requires_target' key are included."""
    tag_targets = {}
    for seq in config.SEQUENCES:
        trigger = seq.get("trigger", {})
        if trigger.get("type") == "rfid" and "requires_target" in trigger:
            tag_targets[trigger["rfid_tag"]] = trigger["requires_target"]
        elif trigger.get("type") == "rfid" and "unless_target" in trigger:
            tag_targets.setdefault(trigger["rfid_tag"], "not " + trigger["unless_target"])
    return tag_targets


async def rfid_processor(state, engine):
    """Thin dispatcher: reads tags from the queue, filters fork tags against
    the operator-selected target, and passes the rest to the engine.

    Args:
        state:  AMRState shared object
        engine: SequenceEngine instance (core/sequence_engine.py)
    """
    tag_targets = _build_target_map()
    if tag_targets:
        logger.info("[RFID Processor] Fork-target gating active: %s", tag_targets)

    logger.info("[RFID Processor] Started.")
    while True:
        tag = await state.rfid_queue.get()
        state.latest_rfid_tag = tag
        logger.debug("[RFID] Tag read: %s", tag)

        # PID auto-tune drives its own test — tags (speed zones, set_track,
        # stops) must not change anything underneath it.
        if state.current_mode == "autotune":
            logger.debug("[RFID] Tag %s ignored — auto-tune running", tag)
            continue

        await engine.on_rfid_tag(tag)