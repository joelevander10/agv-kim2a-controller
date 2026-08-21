"""
rfid_processor.py — RFID tag dispatcher
========================================
Consumes tags from state.rfid_queue and forwards them to the SequenceEngine.

Fork-target filtering
----------------------
Fork sequences ("fork_to_point*") are tagged in the profile JSON with a
`requires_target` field inside their trigger. These tags are physically read
on EVERY pass through the loop regardless of which branch the operator wants
— so before forwarding to the engine, we check the scanned tag's required
target against state.selected_fork_target (set from the HMI at homepost). If
it doesn't match, the tag is dropped here and the engine never sees it — no
change needed in core/sequence_engine.py.

Tags with no `requires_target` (checkpoints, speed zones, trolley sequences,
the merge-revert tag, etc.) are unaffected and always forwarded as before.

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

        required_target = tag_targets.get(tag)
        if required_target is not None and required_target != state.selected_fork_target:
            logger.debug(
                "[RFID] Tag %s ignored — requires target '%s', current target is '%s'",
                tag, required_target, state.selected_fork_target,
            )
            continue

        await engine.on_rfid_tag(tag)