"""
track_select.py — choose which MGS1600 track the steering follows
=================================================================
The MGS1600 always reports a LEFT and a RIGHT track. On a single tape both are
the same; at a real fork they move apart. A magnetic disturbance under the
floor next to the tape can ALSO show up as a second "track" — the AGV would
then steer toward it (it follows the left track by default).

Rule:
  * tracks together (|L − R| ≤ TRACK_SPREAD_MM)  → one tape, use it.
  * tracks apart INSIDE a fork area              → follow the side the RFID
    tag / K-L key chose (state.track_follow), as before.
  * tracks apart OUTSIDE a fork area             → "track lock": follow the
    track nearest the last good position (the real tape cannot jump in one
    10 ms frame) and ignore the other one — a floor spot, or a merge joining.

A fork area starts every time a set_track action (RFID fork tag) or the K / L
key sets the track, and ends once the two tracks have been apart and come back
together for FORK_SETTLE_S (the AGV is on the single branch tape), or after
FORK_ZONE_MAX_S if the split never shows up.

Limits: a disturbance INSIDE a fork area is not filtered; at a fork WITHOUT a
tag the AGV stays on the tape it is on (it no longer takes the left branch by
default). TRACK_LOCK_ENABLED = false restores the old behaviour.
"""
import logging

import config

logger = logging.getLogger(__name__)


class TrackSelector:

    def __init__(self):
        self.fork = False            # inside a fork area
        self._seq = None             # last state.track_set_seq seen
        self._t0 = 0.0
        self._spread_seen = False
        self._together_since = None
        self.locked = False          # currently ignoring a second track
        self.events = 0              # times a second track was ignored
        self.last_event = None       # {"left": L, "right": R, "used": pv}

    def select(self, sensor, follow, set_seq, last_pv, now):
        """Return (pv, how) — pv in mm; how is "single" | "fork" | "locked"."""
        left, right = sensor.get("left_mm"), sensor.get("right_mm")
        follow_pv = right if follow == "right" else left

        if self._seq is None:
            self._seq = set_seq
        elif set_seq != self._seq:                      # a tag / key set the track
            self._seq = set_seq
            self.fork, self._t0 = True, now
            self._spread_seen, self._together_since = False, None
            self._unlock()

        if left is None or right is None:
            return follow_pv, "single"
        spread = abs(left - right)
        apart = spread > config.TRACK_SPREAD_MM

        if self.fork:
            if apart:
                self._spread_seen, self._together_since = True, None
            elif self._spread_seen:
                self._together_since = self._together_since or now
                if now - self._together_since >= config.FORK_SETTLE_S:
                    self.fork = False
                    logger.info("[TRACK] fork passed — track lock active again")
            if self.fork and now - self._t0 > config.FORK_ZONE_MAX_S:
                self.fork = False
                logger.info("[TRACK] fork area timed out (%.0f s) — track lock active again",
                            config.FORK_ZONE_MAX_S)
            if self.fork:
                return follow_pv, "fork"

        if not apart or not config.TRACK_LOCK_ENABLED or last_pv is None:
            self._unlock()
            return follow_pv, "single"

        pv = left if abs(left - last_pv) <= abs(right - last_pv) else right
        if not self.locked:
            self.locked = True
            self.events += 1
            self.last_event = {"left": left, "right": right, "used": pv}
            logger.warning("[TRACK] second track outside a fork (L=%+d R=%+d mm) — "
                           "floor disturbance? following %+d mm, ignoring %+d mm",
                           left, right, pv, right if pv == left else left)
        return pv, "locked"

    def _unlock(self):
        self.locked = False
