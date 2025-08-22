import os
import time
import threading
from typing import Optional
from opendbc.can.packer import CANPacker

class HyundaiCANFD:
    def __init__(self, canbus, packer: CANPacker, log):
        self.canbus = canbus
        self.packer = packer
        self.log = log
        self._adas_disable_done = False
        self._adas_disable_attempted = False
        self._adas_disable_enabled = (os.getenv("HYUNDAI_ADAS_DISABLE", "0") == "1")

    def init(self):
        # Run a bounded disable attempt only if explicitly enabled
        if not self._adas_disable_enabled:
            self.log.info("HYUNDAI_ADAS_DISABLE not set; skipping ADAS disable attempts")
            return
        self._adas_disable_attempted = True
        MAX_TRIES = 3
        for i in range(1, MAX_TRIES + 1):
            ok = False
            try:
                ok = self._attempt_adas_disable_once()
            except Exception as e:
                self.log.warn(f"ADAS disable attempt {i}/{MAX_TRIES} raised: {e}")
            if ok:
                self._adas_disable_done = True
                self.log.info(f"ADAS disable succeeded on attempt {i}")
                break
            else:
                self.log.info(f"ADAS disable failed on attempt {i}/{MAX_TRIES}; backing off")
                time.sleep(0.5)
        if not self._adas_disable_done:
            self.log.warn("ADAS disable did not succeed; continuing with stock ADAS active (no spam)")

    def _attempt_adas_disable_once(self) -> bool:
        """Implement your best-known disable sequence once.
        Return True on success, False on failure. Never loop here."""
        try:
            # TODO: implement tester-present/security access/disable sequence
            pass
        except Exception:
            return False
        return False
