"""
protocol.py - the asercom ASCII protocol (GCtronic "Advanced sercom
protocol", e-puck 1, selector position 3), on top of any transport that
provides write_line(str), read_line() -> str and close().

  D,l,r -> d          set wheel speeds, steps/s in [-1000, 1000]
  E     -> e,l,r      get wheel speeds
  K     -> k          calibrate proximity sensors
  M     -> m,#,#,#..  floor sensors (needs ground-sensor extension)
  N     -> n,#x8..    proximity ps0..ps7 (0 = nothing, 4095 = touching)
  O     -> o,#x8..    ambient light
  Q     -> q,l,r      motor step counters
  V     -> v,Version ...
Replies start with the command letter in lowercase.
"""
from .model import MAX_STEPS


class LinkError(RuntimeError):
    pass


class BackendClosed(RuntimeError):
    """The backend is gone for good (e.g. the PyBullet window was closed):
    stop cleanly instead of retrying."""


class AsercomRobot:
    def __init__(self, transport, retries=2, max_lines=6):
        self.t = transport
        self.retries = retries
        self.max_lines = max_lines
        self.errors = 0

    def _query(self, cmd, expect):
        for _ in range(self.retries + 1):
            self.t.write_line(cmd)
            for _ in range(self.max_lines):
                line = self.t.read_line()
                if not line:
                    break                      # timeout -> resend
                if line.startswith(expect):
                    return line
            self.errors += 1
        raise LinkError(f"no '{expect}' reply to '{cmd}'")

    @staticmethod
    def _ints(line, n=None):
        vals = [int(float(v)) for v in line.split(",")[1:] if v.strip()]
        return vals if n is None else vals[:n]

    def wait_ready(self, timeout=10.0, period=0.5):
        """Send V until the robot answers, then drop whatever is pending.
        A Bluetooth link can take a few seconds to come up after the port is
        opened (about 2.5 s on macOS), and the first command after it comes
        up is often garbled. Returns the version line, or raises LinkError."""
        import time
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.t.write_line("V")
            for _ in range(self.max_lines):
                line = self.t.read_line()
                if not line:
                    break
                if line.startswith("v,"):
                    time.sleep(period)               # let the rest arrive, then drop it
                    while self.t.read_line():
                        pass
                    return line
            time.sleep(period)
        raise LinkError(f"no answer from the robot within {timeout:.0f} s")

    def version(self):
        return self._query("V", "v")

    def set_speed(self, left, right):
        l = max(-MAX_STEPS, min(MAX_STEPS, int(round(left))))
        r = max(-MAX_STEPS, min(MAX_STEPS, int(round(right))))
        self._query(f"D,{l},{r}", "d")
        return l, r

    def stop(self):
        return self.set_speed(0, 0)

    def get_speed(self):
        return self._ints(self._query("E", "e"), 2)

    def get_proximity(self):
        return self._ints(self._query("N", "n"), 8)

    def get_light(self):
        return self._ints(self._query("O", "o"), 8)

    def get_floor(self):
        return self._ints(self._query("M", "m"), 3)

    def get_steps(self):
        return self._ints(self._query("Q", "q"), 2)

    def calibrate(self):
        return self._query("K", "k")

    def close(self):
        self.t.close()
