"""
transport_base.py - in-process robots (mock, sim) that answer asercom
commands exactly like the firmware, so the protocol layer is exercised the
same way for every backend. Subclasses implement the robot itself.
"""
import time


class AsercomServer:
    kind = "base"

    def __init__(self, latency=0.0):
        self.latency = latency     # wall-clock delay per command (realism only)
        self.timeout = 0.5
        self.cmd_l = self.cmd_r = 0
        self._out = []

    # --- to implement
    def apply_speed(self, left, right):     # steps/s; advance the world one control period
        raise NotImplementedError

    def proximity(self):                    # 8 ints, ps0..ps7
        raise NotImplementedError

    def floor(self):                        # 3 ints
        raise NotImplementedError

    def step_counters(self):                # 2 ints
        raise NotImplementedError

    def pose(self):                         # (x, y, yaw_deg) or None
        return None

    def contacts(self):                     # control steps spent touching an obstacle
        return 0

    def layout(self):                       # world for plots: arena side, obstacles, food
        return None

    # --- transport API
    def write_line(self, text):
        if self.latency:
            time.sleep(self.latency)
        parts = text.strip().split(",")
        cmd = parts[0]
        if cmd == "D":
            self.cmd_l = max(-1000, min(1000, int(parts[1])))
            self.cmd_r = max(-1000, min(1000, int(parts[2])))
            self.apply_speed(self.cmd_l, self.cmd_r)
            self._out.append("d")
        elif cmd == "E":
            self._out.append(f"e,{self.cmd_l},{self.cmd_r}")
        elif cmd == "N":
            self._out.append("n," + ",".join(str(int(v)) for v in self.proximity()))
        elif cmd == "O":
            self._out.append("o," + ",".join(["3000"] * 8))
        elif cmd == "M":
            self._out.append("m," + ",".join(str(int(v)) for v in list(self.floor()) + [0, 0]))
        elif cmd == "Q":
            l, r = self.step_counters()
            self._out.append(f"q,{int(l)},{int(r)}")
        elif cmd == "K":
            self._out.append("k")
        elif cmd == "V":
            self._out.append(f"v,Version {self.kind} asercom emulation")
            self._out.append("HW version: emulated")     # asercom 1.2.2 sends a second line
        else:
            self._out.append("z,Command not found")

    def read_line(self):
        return self._out.pop(0) if self._out else ""

    def close(self):
        pass
