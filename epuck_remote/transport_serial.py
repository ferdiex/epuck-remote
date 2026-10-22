"""transport_serial.py - the real robot: Bluetooth SPP serial port
(/dev/rfcomm0 on Linux, /dev/tty.e-puck_XXXX on macOS, COMx on Windows)."""
import time


class SerialTransport:
    kind = "real"

    def __init__(self, port, baud=115200, timeout=0.5, eol="\r"):
        import serial  # pyserial, imported lazily so mock/sim need nothing
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.timeout = timeout
        self.eol = eol
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def write_line(self, text):
        self.ser.write((text + self.eol).encode("ascii"))

    def read_line(self):
        return self.ser.readline().decode("ascii", "replace").strip()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def pose(self):
        return None

    def layout(self):
        return None

    def contacts(self):
        return 0
