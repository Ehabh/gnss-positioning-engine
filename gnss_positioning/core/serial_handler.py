"""
Serial Port Handler for GNSS Receivers.

Manages serial connections to GNSS receivers, including:
    - Port discovery and enumeration
    - Threaded reading with buffering
    - Connection management
    - Receiver configuration commands
    - Optional NMEA sentence extraction from mixed RTCM/NMEA streams
"""

import threading
import logging
import time
from typing import Optional, Callable, List

logger = logging.getLogger(__name__)

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    logger.warning("pyserial not installed. Install with: pip install pyserial")


class SerialHandler:
    """Manages serial connection to a GNSS receiver.

    Runs a background thread to read data from the serial port
    and dispatches raw bytes to a callback.

    Usage:
        handler = SerialHandler()
        handler.on_data = my_callback
        handler.connect('/dev/ttyUSB0', 115200)
        ...
        handler.disconnect()
    """

    def __init__(self, name: str = "Receiver"):
        self.name = name
        self._port: Optional[serial.Serial] = None
        self._read_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()

        # Callback for received data
        self.on_data: Optional[Callable[[bytes], None]] = None
        self.on_nmea: Optional[Callable[[str], None]] = None
        self._line_buffer = bytearray()

        # Statistics
        self.bytes_received = 0
        self.connect_time: Optional[float] = None

    @staticmethod
    def list_ports() -> List[dict]:
        """List available serial ports.

        Returns:
            List of dicts with port info: device, description, hwid
        """
        if not HAS_SERIAL:
            return []

        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append({
                'device': port.device,
                'description': port.description,
                'hwid': port.hwid,
                'manufacturer': port.manufacturer or '',
                'product': port.product or '',
                'vid': port.vid,
                'pid': port.pid,
            })

        # Sort: USB serial devices first
        ports.sort(key=lambda p: (
            0 if 'USB' in (p['description'] or '') else 1,
            p['device']
        ))

        return ports

    def connect(self, port: str, baudrate: int = 115200,
                timeout: float = 1.0) -> bool:
        """Connect to serial port.

        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate: Baud rate (typical default: 115200)
            timeout: Read timeout [s]

        Returns:
            True if connection successful
        """
        if not HAS_SERIAL:
            logger.error("pyserial not available")
            return False

        with self._lock:
            if self._port and self._port.is_open:
                logger.warning(f"{self.name}: Already connected")
                return False

            try:
                self._port = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=timeout,
                )
                self._running = True
                self.connect_time = time.time()
                self.bytes_received = 0

                # Start read thread
                self._read_thread = threading.Thread(
                    target=self._read_loop,
                    name=f"Serial-{self.name}",
                    daemon=True,
                )
                self._read_thread.start()

                logger.info(f"{self.name}: Connected to {port} @ {baudrate}")
                return True

            except serial.SerialException as e:
                logger.error(f"{self.name}: Connection failed: {e}")
                self._port = None
                return False

    def disconnect(self):
        """Disconnect from serial port."""
        self._running = False

        if self._read_thread:
            self._read_thread.join(timeout=2.0)
            self._read_thread = None

        with self._lock:
            if self._port and self._port.is_open:
                try:
                    self._port.close()
                except Exception as e:
                    logger.error(f"{self.name}: Error closing port: {e}")
                self._port = None

        logger.info(f"{self.name}: Disconnected")

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._port is not None and self._port.is_open

    def send(self, data: bytes) -> bool:
        """Send data to the receiver.

        Args:
            data: Bytes to send

        Returns:
            True if sent successfully
        """
        with self._lock:
            if not self._port or not self._port.is_open:
                return False
            try:
                self._port.write(data)
                self._port.flush()
                return True
            except serial.SerialException as e:
                logger.error(f"{self.name}: Send error: {e}")
                return False

    def _read_loop(self):
        """Background thread: continuously read from serial port."""
        logger.debug(f"{self.name}: Read thread started")

        while self._running:
            try:
                with self._lock:
                    if not self._port or not self._port.is_open:
                        break
                    # Check how many bytes are waiting
                    waiting = self._port.in_waiting
                    if waiting > 0:
                        data = self._port.read(waiting)
                    else:
                        # Read with timeout
                        data = self._port.read(1)

                if data:
                    self.bytes_received += len(data)
                    self._extract_nmea_lines(data)
                    if self.on_data:
                        try:
                            self.on_data(data)
                        except Exception as e:
                            logger.error(f"{self.name}: Callback error: {e}")

            except serial.SerialException as e:
                logger.error(f"{self.name}: Read error: {e}")
                self._running = False
                break
            except Exception as e:
                logger.error(f"{self.name}: Unexpected error: {e}")
                time.sleep(0.1)

        logger.debug(f"{self.name}: Read thread stopped")

    def _extract_nmea_lines(self, data: bytes):
        """Extract complete NMEA lines from incoming serial bytes."""
        if self.on_nmea is None:
            return

        self._line_buffer.extend(data)

        # Avoid unbounded growth when binary-only stream has no newlines.
        if len(self._line_buffer) > 8192:
            self._line_buffer = self._line_buffer[-4096:]

        while True:
            line_end = self._line_buffer.find(b"\n")
            if line_end < 0:
                break

            raw_line = bytes(self._line_buffer[:line_end + 1])
            del self._line_buffer[:line_end + 1]

            if not raw_line.startswith(b"$"):
                continue

            try:
                line = raw_line.decode("ascii").strip()
            except UnicodeDecodeError:
                continue

            if "*" not in line:
                continue

            try:
                self.on_nmea(line)
            except Exception as e:
                logger.error(f"{self.name}: NMEA callback error: {e}")

    @property
    def stats(self) -> dict:
        uptime = time.time() - self.connect_time if self.connect_time else 0
        return {
            'connected': self.is_connected,
            'bytes_received': self.bytes_received,
            'uptime_s': uptime,
            'data_rate_bps': self.bytes_received * 8 / uptime if uptime > 0 else 0,
        }
