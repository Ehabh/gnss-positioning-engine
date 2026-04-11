"""
Serial Port Handler for GNSS Receivers.

Manages serial connections to LC29HBA receivers, including:
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
            baudrate: Baud rate (LC29HBA default: 115200)
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

    def send_nmea_command(self, command: str) -> bool:
        """Send an NMEA-format command with checksum.

        Args:
            command: Command without $ and *checksum (e.g., 'PAIR432,1')

        Returns:
            True if sent
        """
        # Compute NMEA checksum (XOR of all characters between $ and *)
        checksum = 0
        for ch in command:
            checksum ^= ord(ch)

        msg = f"${command}*{checksum:02X}\r\n"
        return self.send(msg.encode('ascii'))

    def configure_lc29h_rtcm_only(self) -> bool:
        """Configure LC29HBA for RTCM-only output.

        Disables all NMEA sentences and enables RTCM output.
        Uses Quectel PAIR commands.
        """
        commands = [
            # Disable all NMEA messages
            "PAIR062,0,0",   # GGA off
            "PAIR062,1,0",   # GLL off
            "PAIR062,2,0",   # GSA off
            "PAIR062,3,0",   # GSV off
            "PAIR062,4,0",   # RMC off
            "PAIR062,5,0",   # VTG off
            # Enable RTCM MSM4 output for all constellations
            "PAIR434,1074,1",  # GPS MSM4
            "PAIR434,1084,1",  # GLONASS MSM4
            "PAIR434,1094,1",  # Galileo MSM4
            "PAIR434,1124,1",  # BeiDou MSM4
            # Enable ephemeris output
            "PAIR434,1019,1",  # GPS ephemeris
            "PAIR434,1020,1",  # GLONASS ephemeris
            "PAIR434,1042,1",  # BeiDou ephemeris
            "PAIR434,1046,1",  # Galileo ephemeris
        ]

        success = True
        for cmd in commands:
            if not self.send_nmea_command(cmd):
                success = False
            time.sleep(0.1)  # Small delay between commands

        return success

    def configure_lc29h_rtcm_with_nmea_reference(self) -> bool:
        """Configure receiver for RTCM + reference NMEA (GGA/RMC).

        Keeps GGA and RMC enabled so the app can auto-detect a receiver
        truth/reference trajectory while still consuming RTCM observations.
        """
        commands = [
            # NMEA output: keep only sentences used for reference.
            "PAIR062,0,1",   # GGA on
            "PAIR062,1,0",   # GLL off
            "PAIR062,2,0",   # GSA off
            "PAIR062,3,0",   # GSV off
            "PAIR062,4,1",   # RMC on
            "PAIR062,5,0",   # VTG off
            # RTCM observation output
            "PAIR434,1074,1",  # GPS MSM4
            "PAIR434,1084,1",  # GLONASS MSM4
            "PAIR434,1094,1",  # Galileo MSM4
            "PAIR434,1124,1",  # BeiDou MSM4
            # Ephemeris output
            "PAIR434,1019,1",
            "PAIR434,1020,1",
            "PAIR434,1042,1",
            "PAIR434,1046,1",
        ]

        success = True
        for cmd in commands:
            if not self.send_nmea_command(cmd):
                success = False
            time.sleep(0.1)
        return success

    def configure_lc29h_base_station(self, x: float, y: float, z: float) -> bool:
        """Configure LC29HBA as a base station.

        Sets the receiver to fixed position mode and enables
        RTCM output for corrections.

        Args:
            x, y, z: Known ECEF position [m]
        """
        commands = [
            # Set survey-in or fixed position mode
            # This is a placeholder - actual LC29H commands may differ
            f"PAIR432,1",      # Enable base station mode
            # Set known position (ECEF)
            f"PAIR433,{x:.4f},{y:.4f},{z:.4f}",
            # Enable RTCM output
            "PAIR434,1005,1",  # Station ARP
            "PAIR434,1074,1",  # GPS MSM4
            "PAIR434,1084,1",  # GLONASS MSM4
            "PAIR434,1094,1",  # Galileo MSM4
            "PAIR434,1124,1",  # BeiDou MSM4
            "PAIR434,1019,1",  # GPS ephemeris
            "PAIR434,1020,1",  # GLONASS ephemeris
            "PAIR434,1042,1",  # BeiDou ephemeris
            "PAIR434,1046,1",  # Galileo ephemeris
        ]

        success = True
        for cmd in commands:
            if not self.send_nmea_command(cmd):
                success = False
            time.sleep(0.1)

        return success

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
