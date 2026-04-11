"""
RTCM3 Frame Extractor.

Extracts complete RTCM3 frames from a raw byte stream.

RTCM3 frame structure:
    Byte 0:      Preamble = 0xD3
    Bytes 1-2:   6 reserved bits (must be 0) + 10-bit message length
    Bytes 3..N:  Message data (length bytes)
    Bytes N+1..N+3: CRC-24Q checksum (3 bytes)

Total frame size = 3 (header) + length + 3 (CRC) = length + 6 bytes.

Reference: RTCM Standard 10403.3, Section 4.1
"""

import logging
from typing import Optional, Callable

logger = logging.getLogger(__name__)

RTCM3_PREAMBLE = 0xD3
RTCM3_HEADER_SIZE = 3
RTCM3_CRC_SIZE = 3

# CRC-24Q lookup table
_CRC24Q_TABLE = None


def _build_crc24q_table():
    global _CRC24Q_TABLE
    if _CRC24Q_TABLE is not None:
        return
    poly = 0x1864CFB
    table = []
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= poly
        table.append(crc & 0xFFFFFF)
    _CRC24Q_TABLE = table


def _crc24q(data: bytes) -> int:
    """Compute CRC-24Q checksum."""
    _build_crc24q_table()
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC24Q_TABLE[((crc >> 16) ^ byte) & 0xFF]
    return crc


class RTCM3FrameExtractor:
    """Extracts complete RTCM3 frames from a raw byte stream.

    Handles partial frames across multiple feed() calls (stream-safe).

    Usage:
        extractor = RTCM3FrameExtractor()
        extractor.on_frame = lambda msg_type, data: print(msg_type)
        extractor.feed(raw_bytes)
    """

    def __init__(self):
        self._buffer = bytearray()
        self.on_frame: Optional[Callable[[int, bytes], None]] = None

        # Statistics
        self.frames_received = 0
        self.crc_errors = 0
        self.bytes_processed = 0

    def feed(self, data: bytes):
        """Feed raw bytes into the extractor.

        Automatically extracts and dispatches complete RTCM3 frames.

        Args:
            data: Raw bytes from the serial port
        """
        self._buffer.extend(data)
        self.bytes_processed += len(data)
        self._extract_frames()

    def _extract_frames(self):
        """Scan buffer and extract all complete frames."""
        while len(self._buffer) >= RTCM3_HEADER_SIZE:
            # Find preamble
            if self._buffer[0] != RTCM3_PREAMBLE:
                # Discard bytes until preamble found
                idx = self._buffer.find(RTCM3_PREAMBLE)
                if idx == -1:
                    self._buffer.clear()
                    return
                del self._buffer[:idx]
                continue

            # Need at least 3 header bytes to determine message length
            if len(self._buffer) < RTCM3_HEADER_SIZE:
                return

            # Reserved bits (6) must be 0; length is lower 10 bits of bytes 1-2
            if self._buffer[1] & 0xFC:
                # Reserved bits non-zero — not a valid header, skip preamble
                del self._buffer[0]
                continue

            msg_length = ((self._buffer[1] & 0x03) << 8) | self._buffer[2]
            frame_size = RTCM3_HEADER_SIZE + msg_length + RTCM3_CRC_SIZE

            # Wait for complete frame
            if len(self._buffer) < frame_size:
                return

            frame = bytes(self._buffer[:frame_size])

            # Verify CRC-24Q
            expected_crc = _crc24q(frame[:-RTCM3_CRC_SIZE])
            actual_crc = (
                (frame[-3] << 16) |
                (frame[-2] << 8) |
                frame[-1]
            )

            if expected_crc != actual_crc:
                logger.debug(f"CRC mismatch (expected={expected_crc:06X}, "
                             f"got={actual_crc:06X}) — skipping byte")
                self.crc_errors += 1
                del self._buffer[0]
                continue

            # Valid frame — extract message type (first 12 bits of data)
            if msg_length >= 2:
                msg_type = (
                    (frame[RTCM3_HEADER_SIZE] << 4) |
                    (frame[RTCM3_HEADER_SIZE + 1] >> 4)
                )
                msg_data = frame[RTCM3_HEADER_SIZE:RTCM3_HEADER_SIZE + msg_length]

                self.frames_received += 1
                logger.debug(f"RTCM3 frame: type={msg_type} len={msg_length}")

                if self.on_frame:
                    try:
                        self.on_frame(msg_type, msg_data)
                    except Exception as e:
                        logger.error(f"on_frame callback error: {e}")

            del self._buffer[:frame_size]

    @property
    def stats(self) -> dict:
        return {
            'frames_received': self.frames_received,
            'crc_errors': self.crc_errors,
            'bytes_processed': self.bytes_processed,
            'buffer_size': len(self._buffer),
        }
