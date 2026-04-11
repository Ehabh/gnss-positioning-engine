"""
Bit-level reader for RTCM3 message parsing.

RTCM3 messages are bit-packed, so we need to read arbitrary
numbers of bits from a byte stream. This reader handles that
efficiently.
"""

import struct


class BitReader:
    """Read arbitrary-length bit fields from a byte buffer.

    RTCM3 messages encode fields as bit-packed integers (signed and unsigned),
    with field widths that don't align to byte boundaries.

    Usage:
        reader = BitReader(message_bytes)
        msg_type = reader.read_uint(12)   # 12-bit message type
        station_id = reader.read_uint(12) # 12-bit station ID
        tow = reader.read_uint(30)        # 30-bit GPS epoch time
        ...
    """

    def __init__(self, data: bytes):
        self._data = data
        self._bit_pos = 0
        self._total_bits = len(data) * 8

    @property
    def bits_remaining(self) -> int:
        return self._total_bits - self._bit_pos

    @property
    def bit_position(self) -> int:
        return self._bit_pos

    def read_uint(self, n_bits: int) -> int:
        """Read unsigned integer of n_bits width.

        Args:
            n_bits: Number of bits to read (1-64)

        Returns:
            Unsigned integer value
        """
        if n_bits <= 0:
            return 0
        if self._bit_pos + n_bits > self._total_bits:
            raise ValueError(
                f"Not enough bits: need {n_bits}, have {self.bits_remaining}"
            )

        value = 0
        remaining = n_bits

        while remaining > 0:
            byte_idx = self._bit_pos >> 3
            bit_offset = self._bit_pos & 7
            bits_available = 8 - bit_offset
            bits_to_read = min(remaining, bits_available)

            # Extract bits from current byte
            mask = (1 << bits_to_read) - 1
            shift = bits_available - bits_to_read
            bits = (self._data[byte_idx] >> shift) & mask

            value = (value << bits_to_read) | bits
            self._bit_pos += bits_to_read
            remaining -= bits_to_read

        return value

    def read_int(self, n_bits: int) -> int:
        """Read signed integer (two's complement) of n_bits width.

        Args:
            n_bits: Number of bits to read (1-64)

        Returns:
            Signed integer value
        """
        value = self.read_uint(n_bits)
        # Check sign bit
        if n_bits > 0 and (value & (1 << (n_bits - 1))):
            value -= (1 << n_bits)
        return value

    def read_bool(self) -> bool:
        """Read a single bit as boolean."""
        return self.read_uint(1) == 1

    def read_bits(self, n_bits: int) -> int:
        """Alias for read_uint."""
        return self.read_uint(n_bits)

    def skip(self, n_bits: int):
        """Skip n_bits forward."""
        self._bit_pos += n_bits

    def read_float_scaled(self, n_bits: int, scale: float,
                           signed: bool = True) -> float:
        """Read integer and apply scale factor.

        Common pattern in RTCM: value = integer * scale_factor

        Args:
            n_bits: Number of bits
            scale: Scale factor to multiply
            signed: Whether to read as signed

        Returns:
            Scaled float value
        """
        if signed:
            return self.read_int(n_bits) * scale
        else:
            return self.read_uint(n_bits) * scale

    def read_bitmask(self, n_bits: int) -> list:
        """Read a bitmask and return list of set bit indices (1-based).

        Used for MSM satellite and signal masks.

        Args:
            n_bits: Number of bits in mask

        Returns:
            List of 1-based indices where bits are set
        """
        mask = self.read_uint(n_bits)
        indices = []
        for i in range(n_bits):
            if mask & (1 << (n_bits - 1 - i)):
                indices.append(i + 1)
        return indices