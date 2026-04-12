"""
GNSS Processing Pipeline.

Central coordinator that connects:
    Serial Handler → RTCM Parser → Positioning Engine → Solution Output

Manages data flow between components and dispatches solutions to the UI.
"""

import logging
import time
import threading
from typing import Optional, Callable, Dict
from enum import Enum, auto
import numpy as np

from .constants import Constellation, FixType
from .data_types import (
    EpochObservations, PositionSolution, EphemerisStore,
    BaseStationInfo, RawObservation,
)
from .session_logger import SessionLogger
from .serial_handler import SerialHandler
from ..parsers.rtcm3_frame import RTCM3FrameExtractor
from ..parsers.rtcm3_msm import MSM4Decoder
from ..parsers.nmea import NMEASentenceParser
from ..parsers.rtcm3_ephemeris import (
    decode_gps_ephemeris, decode_glonass_ephemeris,
    decode_galileo_ephemeris, decode_beidou_ephemeris,
    decode_station_arp,
)
from ..engines.sps import SPSEngine
from ..engines.dgnss import DGNSSEngine
from ..engines.rtk import RTKEngine
from ..utils.time_utils import gps_time_now
from ..utils.coordinates import lla_to_ecef, ecef_to_enu

logger = logging.getLogger(__name__)


class PositioningMode(Enum):
    SPS = auto()
    DGNSS = auto()
    RTK = auto()


class GNSSPipeline:
    """Main GNSS processing pipeline.

    Coordinates all components from serial input to position output.
    """

    def __init__(self):
        # Data stores
        self.ephemeris = EphemerisStore()
        self.base_info: Optional[BaseStationInfo] = None

        # Serial handlers
        self.rover_serial = SerialHandler(name="Rover")
        self.base_serial = SerialHandler(name="Base")

        # RTCM parsers
        self._rover_frame_extractor = RTCM3FrameExtractor()
        self._base_frame_extractor = RTCM3FrameExtractor()
        self._msm_decoder = MSM4Decoder()
        self._nmea_parser = NMEASentenceParser()
        self._session_logger = SessionLogger()

        # Positioning engines
        self._sps_engine = SPSEngine(self.ephemeris)
        self._dgnss_engine = DGNSSEngine(self.ephemeris)
        self._rtk_engine = RTKEngine(self.ephemeris)

        # Current mode
        self.mode = PositioningMode.SPS

        # GPS week (will be updated from RTCM)
        self._gps_week, _ = gps_time_now()

        # Callbacks
        self.on_solution: Optional[Callable[[PositionSolution], None]] = None
        self.on_satellites: Optional[Callable[[EpochObservations], None]] = None
        self.on_ephemeris_update: Optional[Callable[[dict], None]] = None
        self.on_status: Optional[Callable[[str], None]] = None
        self.on_reference: Optional[Callable[[dict], None]] = None

        # Wire up frame extractors
        self._rover_frame_extractor.on_frame = self._on_rover_frame
        self._base_frame_extractor.on_frame = self._on_base_frame

        # Wire up serial handlers
        self.rover_serial.on_data = self._rover_frame_extractor.feed
        self.base_serial.on_data = self._base_frame_extractor.feed
        self.rover_serial.on_nmea = self._on_rover_nmea

        # Statistics
        self.epochs_processed = 0
        self.solutions_computed = 0

        # Pending observations accumulator
        # (MSM messages for different constellations arrive separately
        #  but belong to the same epoch)
        self._pending_obs: Dict[float, EpochObservations] = {}

        # Processing lock
        self._process_lock = threading.Lock()
        self._reference_lock = threading.Lock()
        self._reference_position: Optional[dict] = None

    def set_mode(self, mode: PositioningMode):
        """Set the positioning mode."""
        self.mode = mode
        logger.info(f"Positioning mode set to: {mode.name}")

        if self.on_status:
            self.on_status(f"Mode: {mode.name}")

    def connect_rover(self, port: str, baudrate: int = 115200) -> bool:
        """Connect to the rover receiver.

        Args:
            port: Serial port path
            baudrate: Baud rate

        Returns:
            True if connected
        """
        success = self.rover_serial.connect(port, baudrate)
        if success and self.on_status:
            self.on_status(f"Rover connected: {port}")
        return success

    def connect_base(self, port: str, baudrate: int = 115200) -> bool:
        """Connect to the base station receiver.

        Args:
            port: Serial port path
            baudrate: Baud rate

        Returns:
            True if connected
        """
        success = self.base_serial.connect(port, baudrate)
        if success and self.on_status:
            self.on_status(f"Base connected: {port}")
        return success

    def configure_base_station(self, x: float, y: float, z: float) -> bool:
        """Register base station ECEF position for DGNSS/RTK engines."""
        self.base_info = BaseStationInfo(
            station_id=0, x_ecef=x, y_ecef=y, z_ecef=z
        )
        self._dgnss_engine.set_base_station(self.base_info)
        self._rtk_engine.set_base_station(self.base_info)
        return True

    def disconnect_all(self):
        """Disconnect all serial connections."""
        self.rover_serial.disconnect()
        self.base_serial.disconnect()
        self.stop_session_logging()

    def start_session_logging(self, file_path: str) -> bool:
        """Start JSONL session logging for post-processing."""
        ok = self._session_logger.start(file_path)
        if ok and self.on_status:
            self.on_status(f"Logging to: {file_path}")
        return ok

    def stop_session_logging(self):
        """Stop active session logging."""
        if self._session_logger.is_active:
            path = self._session_logger.path
            self._session_logger.stop()
            if self.on_status and path:
                self.on_status(f"Log saved: {path}")

    @property
    def reference_position(self) -> Optional[dict]:
        with self._reference_lock:
            if self._reference_position is None:
                return None
            return dict(self._reference_position)

    def _on_rover_frame(self, msg_type: int, data: bytes):
        """Handle a decoded RTCM frame from the rover receiver."""
        self._process_rtcm_frame(msg_type, data, is_base=False)

    def _on_base_frame(self, msg_type: int, data: bytes):
        """Handle a decoded RTCM frame from the base station."""
        self._process_rtcm_frame(msg_type, data, is_base=True)

    def _process_rtcm_frame(self, msg_type: int, data: bytes, is_base: bool):
        """Process a single RTCM3 frame.

        Routes to appropriate decoder based on message type.
        """
        source = "Base" if is_base else "Rover"
        self._session_logger.log_event(
            "rtcm_frame",
            {"source": source, "msg_type": msg_type, "length": len(data)},
        )

        # MSM4 and MSM7 observation messages
        # MSM4: 1074/1084/1094/1124  MSM7: 1077/1087/1097/1127
        if msg_type in (1074, 1077, 1084, 1087, 1094, 1097, 1124, 1127):
            epoch = self._msm_decoder.decode(msg_type, data, self._gps_week)
            if epoch:
                if is_base:
                    self._handle_base_observations(epoch)
                else:
                    self._handle_rover_observations(epoch)

        # Ephemeris messages
        elif msg_type == 1019:
            eph = decode_gps_ephemeris(data)
            if eph:
                self.ephemeris.update_gps(eph)
                self._notify_ephemeris_update()

        elif msg_type == 1020:
            eph = decode_glonass_ephemeris(data)
            if eph:
                self.ephemeris.update_glonass(eph)
                # Update MSM decoder with GLONASS frequency channels
                self._msm_decoder.set_glonass_channel(
                    eph.svn, eph.freq_channel)
                self._notify_ephemeris_update()

        elif msg_type in (1045, 1046):
            eph = decode_galileo_ephemeris(data)
            if eph:
                self.ephemeris.update_galileo(eph)
                self._notify_ephemeris_update()

        elif msg_type == 1042:
            eph = decode_beidou_ephemeris(data)
            if eph:
                self.ephemeris.update_beidou(eph)
                self._notify_ephemeris_update()

        # Station ARP
        elif msg_type in (1005, 1006):
            info = decode_station_arp(data)
            if info and is_base:
                self.base_info = BaseStationInfo(**info)
                self._dgnss_engine.set_base_station(self.base_info)
                self._rtk_engine.set_base_station(self.base_info)
                logger.info(f"Base station ARP received: "
                           f"({info['x']:.3f}, {info['y']:.3f}, {info['z']:.3f})")
                self._session_logger.log_event("base_station_arp", info)

    def _handle_rover_observations(self, epoch: EpochObservations):
        """Accumulate rover observations and trigger processing at epoch boundaries.

        MSM messages for different constellations share the same GPS TOW but
        arrive as separate RTCM frames within a few milliseconds of each other.
        Epoch boundary detection: when a strictly newer TOW arrives, the previous
        TOW's accumulation is complete — process it before starting the new one.

        TOW values are rounded to the nearest millisecond to absorb sub-ms
        floating-point drift introduced by the GLONASS time-system conversion.
        """
        # Determine the GPS-TOW accumulation key.
        # BeiDou MSM4 epoch time is in BDT (BeiDou Time = GPS time − 14 s).
        # Adding 14 s converts BDT to GPS TOW so BeiDou observations land in
        # the same epoch bucket as GPS/Galileo/GLONASS without affecting the
        # timestamp stored in each RawObservation (which must stay in BDT for
        # correct satellite-position computation in the orbit engine).
        is_beidou = (
            epoch.observations and
            epoch.observations[0].constellation == Constellation.BEIDOU
        )
        acc_tow = round(epoch.timestamp_gps + (14.0 if is_beidou else 0.0), 3)

        with self._process_lock:
            # Any pending epoch with a strictly older GPS TOW is now complete.
            # Process it so all constellation observations enter WLS together.
            for old_tow in sorted(self._pending_obs.keys()):
                if old_tow < acc_tow - 0.001:
                    self._process_pending_epoch(old_tow)

            # Accumulate observations for the current epoch.
            # The EpochObservations timestamp is GPS TOW (acc_tow) so the SPS
            # engine uses a consistent TOW across all constellations.
            if acc_tow not in self._pending_obs:
                self._pending_obs[acc_tow] = EpochObservations(
                    timestamp_gps=acc_tow,
                    week=epoch.week,
                )
            self._pending_obs[acc_tow].observations.extend(epoch.observations)

    def _process_pending_epoch(self, tow: float):
        """Process accumulated observations for an epoch."""
        if tow not in self._pending_obs:
            return

        epoch = self._pending_obs.pop(tow)

        # Notify UI about satellites
        if self.on_satellites:
            self.on_satellites(epoch)
        self._session_logger.log_event(
            "epoch_observations",
            {
                "tow_s": epoch.timestamp_gps,
                "week": epoch.week,
                "num_obs": len(epoch.observations),
                "num_sats": epoch.satellite_count,
            },
        )

        self.epochs_processed += 1

        # Process based on mode
        if self.mode == PositioningMode.SPS:
            solution = self._sps_engine.process_epoch(epoch)
        elif self.mode == PositioningMode.DGNSS:
            solution = self._dgnss_engine.process_epoch(epoch)
        elif self.mode == PositioningMode.RTK:
            solution = self._rtk_engine.process_epoch(epoch)
        else:
            return

        if solution.is_valid:
            self.solutions_computed += 1
            ref_error = self._compute_reference_error(solution)
            if ref_error:
                self._session_logger.log_event("reference_error", ref_error)

        # Dispatch solution
        self._session_logger.log_event(
            "position_solution",
            {
                "timestamp_gps": solution.timestamp_gps,
                "week": solution.week,
                "fix_type": solution.fix_type.name,
                "valid": solution.is_valid,
                "num_satellites": solution.num_satellites,
                "latitude": solution.latitude,
                "longitude": solution.longitude,
                "altitude": solution.altitude,
                "sigma_horizontal": solution.sigma_horizontal,
                "sigma_vertical": solution.sigma_vertical,
            },
        )
        if self.on_solution:
            self.on_solution(solution)

    def _handle_base_observations(self, epoch: EpochObservations):
        """Handle base station observations."""
        if self.mode == PositioningMode.DGNSS:
            self._dgnss_engine.update_base_observations(epoch)
        elif self.mode == PositioningMode.RTK:
            self._rtk_engine.update_base_observations(epoch)

    def _notify_ephemeris_update(self):
        """Notify UI about ephemeris store changes."""
        if self.on_ephemeris_update:
            self.on_ephemeris_update(self.ephemeris.summary)

    def _on_rover_nmea(self, sentence: str):
        """Handle NMEA sentence from rover stream for auto-reference."""
        parsed = self._nmea_parser.parse_reference(sentence)
        if parsed is None or not parsed.is_valid_fix:
            return

        with self._reference_lock:
            # Merge sentence types: keep latest lat/lon from either, prefer
            # altitude from GGA, and retain extra RMC/GGA metadata.
            prev = dict(self._reference_position) if self._reference_position else {}
            ref = {
                "source": parsed.source,
                "latitude": parsed.latitude,
                "longitude": parsed.longitude,
                "altitude": prev.get("altitude", 0.0),
                "fix_quality": prev.get("fix_quality", 0),
                "num_satellites": prev.get("num_satellites", 0),
                "speed_knots": prev.get("speed_knots", 0.0),
                "course_deg": prev.get("course_deg", 0.0),
                "timestamp_utc": parsed.timestamp_utc.isoformat() if parsed.timestamp_utc else None,
                "updated_unix": time.time(),
            }

            if parsed.sentence_type == "GGA":
                ref["altitude"] = parsed.altitude_m
                ref["fix_quality"] = parsed.fix_quality
                ref["num_satellites"] = parsed.num_satellites
            elif parsed.sentence_type == "RMC":
                ref["speed_knots"] = parsed.speed_knots
                ref["course_deg"] = parsed.course_deg

            self._reference_position = ref

        self._session_logger.log_event("nmea_reference", ref)
        if self.on_reference:
            self.on_reference(dict(ref))

    def _compute_reference_error(self, solution: PositionSolution) -> Optional[dict]:
        """Compute ENU/2D/3D error against latest NMEA-derived reference."""
        ref = self.reference_position
        if not ref:
            return None

        try:
            ref_ecef = lla_to_ecef(ref["latitude"], ref["longitude"], ref.get("altitude", 0.0))
            dx = float(solution.x - ref_ecef[0])
            dy = float(solution.y - ref_ecef[1])
            dz = float(solution.z - ref_ecef[2])
            enu = ecef_to_enu(dx, dy, dz, ref["latitude"], ref["longitude"])
            err_2d = float(np.hypot(enu[0], enu[1]))
            err_3d = float(np.linalg.norm([dx, dy, dz]))
        except Exception:
            return None

        return {
            "reference_source": ref.get("source", "unknown"),
            "enu_e_m": float(enu[0]),
            "enu_n_m": float(enu[1]),
            "enu_u_m": float(enu[2]),
            "error_2d_m": err_2d,
            "error_3d_m": err_3d,
        }

    @property
    def stats(self) -> dict:
        return {
            'mode': self.mode.name,
            'epochs_processed': self.epochs_processed,
            'solutions_computed': self.solutions_computed,
            'reference': self.reference_position,
            'logging': {
                'active': self._session_logger.is_active,
                'path': self._session_logger.path,
            },
            'ephemeris': self.ephemeris.summary,
            'rover_serial': self.rover_serial.stats,
            'base_serial': self.base_serial.stats if self.base_serial.is_connected else None,
            'rover_rtcm': self._rover_frame_extractor.stats,
        }
