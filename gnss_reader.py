# ===========================
# File: gnss_reader.py
# Description: Read and parse NMEA GNSS data from USB or log file
# Supports multiple NMEA sentence types: GGA, RMC, VTG, GSV, HDT
# ===========================

import serial
from serial.tools import list_ports
from dataclasses import dataclass
from datetime import datetime, date, time
from typing import List, Optional, Generator

# ---------- Data classes for NMEA types ----------

# GGA
@dataclass
class GGAData:
    time_utc: Optional[time]  # Time of fix (hhmmss.ss)
    latitude: Optional[float]  # Latitude in decimal degrees
    latitude_dir: Optional[str]  # 'N' or 'S'
    longitude: Optional[float]  # Longitude in decimal degrees
    longitude_dir: Optional[str]  # 'E' or 'W'
    fix_quality: Optional[int]  # GPS quality indicator (0 = invalid, 1 = GPS fix, etc.)
    num_satellites: Optional[int]  # Number of satellites in use
    horizontal_dilution: Optional[float]  # HDOP
    altitude: Optional[float]  # Altitude above mean sea level
    altitude_units: Optional[str]  # Units of altitude (usually 'M')
    geoid_separation: Optional[float]  # Height of geoid above WGS84 ellipsoid
    geoid_units: Optional[str]  # Units of geoid separation

# RMC
@dataclass
class RMCData:
    time_utc: Optional[time]  # Time of fix
    status: Optional[str]  # Status A = active or V = void
    latitude: Optional[float]
    latitude_dir: Optional[str]
    longitude: Optional[float]
    longitude_dir: Optional[str]
    speed_knots: Optional[float]  # Speed over ground in knots
    track_angle: Optional[float]  # Track angle in degrees
    date_utc: Optional[date]  # Date (ddmmyy)
    mag_variation: Optional[float]  # Magnetic variation
    mag_var_dir: Optional[str]  # Magnetic variation direction

# VTG
@dataclass
class VTGData:
    track_true: Optional[float]  # Track made good in degrees True
    track_magnetic: Optional[float]  # Track made good in degrees Magnetic
    speed_knots: Optional[float]  # Speed over ground in knots
    speed_kmph: Optional[float]  # Speed over ground in km/h
    mode: Optional[str]  # Mode indicator

# Satellite info for GSV sentence
@dataclass
class SatelliteInfo:
    prn: Optional[int]  # Satellite PRN number
    elevation: Optional[int]  # Elevation in degrees (0–90)
    azimuth: Optional[int]  # Azimuth in degrees (0–359)
    snr: Optional[int]  # Signal to Noise Ratio (SNR)

# GSV
@dataclass
class GSVData:
    total_messages: Optional[int]  # Total number of messages of this type in the cycle
    message_number: Optional[int]  # Message number
    total_satellites: Optional[int]  # Total satellites in view
    satellites: List[SatelliteInfo]  # List of satellites in this message

# HDT
@dataclass
class HDTData:
    heading: Optional[float]  # Heading in degrees true

# ---------- GNSSReader class for parsing ----------

class GNSSReader:
    # Reads NMEA GNSS data from a live USB serial device or a log file.
    # Auto-detects USB GNSS ports and supports parsing of GGA, RMC, VTG, GSV, HDT.
    def __init__(self, port: Optional[str] = None, baudrate: int = 9600, log_file: Optional[str] = None, checksum_required: bool = False):
        # If log_file is provided, open file instead of serial
        self.checksum_required = checksum_required
        self.ser = None
        self.log_file = log_file
        if log_file:
            self.file = open(log_file, 'r')
        else:
            # Auto-detect USB GNSS port if not specified
            if port is None:
                ports = self.detect_gnss_ports()
                if not ports:
                    raise IOError("No GNSS USB serial ports found")
                port = ports[0]
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=1)

    @staticmethod
    def detect_gnss_ports() -> List[str]:
        # Detect likely GNSS USB serial ports based on common names
        ports = []
        for info in list_ports.comports():
            desc = (info.description or "").lower()
            if any(keyword in desc for keyword in ['gps', 'gnss', 'u-blox', 'serial']):
                ports.append(info.device)
        if not ports:
            for info in list_ports.comports():
                device = info.device
                if device.startswith('/dev/ttyUSB') or device.startswith('/dev/ttyACM'):
                    ports.append(device)
        return ports

    def read_sentences(self) -> Generator[object, None, None]:
        # Reads and parses NMEA sentences one by one
        # Yields parsed dataclass objects (GGAData, RMCData, etc.)
        while True:
            line = None
            try:
                if self.log_file:
                    line = self.file.readline()
                    if not line:
                        break
                else:
                    raw = self.ser.readline()
                    if not raw:
                        continue
                    line = raw.decode('ascii', errors='ignore')
            except Exception:
                continue

            line = line.strip()
            if not line or not line.startswith('$'):
                continue

            try:
                parsed = self.parse_sentence(line)
                if parsed:
                    yield parsed
            except ValueError:
                continue

    def parse_sentence(self, line: str):
        # Parse a single NMEA sentence string and return a dataclass instance
        # Handle checksum verification if enabled
        if '*' in line:
            body, cksum_str = line[1:].split('*', 1)
            calc_cksum = 0
            for ch in body:
                calc_cksum ^= ord(ch)
            sent_cksum = int(cksum_str.strip(), 16)
            if self.checksum_required and calc_cksum != sent_cksum:
                raise ValueError("Checksum does not match")
        else:
            body = line[1:]

        # Extract fields and sentence type
        fields = body.split(',')
        msg_type = fields[0][2:]

        # Dispatch to specific parsers
        if msg_type == 'GGA':
            return self._parse_gga(fields)
        elif msg_type == 'RMC':
            return self._parse_rmc(fields)
        elif msg_type == 'VTG':
            return self._parse_vtg(fields)
        elif msg_type == 'GSV':
            return self._parse_gsv(fields)
        elif msg_type == 'HDT':
            return self._parse_hdt(fields)
        return None

    #
    

