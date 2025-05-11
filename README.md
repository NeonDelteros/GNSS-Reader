# GNSSReader Python Module

This module provides a high-level parser for GNSS data in NMEA format. It is designed to be used in robotics, tracking, or positioning systems where GPS/GNSS data is streamed via USB or simulated via log files.

## Module Contents

### `gnss_reader.py`
- Reads NME sentences from:
  - USB GNSS devices (via auto-detected serial port)
  - `.log` files (for simulation/testing)
- Parses sentence types:
  - `$GPGGA` – GPS fix data
  - `$GPRMC` – Recommended minimum data (position, speed, date)
  - `$GPVTG` – Track made good and speed
  - `$GPGSV` – Satellites in view
  - `$GPHDT` – Heading

Returns structured data using Python `@dataclass` types like `GGAData`, `RMCData`, `VTGData`, etc.

### `test_gnss_reader.py` (example usage)
Reads from a log file or USB and prints parsed data for visual inspection.

---

## How to Use

### 1. Install requirements

```bash
pip install pyserial
```

### 2. Read from a real GNSS USB device

```python
from gnss_reader import GNSSReader, GGAData

gnss = GNSSReader(baudrate=9600)
for msg in gnss.read_sentences():
    if isinstance(msg, GGAData):
        print(f"Lat: {msg.latitude}, Lon: {msg.longitude}, Alt: {msg.altitude}")
```

### 3. Or read from a `.log` file (test mode)

```python
gnss = GNSSReader(log_file="output1.log")
for msg in gnss.read_sentences():
    print(msg)
```

---

## Testing with `test_gnss_reader.py`

```bash
python3 test_gnss_reader.py
```

---

## Notes

- This module avoids `pynmea2` for more flexible parsing and custom extension.
- Designed to be **imported**, not run directly.

---
