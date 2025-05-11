# 📘 GNSS Depth Control System – README

## Overview

This project provides a modular GNSS parser and depth control system for underwater equipment such as a tethered camera system. It enables autonomous control of vertical positioning based on real-time GNSS location and pre-mapped seabed depth data.

### Components

- `gnss_reader.py` – Reads and parses NMEA data from USB GNSS receivers or log files.
- `DEPTH_1.py` – Main controller that adjusts depth automatically using parsed GNSS position and seabed grid.
- `test_gnss_reader.py` – A test harness to verify GNSS parsing using a log file or live USB stream.

## 📦 What the Code Does

### `gnss_reader.py`
- Auto-detects connected GNSS USB devices.
- Parses major NMEA sentence types:
  - `GGA`: Position, altitude
  - `RMC`: Recommended minimum data (time, speed, date)
  - `VTG`: Ground speed and track
  - `GSV`: Satellite info
  - `HDT`: True heading
- Returns structured dataclasses for easy downstream use.

### `DEPTH_1.py`
- Uses `GNSSReader` to get GPS data in real time.
- Queries seabed depth using an R-tree spatial index on `grid_min.csv`.
- Computes target depth offset (e.g. camera 1.5m above seabed).
- Applies PID control to move a motorized spool up/down accordingly.
- Switches between auto and manual mode via a hardware GPIO button.

## 🔧 How to Set Up

### 1. Hardware Requirements
- Raspberry Pi (with GPIO access)
- GNSS USB receiver (outputs NMEA)
- Motor driver + DC motor + limit switches
- Mode switch button (manual/auto)
- Seabed depth grid file: `grid_min.csv`

### 2. Software Dependencies

Install required packages:

```bash
sudo apt install python3-pip
pip3 install pyserial pandas numpy rtree
```

Enable I2C, SPI, and GPIO if needed on Raspberry Pi.

## 📂 File Structure

```
/project-dir/
├── gnss_reader.py
├── DEPTH_1.py
├── test_gnss_reader.py
├── output1.log
├── grid_min.csv
└── README.md
```

## 🧩 How to Integrate

### Step 1: Use `GNSSReader` in your code

```python
from gnss_reader import GNSSReader, GGAData

gnss = GNSSReader(baudrate=9600)
for msg in gnss.read_sentences():
    if isinstance(msg, GGAData):
        print(msg.latitude, msg.longitude)
```

### Step 2: Replace `pynmea2` or direct serial use in your project with this module.

### Step 3: Connect parsed latitude/longitude to your control system.

## 🚀 Running the System

### Option 1: Live GNSS Device

```bash
python3 DEPTH_1.py
```

Make sure:
- `grid_min.csv` is present
- GNSS outputs `$GPGGA`

### Option 2: Simulated Log File

```python
GNSSReader(log_file="output1.log")
```

## 🧪 Testing the Parser Only

```bash
python3 test_gnss_reader.py
```

Output will show parsed GNSS info like:
- Coordinates
- Speed
- Satellite count
- Heading

## 🔍 Notes

- Extend `gnss_reader.py` with more NMEA types as needed.
- Designed for modular testing and integration.
- Suitable for Raspberry Pi marine robotics, drones, or autonomous boats.

## 📞 Support / Contributions

Feel free to extend this project or reach out for help integrating into other systems.