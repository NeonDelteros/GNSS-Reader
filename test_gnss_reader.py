from gnss_reader import GNSSReader, GGAData, RMCData, VTGData, GSVData, HDTData

# Use GNSSReader with a log file or real USB input
reader = GNSSReader(log_file="output1.log")  # or use GNSSReader(baudrate=9600)

for msg in reader.read_sentences():
    if isinstance(msg, GGAData):
        print(f"GGA: Lat={msg.latitude}, Lon={msg.longitude}, Alt={msg.altitude}m")
    elif isinstance(msg, RMCData):
        print(f"RMC: Speed={msg.speed_knots} knots, Date={msg.date_utc}")
    elif isinstance(msg, VTGData):
        print(f"VTG: Speed={msg.speed_kmph} km/h, Track={msg.track_true}")
    elif isinstance(msg, GSVData):
        print(f"GSV: Satellites in view: {msg.total_satellites}")
    elif isinstance(msg, HDTData):
        print(f"HDT: Heading = {msg.heading}°")
