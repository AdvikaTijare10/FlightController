import datetime
log_filename = f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# Open file once (IMPORTANT)
log_file = open(log_filename, mode='w', newline='')
writer = csv.writer(log_file)

# Write header
writer.writerow(['Timestamp', 'Roll', 'Pitch', 'Yaw', 'Altitude', 'Temp', 'Pressure'])

# Logging function
def log_to_csv(r, p, y, alt, temp, pressure):
    writer.writerow([
        datetime.now().isoformat(),
        r, p, y, alt, temp, pressure
    ])

# Logging control
log_counter = 0
