
# ================= IMPORTS =================
import serial
import time
import math
from flask import Flask, jsonify
from flask_cors import CORS
import csv
from datetime import datetime


# ================= CONFIGURATION =================
PORT = 'COM5'
BAUD_RATE = 115200

ACC_SCALE = 16384.0
GYRO_SCALE = 131.0
BETA = 0.1   # Madgwick filter gain


# ================= GLOBAL STATE =================

# Quaternion (orientation)
q0, q1, q2, q3 = 1, 0, 0, 0
prev_time = None

# Gyroscope bias
gyro_bias_x = 0
gyro_bias_y = 0
gyro_bias_z = 0

# Offset calibration
roll_offset = 0
pitch_offset = 0

# Latest values (shared with frontend)
latest_roll = 0
latest_pitch = 0
latest_yaw = 0

latest_temp = 0
latest_pressure = 0
latest_altitude = 0


# ================= CSV LOGGING SETUP =================

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


# ================= SERIAL PARSING =================

# Parse IMU line
def parse_imu(line):
    try:
        if "ACC:" in line and "GYRO:" in line:
            acc_part, gyro_part = line.split("|")

            acc_vals = acc_part.replace("ACC:", "").strip().split(",")
            gyro_vals = gyro_part.replace("GYRO:", "").strip().split(",")

            return (
                float(acc_vals[0]), float(acc_vals[1]), float(acc_vals[2]),
                float(gyro_vals[0]), float(gyro_vals[1]), float(gyro_vals[2])
            )
    except:
        return None
    return None


# Parse BMP line
def parse_bmp(line):
    try:
        if "BMP:" in line:
            vals = line.replace("BMP:", "").strip().split(",")
            return float(vals[0]), float(vals[1]), float(vals[2])
    except:
        return None
    return None


# ================= MADGWICK FILTER =================

def madgwick_update(ax, ay, az, gx, gy, gz, dt):
    global q0, q1, q2, q3

    # Normalize accelerometer
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0:
        return
    ax /= norm
    ay /= norm
    az /= norm

    # Convert gyro to radians
    gx = math.radians(gx)
    gy = math.radians(gy)
    gz = math.radians(gz)

    # Gradient descent step
    f1 = 2*(q1*q3 - q0*q2) - ax
    f2 = 2*(q0*q1 + q2*q3) - ay
    f3 = 2*(0.5 - q1*q1 - q2*q2) - az

    grad0 = -2*q2*f1 + 2*q1*f2
    grad1 = 2*q3*f1 + 2*q0*f2 - 4*q1*f3
    grad2 = -2*q0*f1 + 2*q3*f2 - 4*q2*f3
    grad3 = 2*q1*f1 + 2*q2*f2

    norm = math.sqrt(grad0*grad0 + grad1*grad1 + grad2*grad2 + grad3*grad3)
    if norm == 0:
        return

    grad0 /= norm
    grad1 /= norm
    grad2 /= norm
    grad3 /= norm

    # Quaternion update
    qDot0 = 0.5 * (-q1*gx - q2*gy - q3*gz) - BETA * grad0
    qDot1 = 0.5 * (q0*gx + q2*gz - q3*gy) - BETA * grad1
    qDot2 = 0.5 * (q0*gy - q1*gz + q3*gx) - BETA * grad2
    qDot3 = 0.5 * (q0*gz + q1*gy - q2*gx) - BETA * grad3

    q0 += qDot0 * dt
    q1 += qDot1 * dt
    q2 += qDot2 * dt
    q3 += qDot3 * dt

    # Normalize quaternion
    norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    q0 /= norm
    q1 /= norm
    q2 /= norm
    q3 /= norm


# Convert quaternion → Euler
def get_euler():
    roll = math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
    pitch = math.asin(2*(q0*q2 - q3*q1))
    yaw = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


# ================= SERIAL INIT =================
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
time.sleep(2)


# ================= GYRO CALIBRATION =================
print("Calibrating gyro... Keep still")

samples = 200
bx = by = bz = 0

count = 0
while count < samples:
    line = ser.readline().decode(errors='ignore').strip()
    data = parse_imu(line)
    if not data:
        continue

    _, _, _, gx, gy, gz = data
    bx += gx
    by += gy
    bz += gz
    count += 1

gyro_bias_x = bx / samples
gyro_bias_y = by / samples
gyro_bias_z = bz / samples

print("Gyro calibrated")


# ================= OFFSET CALIBRATION =================
print("Calibrating offsets... Keep flat")

samples = 100
r_sum = p_sum = 0
count = 0

while count < samples:
    line = ser.readline().decode(errors='ignore').strip()
    data = parse_imu(line)
    if not data:
        continue

    ax, ay, az, gx, gy, gz = data

    ax /= ACC_SCALE
    ay /= ACC_SCALE
    az /= ACC_SCALE

    gx = (gx - gyro_bias_x) / GYRO_SCALE
    gy = (gy - gyro_bias_y) / GYRO_SCALE
    gz = (gz - gyro_bias_z) / GYRO_SCALE

    current_time = time.time()

    if prev_time is None:
        prev_time = current_time
        continue

    dt = current_time - prev_time
    prev_time = current_time

    madgwick_update(ax, ay, az, gx, gy, gz, dt)
    r, p, _ = get_euler()

    r_sum += r
    p_sum += p
    count += 1

roll_offset = r_sum / samples
pitch_offset = p_sum / samples

print("Offsets calibrated")


# ================= FLASK  =================
app = Flask(__name__)
CORS(app)


@app.route("/data")
def get_data():
    return jsonify({
        "roll": round(latest_roll, 2),
        "pitch": round(latest_pitch, 2),
        "yaw": round(latest_yaw, 2),
        "temp": round(latest_temp, 2),
        "pressure": round(latest_pressure, 2),
        "altitude": round(latest_altitude, 2)
    })




def update_loop():
    global prev_time
    global latest_roll, latest_pitch, latest_yaw
    global latest_temp, latest_pressure, latest_altitude
    global log_counter

    while True:

        # Read serial
        line = ser.readline().decode(errors='ignore').strip()

        # BMP DATA
        bmp = parse_bmp(line)
        if bmp:
            latest_temp, latest_pressure, latest_altitude = bmp
            continue

        # IMU DATA
        imu = parse_imu(line)
        if not imu:
            continue

        ax, ay, az, gx, gy, gz = imu

        # Scale data
        ax /= ACC_SCALE
        ay /= ACC_SCALE
        az /= ACC_SCALE

        gx = (gx - gyro_bias_x) / GYRO_SCALE
        gy = (gy - gyro_bias_y) / GYRO_SCALE
        gz = (gz - gyro_bias_z) / GYRO_SCALE

        # Time step
        current_time = time.time()

        if prev_time is None:
            prev_time = current_time
            continue

        dt = current_time - prev_time
        prev_time = current_time

        # Filter update
        madgwick_update(ax, ay, az, gx, gy, gz, dt)

        # Convert to angles
        roll, pitch, yaw = get_euler()

        roll -= roll_offset
        pitch -= pitch_offset

        # Store values
        latest_roll = roll
        latest_pitch = pitch
        latest_yaw = yaw

        # Controlled logging (every 10 cycles)
        log_counter += 1
        if log_counter % 10 == 0:
            log_to_csv(roll, pitch, yaw,
                       latest_altitude, latest_temp, latest_pressure)

        # Stable loop timing
        time.sleep(0.02)


# ================= RUN =================
if __name__ == "__main__":
    import threading

    threading.Thread(target=update_loop, daemon=True).start()
    app.run(port=5000)