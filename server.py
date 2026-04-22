import serial
import time
import math
from flask import Flask, jsonify
from flask_cors import CORS
import csv
from datetime import datetime


# ================= CONFIG =================
PORT = 'COM5'
BAUD_RATE = 115200

ACC_SCALE = 16384.0
GYRO_SCALE = 131.0
BETA = 0.1

# ==========================================

# Quaternion
q0, q1, q2, q3 = 1, 0, 0, 0
prev_time = None

# Gyro bias
gyro_bias_x = 0
gyro_bias_y = 0
gyro_bias_z = 0

# Offsets
roll_offset = 0
pitch_offset = 0

# Latest values (shared with Flask)
latest_roll = 0
latest_pitch = 0
latest_yaw = 0

# BMP values
latest_temp = 0
latest_pressure = 0
latest_altitude = 0

#==================LOGGING TO CSV===================#
# Define log file name with timestamp
log_filename = f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# Write Header
with open(log_filename, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Timestamp', 'Roll', 'Pitch', 'Yaw', 'Altitude', 'Temp','Pressure'])

def log_to_csv(r, p, y, alt, temp, pressure):
    with open(log_filename, mode='a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([datetime.now().isoformat(), r, p, y, alt, temp,pressure])



# ================= PARSER =================
def parse_line(line):
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


def parse_bmp(line):
    try:
        if "BMP:" in line:
            vals = line.replace("BMP:", "").strip().split(",")

            return (
                float(vals[0]),  # temp
                float(vals[1]),  # pressure
                float(vals[2])   # altitude
            )
    except:
        return None
    return None
# ================= MADGWICK =================
def madgwick_update(ax, ay, az, gx, gy, gz, dt):
    global q0, q1, q2, q3

    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0:
        return
    ax /= norm
    ay /= norm
    az /= norm

    gx = math.radians(gx)
    gy = math.radians(gy)
    gz = math.radians(gz)

    f1 = 2*(q1*q3 - q0*q2) - ax
    f2 = 2*(q0*q1 + q2*q3) - ay
    f3 = 2*(0.5 - q1*q1 - q2*q2) - az

    J_11or24 = 2*q2
    J_12or23 = 2*q3
    J_13or22 = 2*q0
    J_14or21 = 2*q1
    J_32 = 2*J_14or21
    J_33 = 2*J_11or24

    grad0 = J_14or21*f2 - J_11or24*f1
    grad1 = J_12or23*f1 + J_13or22*f2 - J_32*f3
    grad2 = J_12or23*f2 - J_33*f3 - J_13or22*f1
    grad3 = J_14or21*f1 + J_11or24*f2

    norm = math.sqrt(grad0*grad0 + grad1*grad1 + grad2*grad2 + grad3*grad3)
    if norm == 0:
        return

    grad0 /= norm
    grad1 /= norm
    grad2 /= norm
    grad3 /= norm

    qDot0 = 0.5 * (-q1*gx - q2*gy - q3*gz) - BETA * grad0
    qDot1 = 0.5 * (q0*gx + q2*gz - q3*gy) - BETA * grad1
    qDot2 = 0.5 * (q0*gy - q1*gz + q3*gx) - BETA * grad2
    qDot3 = 0.5 * (q0*gz + q1*gy - q2*gx) - BETA * grad3

    q0 += qDot0 * dt
    q1 += qDot1 * dt
    q2 += qDot2 * dt
    q3 += qDot3 * dt

    norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    q0 /= norm
    q1 /= norm
    q2 /= norm
    q3 /= norm

def get_euler():
    roll = math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
    pitch = math.asin(2*(q0*q2 - q3*q1))
    yaw = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# ================= SERIAL =================
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# ================= GYRO CALIBRATION =================
print("Calibrating gyro... Keep still")

samples = 200
bx = by = bz = 0
count = 0

while count < samples:
    line = ser.readline().decode(errors='ignore').strip()
    data = parse_line(line)
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

print("Gyro calibrated\n")

# ================= OFFSET CALIBRATION =================
print("Calibrating offsets... Keep flat")

samples = 100
r_sum = p_sum = count = 0

while count < samples:
    line = ser.readline().decode(errors='ignore').strip()
    data = parse_line(line)
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

print("Offset calibrated\n")

# ================= FLASK =================
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

# ================= MAIN LOOP =================
def update_loop():
    global prev_time, latest_roll, latest_pitch, latest_yaw,latest_temp, latest_pressure, latest_altitude
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        bmp_data = parse_bmp(line)
        if bmp_data:
            latest_temp, latest_pressure, latest_altitude = bmp_data
            continue  # skip IMU processing for this line
        imu_data = parse_line(line)

        if not imu_data:
            continue

        ax, ay, az, gx, gy, gz = imu_data

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

        roll, pitch, yaw = get_euler()

        roll -= roll_offset
        pitch -= pitch_offset

        latest_roll = roll
        latest_pitch = pitch
        latest_yaw = yaw
        # log_to_csv(latest_roll,latest_pitch,latest_yaw,latest_altitude,latest_temp, latest_pressure)
        # time.sleep(0.05)
            

# ================= RUN =================
if __name__ == "__main__":
    import threading

    threading.Thread(target=update_loop, daemon=True).start()
    app.run(port=5000)


