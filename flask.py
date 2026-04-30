from flask import Flask, jsonify
from flask_cors import CORS
from ReadingIMU_BMPdata import parse_imu_data , parse_bmp_data , init_serial
from magdwick_filter import get_euler
from logCSV import log_to_csv
ser = init_serial()
from magdwick_filter import madgwick_update
import time

ACC_SCALE = 16384.0
GYRO_SCALE = 131.0
gyro_bias_x = 0
gyro_bias_y = 0
gyro_bias_z = 0
roll_offset = 0
pitch_offset = 0

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
        bmp = parse_bmp_data(line)
        if bmp:
            latest_temp, latest_pressure, latest_altitude = bmp
            continue

        # IMU DATA
        imu = parse_imu_data(line)
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