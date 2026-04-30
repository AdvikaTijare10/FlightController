from ReadingIMU_BMPdata import parse_imu_data
from ReadingIMU_BMPdata import init_serial

ser = init_serial()


print("Calibrating gyro... Keep still")

samples = 200
bx = by = bz = 0

count = 0
while count < samples:
    line = ser.readline().decode(errors='ignore').strip()
    data = parse_imu_data(line)
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
