import serial
import time

PORT ='COM5'
BAUD_RATE =115200


def parse_imu_data(line):
     
     parts= line.split('|')
     acc_part = parts[0]
     gyro_part = parts[1]

     acc_data= acc_part[4:].split(',')
     gyro_data= gyro_part[5:].split(',')

     ax = float (acc_data[0])
     ay= float(acc_data[1])
     az= float(acc_data[2])

     gx= float(gyro_data[0]) 
     gy= float (gyro_data[1])
     gz= float (gyro_data[2])

     return [ax,ay,az,gx,gy,gz]

def parse_bmp_data(line):
    try:
        bmp_data = line[4:].split(',')

        if len(bmp_data) < 3:
            return None  # not enough data

        temp = float(bmp_data[0])
        pressure = float(bmp_data[1])
        altitude = float(bmp_data[2])

        return [temp, pressure, altitude]

    except (ValueError, IndexError):
        return None


def init_serial():
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    return ser

def get_line(ser):
    if ser.in_waiting > 0:
        return ser.readline().decode('utf-8', errors='ignore').strip()
    return None


# keep main ONLY for standalone running
def main():
    ser = init_serial()
    while True:
        line = get_line(ser)
        if line:
            print(line)


if __name__ == "__main__":
    main()