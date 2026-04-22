# ✈ Flight Controller 

A real-time flight telemetry dashboard that visualizes IMU (Inertial Measurement Unit) and BMP sensor data.
The system processes raw sensor data, performs sensor fusion using the Madgwick algorithm, and displays aircraft orientation and environmental data on a web-based dashboard.

---

##  Features

* Real-time Roll, Pitch, Yaw visualization
* Artificial Horizon (Attitude Indicator)
* Compass (Yaw-based heading)
* Live graphs using Chart.js
* Sensor data display:

  * Temperature
  * Pressure
  * Altitude
* Alert system:

  * Extreme orientation detection
  * Sensor freeze detection
* CSV logging of flight data
* Backend API using Flask

---

## 🧠 System Architecture

```
IMU + BMP Sensors → Serial (COM Port) → Python Backend → Flask API → Web Dashboard
```

---

## 📂 Project Structure

```
Flight Controller Dashboard/
│
├── server.py              # Backend (sensor processing + API + logging)
├── index.html             # Frontend dashboard UI
├── flight_log_*.csv       # Auto-generated logs
├── .gitignore
├── README.md

```

---

## ⚙️ Technologies Used

### Backend

* Python
* Flask
* PySerial
* CSV (data logging)

### Frontend

* HTML, CSS, JavaScript
* Chart.js

### Hardware

* IMU Sensor (Accelerometer + Gyroscope)
* BMP Sensor (Temperature, Pressure, Altitude)

---
## Dashboard Preview
<img width="1920" height="812" alt="Image" src="https://github.com/user-attachments/assets/fe2c8d44-ab24-4e3c-9a51-8189b0f74cf1" />
<img width="1920" height="799" alt="Image" src="https://github.com/user-attachments/assets/eab672f5-cdf2-4b0e-b8c8-55914cac543b" />
<img width="1920" height="818" alt="Image" src="https://github.com/user-attachments/assets/24261147-4ed0-4a1f-9c59-7b244d7637a0" />

https://github.com/user-attachments/assets/1550c2e5-718c-4f87-9541-b88d7c234211


## 📦 Requirements

Install dependencies:

```bash
pip install flask flask-cors pyserial
```

---

## 🔌 Hardware Setup

* Connect IMU + BMP sensor to microcontroller
* Ensure serial data format matches:

### IMU Format:

```
ACC:x,y,z | GYRO:x,y,z
```

### BMP Format:

```
BMP:temp,pressure,altitude
```

* Update COM port in `server.py`:

```python
PORT = 'COM5'
```

---

## 🧮 Core Logic

### 1. Sensor Parsing

* Extract accelerometer and gyroscope values
* Extract BMP sensor values separately

### 2. Sensor Fusion

* Uses **Madgwick Algorithm** for orientation estimation
* Converts raw IMU data → quaternion → Euler angles

### 3. Calibration

* Gyroscope bias calibration (stationary)
* Roll & pitch offset calibration (flat surface)

### 4. Data Processing

* Normalize accelerometer values
* Remove gyro bias
* Compute delta time (dt)
* Update quaternion

### 5. Output

* Roll, Pitch, Yaw
* Temperature, Pressure, Altitude

---

## 🌐 API Endpoint

```
GET /data
```

Returns:

```json
{
  "roll": float,
  "pitch": float,
  "yaw": float,
  "temp": float,
  "pressure": float,
  "altitude": float
}
```

---

## 📊 Frontend Features

The dashboard ():

* Fetches data every 100ms
* Updates:

  * Artificial horizon (roll + pitch)
  * Compass (yaw)
  * Live graphs
* Displays alerts:

  * Extreme orientation (>60°)
  * Sensor freeze detection

---

## 🧾 Data Logging

* Automatically creates CSV file:

```
flight_log_YYYYMMDD_HHMMSS.csv
```

* Logged fields:

  * Timestamp
  * Roll, Pitch, Yaw
  * Altitude
  * Temperature
  * Pressure

---

## ▶️ How to Run

### Step 1: Start Backend

```bash
python server.py
```

* Performs:

  * Gyro calibration
  * Offset calibration
  * Starts Flask server at `http://127.0.0.1:5000`

---

### Step 2: Open Dashboard

* Open `index.html` in browser

---

## ⚠️ Notes

* Keep device **still during calibration**
* Ensure correct COM port
* Serial baud rate must match:

```python
BAUD_RATE = 115200
```

---

## 🔮 Future Improvements

- Improve sensor calibration and bias correction  
- Reduce noise in IMU readings for more stable output  
- Implement a Kalman filter for better sensor fusion accuracy  
- Optimize data update rate between backend and frontend  
- Handle sensor failures and communication errors more robustly  

---

## 📌 Summary

This project demonstrates:

* Sensor fusion (IMU + BMP)
* Real-time data processing
* Full-stack integration (Python + Web)
* Flight instrumentation concepts

---

