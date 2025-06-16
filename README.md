
# OriSense

**OriSense** is an open-source Arduino-based project designed for interacting with the Adafruit MPU6050 6-DoF accelerometer and gyroscope sensor. This collection of sketches provides a versatile toolkit for reading, processing, and analyzing motion data, from basic accelerometer and gyroscope readings to advanced features like motion detection, distance tracking, and orientation estimation. Whether you're building a robotics project, a motion-sensing wearable, or an IoT device, OriSense empowers you to harness the full potential of the MPU6050.

Let’s dive in and start sensing motion with OriSense! 🚀

# Features

Here’s what **OriSense** brings to the table:

- 📏 **Comprehensive Motion Sensing**: Read accelerometer (m/s²), gyroscope (rad/s or °/s), and temperature (°C) data from the MPU6050.
- 🛠️ **Advanced Data Processing**:
  - Calibration with EEPROM storage for persistent offsets (`MPU-Viewer.ino`).
  - Moving average filtering for noise reduction.
  - Complementary filter for roll and pitch estimation.
- 🚨 **Motion Detection**: Configurable motion interrupts for detecting significant movements (`MotionSense.ino`).
- 📍 **Distance Tracking**: Estimate velocity and distance using accelerometer data with drift correction (`DistanceSense.ino`).
- 📜 **Flexible Output Formats**: Serial output tailored for easy parsing by external tools (e.g., Python dashboards).
- 🧰 **Modular Sketches**: Individual scripts for specific use cases (e.g., single-axis acceleration, rotation, motion detection).
- ⚙️ **Cross-Platform Compatibility**: Compatible with Arduino boards supporting I2C (e.g., Uno, Nano, ESP32).

> “OriSense turns raw sensor data into meaningful motion insights—perfect for any project!” — A happy maker

# Use Cases

**OriSense** is ideal for:
* **Robotics**: Track orientation, detect motion, or estimate movement for autonomous robots.
* **Wearables**: Monitor activity, gestures, or steps in fitness trackers or smart clothing.
* **IoT Devices**: Integrate motion data into connected systems for home automation or monitoring.
* **Educational Projects**: Teach sensor interfacing, data processing, and motion physics.
* **Prototyping**: Quickly test MPU6050 features for custom hardware or research applications.

# Installation

### Prerequisites
* Arduino IDE 2.0+ or compatible platform
* Adafruit MPU6050 sensor module
* Arduino board with I2C support (e.g., Arduino Uno, Nano, ESP32)
* Libraries:
  - `Adafruit_MPU6050`
  - `Adafruit_Sensor`
  - `Wire` (standard Arduino library)
  - `EEPROM` (standard Arduino library, used in `MPU-Viewer.ino`)

### Steps
1. **Install Arduino Libraries**:
   Open the Arduino IDE, go to `Sketch > Include Library > Manage Libraries`, and install:
   - `Adafruit MPU6050` by Adafruit
   - `Adafruit Sensor` by Adafruit
   Ensure the `Wire` and `EEPROM` libraries are included (they come with the Arduino IDE).

2. **Clone or Download**:
   Clone the repository or download the project files:
   ```bash
   git clone https://github.com/your-repo/OriSense.git
   cd OriSense
   ```

3. **Connect Hardware**:
   - Connect the MPU6050 to your Arduino:
     - `VCC` to 3.3V or 5V (check your module’s specs)
     - `GND` to GND
     - `SCL` to SCL (e.g., A5 on Uno)
     - `SDA` to SDA (e.g., A4 on Uno)
   - Verify connections and ensure proper power supply.

4. **Upload a Sketch**:
   - Open a `.ino` file (e.g., `MPU-Viewer.ino`) in the Arduino IDE.
   - Select your board and port under `Tools > Board` and `Tools > Port`.
   - Click `Upload` to program the Arduino.
   - Open the Serial Monitor (`Tools > Serial Monitor`) at 115200 baud to view output.

# Project Structure

```
OriSense/
├── MPU-Viewer.ino         # Advanced demo with calibration, filtering, and orientation
├── Accelerometer_x.ino    # Basic accelerometer X-axis readings
├── Accelerometer_y.ino    # Basic accelerometer Y-axis readings
├── Accelerometer_z.ino    # Basic accelerometer Z-axis readings
├── Rotation_x.ino         # Basic accelerometer X-axis readings (duplicate)
├── Rotation_y.ino         # Basic accelerometer Y-axis readings
├── Rotation_z.ino         # Basic accelerometer Z-axis readings
├── MotionSense.ino        # Motion detection with interrupt-based triggers
├── RotationSense.ino      # Gyroscope readings in rad/s and °/s
├── DistanceSense.ino      # Distance and velocity estimation from accelerometer
└── README.md              # Project documentation
```

⚠️ **Note**: The `Rotation_x.ino` sketch is a duplicate of `Accelerometer_x.ino`. Consider removing or renaming it to avoid confusion.

# Libraries Used

The sketches rely on the following libraries:

1. **All Sketches**:
   - `Adafruit_MPU6050` (external, from Adafruit)
   - `Adafruit_Sensor` (external, from Adafruit)
   - `Wire` (standard Arduino library)

2. **MPU-Viewer.ino** (additional):
   - `EEPROM` (standard Arduino library)

**External Dependencies**:
- Install `Adafruit_MPU6050` and `Adafruit_Sensor` via the Arduino Library Manager.
- No other external libraries are required.

# Usage

Each sketch is designed for a specific MPU6050 use case. Below is an overview of how to use the main sketches and interpret their output.

### Main Sketches

1. **MPU-Viewer.ino** (Advanced Demo):
   - **Purpose**: Full-featured demo with calibration, moving average filtering, complementary filter for orientation, and periodic recalibration checks.
   - **Output**: Formatted serial data (115200 baud):
     ```
     Adafruit MPU6050 Advanced Demo!
     MPU6050 Found!
     Loaded calibration from EEPROM
     Setup complete!
     Acceleration X: -0.01, Y: 0.01, Z: 9.79
     Rotation X: -0.00, Y: 0.00, Z: 0.00
     Rotation X: -0.04, Y: 0.03, Z: 0.01
     Estimated Roll: 0.12 °
     Estimated Pitch: -0.08 °
     Temperature: 26.34
     ---
     ```
   - **Usage**: Upload the sketch, open the Serial Monitor, and keep the sensor stationary during initial calibration (if not pre-calibrated). Use with a Python dashboard (e.g., `Test_1.py`) for real-time monitoring.

2. **Accelerometer_x.ino**, **Accelerometer_y.ino**, **Accelerometer_z.ino**:
   - **Purpose**: Basic demos for single-axis accelerometer readings (X, Y, Z) with positive and negative values.
   - **Output** (e.g., `Accelerometer_x.ino`):
     ```
     Adafruit MPU6050 test!
     MPU6050 Found!
     Accelerometer range set to: +-8G
     Accel X: 0.02 , -X: -0.02
     ---
     ```
   - **Usage**: Upload the desired sketch to focus on a specific axis. Useful for simple testing or calibration.

3. **RotationSense.ino**:
   - **Purpose**: Reads gyroscope data in both radians per second (rad/s) and degrees per second (°/s).
   - **Output**:
     ```
     Adafruit MPU6050 test!
     MPU6050 Found!
     Rotation X: 0.00, Y: 0.01, Z: -0.01 rad/s
     Rotation X: 0.00°/s, Y: 0.57°/s, Z: -0.57°/s
     ```
   - **Usage**: Upload to monitor rotational velocity. Ideal for projects needing angular rate data.

4. **MotionSense.ino**:
   - **Purpose**: Detects significant motion using MPU6050’s interrupt feature and reports acceleration and gyroscope data.
   - **Output** (on motion detection):
     ```
     AccelX:0.03,AccelY:0.01,AccelZ:9.80,GyroX:0.00,GyroY:0.01,GyroZ:-0.01
     Accelerometer Magnitude:9.80
     ```
   - **Usage**: Upload and connect the MPU6050’s interrupt pin to your Arduino. Monitor for motion-triggered events.

5. **DistanceSense.ino**:
   - **Purpose**: Estimates velocity and distance by integrating accelerometer data with drift correction.
   - **Output** (when moving):
     ```
     Calibrating MPU6050...
     Calibration successful.
     AccelX (cm/s^2): 150.00  Velocity (cm/s): 2.50  SessionDist (cm): 10.25  TotalDist (cm): 15.75
     ```
   - **Usage**: Upload and ensure the sensor is stationary during calibration. Use for motion tracking applications.

### Example Workflow
1. Connect the MPU6050 to your Arduino and open the Arduino IDE.
2. Load `MPU-Viewer.ino` for a comprehensive demo.
3. Upload the sketch and open the Serial Monitor at 115200 baud.
4. Observe the output, which includes calibrated acceleration, rotation, orientation, and temperature data.
5. For specific needs, upload other sketches (e.g., `MotionSense.ino` for motion detection).
6. Pair with a Python script (e.g., `Test_1.py`) to log or visualize data.

### Serial Output Parsing
The sketches (especially `MPU-Viewer.ino`) produce structured output with delimiters (e.g., commas, newlines, `---`) for easy parsing. Example Python parsing for `MPU-Viewer.ino`:
```python
import serial
ser = serial.Serial('/dev/ttyACM0', 115200)
while True:
    line = ser.readline().decode('utf-8').strip()
    if line.startswith('Acceleration'):
        accel = [float(x.split(': ')[1]) for x in line.split(', ')]
        print(f"Accel X: {accel[0]}, Y: {accel[1]}, Z: {accel[2]}")
```

# Customization

To adapt **OriSense** for your project, modify the sketches to suit your needs. The `MPU-Viewer.ino` sketch is the most feature-rich and a good starting point for customization, while others are simpler for focused applications.

### Why `MPU-Viewer.ino`?
- **Comprehensive Features**: Includes calibration, filtering, and orientation estimation, making it highly extensible.
- **Structured Output**: Easy to parse for integration with external tools.
- **Flexible Configuration**: Adjust constants (e.g., `WINDOW_SIZE`, `alpha`) or add new processing logic.

### What Can You Edit?
- **Sensor Settings**: Change accelerometer range (`MPU6050_RANGE_8_G`), gyro range (`MPU6050_RANGE_500_DEG`), or filter bandwidth (`MPU6050_BAND_21_HZ`) in `setup()`.
- **Data Processing**: Modify `movingAverage`, `complementary filter`, or add new filters (e.g., Kalman).
- **Output Format**: Customize `Serial.print` statements for different data formats (e.g., JSON, CSV).
- **Calibration Logic**: Adjust `calibrateIMU` or `checkCalibrationNeeded` thresholds for specific environments.
- **New Features**: Add commands to trigger actions via serial input (e.g., recalibrate on demand).

### Example: Add JSON Output to `MPU-Viewer.ino`
Modify the `loop()` function to output data in JSON format:
```cpp
void loop() {
  // ... existing code until data processing ...
  // After processing accelX, accelY, accelZ, gyroX, gyroY, gyroZ, roll, pitch, temp
  Serial.print("{\"accel\":{");
  Serial.print("\"x\":"); Serial.print(accelX, 2);
  Serial.print(",\"y\":"); Serial.print(accelY, 2);
  Serial.print(",\"z\":"); Serial.print(accelZ, 2);
  Serial.print("},\"gyro\":{");
  Serial.print("\"x\":"); Serial.print(gyroX, 2);
  Serial.print(",\"y\":"); Serial.print(gyroY, 2);
  Serial.print(",\"z\":"); Serial.print(gyroZ, 2);
  Serial.print("},\"roll\":"); Serial.print(roll, 2);
  Serial.print(",\"pitch\":"); Serial.print(pitch, 2);
  Serial.print(",\"temp\":"); Serial.print(temp.temperature, 2);
  Serial.println("}");
  Serial.println("---");
  delay(700);
}
```

**Output**:
```
{"accel":{"x":-0.01,"y":0.01,"z":9.79},"gyro":{"x":0.00,"y":0.00,"z":0.00},"roll":0.12,"pitch":-0.08,"temp":26.34}
---
```

### Example: Add Serial Command for Recalibration
Add serial input to trigger recalibration in `MPU-Viewer.ino`:
```cpp
void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n').trim();
    if (cmd == "calibrate") {
      calibrateIMU();
    }
  }
  // ... existing loop code ...
}
```

**Usage**: Send `calibrate` via the Serial Monitor to recalibrate the sensor.

### Notes on Customization
- **Constants**: Adjust `WINDOW_SIZE` (filter window), `alpha` (complementary filter weight), or `varianceThreshold` (calibration check) for performance tuning.
- **Integration**: Use `DistanceSense.ino`’s velocity/distance logic in `MPU-Viewer.ino` for motion tracking.
- **Error Handling**: Enhance `isnan` checks or add retry logic for robust operation.
- **External Tools**: Pair with Python (e.g., `Test_1.py`) or Processing for data visualization.

# Future Enhancements

- 📊 **Real-Time Visualization**: Integrate with Processing or Python to plot motion data.
- 📡 **Wireless Support**: Add Bluetooth or WiFi (e.g., ESP32) for remote monitoring.
- 🧠 **Advanced Filtering**: Implement Kalman or Madgwick filters for better orientation estimation.
- 📄 **Data Logging**: Store data on an SD card or EEPROM for offline analysis.
- ⚡ **Low-Power Mode**: Optimize for battery-powered applications.

# Extra Goodies

### Table of Supported Boards
| Board       | Status       | Notes                       |
|-------------|--------------|-----------------------------|
| Arduino Uno | ✅ Supported | Standard I2C pins (A4, A5)  |
| Arduino Nano| ✅ Supported | Compact form factor         |
| ESP32       | ✅ Supported | Wireless capabilities       |
| Others      | ✅ Supported | Any I2C-capable Arduino     |

### Horizontal Rule
***
Ready to sense motion with OriSense? Let’s build something dynamic! 🚀

# License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more information.

# Conclusion

**OriSense** is a versatile toolkit for working with the Adafruit MPU6050, offering everything from basic sensor readings to advanced motion processing and distance tracking. Its modular sketches cater to beginners and experts alike, making it perfect for robotics, wearables, IoT, and educational projects. By customizing `MPU-Viewer.ino` or combining features from other sketches, you can tailor OriSense to your specific needs. As an open-source project, it invites contributions to expand its capabilities. Connect your MPU6050, upload a sketch, and start exploring the world of motion sensing today! 🚀
