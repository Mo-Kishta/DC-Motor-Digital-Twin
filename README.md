# DC Motor Digital Twin 🔄⚙️

Welcome to the **DC Motor Digital Twin** project! This repository contains a complete Virtual-to-Physical (V2P) system that bridges a 3D software simulation with real-world physical hardware in real-time.

## 📁 Repository Structure
As seen in the repository, this project is a complete package containing both the hardware firmware and the software application:
* 📂 **`Arduino-DC-Motor-Code/`**: Contains the Arduino firmware (`.ino`) responsible for motor control, PID calculations, sensor reading, and serial communication.
* 📂 **`MotorTwinApp/`**: Contains the complete Unity 3D project for the interactive dashboard and digital twin visualization.

> **⚠️ IMPORTANT:** The Arduino code provided in this repository is specifically designed to work seamlessly with the included Unity application. They communicate via a custom Serial protocol. To experience the full Digital Twin functionality, both the firmware and the Unity app must be used together.

## 🚀 Project Overview
The system establishes a seamless bidirectional communication link between a Unity 3D dashboard and a physical microcontroller setup. 
* **Virtual to Physical:** Commands from the Unity UI (Target Speed, Target Angle, Auto/Manual modes) are sent to the Arduino to control the physical motor.
* **Physical to Virtual:** Real-time telemetry data (Current RPM, Current Angle, Motor Current, Voltage) is read by the Arduino sensors and sent back to Unity to update the 3D model and dashboard instantaneously.

## 🛠️ Hardware Requirements
* Arduino Board (e.g., Uno/Mega)
* DC Motor with Quadrature Encoder
* Motor Driver (e.g., L298N or similar)
* ACS712 Current Sensor
* Potentiometer (for manual physical control)
* Power Supply (12V)

## 💻 Software & Control Systems
* **Supported OS:** Windows (Required for standard Serial COM port communication).
* **Unity 3D & C#:** Used to design the interactive Digital Twin dashboard.
* **Arduino C/C++:** Used for microcontroller firmware.
* **PID Control:** The system features advanced internal algorithms, utilizing a **Speed PID** to maintain target RPM and a **Position PID** for precise angular targeting.

## 🏃‍♂️ How to Run
1. Flash the firmware from the `Arduino-DC-Motor-Code` folder to your Arduino.
2. Ensure your hardware components are wired correctly according to the pin definitions in the code.
3. Open the `MotorTwinApp` project in Unity, or run the compiled Windows executable (`.exe`) if you built one.
4. Select the correct COM port in the Unity UI to establish the Serial connection.
5. Start controlling your physical motor directly from the virtual dashboard!

---
*Developed by **Mohammed Kishta** - Mechatronics Engineering*
