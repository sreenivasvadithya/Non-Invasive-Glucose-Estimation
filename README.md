# Non-Invasive Glucose Estimation Using Optical Sensors
This repository contains the complete design, code, and documentation for a non-invasive glucose monitoring prototype. The system uses near-infrared (IR 940 nm) and red LEDs to illuminate the skin. Reflected light is detected by a BPW34 photodiode, amplified with LM358 op-amps, and digitized using an ESP32 microcontroller. Data is transmitted via Wi-Fi to Firebase for real-time monitoring and analysis.

The project demonstrates how low-cost optical sensing and cloud connectivity can be combined to estimate glucose levels without blood sampling. While the initial prototype shows limitations in sensitivity and ambient light interference, it provides a foundation for future improvements in wearable glucose monitoring solutions.

# Features
Multi-wavelength optical sensing (IR and red LEDs)

Real-time analog signal acquisition

ESP32 Wi-Fi integration with Firebase

Visual LED indicators for glucose levels

Sample machine learning scripts for data analysis

Complete hardware schematics and firmware

# Repository Contents
/firmware: ESP32 Arduino code

/hardware: Circuit diagrams and component lists

/scripts: Google Colab notebooks for data processing

/docs: User guides and setup instructions

# Quick Start
Assemble the circuit as shown in /hardware.

Flash the ESP32 firmware using Arduino IDE.

Configure Firebase credentials in the code.

Start logging and analyzing data in real time.
