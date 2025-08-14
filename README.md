## IEP\_Urban\_Gardening\_System\_Minimal

Collaboration between Prabhu Sumathi Vanavan and Abinaash Abirama Sundaram of Singapore Polytechnic (Year 1 Computer Engineering DCPE stduents) 

An Arduino-based environmental monitoring and automation system designed for small-scale urban gardening. This project utilizes the RichShield sensor suite to track and respond to key greenhouse parameters, specifically temperature, humidity, and ambient light levels.

### Technical Overview

**Microcontroller**: Arduino Uno-compatible board
**Peripheral Interface**: RichShield development board

**Sensor Inputs**:

* DHT11: Digital temperature and humidity sensor
* LDR: Analog light sensor for ambient brightness detection

**Output Devices**:

* TM1637 4-digit 7-segment display: Displays live sensor data (e.g., temperature, light level)
* RGB and yellow LEDs: Indicate system status (normal, warning, panic)
* Passive buzzer: Audible alert in panic conditions

### System Behavior

* **Real-Time Monitoring**: Periodic polling of sensor values with basic data smoothing to reduce noise.
* **State Detection**:

  * **Normal**: All readings are within predefined thresholds.
  * **Warning**: One or more readings slightly outside optimal range; yellow LED blinks.
  * **Panic**: Readings significantly out of range; red LED and buzzer are activated.
* **Recovery Logic**: On return to safe conditions from a panic state, a green LED sequence is triggered to indicate recovery.
* **System Stability**: Watchdog timer is used to auto-reset the system in case of software lockups.

### Design Intent

This project is built under strict hardware constraints—only the Arduino Uno and RichShield are used. It demonstrates a minimal yet functional environmental monitoring system suitable for educational purposes, prototypes, or basic automation in urban gardening setups.

---

**Note**: This is a student project. The code is provided as-is and may be modified or reused freely for educational and non-commercial use.
