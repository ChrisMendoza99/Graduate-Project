# 📡 ESP32-C5 Multi-Sensor Hub 🌐

This project implements a comprehensive multi-sensor hub using the ESP32-C5 microcontroller. It integrates data from various sensors (temperature, humidity, IMU, ambient light), transmits this data over a CAN bus (with optional encryption using Ascon-128), and provides Bluetooth Low Energy (BLE) connectivity for remote access. The system also includes an LED strip for visual feedback.

## 🚀 Key Features

*   **Multi-Sensor Integration:** Reads data from AHT20 (temperature/humidity), LSM6DSO32 (IMU), and VEML7700 (ambient light) sensors. 🌡️
*   **CAN Bus Communication:** Transmits sensor data over a CAN bus using the TWAI controller. 🚗
*   **Encryption Options:** Supports CAN bus message encryption using Ascon-128 algorithms for secure communication. 🔒
*   **Bluetooth Low Energy (BLE):** Enables remote access to sensor data via BLE using the NimBLE stack. 📱
*   **LED Strip Control:** Provides visual feedback through an LED strip, indicating system status or sensor readings. 💡
*   **User-Friendly Flashing:** Includes a bash script for easy flashing and monitoring of ESP32 devices. ⚡

## 🛠️ Tech Stack

*   **Microcontroller:** ESP32-C5
*   **Real-Time Operating System (RTOS):** FreeRTOS
*   **Wireless Communication:**
    *   Bluetooth: NimBLE
*   **Serial Communication:**
    *   CAN: TWAI (Two-Wire Automotive Interface)
*   **Sensors:**
    *   AHT20 (Temperature & Humidity)
    *   LSM6DSO32 (IMU - Accelerometer & Gyroscope)
    *   VEML7700 (Ambient Light)
*   **Encryption:**
    *   AES-CCM (using mbedtls)
    *   Ascon-128
*   **Programming Language:** C
*   **Build System:** CMake, ESP-IDF
*   **Component Manager:** ESP-IDF Component Manager
*   **LED Strip Library:** `espressif/led_strip`
*   **Bash Scripting:** For device flashing and monitoring.

## 📂 Project Structure

```
├── CMakeLists.txt
├── main
│   ├── canaes
│   │   ├── canaes.c
│   │   └── canaes.h
│   ├── canascon
│   │   ├── canascon.c
│   │   └── canascon.h
│   ├── esp32_ascon
│   │   ├── ascon.h
│   │   ├── core.c
│   │   ├── decrypt.c
│   │   ├── encrypt.c
│   │   └── constants.h
│   ├── bluetooth
│   │   ├── ble_fdcan_sens.h
│   │   └── gatt_svr.c
|   |   └── esp_ble.c
│   ├── i2c_devices
│   │   ├── AHT20.h
│   │   ├── lsm6d.h
│   │   └── veml.h
│   ├── grad_proj.c
│   └── idf_component.yml
├── device_selector.sh
└── README.md
```

## 📸 Screenshots
![App Screenshot](https://github.com/ChrisMendoza99/Graduate-Project/blob/main/pictures/ESP32C5_StackBoard.jpg)
![App Screenshot](https://github.com/ChrisMendoza99/Graduate-Project/blob/main/pictures/IMG_6477.png)

## 📝 License

This project is licensed under the [MIT License](LICENSE) - see the `LICENSE` file for details.
