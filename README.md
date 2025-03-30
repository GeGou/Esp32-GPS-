# Esp32-GPS- 🚀

> GPS project using Esp32 (especially Xiao Esp32-C3) and GSM communication via MQTT Broker. It also can detected a BLE tag to know when to sent the GPS Data and when not to.

![GitHub License](https://img.shields.io/github/license/GeGou/Esp32-GPS-)
![GitHub stars](https://img.shields.io/github/stars/GeGou/Esp32-GPS-)
![GitHub forks](https://img.shields.io/github/forks/GeGou/Esp32-GPS-)

## ✨ Features
- It only wakes up when the tilt sensor receives vibration, for better battery management.
- It can detect every BLE tag based on tag's mac address.

## 📦 Installation

### Prerequisites
Make sure you have the following installed:
- [PlatformIO](https://platformio.org/) (if using ESP32)
- Arduino IDE (if applicable)

## 🚀 Usage
1. Connect the device to power.
2. Modify `config.h` for GPRS/MQTT settings.
3. Upload the code to ESP32.
4. Open the serial monitor to see logs.

## 📁 Project Structure
```
📦 GeGou/Esp32-GPS-
 ┣ 📂 include          # Header files (.h)
 ┣ 📂 lib              # Custom libraries
 ┣ 📂 src              # Main source code
 ┃ ┣ 📜 main.cpp       # Entry point
 ┃ ┣ 📜 config.cpp     # Configuration definitions
 ┃ ┗ 📜 config.h       # Configuration declarations
 ┣ 📜 platformio.ini   # PlatformIO configuration
 ┣ 📜 README.md        # This file
 ┗ 📜 LICENSE          # License file
```

## 🛠 Configuration
Modify the `include/config.h` file to match your setup:
```cpp
#define WIFI_SSID "YourWiFi"
#define WIFI_PASSWORD "YourPassword"
```

## 🤝 Contributing
Want to contribute? Follow these steps:
1. Fork this repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Commit your changes (`git commit -m "Add new feature"`).
4. Push to your branch (`git push origin feature-branch`).
5. Open a Pull Request.

## 📜 License
This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

## 📞 Support
If you have any questions, create an **Issue** or reach out via:
- 📧 Email:
- 💬 Discord:
