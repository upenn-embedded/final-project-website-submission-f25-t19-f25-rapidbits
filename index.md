# RapidBits Final Project Website

Welcome to our final project website for **F25-T19 RapidBits**.

## Project Overview

This project presents a fully integrated Indoor Air Quality Monitor capable of tracking four key environmental indicators: CO₂, TVOC, temperature, and humidity. The system is built around the ATmega328PB microcontroller, which collects gas measurements from the SGP30 via I²C and temperature–humidity data from the DHT11 through a single-wire digital interface.

A 1.8-inch ST7735 LCD provides a clear and intuitive real-time display, showing numerical readings alongside bar-graph visualizations for temperature, humidity, CO₂, and TVOC levels. To enhance user awareness, the device includes an RGB LED and an active buzzer that automatically respond to poor air quality by signaling warning or alarm states.

The system also supports UART communication for debugging and data monitoring, and incorporates an ESP32 module to enable Wi-Fi-based cloud logging. Together, these features demonstrate a complete embedded sensing platform that integrates measurement, processing, visualization, and user feedback to help users understand and respond to changes in indoor environmental conditions.

## Team Members

- Fengyu Wu  
- Qinyan Zhang
- Weiye Zhai

![1765236560228](image/index/1765236560228.png)

## 1. Final Product Video

The following video demonstrates the full functionality of our Indoor Air Quality Monitor.  
It is under 5 minutes and highlights all key features of the final system, including:

- Real-time sensing of CO₂, TVOC, temperature, and humidity  
- 1 Hz data acquisition using the SGP30 (I²C) and DHT11 (single-wire) sensors  
- Live LCD updates with graphical bar indicators  
- Automatic LED and buzzer alarms for poor air quality  
- UART output for debugging and data verification  
- System responsiveness to environmental changes  
- Integration with the ESP32 module for cloud connectivity  

🎥 **Watch the Final Project Video:**  
[**Final Project Video Link**](https://drive.google.com/file/d/1Af4kr8z-wDgtJuuS6ySc6BVJDjWGENj6/view?usp=sharing)

## 2. Images of the Final Product

Below are several views of our completed Indoor Air Quality Monitor, including both the exterior enclosure and the internal electronics. These photos highlight the full integration of sensors, display module, indicators, and MCU hardware.
### **400×400 image **
![1765238678673](image/index/1765238678673.png)

### **Exterior Views**
The exterior images show the assembled device, including the LCD interface, RGB LED indicator, and overall form factor.

![1765238870214](image/index/1765238870214.png)
![1765238919365](image/index/1765238919365.png)

---

### **Interior Views**
The internal photos reveal the embedded system architecture, showing the ATmega328PB controller board, SGP30 and DHT11 sensors, ST7735 LCD wiring through the level shifter, RGB LED, buzzer, and ESP32 Wi-Fi module.

![interior1](image/README/interior1.png)
![interior2](image/README/interior2.png)

---



## Video
Video is available in the `video` folder.