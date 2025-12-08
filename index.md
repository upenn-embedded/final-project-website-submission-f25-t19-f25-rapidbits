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

## Code
[GitHub Repository](./Code)

## Images
![Example](./image/example.jpg)

## Video
Video is available in the `video` folder.