[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/a-5mB3GB)
# final-project-skeleton

**Team Number:**

**Team Name:**

| Team Member Name | Email Address       |
|------------------|---------------------|
| Fengyu Wu        | ericwu77@seas.upenn.edu           |
| Qinyan Zhang     | zqy@seas.upenn.edu                |
| Weiye Zhai       | zhaiwy@seas.upenn.edu             |

**GitHub Repository URL:**

**GitHub Pages Website URL:** [for final submission]*

## Final Project Proposal

### 1. Abstract

*In a few sentences, describe your final project.*

The Indoor Air Quality Monitor is a compact system designed to track key environmental parameters such as carbon dioxide, volatile organic compounds, temperature, and humidity. It uses a microcontroller to collect data from integrated sensors and display real-time air quality information on a screen. Visual and audio indicators provide immediate feedback when conditions become unhealthy. The design emphasizes efficient communication, power management, and data processing to ensure reliable operation in varied indoor environments. By combining sensing, control, and user interaction in a single platform, this project demonstrates how embedded systems can support healthier and more responsive living spaces.

### 2. Motivation

*What is the problem that you are trying to solve? Why is this project interesting? What is the intended purpose?*

The modern lifestyles lead to longer hours spent indoors, maintaining healthy indoor air quality has become increasingly important. Poor ventilation, household chemicals, and electronic devices can cause a increase of carbon dioxide and volatile organic compounds, negatively affecting comfort, concentration, and overall health and people remain unaware of the quality of the air they breathe each day. Developing an indoor air quality monitoring system provides a practical and affordable way to raise awareness and promote healthier living environments. By combining sensor data with clear visual and audio feedback, such systems help users quickly identify and respond to poor air conditions. This project aims to demonstrate how embedded sensing and control technologies can be integrated to support real-time environmental awareness, energy efficiency, and personal well-being.

### 3. System Block Diagram

*Show your high level design, as done in WS1 and WS2. What are the critical components in your system? How do they communicate (I2C?, interrupts, ADC, etc.)? What power regulation do you need?*

![1761166661147](image/README/1761166661147.png)

The system is powered from a 5 V USB source on the ATmega328PB Xplained Mini board.
The ATmega328PB serves as the main controller and communicates with all peripherals over digital interfaces.
The SGP30 gas sensor and the DHT20 temperature and humidity sensor are both connected via the I²C bus (TWI0), using pins PC4 (SDA) and PC5 (SCL).
The SGP30 operates from the 5 V rail through its onboard 1.8 V LDO and internal I²C level shifting, while the DHT20 is 5 V tolerant and directly shares the same bus without additional conversion.

The OLED display (AOM12864A0-0.96WW-ANO, SSD1306 controller) operates at 3.3 V and is driven through the SPI interface (PB5–SCK, PB3–MOSI, PB2–CS, PB1–DC, PB0–RST).Because the microcontroller operates at 5 V logic, all SPI lines pass through a SN74LVC125 level shifter, which converts the 5 V signals down to 3.3 V before reaching the OLED.

For user feedback, PWM and GPIO outputs control an active buzzer (for air-quality alarm) and an RGB LED (for real-time air-quality indication). All subsystems share a common ground, ensuring reliable communication across the mixed-voltage domains (5 V and 3.3 V).

### 4. Design Sketches

*What will your project look like? Do you have any critical design features? Will you need any special manufacturing techniques to achieve your vision, like power tools, laser cutting, or 3D printing?  Submit drawings for this section.*

![DesignSketch](image/README/DesignSketch.png)

### 5. Software Requirements Specification (SRS)

SRS 01 – The SGP30 gas sensor and DHT20 temperature and humidity sensor continuously monitor the indoor environment. Data from both sensors are collected by the ATmega328PB every 1 second over the I²C (TWI0) interface. Each reading is filtered and displayed on the OLED screen in real time.

SRS 02 – The system calculates and updates air-quality indicators (CO₂ and TVOC levels) every second, comparing them against preset thresholds (600 ppm and 200 ppb). If three consecutive readings exceed these limits, an alarm state is triggered.

SRS 03 – The RGB LED and buzzer provide real-time feedback. The LED color changes according to air quality — green (good), yellow (moderate), and red (poor) — while the buzzer emits a 4 kHz tone in poor-air conditions.

SRS 04 – The LCD display (ST7735, SPI interface) updates at 1 Hz, showing current CO₂, TVOC, temperature, humidity, and system status (normal / warning / sensor fault).

SRS 05 – The firmware performs sensor-presence checks on every startup. If a sensor is disconnected or unresponsive, the system enters degraded mode, displays a warning message, and continues operating with available sensors.

SRS 06 – The ESP32 Wi-Fi module receives temperature, humidity, CO₂, and TVOC data from the ATmega328PB via UART and uploads them to a cloud server in real time. All measurements are stored remotely for long-term logging, enabling users to access historical air-quality records through a connected computer.

SRS 07 – All I²C and SPI communications include checksum or CRC verification where applicable, and the firmware retries each failed transaction up to three times before reporting a communication error.


**5.1 Definitions, Abbreviations**

SGP30: Gas sensor module measuring CO₂ equivalent and Total Volatile Organic Compounds (TVOC).  
DHT20: Temperature and humidity sensor providing environmental measurements.  
SSD1306: Organic Light-Emitting Diode display module driven by SSD1306 controller IC.

**5.2 Functionality**

| ID     | Description                                                                                                                                                                                                              |
| ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| SRS-01 | The system reads air data from the sensors every second.                     |
| SRS-02 | The system checks air quality and triggers an alarm if CO₂ or TVOC stay high.      |
| SRS-03 |The RGB LED and buzzer give visual and sound alerts for air quality. |
| SRS-04 | The LCD screen updates every second to show current sensor readings.       |
| SRS-05 | The system checks all sensors at startup and shows a warning if any fail.      |
| SRS-06 | ESP32 uploads sensor data to the cloud for remote logging.        |
| SRS-07 | The system verifies I²C and SPI data and retries failed communication.  |


### 6. Hardware Requirements Specification (HRS)

HRS 01 – The project is built around the ATmega328PB Xplained Mini microcontroller board, which manages all sensors, display, and actuators through digital I/O, I²C, and SPI interfaces.

HRS 02 – The SGP30 gas sensor (powered from 5 V with internal 1.8 V regulator) measures CO₂ and TVOC concentrations. It communicates via I²C using pins PC4 (SDA) and PC5 (SCL).

HRS 03 – The DHT11 sensor (5 V-tolerant) measures temperature and humidity, providing environmental context for air-quality calculations.

HRS 04 – The LCD display (ST7735 controller, 1.8″, 128x160) operates at 5 V and connects through the SPI interface (PB5 – SCK, PB3 – MOSI, PB2 – CS, PB0 – DC, PB1 – RST).

HRS 05 – A buzzer driven by a virtual PWM output generated by the software on PC3 produces an audible 4 kHz alert tone during poor-air conditions.

HRS 06 – Three RGB LED provide visual indication of air-quality levels, with each color channel limited to ≤10 mA current through resistors.

HRS 07 – All modules share a common ground. The 5 V USB supply powers the ATmega328PB and all peripherals

**6.1 Definitions, Abbreviations**

SGP30: Gas sensor module measuring CO₂ equivalent and Total Volatile Organic Compounds (TVOC).  
DHT11: Temperature and humidity sensor providing environmental measurements.  
ST7735: 1.8 inch LCD module driven by the ST7735 controller IC, communicating via an SPI interface.

**6.2 Functionality**

| ID     | Description                                                                                                                        |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| HRS-01 | The ATmega328PB board controls all sensors, display, and output parts. |
| HRS-02 | The SGP30 gas sensor measures CO₂ and TVOC through the I²C bus.   |
| HRS-03 | The DHT11 sensor reads temperature and humidity .     |
| HRS-04 | The LCD display (ST7735) runs at 5 V via SPI and shows system data.   |
| HRS-05 | The buzzer makes a 4 kHz sound when air quality is poor.  |
| HRS-06 | Three different color LEDs show air quality level.   |
| HRS-07 | All modules share a 5 V USB power and common ground. |

### 7. Bill of Materials (BOM)

*What major components do you need and why? Try to be as specific as possible. Your Hardware & Software Requirements Specifications should inform your component choices.*

[BOM](https://docs.google.com/spreadsheets/d/1OiZFcbvLDSlO1WmZcO8CbIbPxtMEYcChFjx83_pbv7g/edit?gid=2071228825#gid=2071228825)

### 8. Final Demo Goals

*How will you demonstrate your device on demo day? Will it be strapped to a person, mounted on a bicycle, require outdoor space? Think of any physical, temporal, and other constraints that could affect your planning.*

By the final demonstration, we expect to present a fully functional prototype of the Indoor Air Quality Monitor that integrates all key components, including sensors for detecting CO₂, VOCs, temperature, and humidity, as well as visual and audio indicators for feedback. The system will continuously collect and display real-time environmental data, showing clear responses to changes in air quality. During the demo, we will validate the device’s accuracy and responsiveness by simulating various indoor conditions, such as increased CO₂ or VOC levels.

### 9. Sprint Planning

*You've got limited time to get this project done! How will you plan your sprint milestones? How will you distribute the work within your team? Review the schedule in the final project manual for exact dates.*

| Milestone  | Functionality Achieved | Distribution of Work |
| ---------- | ---------------------- | -------------------- |
| Sprint #1  |Finish circuit design and wiring. Connect sensors to the microcontroller. Test I²C and GPIO communication.                        |Fengyu Wu: coding, debugging, Qinyan Zhang: coding，wiring, Weiye Zhai: coding, documentation                      |
| Sprint #2  |Read data from all sensors. Show values on the display. Test LED color changes and buzzer alerts.                        |Fengyu Wu: coding, debugging, Qinyan Zhang: coding，wiring, Weiye Zhai: coding, documentation                      |
| MVP Demo   |Working prototype can measure CO₂, VOC, temperature, and humidity. Display and alerts respond correctly.                        |Fengyu Wu: coding, debugging, Qinyan Zhang: coding，wiring, Weiye Zhai: coding, documentation                      |
| Final Demo |Fully functional and stable system. Clear display, accurate readings, and fast alert response.                        |Fengyu Wu: coding, debugging, Qinyan Zhang: coding，wiring, Weiye Zhai: coding, documentation                      |

**This is the end of the Project Proposal section. The remaining sections will be filled out based on the milestone schedule.**

## Sprint Review #1

### 1.Progress over the past week


After the Team Advisor Meeting, our group revised the **Bill of Materials (BOM)** based on component availability and compatibility feedback.  

![1762993741434](image/README/1762993741434.png)

We switched our primary supplier from **Digi-Key** to **Adafruit**, as the previously selected OLED display could not be sourced from alternative vendors.  

![1762993788118](image/README/1762993788118.png)

To ensure long-term availability and better support, we decided to replace the OLED module with an **LCD screen**.  
Accordingly, we updated and tested the **LCD driver library** to ensure compatibility with the ATmega328PB platform and our future system requirements.

In parallel, we completed the implementation framework for **SRS 01**:

> *The SGP30 gas sensor and DHT20 temperature and humidity sensor continuously monitor the indoor environment.  
> Data from both sensors are collected by the ATmega328PB every 1 second over the I²C (TWI0) interface.)*

This includes:
- Initializing the **TWI0 (I²C)** communication bus and verifying successful device acknowledgment for both sensors.  
- Implementing the periodic sampling routine with a **1-second timer interrupt**.  
- Structuring the **data acquisition loop** to store and prepare environmental readings for later integration with the display and data logging modules.

**Proof of Work:**
- Updated **BOM** and **LCD driver library** uploaded to GitHub.  
- Code framework for **SRS 01** committed to the repository (`/src/sensor_readout/`).  

### Current state of project

This week, we received all previously required components for our system, including the gas sensor and the temperature–humidity sensor. After discussing with our project manager, we decided not to purchase a new OLED display, as the shipping time would be too long and the cost relatively high. Instead, we will reuse the LCD display module from earlier labs to implement the visual output functionality for our final project. This allows us to stay on schedule while still meeting the project’s display requirements.

### Next week's plan

Next week, our main focus will be getting stable and reliable sensor readings. We will refine the sensor-reading code, verify consistent CO₂, TVOC, temperature, and humidity data, and make sure both sensors respond correctly. Once the readings are stable, we will connect this data to the LCD so the basic values can be shown in real time. After the display is working smoothly, we will start adding simple air-quality status logic as preparation for future alert functions.

## Sprint Review #2

### 1.Progress over the past week

This week, the project reached a major milestone with all core functions successfully implemented. The following components are now operational:

•	SGP30/DHT20 environmental data acquisition: Stable readings of temperature, humidity, TVOC, and eCO₂.

•	UART data reception and parsing: Reliable serial communication enabling real-time debugging and parameter monitoring.

•	Alarm logic implementation: Automatic alarm triggering based on gas concentration and temperature/humidity thresholds, with output to LED/buzzer (depending on project configuration).

•	LED air-quality indication logic: LED indicators correctly reflect real-time air-quality levels, providing fast and intuitive feedback.

•	All major source code has been uploaded to GitHub, including drivers, logic modules, interrupt handlers, and the main execution loop.

These functions have undergone multiple rounds of testing and are now operating in a stable state.

### 2. Current state of project

The main system pipeline is fully operational:

•	All hardware components are functioning correctly, with no faults or abnormal temperature rise.

•	I²C, SPI, and UART peripherals have all been validated.

•	The complete processing chain—sensors → MCU → system logic → LED indicators—is confirmed stable.

•	The system can accurately and promptly reflect environmental changes, including variations in temperature, humidity, TVOC, and CO₂, through both LED output and serial data.

The LCD display task is currently in progress. Initialization and communication have been completed, and work is ongoing to finalize the display format and refresh strategy.

Overall, the project has entered a mature and stable development stage, with remaining work focused mainly on feature enhancement and user-interface improvements.

### 3. Next week's plan

Next week, we will finish the LCD display and add the ESP32 Wi-Fi module so the system can send sensor data to the cloud for remote viewing. We will also use FreeRTOS to organize tasks for sensing, display, and communication, so the system runs smoothly and without blocking.

## MVP Demo

1. Show a system block diagram & explain the hardware implementation.  
![1764205108326](image/README/1764205108326.png)

The system is powered from a 5 V USB source through the ATmega328PB Xplained Mini board, which works as the main controller. The SGP30 gas sensor communicates with the microcontroller over the I²C bus (TWI0) using pins PC4 (SDA) and PC5 (SCL). The SGP30 is powered from the 5 V rail but uses its onboard 1.8 V LDO and internal I²C level shifting to meet its core voltage requirements. The DHT11 temperature and humidity sensor is also powered from the 5 V rail and connects to the ATmega328PB through a single-wire digital interface on a GPIO pin (PD2), separate from the I²C bus.

The LCD module (ST7735-based color display) operates at 3.3 V and is driven through the SPI interface. SPI signals from the ATmega328PB—SCK, MOSI, CS, DC, and RESET—are 5 V logic, so they first pass through an SN74LVC125 level shifter, which converts them down to 3.3 V before they reach the LCD. A 3.3 V rail derived from the 5 V supply powers the LCD and other 3.3 V devices, while the ATmega328PB and most sensors remain on the 5 V rail.

For user feedback, a PWM output from the ATmega328PB controls an active buzzer that generates audible alarms when air quality becomes poor, and GPIO pins drive an RGB LED that shows real-time air-quality status with green, yellow, and red colors. An ESP32 module is connected to the ATmega328PB via UART and sends processed air-quality data to a laptop over Wi-Fi, while the USB interface provides direct serial communication for debugging and data logging.

2. Explain your firmware implementation, including application logic and critical drivers you've written.  
Our firmware for the ATmega328PB implements a complete indoor air-quality monitor that periodically reads temperature/humidity (DHT11), air quality (SGP30), and presents the results on an ST7735 LCD, a tri-color LED, and a buzzer on PC3 for audible alarms.

**Initialization steps:**  
The `main()` function performs all initialization and then enters a periodic measurement loop.  
• `uart_init()` - configures the UART for serial debugging output.  
• `timer1_init_us()` – sets up Timer1 with a prescaler of 8 on a 16 MHz clock, achieving a 2 MHz tick (0.5 µs per count). This timer is used by the DHT11 timing routines.  
• `twi0_init()` - sets up TWI0 (I²C) for the SGP30 on PC4/PC5.
• `sgp30_init()` - sends the “init air quality” command to the SGP30 after a startup delay.  
• `buzzer_init()` – sets PC3 as a digital output and drives it low; this pin is later toggled in software to generate audible tones.  
• `lcd_init()` and `lcd_draw_static_ui()` – initialize the ST7735 LCD and draw the static UI layout.  

**Main loop:**  

1. Read DHT11 using `dht_read(&h, &t)`:
   On success: The firmware prints humidity (h) and temperature (t) via UART. Calls `sgp30_update_humidity_from_dht(t, h)` to provide absolute-humidity compensation data to the SGP30.  
   On failure: An error code is printed via UART, and the SGP30 compensation is skipped for that cycle.  
2. Read SGP30 air quality using `sgp30_measure_iaq(&co2, &tvoc)`:
   If the readings are valid (CRC passes): The firmware prints CO2 (ppm) and TVOC (ppb) to UART. Calls `update_air_quality_led(co2, tvoc)` to update the tri-color LED and buzzer based on air quality. If the DHT11 read earlier in the cycle was also valid, calls `lcd_update_readings(t, h, co2, tvoc)` to refresh the LCD UI.
   If the SGP30 read fails (CRC error): Prints an error message over UART.
3. A `_delay_ms(1000)` at the end of each loop sets an approximate 1 Hz update rate.

**DHT11 driver**:
The DHT11 is connected on PD2 and is driven with a custom bit-banged single-wire driver that relies on Timer1 for precise timing. PD2 is configured as an output to issue the start signal. `wait_for_level(level, timeout_us)` uses Timer1 as a 0.5 µs-resolution counter to wait for the data line to reach a specified logic level, with a timeout to avoid hanging if the sensor misbehaves. `dht_read_bit()` decodes a single bit from the DHT11. `dht_read()` issues the start sequence, waits for the sensor’s response, then reads 40 bits into a 5-byte buffer. It validates the data using the checksum `data[4] == data[0] + data[1] + data[2] + data[3]`. On success, it returns integer humidity (`data[0]`) and temperature.

**SGP30 driver**:
The SGP30 gas sensor is accessed over TWI0 (I²C) with address `0x58`. `twi0_start()`, `twi0_write()`, `twi0_read_ack()`, `twi0_read_nack()`, and `twi0_stop()` encapsulate the manipulation of TWI0 registers (TWCR0, TWDR0, TWSR0) and handle start conditions, data phase, acknowledgements, and stop conditions. `sgp30_write_cmd(uint16_t cmd)` sends a 16-bit command to the sensor (MSB first). `sgp30_read_word(uint8_t last)` reads a 16-bit word plus an 8-bit CRC and verifies it using `sgp_crc()`. If the CRC check fails, it returns `0xFFFF` to indicate an error. `sgp30_init()` sends the Init_Air_Quality command (`0x2003`) after a power-on delay. `sgp30_measure_iaq(&co2, &tvoc)` sends the Measure_IAQ command (`0x2008`), waits, then reads two words corresponding to CO₂ (ppm) and TVOC (ppb). `calc_abs_humidity(T, RH)` computes absolute humidity from temperature and relative humidity using a standard psychrometric formula. `sgp30_update_humidity_from_dht(t, h)` converts the DHT11’s t and h readings into absolute humidity, clamps it into a valid range, formats it into the fixed-point representation expected by the SGP30, and sends it using `sgp30_set_absolute_humidity_raw()`.

**LCD UI and graphics driver**:
The LCD UI is rendered on an ST7735 using the `ST7735` and `LCD_GFX` libraries, with a custom layout drawn `in lcd_draw_static_ui()`: the screen is cleared to black, “T:” and “H:” labels are drawn at the top, and two vertical gauges (rectangular tube + circular bulb) visualize temperature (red) and humidity (blue). The bottom row shows “CO2:” and “TVOC:” labels and their numeric values, with CO₂ text color (green/yellow/red) chosen to match the air-quality thresholds. In `lcd_update_temp_bar()` and `lcd_update_hum_bar()`, the current temperature (0–80 °C) and humidity (0–100 %) are mapped to vertical fill bars inside the gauges. In `lcd_update_readings()`, CO₂ (ppm) and TVOC (ppb) are updated along with two horizontal bar graphs on row 110: CO₂ uses a color-matched bar with a white border, while TVOC uses a cyan bar with its own white frame.

**LED and buzzer alarm logic**:
The LED and buzzer alarm system uses simple macros to guarantee only one LED (green/yellow/red) is active at a time, and `update_air_quality_led()` selects the color based on CO₂ and TVOC thresholds while also generating different buzzer tones entirely through software. PC3 is toggled at different rates and durations: no beep in the “good” range, a short low-frequency beep for “moderate” air, and a long higher-frequency beep for “poor” air. This provides a clear, hardware-simple alarm mechanism that matches air-quality levels.

3. Demo your device.
   [MVP demo](https://drive.google.com/file/d/1ZKFtCWNjSGloMHTX5XSS8Vtoo1BYdIxz/view?usp=drive_link)

4. Have you achieved some or all of your Software Requirements Specification (SRS)?

We have achieved the majority of our Software Requirements Specification. The system successfully samples CO₂, TVOC, temperature, and humidity once per second via the I²C (TWI0) interface and evaluates the measurements against preset air-quality thresholds. When CO₂ or TVOC values remain above the limits for three consecutive readings, the system enters an alarm state and activates both the RGB LED and buzzer. The LCD display refreshes at 1 Hz over the SPI interface and shows real-time sensor values, vertical temperature and humidity bar graphs, and horizontal CO₂ and TVOC bar indicators. Sensor-presence checks are performed at startup, and CRC validation ensures reliable sensor communication. Wi-Fi data uploading through the ESP32 module is currently in development and will be completed for the final demonstration.

   1. Show how you collected data and the outcomes.

   Sensor readings are sampled once per second and displayed on the LCD in real time. We monitor the values through both the LCD interface and UART terminal output to verify system behavior. After three consecutive high readings, the RGB LED switches to red and the buzzer produces a 4 kHz tone. The LCD bar graphs and color indicators update immediately to reflect the poor air-quality state, demonstrating correct real-time processing and alarm logic.

5. Have you achieved some or all of your Hardware Requirements Specification (HRS)?

We have achieved all major hardware requirements. The ATmega328PB successfully interfaces with the SGP30 and DHT20 sensors through the I²C bus, retrieving stable measurements every second. The ST7735 LCD is fully integrated through the SPI interface and reliably displays all sensor values and system status. The RGB LED and active buzzer provide clear visual and audible feedback.

   1. Show how you collected data and the outcomes.

   We verify hardware operation by continuously displaying sensor values on the LCD while observing UART logs. Changes in temperature, humidity, and VOC concentration appear instantly on the screen, confirming stable I²C communication and correct data acquisition. When air-quality thresholds are exceeded, the system immediately activates the corresponding LED color and audible buzzer alert.

6. Show off the remaining elements that will make your project whole: mechanical casework, supporting graphical user interface (GUI), web portal, etc.

For the remaining elements, we plan to build a mechanical enclosure to protect the electronics and improve the usability and portability of the device. We are also adding Wi-Fi connectivity so the system can upload air-quality data to a web portal for remote monitoring and historical data analysis.

7. What is the riskiest part remaining of your project?

The most challenging and high-risk component of our project is the newly added ESP32 Wi-Fi module. Programming the ESP32 through the Arduino IDE is highly unstable; the board sometimes enters download mode and flashes successfully, but other times it disconnects randomly during uploading. We have tested multiple ESP32 units, different USB cables, and different computers, and the problem still persists, making reliable firmware flashing unpredictable. In addition, the data-transfer logic between the ATmega328PB and the ESP32 introduces further complexity. Since the primary UART interface on the ATmega328PB is occupied by the USB debugging connection, we must implement a new custom UART protocol to send four sensor values to the ESP32. This requires extra parsing logic and synchronization on both sides, increasing development risk and integration difficulty.

   1. How do you plan to de-risk this?

   We plan to de-risk the ESP32 integration by first trying different flashing configurations and upload parameters in the Arduino IDE to eliminate the programming instability during firmware uploads. In addition, we implement a simple custom UART protocol to send the four sensor values from the ATmega328PB to the ESP32, ensuring reliable and synchronized data transfer between the two devices.

8. What questions or help do you need from the teaching team?

Most core functions of our system are already implemented. For the remaining work, we would appreciate guidance on integrating the Wi-Fi module and sending sensor data to the cloud. We also hope to receive feedback on our final demo, including suggestions for how to best organize the display information and physical packaging.

## Final Project Report

Don't forget to make the GitHub pages public website!
If you’ve never made a GitHub pages website before, you can follow this webpage (though, substitute your final project repository for the GitHub username one in the quickstart guide):  [https://docs.github.com/en/pages/quickstart](https://docs.github.com/en/pages/quickstart)

### 1. Video

[Final Project Video](https://drive.google.com/file/d/1B1SKHOs2A9I9UX17AUFNReegv7i-G88d/view?usp=drive_link)

* The video must demonstrate your key functionality.
* The video must be 5 minutes or less.
* Ensure your video link is accessible to the teaching team. Unlisted YouTube videos or Google Drive uploads with SEAS account access work well.
* Points will be removed if the audio quality is poor - say, if you filmed your video in a noisy electrical engineering lab.

### 2. Images

![exterior1](image\README\exterior1.png)

![exterior2](image\README\exterior2.png)

![exterior3](image\README\exterior3.png)

![exterior4](image\README\exterior4.png)

![interior1](image\README\interior1.png)

![interior2](image\README\interior2.png)

*Include photos of your device from a few angles. If you have a casework, show both the exterior and interior (where the good EE bits are!).*

### 3. Results

*What were your results? Namely, what was the final solution/design to your problem?*

The final result of our project is a fully working Indoor Air Quality Monitor that measures CO₂, TVOC, temperature, and humidity in real time. The ATmega328PB reads sensor data every second, shows the values on the LCD screen, and gives fast feedback through an RGB LED and buzzer when air quality becomes poor. We also added UART communication for debugging and Wi-Fi support through an ESP32 to upload sensor data to the cloud. The system responds quickly to environmental changes, is stable during long-term operation, and provides clear visual and sound alerts to help users understand the air quality around them.

#### 3.1 Software Requirements Specification (SRS) Results

*Based on your quantified system performance, comment on how you achieved or fell short of your expected requirements.*

*Did your requirements change? If so, why? Failing to meet a requirement is acceptable; understanding the reason why is critical!*

*Validate at least two requirements, showing how you tested and your proof of work (videos, images, logic analyzer/oscilloscope captures, etc.).*

| ID     | Description                                                                                               | Validation Outcome                                                                          |
| ------ | --------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| SRS-01 | The IMU 3-axis acceleration will be measured with 16-bit depth every 100 milliseconds +/-10 milliseconds. | Confirmed, logged output from the MCU is saved to "validation" folder in GitHub repository. |

#### 3.2 Hardware Requirements Specification (HRS) Results

*Based on your quantified system performance, comment on how you achieved or fell short of your expected requirements.*

*Did your requirements change? If so, why? Failing to meet a requirement is acceptable; understanding the reason why is critical!*

*Validate at least two requirements, showing how you tested and your proof of work (videos, images, logic analyzer/oscilloscope captures, etc.).*

| ID     | Description                                                                                                                        | Validation Outcome                                                                                                      |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| HRS-01 | A distance sensor shall be used for obstacle detection. The sensor shall detect obstacles at a maximum distance of at least 10 cm. | Confirmed, sensed obstacles up to 15cm. Video in "validation" folder, shows tape measure and logged output to terminal. |
|        |                                                                                                                                    |                                                                                                                         |

### 4. Conclusion

Reflect on your project. Some questions to address:

* What did you learn from it?
* What went well?
* What accomplishments are you proud of?
* What did you learn/gain from this experience?
* Did you have to change your approach?
* What could have been done differently?
* Did you encounter obstacles that you didn’t anticipate?
* What could be a next step for this project?

This project taught us how to design and build a full embedded system that connects sensors, a display, alarms, and cloud communication. We are proud that the system can reliably read CO₂, TVOC, temperature, and humidity and show clear feedback through the LCD, LED, and buzzer. One challenge we faced was the ESP32, which often failed to connect or stay stable during Wi-Fi setup, making it difficult to upload code and slowing our progress. We also experienced issues with the LCD wiring, where slight movement caused the screen to show random lines or distorted colors, leading us to rebuild the connections for better stability. These problems taught us the importance of good hardware connections and careful planning when adding wireless modules.

## References

Fill in your references here as you work on your final project. Describe any libraries used here.

