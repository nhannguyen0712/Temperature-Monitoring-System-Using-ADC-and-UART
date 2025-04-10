# **ATmega324PA Temperature Monitoring System with RTC and UART Alert**

## **Project Overview**
This project builds a **temperature monitoring system** using an **ATmega324PA** microcontroller, **LM35 analog temperature sensor**, **DS3231 real-time clock**, and a **16x2 LCD display**. The system reads analog temperature data via **ADC**, timestamps it with the RTC, and displays the information on an LCD. If the temperature exceeds a set threshold, it sends a warning through **UART** to a PC terminal.

## **Project Diagram**
![alt text](diagram.png)

## **Features**
- **LM35 Temperature Sensor**: Outputs analog voltage (10mV/°C) proportional to ambient temperature.
- **DS3231 RTC Module**: Provides accurate real-time clock over I2C.
- **LCD Display**: Shows current temperature, time, and date.
- **Temperature Alert System**: Sends a **UART alert** if the temperature exceeds a defined threshold.
- **UART Logging**: Communicates alert events via FT232R USB-to-Serial module.

## **Technology**
- **Microcontroller**: **ATmega324PA**
- **Programming Language**: **C**
- **Components**:
  - **LM35** connected to **ADC0 (PA0)**
  - **DS3231 RTC** connected via **I2C (TWI on PD0/PD1)**
  - **16x2 LCD** in 4-bit mode (connected to **PORTB/PORTD**)
  - **UART** via **PD0 (RX) and PD1 (TX)**
  - **USB-to-Serial module (FT232R)** for PC communication

## **How It Works**
1. The **LM35** outputs an analog voltage based on temperature.
2. The **ADC** digitizes this voltage to compute temperature in Celsius.
3. The **DS3231 RTC** provides current date and time via I2C.
4. Every second:
   - The **LCD** displays the temperature, date, and time.
   - If temperature exceeds a preset threshold (e.g. 35°C), a **UART warning** is sent to the PC.
5. The alert is only triggered once per over-threshold event to avoid spamming.

## **Files in the Project**
- **main.c**: Complete implementation of the monitoring system including:
  - LCD driver functions
  - I2C-based RTC read
  - ADC-based temperature read
  - UART alert messaging

## **Setup & Usage**
1. **Hardware Connections**:
   - **LM35** → **ADC0 (PA0)**
   - **RTC Module (DS3231)** → **SCL/SDA (PD0/PD1)**
   - **16x2 LCD** → **LCD_RS/LCD_EN (PB0/PB1), LCD_D4–D7 (PD4–PD7)**
   - **FT232R USB-to-Serial** → **PD0 (RX), PD1 (TX)**
2. **Software**:
   - Compile and upload the `main.c` to the **ATmega324PA** using **AVR-GCC** or **Atmel Studio**.
   - Open a serial monitor (e.g., PuTTY) at **9600 baud**.
   - Monitor live temperature updates and UART alerts on high temperature events.

## **How to Run the Code**
1. Assemble the hardware on a breadboard or simulate using **Proteus**.
2. Flash the compiled firmware to the **ATmega324PA**.
3. Power the system and open a terminal window.
4. Watch temperature, date, and time on the LCD and get alerts in the terminal if temperature exceeds the threshold.

## **Responsibilities**
- Developed and tested a complete embedded solution for real-time temperature monitoring.
- Integrated **ADC, I2C, UART**, and **LCD** in a cohesive system.
- Built a minimal but robust **UART alert mechanism** for thermal events.
- Verified performance on **hardware and simulation** using **Proteus** and **serial monitor tools**.

## **Conclusion**
This project showcases the integration of sensor acquisition, digital communication, and user display in embedded systems. By combining an **RTC**, **temperature sensor**, and **alert system**, it provides a scalable base for data logging, environmental control, or safety applications using **ATmega324PA**.
