# **ATmega324PA Temperature Monitoring System Using ADC and UART**

## **Project Overview**
This project demonstrates how to build a **temperature monitoring system** using an **ATmega324PA** microcontroller, **LM35 temperature sensor**, **UART communication**, and a **16x2 LCD display**. The system reads the analog temperature data via **ADC**, processes it into a Celsius value, and displays the result both on an **LCD** and a **PC terminal** via **UART**. The architecture ensures real-time updates using **interrupts** and **double buffering** for efficient data handling.

## **Project Diagram**
![alt text](diagram.png)

## **Features**
- **LM35 Analog Temperature Sensor**: Converts temperature into analog voltage (10mV/°C).
- **ADC (Analog-to-Digital Converter)**: Samples the analog voltage and converts it into digital temperature data.
- **LCD Display**: Shows the real-time temperature in Celsius.
- **UART Communication**: Sends temperature data to PC terminal tools (e.g., PuTTY).
- **Double Buffering**: Prevents data loss during ADC read/processing.
- **Interrupt-Driven**: ADC and UART operations are triggered using interrupts for responsive data handling.

## **Technology**
- **Microcontroller**: **ATmega324PA**
- **Programming Language**: **C**
- **Components**:
  - **LM35** connected to **ADC0 (PA0)**
  - **16x2 LCD** in **4-bit mode** connected to **PORTC**
  - **UART** via **PD0 (RX) and PD1 (TX)**
  - **USB-to-Serial module (CP2102)** for PC interface

## **How It Works**
1. The **LM35 sensor** outputs an analog voltage proportional to the surrounding temperature.
2. The **ADC** periodically samples the analog signal and converts it into a digital value.
3. Using a conversion formula, the digital value is processed to obtain the temperature in Celsius.
4. The result is:
   - Displayed on a **16x2 LCD**
   - Transmitted via **UART** to a connected PC terminal
5. The system uses **double buffering** to store ADC values and prevent race conditions.
6. **Interrupts** are used for both ADC and UART to ensure real-time responsiveness.

## **Files in the Project**
- **main.c**: Main application logic including initialization, temperature calculation, and display logic.
- **adc.c / adc.h**: Handles ADC configuration, ISR, and buffering.
- **lcd.c / lcd.h**: LCD control and display routines.
- **uart.c / uart.h**: UART communication setup and send functions.
- **timer.c / timer.h** *(optional)*: Timer setup for periodic ADC trigger.

## **Setup & Usage**
1. **Hardware**:
   - Connect the **LM35** sensor to **ADC0 (PA0)**.
   - Interface the **16x2 LCD** in 4-bit mode using **PORTC**.
   - Connect **CP2102 module** for UART via **PD0/PD1** to the PC.
2. **Software**:
   - Compile and upload the project code to the **ATmega324PA**.
   - Open a terminal tool on PC (e.g., PuTTY) at **9600 baud**.
   - Observe real-time temperature updates on both the **LCD** and terminal.

## **How to Run the Code**
1. Wire up the components on a breadboard or simulate using **Proteus**.
2. Upload the compiled hex file to the **ATmega324PA**.
3. Open the terminal tool and power the system.
4. Temperature will start displaying and updating every second.

## **Responsibilities**  
- Designed and implemented a complete **temperature monitoring system** using **ADC, UART, and LCD** on **ATmega324PA**.  
- Developed **interrupt-driven routines** for ADC sampling and UART transmission.  
- Used **double buffering** to ensure consistent data flow without delays.  
- Tested and validated the system using **Proteus simulation** and live debugging via terminal.

## **Conclusion**
This project demonstrates a real-world embedded system integrating **sensors**, **digital communication**, and **user interface**. It reflects robust embedded software practices like **interrupt-driven design**, **buffer management**, and **real-time data processing**, all built on a compact and efficient **ATmega324PA** platform.
