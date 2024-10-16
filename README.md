# **Temperature-Controlled Fan System Using ESP32-S2-Solo-2**

This project demonstrates how to build a **temperature-controlled fan system** using the **ESP32-S2-Solo-2 microcontroller**, a **DHT22 temperature sensor**, and a **status LED**. The fan automatically turns ON when the temperature exceeds a threshold, with the status LED providing a visual indication. 

---

## **Table of Contents**
1. [Overview](#overview)  
2. [Hardware Components](#hardware-components)  
3. [Circuit Diagram and Wiring](#circuit-diagram-and-wiring)  
4. [Software Setup](#software-setup)  
5. [Code Explanation](#code-explanation)  
6. [How It Works](#how-it-works)  
7. [Usage](#usage)  
8. [Troubleshooting](#troubleshooting)  
9. [Conclusion](#conclusion)  

---

## **Overview**
This system reads the ambient temperature using the **DHT22 temperature sensor**. When the temperature exceeds **26°C**, the fan and the status LED are turned on. When the temperature falls below the threshold, both the fan and LED are turned off.

---

## **Hardware Components**
1. **ESP32-S2-Solo-2 Microcontroller**  
2. **DHT22 Temperature and Humidity Sensor**  
3. **5V DC Fan**  
4. **Status LED (optional)**  
5. **220Ω Resistor** (for LED)  
6. **Breadboard and Jumper Wires**  
7. **Power Source** (Battery or USB cable)  

---

## **Circuit Diagram and Wiring**

| **Component**   | **ESP32 Pin** | **Notes** |
|-----------------|---------------|-----------|
| DHT22 Sensor    | GPIO 34       | Sensor reads the ambient temperature. Connect the **VCC** to 3.3V, **GND** to GND, and **DATA** to GPIO 34. |
| Fan             | GPIO 21       | Control pin for the fan. Connect one side to GPIO 21 and the other to ground. Use a transistor or relay if needed for higher currents. |
| Status LED      | GPIO 16       | Indicates whether the fan is active. Use a 220Ω resistor in series with the LED. |

---

## **Software Setup**

1. **Arduino IDE Installation**  
   - Download and install the **Arduino IDE** from [here](https://www.arduino.cc/en/software).

2. **Install ESP32 Board Support**  
   - Go to **File > Preferences** in the Arduino IDE.  
   - Add the following URL to the "Additional Board Manager URLs":  
     `https://dl.espressif.com/dl/package_esp32_index.json`  
   - Go to **Tools > Board > Board Manager**, search for "ESP32", and install the board package.

3. **Install DHT Library**  
   - Go to **Sketch > Include Library > Manage Libraries**.  
   - Search for **DHT sensor library** and install the one by **Adafruit**.

4. **Select ESP32 Board**  
   - Go to **Tools > Board** and select **ESP32-S2 Dev Module**.  
   - Select the appropriate **COM port** from **Tools > Port**.

---

## **Code Explanation**

The following code initializes the fan control system and the DHT22 temperature sensor. It reads the temperature every 2 seconds and toggles the fan and LED based on the threshold (26°C).

### **Code**

```cpp
#include <DHT.h>  // Include DHT library for temperature/humidity sensor

#define DHTPIN 34       // Pin where the DHT sensor is connected
#define DHTTYPE DHT22   // Specify the type of DHT sensor (DHT11 or DHT22)

DHT dht(DHTPIN, DHTTYPE);  // Create a DHT object

const int fanPin = 21;         // Fan connected to GPIO 21
const int statusLedPin = 16;   // Status LED connected to GPIO 16
float temperature;             // Variable to hold temperature value

void setup() {
  Serial.begin(115200);  // Initialize serial communication
  
  // Set initial pin modes
  pinMode(fanPin, OUTPUT);
  pinMode(statusLedPin, OUTPUT);

  // Ensure the fan and LED start OFF
  digitalWrite(fanPin, LOW);
  digitalWrite(statusLedPin, LOW);

  // Initialize the DHT sensor
  dht.begin();  

  Serial.println("System Initialized. Waiting for sensor data...");
}

void loop() {
  // Read temperature from the DHT sensor
  temperature = dht.readTemperature();  // Read temperature as Celsius

  // Check if reading was successful
  if (isnan(temperature)) {
    Serial.println("Failed to read temperature! Retrying...");
    delay(2000);  // Wait before retrying
    return;  // Exit the loop if reading fails
  }

  // Print temperature to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  // Fan control logic
  if (temperature > 26.0) {  // Threshold for fan activation
    if (digitalRead(fanPin) == HIGH) {  // Check if fan is ON
      digitalWrite(fanPin, LOW);      // Turn ON the fan
      digitalWrite(statusLedPin, HIGH);  // Turn ON the status LED
      Serial.println("Fan ON. Status LED ON.");
    }
  } else {
    if (digitalRead(fanPin) == LOW) {  // Check if fan is OFF
      digitalWrite(fanPin, HIGH);       // Turn OFF the fan
      digitalWrite(statusLedPin, LOW);  // Turn OFF the status LED
      Serial.println("Fan OFF. Status LED OFF.");
    }
  }

  // Wait a short period before the next reading
  delay(2000);  // Read every 2 seconds
}
