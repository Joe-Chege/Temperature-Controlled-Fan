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
