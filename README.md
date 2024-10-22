# BQ34Z100 Library for Arduino

A comprehensive Arduino library for interfacing with the Texas Instruments BQ34Z100-G1 battery fuel gauge over I2C. This library provides easy access to key battery metrics such as state of charge (SOC), state of health (SOH), current, and more, enabling efficient monitoring and management of battery performance.

## Features

- Read battery State of Charge (SOC) and State of Health (SOH)
- Monitor instantaneous voltage, current draw, and temperature
- Perform calibration routines for precise measurements
- Control and configure battery parameters via data flash
- Supports both internal and external temperature sensors

## Installation

1. **Download the Library**: Clone or download the repository as a ZIP file.
2. **Include the Library**: In the Arduino IDE, go to `Sketch` > `Include Library` > `Add .ZIP Library` and select the downloaded ZIP file.

## Example Usage

Here's a basic example to get started with the BQ34Z100 library:

```cpp
#include "BQ34Z100.h"

BQ34Z100 *batteryManager;

void setup() {
  Serial.begin(115200);
  
  Wire.begin(21, 22); // Initialize I2C with SDA on pin 21 and SCL on pin 22
  batteryManager = new BQ34Z100(21, 22);
}

void loop() {
  int stateOfCharge = batteryManager->getSOC();
  int stateOfHealth = batteryManager->getStateOfHealth();
  int loadCurrent = batteryManager->getCurrent();

  Serial.printf("Charge Level: %d%%\n", stateOfCharge);
  Serial.printf("Battery Health: %d%%\n", stateOfHealth);
  Serial.printf("Load Current: %d mA\n", loadCurrent);

  delay(1000); // Wait for 1 second before the next reading
}
```

## Functions Overview

- **`getSOC()`**: Returns the State of Charge (SOC) of the battery as a percentage (0-100%).
- **`getStateOfHealth()`**: Returns the State of Health (SOH) of the battery as a percentage (0-100%), which indicates the remaining usable capacity.
- **`getCurrent()`**: Reads the current draw in mA.
- **`getVoltage()`**: Reads the battery voltage in mV.
- **`getTemperature()`**: Reads the temperature in degrees Celsius.

## Detailed Description

The BQ34Z100-G1 is a highly accurate battery fuel gauge that can monitor and report key metrics for battery packs. It uses TI's patented Impedance Track™ technology to provide precise measurements of remaining battery capacity. This library simplifies communication with the BQ34Z100-G1 over I2C, offering a straightforward API to retrieve battery status, perform calibration, and configure the fuel gauge.

### Supported Battery Metrics

- **State of Charge (SOC)**: The remaining battery percentage.
- **State of Health (SOH)**: The battery's remaining capacity compared to its original capacity.
- **Current Draw**: Real-time current draw from the battery (useful for tracking load).
- **Voltage**: Battery voltage measurement.
- **Temperature**: Internal or external temperature measurement for thermal management.

## Calibration and Configuration

The library includes methods for entering calibration mode, setting battery parameters, and adjusting calibration values such as the voltage divider ratio and sense resistor value. To ensure accuracy, it's recommended to perform these routines when deploying the BQ34Z100-G1 in a new environment or with a different battery configuration.

For detailed calibration instructions, refer to the [TI BQ34Z100-G1 datasheet](http://www.ti.com/lit/ds/symlink/bq34z100-g1.pdf).

### Example Calibration

```cpp
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  batteryManager = new BQ34Z100(21, 22);

  // Enable calibration mode
  batteryManager->enableCal();
  batteryManager->enterCal();

  // Set the sense resistor value for current measurements
  batteryManager->setSenseResistor();

  // Exit calibration mode
  batteryManager->exitCal();
}
```

## Hardware Setup

### I2C Pins

- **Arduino Uno**:
  - **SDA**: A4
  - **SCL**: A5
- **ESP32** (example above):
  - **SDA**: GPIO 21
  - **SCL**: GPIO 22

Connect the BQ34Z100-G1 sensor's I2C pins to the appropriate Arduino I2C pins as listed above. Ensure proper power connections and battery connection to the BQ34Z100-G1.

### Wiring Diagram

- Connect the **SDA** pin of the BQ34Z100-G1 to the **SDA** pin of the Arduino.
- Connect the **SCL** pin of the BQ34Z100-G1 to the **SCL** pin of the Arduino.
- Connect the **VCC** pin of the BQ34Z100-G1 to a 3.3V or 5V power source (depending on the voltage requirements of your module).
- Connect the **GND** pin of the BQ34Z100-G1 to the GND of the Arduino.

## Note

This library is designed for easy integration into battery-powered Arduino projects that require accurate monitoring of battery charge, health, and usage.

### Calibration Tips

For accurate readings, it is essential to calibrate the sensor to match your specific battery and circuit. Refer to the [BQ34Z100-G1 datasheet](http://www.ti.com/lit/ds/symlink/bq34z100-g1.pdf) for detailed instructions on calibration and configuration.

Ensure the following before performing calibration:

- The battery is fully charged.
- The sense resistor value is correctly set in the library.
- Voltage and current measurements match those of an external calibrated meter.

Proper calibration helps improve the accuracy of SOC (State of Charge) and SOH (State of Health) readings.

