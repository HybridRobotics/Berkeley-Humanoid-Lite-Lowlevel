#include "HX711.h"
#include "Adafruit_INA3221.h"
#include <Wire.h>

#define LEFT_LOADCELL_SCK_PIN   4
#define LEFT_LOADCELL_DOUT_PIN  5
#define RIGHT_LOADCELL_SCK_PIN   6
#define RIGHT_LOADCELL_DOUT_PIN  7


/*
  Circuit Connection

  | Device      | Device Pin | Arduino Pin |
  | ----------- | ---------- | ----------- |
  | Left HX711  | VCC        | 3V3         |
  |             | GND        | GND         |
  |             | SCK        | D4          |
  |             | DOUT       | D5          |
  | Right HX711 | VCC        | 3V3         |
  |             | GND        | GND         |
  |             | SCK        | D6          |
  |             | DOUT       | D7          |
  | INA3221     | VCC        | 3V3         |
  |             | GND        | GND         |
  |             | SCK        | I2C SDA     |
  |             | SDA        | I2C SCL     |
 */



float left_calibration_factor = 26600 * 0.2;
float right_calibration_factor = 25950 * 0.2;

float filter_alpha = 0.05;


HX711 left_scale;
HX711 right_scale;

Adafruit_INA3221 ina3221;

float voltage_average = 0.;
float current_average = 0.;
float torque_average = 0.;

uint8_t channel = 0;


void calibrate() {
  left_scale.set_scale(1);
  right_scale.set_scale(1);

  left_scale.tare(100);
  right_scale.tare(100);

  float left_reading_average = 0;
  float right_reading_average = 0;
  float alpha = 0.005;
  
  while (1) {
    left_reading_average = alpha * left_scale.get_units() + (1 - alpha) * left_reading_average;
    right_reading_average = alpha * right_scale.get_units() + (1 - alpha) * right_reading_average;

    
    Serial.print("L:");
    Serial.print(left_reading_average, 2); //scale.get_units() returns a float
    Serial.print(",R:");
    Serial.print(right_reading_average, 2); //scale.get_units() returns a float
    Serial.println();
  }
}


void setup() {
  Serial.begin(115200);

  if (!ina3221.begin(0x40, &Wire)) { // can use other I2C addresses or buses
    Serial.println("Failed to find INA3221 chip");
    while (1) {

    }
  }

  ina3221.setAveragingMode(INA3221_AVG_16_SAMPLES);
  ina3221.setShuntResistance(0, 0.003);
  ina3221.setShuntResistance(1, 0.1);
  ina3221.setShuntResistance(2, 0.1);



  left_scale.begin(LEFT_LOADCELL_DOUT_PIN, LEFT_LOADCELL_SCK_PIN, 64);
  right_scale.begin(RIGHT_LOADCELL_DOUT_PIN, RIGHT_LOADCELL_SCK_PIN, 64);

  left_scale.set_scale(left_calibration_factor);
  right_scale.set_scale(right_calibration_factor);

  left_scale.tare(100);
  right_scale.tare(100);

  // calibrate();
}

void loop() {

  float voltage = ina3221.getBusVoltage(channel);
  float current = ina3221.getCurrentAmps(channel);
  
  voltage_average = filter_alpha * voltage + (1 - filter_alpha) * voltage_average;
  current_average = filter_alpha * current + (1 - filter_alpha) * current_average;

  float force_left = left_scale.get_units();
  float force_right = right_scale.get_units();
  float distance = 0.050;
  float torque = -(force_left + force_right) * distance;

  torque_average = filter_alpha * torque + (1 - filter_alpha) * torque_average;

  Serial.print("{\"V\":");
  Serial.print(voltage_average, 6);
  Serial.print(",\"I\":");
  Serial.print(current_average, 6);
  Serial.print(",\"T\":");
  Serial.print(torque_average, 6);
  Serial.println("}");
}