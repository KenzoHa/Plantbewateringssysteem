/* SOURCES:
https://canvas.kdg.be/courses/49816/pages/bodemvochtigheidssensoren --> voorbeeld 'value-mapping' soil sensors
https://claude.ai --> AI-tool
https://chatgpt.com --> AI-tool
*/

// LIBRARIES:
#include <Arduino.h>
#include <config.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// GLOBAL VARIABLES: 
int resValue;
int capValue;

OneWire ds(DS18S20_Pin);

// FUNCTIONS:
void readSoilSensors() {
  // Read both sensors
  resValue = analogRead(RES_SOILSENSOR);
  capValue = analogRead(CAP_SOILSENSOR);
}

int calculateMoisturePercentage(int sensorValue, int sensorMinValue, int sensorMaxValue) { //return percentage for soil sensors (based on analog value)
  // Convert sensor value to percentage
  return map(sensorValue, sensorMinValue, sensorMaxValue, 0, 100);
}

void printSoilData() { //show raw-value and percentage in SerialMonitor
  // Calculate percentages for both sensors
  int resMoisturePercentage = calculateMoisturePercentage(resValue, RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN, RESISTIEVE_SENSOR_NAT_INTERVAL_MAX);
  int capMoisturePercentage = calculateMoisturePercentage(capValue,CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN, CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX);
  
  // Print all sensor data
  Serial.print("Resistance type soil sensor value: ");
  Serial.print(resValue);
  Serial.print(" - Soil moisture percentage: ");
  Serial.print(resMoisturePercentage);
  Serial.println("%");
  
  Serial.print("Capacitance type soil sensor value: ");
  Serial.print(capValue);
  Serial.print(" - Soil moisture percentage: ");
  Serial.print(capMoisturePercentage);
  Serial.println("%");
  Serial.println("----------------");
}

float getTemp() {  //returns the temperature from one DS18S20 in °Celsius
  byte data[12];
  byte addr[8];

  if (!ds.search(addr)) {
      //no more sensors on chain, reset search
      Serial.println("no more sensors on chain, reset search!");
      ds.reset_search();
      return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}

void printTempData() {
  float temperature = getTemp();
    Serial.print("De temperatuur is: ");
    Serial.print(temperature);
    Serial.println(" °C");
  }

void setup() {
  Serial.begin(115200);
}

void loop() {
  readSoilSensors();
  printSoilData();
  delay(1000);

  getTemp();
  printTempData();
  delay(1000);
}
