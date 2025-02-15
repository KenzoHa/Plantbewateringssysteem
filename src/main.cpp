/* BRONNEN:
https://canvas.kdg.be/courses/49816/pages/bodemvochtigheidssensoren --> voorbeeld 'value-mapping' bodemvochtigheidssensoren
https://wiki.dfrobot.com/Waterproof_DS18B20_Digital_Temperature_Sensor__SKU_DFR0198_ --> voorbeeld getTemp()-functie voor DS18B20
https://claude.ai --> AI-tool
https://chatgpt.com --> AI-tool
*/

// LIBRARIES:
#include <Arduino.h>
#include <config.h>                     //Pin-definities, Variabelen en Constanten (grenswaarden, timing, statussen, etc.)
#include <OneWire.h>                    //Nodig voor temp.sensor DS18B20
#include <DallasTemperature.h>          //Nodig voor temp.sensor DS18B20

// FUNCTIONS:
void getBodemvochtValue() {// Leest de analoge waarden van beide bodemvochtigheidssensoren en slaat deze op in globale variabelen
  resSensorValue = analogRead(RES_SOILSENSOR);
  capSensorValue = analogRead(CAP_SOILSENSOR);
}

int berekenBodemvochtPercentage(int sensorValue, int sensorMinValue, int sensorMaxValue) {// Zet de ruwe sensorwaarde om naar een percentage (0-100%) op basis van de min/max grenzen
  // 'map'' sensor value naar percentage
  return map(sensorValue, sensorMinValue, sensorMaxValue, 0, 100);
}

int bepaalBodemvochtStatus(int sensorValue, bool isResistief) {// Bepaalt de status (DROOG/VOCHTIG/NAT) van een bodemvochtigheidssensor op basis van de gemeten waarde
  if (isResistief) {
      // Voor resistieve sensor
      if (sensorValue >= RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN && sensorValue <= RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX) {
          return STATUS_DROOG;
      }
      else if (sensorValue >= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && sensorValue <= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX) {
          return STATUS_VOCHTIG;
      }
      else if (sensorValue >= RESISTIEVE_SENSOR_NAT_INTERVAL_MIN && sensorValue <= RESISTIEVE_SENSOR_NAT_INTERVAL_MAX) {
          return STATUS_NAT;
      }
      else {
          return STATUS_ERROR;
      }
  } 
  else {
      // Voor capacitieve sensor
      if (sensorValue >= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN && sensorValue <= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX) {
          return STATUS_DROOG;
      }
      else if (sensorValue >= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && sensorValue <= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX) {
          return STATUS_VOCHTIG;
      }
      else if (sensorValue >= CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN && sensorValue <= CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX) {
          return STATUS_NAT;
      }
      else {
          return STATUS_ERROR;
      }
  }
}

void printBodemvochtData() {// Print de ruwe waarden, percentages en bodemvochtstatus van beide bodemvochtigheidssensoren naar de Serial Monitor
  int resMoisturePercentage = berekenBodemvochtPercentage(resSensorValue, RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN, RESISTIEVE_SENSOR_NAT_INTERVAL_MAX);
  int capMoisturePercentage = berekenBodemvochtPercentage(capSensorValue,CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN, CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX);
  
  // Print resistieve sensor data
  Serial.print("Resistance type soil sensor value: ");
  Serial.print(resSensorValue);
  Serial.print(" - Soil moisture percentage: ");
  Serial.print(resMoisturePercentage);
  Serial.print("% - Status: ");
  
  if (resBodemStatus == STATUS_DROOG){
    Serial.println("DROOG");
  }
  else if (resBodemStatus == STATUS_VOCHTIG){
    Serial.println("VOCHTIG");
  }
  else if (resBodemStatus == STATUS_NAT){
    Serial.println("NAT");
  }
  else{
    Serial.println("ERROR");
  }
  
  // Print capacitieve sensor data
  Serial.print("Capacitance type soil sensor value: ");
  Serial.print(capSensorValue);
  Serial.print(" - Soil moisture percentage: ");
  Serial.print(capMoisturePercentage);
  Serial.print("% - Status: ");
  
  if (capBodemStatus == STATUS_DROOG){
    Serial.println("DROOG");
  }
  else if (capBodemStatus == STATUS_VOCHTIG){
    Serial.println("VOCHTIG");
  }
  else if (capBodemStatus == STATUS_NAT){
    Serial.println("NAT");
  }
  else{
    Serial.println("ERROR");
  }
  
  Serial.println("----------------");
}

float getTemp() {// Leest de temperatuur van de DS18B20 sensor en geeft deze terug in graden Celsius
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

void printTempValue() {// Print de gemeten temperatuur naar de Serial Monitor
  float temperature = getTemp();
    Serial.print("De temperatuur is: ");
    Serial.print(temperature);
    Serial.println(" °C");
  }

void controleerBewatering() {// Controleert of automatische bewatering nodig is op basis van temperatuur en bodemvochtigheid
  unsigned long hudigeTijd = millis();
  float temperatuur = getTemp();
  
  // Eerst checken of de temperatuur niet te laag is (prioriteit)
  if (temperatuur <= TEMP_MIN) {
    Serial.println("Bewatering geblokkeerd - Temperatuur te laag! (< 5°C buiten)");
    digitalWrite(RELAY_PIN, LOW);  // Zeker zijn dat bewatering uit staat
    return;  // Stop de functie hier
  }
  
  // Check of de grond te droog is OF de temperatuur te hoog is
  if (resBodemStatus == STATUS_DROOG || capBodemStatus == STATUS_DROOG || temperatuur > TEMP_MAX) {
    // Check of we lang genoeg gewacht hebben sinds laatste bewatering
    if (bewateringKanAan && (hudigeTijd - laatsteBewateringTijd >= BEWATERING_INTERVAL)) {
      // Start een nieuwe bewatering
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Bewatering AAN - Reden:");
      
      // Print de reden(en) voor bewatering
      if (resBodemStatus == STATUS_DROOG || capBodemStatus == STATUS_DROOG) {
        Serial.println("- Grond is te droog");
      }
      if (temperatuur > TEMP_MAX) {
        Serial.println("- Temperatuur is te hoog");
      }
      
      bewateringKanAan = false;
      laatsteBewateringTijd = hudigeTijd;
    }
  }
  
  // Check of huidige bewatering klaar is
  if (!bewateringKanAan && (hudigeTijd - laatsteBewateringTijd >= BEWATERING_DUUR)) {
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("Bewatering UIT - Wachten voor volgende beurt");
    bewateringKanAan = true;
  }
}

void controleerDrukknop() {// Controleert of de panic button is ingedrukt en voert een vaste bewatering uit (3 seconden)
  unsigned long hudigeTijd = millis();
  
  // Als de knop wordt ingedrukt en er nog geen bewatering bezig is
  if (digitalRead(BUTTON_PIN) == HIGH && !knopIngedrukt) {
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println("Bewatering AAN - Handmatig via drukknop");
    knopIngedrukt = true;
    knopBewateringStart = hudigeTijd;
  }
  
  // Als de bewatering via knop bezig is, controleer of tijd verstreken is
  if (knopIngedrukt && (hudigeTijd - knopBewateringStart >= KNOP_BEWATERING_DUUR)) {
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("Bewatering UIT - Handmatige bewatering klaar");
    knopIngedrukt = false;
  }
}

void setup() {// Initialiseert de pinnen en Serial communicatie
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Relais start uitgeschakeld (veiliger)
  pinMode(BUTTON_PIN, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Controleer eerst de drukknop (heeft prioriteit)
  controleerDrukknop();
  
  // Alleen automatische bewatering als er geen handmatige bewatering bezig is
  if (!knopIngedrukt) {
      controleerBewatering();
  }
  
  // Update sensors elke 2 seconden
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
      // Lees sensordata
      getBodemvochtValue();
      
      // Bereken statussen
      resBodemStatus = bepaalBodemvochtStatus(resSensorValue, true);
      capBodemStatus = bepaalBodemvochtStatus(capSensorValue, false);
      
      // Print alle data
      printBodemvochtData();
      printTempValue();
      
      lastSensorRead = currentTime;
  }
}
