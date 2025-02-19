/* BRONNEN:
https://canvas.kdg.be/courses/49816/pages/basisstructuur-code-en-configuratie-van-het-plantbewateringssysteem --> basisstructuur code
https://canvas.kdg.be/courses/49816/pages/bodemvochtigheidssensoren --> voorbeeld 'value-mapping' bodemvochtigheidssensoren
https://wiki.dfrobot.com/Waterproof_DS18B20_Digital_Temperature_Sensor__SKU_DFR0198_ --> voorbeeld getTemp()-functie voor DS18B20
https://www.perplexity.ai/ --> AI-tool
https://claude.ai --> AI-tool
*/

// Libraries:
#include <Arduino.h>
#include <config.h>            //Pin-definities en CONSTANTEN (grenswaarden, timing, statussen, etc.)
#include <OneWire.h>           //Nodig voor temp.sensor DS18B20
#include <DallasTemperature.h> //Nodig voor temp.sensor DS18B20

// (Globale) variabelen:
unsigned long LaatsteSensorLezing = 0;
unsigned long bewateringStartTijd = 0;
unsigned long knopStartTijd = 0;

int huidigeBewatering = BEWATERING_UIT;
int bewateringType = 0;
bool knopIngedrukt = false;

OneWire ds(DS18S20_Pin);

int resSensorValue;
int capSensorValue;
int resBodemStatus;
int capBodemStatus;

// Functies:
void getBodemvochtValue()// Leest de analoge waarden van beide bodemvochtigheidssensoren en slaat deze op in globale variabelen
{ 
  resSensorValue = analogRead(RES_SOILSENSOR);
  capSensorValue = analogRead(CAP_SOILSENSOR);
}

int berekenBodemvochtPercentage(int sensorValue, int sensorMinValue, int sensorMaxValue)// Zet de ruwe sensorwaarde om naar een percentage (0-100%) op basis van de min/max grenzen
{ 
  return map(sensorValue, sensorMinValue, sensorMaxValue, 0, 100);
}

int bepaalBodemvochtStatus(int sensorValue, bool isResistief)// Bepaalt de status (DROOG/VOCHTIG/NAT) van een bodemvochtigheidssensor op basis van de gemeten waarde
{ 
  // Voor resistieve sensor:
  if (isResistief){
    if (sensorValue >= RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN && sensorValue <= RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX){
      return STATUS_DROOG;
    }
    else if (sensorValue >= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && sensorValue <= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX){
      return STATUS_VOCHTIG;
    }
    else if (sensorValue >= RESISTIEVE_SENSOR_NAT_INTERVAL_MIN && sensorValue <= RESISTIEVE_SENSOR_NAT_INTERVAL_MAX){
      return STATUS_NAT;
    }
  }
  // Voor capacitieve sensor:
  else{
    if (sensorValue >= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN && sensorValue <= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX){
      return STATUS_DROOG;
    }
    else if (sensorValue >= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && sensorValue <= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX){
      return STATUS_VOCHTIG;
    }
    else if (sensorValue >= CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN && sensorValue <= CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX){
      return STATUS_NAT;
    }
  }
}

void printBodemvochtData()// Print de ruwe waarden, percentages en bodemvochtstatus van beide bodemvochtigheidssensoren naar de Serial Monitor
{ 
  int resMoisturePercentage = berekenBodemvochtPercentage(resSensorValue, RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN, RESISTIEVE_SENSOR_NAT_INTERVAL_MAX);
  int capMoisturePercentage = berekenBodemvochtPercentage(capSensorValue, CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN, CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX);

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

  Serial.println("----------------");
}

int bepaalPrioriteit()//Bij niet-overeenkomende status krijgt de capacitieve sensor prioriteit/voorrang want deze meet nauwkeuriger
{ 
  if (resBodemStatus == capBodemStatus){
    return capBodemStatus; // of resBodemStatus (ze zijn toch gelijk)
  }
  else{
    return capBodemStatus;
  }
}

float getTemp()// Leest de temperatuur van de DS18B20 sensor en geeft deze terug in graden Celsius
{ 
  byte data[12];
  byte addr[8];

  if (!ds.search(addr))
  {
    // no more sensors on chain, reset search
    Serial.println("no more sensors on chain, reset search!");
    ds.reset_search();
    return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28)
  {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++)
  { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); // using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}

void printTempValue()// Print de gemeten temperatuur naar de Serial Monitor
{ 
  float temperature = getTemp();
  Serial.print("De temperatuur is: ");
  Serial.print(temperature);
  Serial.println(" °C");
}

int bepaalBewateringType(float temperatuur, int bodemStatus)// Functie om type bewatering te bepalen a.d.h.v. temperatuur en bodemstatus
{
  // Als temperatuur te laag is (< 5°C), geen bewatering
  if (temperatuur < TEMP_MIN){
    return BEWATERING_UIT;
  }
  // Als bodem DROOG is
  if (bodemStatus == STATUS_DROOG){
    // Als temperatuur > 25°C, dan LANG bewateren
    if (temperatuur > TEMP_MAX){
      return BEWATERING_LANG;
    }
    // Anders KORT bewateren
    else{
      return BEWATERING_KORT;
    }
  }
  // In alle andere gevallen (grond is niet droog), geen bewatering nodig
  else{
    return BEWATERING_UIT;
  }
}

void zetWaterpompAan(int bewateringsType)// Functie om pomp aan te zetten a.d.h.v. bewateringstype
{
  digitalWrite(RELAY_PIN, HIGH);
  huidigeBewatering = BEWATERING_AAN;
  bewateringType = bewateringsType;
  bewateringStartTijd = millis();

  String type = "";
  if (bewateringsType == BEWATERING_TYPE_KORT){
    type = "KORT";
  }
  else if (bewateringsType == BEWATERING_TYPE_LANG){
    type = "LANG";
  }
  else if (bewateringsType == BEWATERING_TYPE_KNOP){
    type = "KNOP";
  }
  Serial.println("Bewatering AAN - Type: " + type);
}

void zetWaterpompUit()//Functie om pomp uit te zetten en statussen te 'resetten'
{
  digitalWrite(RELAY_PIN, LOW);
  huidigeBewatering = BEWATERING_UIT;
  bewateringType = 0;
  Serial.println("Bewatering UIT");
}

void leesSensoren() {
  // Lees sensordata
  getBodemvochtValue();
  float temperatuur = getTemp();

  // Bereken bodemstatussen
  resBodemStatus = bepaalBodemvochtStatus(resSensorValue, true);
  capBodemStatus = bepaalBodemvochtStatus(capSensorValue, false);

  // Print alle data
  printBodemvochtData();
  printTempValue();
}

void controleerBewatering()// Als bewatering al actief is, check of deze gestopt moet worden 
{  
  if (huidigeBewatering == BEWATERING_AAN){
    unsigned long verstrekenTijd = millis() - bewateringStartTijd;
    bool moetStoppen = false;

    if (bewateringType == BEWATERING_KORT){
      moetStoppen = verstrekenTijd >= BEWATERING_KORT;
    }
    else if (bewateringType == BEWATERING_LANG){
      moetStoppen = verstrekenTijd >= BEWATERING_LANG;
    }
    else if (bewateringType == BEWATERING_KNOP){
      moetStoppen = verstrekenTijd >= BEWATERING_KNOP;
    }

    if (moetStoppen) {
      zetWaterpompUit();
      if (bewateringType == BEWATERING_KNOP){
        knopIngedrukt = false;
      }
    }
    return;
  }

  // Controleer of nieuwe bewatering nodig is
  float temperatuur = getTemp();
  int prioriteitStatus = bepaalPrioriteit();
  bewateringType = bepaalBewateringType(temperatuur, prioriteitStatus);

  if (bewateringType != BEWATERING_UIT && !knopIngedrukt) {
      zetWaterpompAan(bewateringType);
  }
}

void controleerDrukknop()//Functie om pomp aan te zetten indien er op de knop wordt gedrukt 
{
  if (digitalRead(BUTTON_PIN) == HIGH && huidigeBewatering == BEWATERING_UIT && !knopIngedrukt) {
    knopIngedrukt = true;
    knopStartTijd = millis();
    zetWaterpompAan(BEWATERING_KNOP);
  }
}

void setup()
{ // Initialiseert de pinnen en Serial communicatie
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(RELAY_PIN, LOW); // Relais start uitgeschakeld (veiliger)
  
}

void loop()
{
  unsigned long huidigeMillis = millis();

  // Controleer eerst de drukknop (heeft prioriteit), we plaatsen deze functie hier zodat de drukknop altijd geactiveerd kan worden en niet enkel bij een sensor lezing
  controleerDrukknop();

  // We plaatsen deze functie hier zodat de bewatering continue wordt gecontroleerd en niet enkel bij een sensor lezing
  controleerBewatering();

  // Voer sensormetingen uit om de 15 seconden, verwerk data en toon in seriële monitor
  if (huidigeMillis - LaatsteSensorLezing >= SENSOR_LEES_INTERVAL)
  {
    leesSensoren();
    LaatsteSensorLezing = huidigeMillis;
  }
}