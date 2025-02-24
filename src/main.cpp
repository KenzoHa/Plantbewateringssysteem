/* BRONNEN:
https://canvas.kdg.be/courses/49816/pages/basisstructuur-code-en-configuratie-van-het-plantbewateringssysteem --> basisstructuur code
https://canvas.kdg.be/courses/49816/pages/bodemvochtigheidssensoren --> voorbeeld 'value-mapping' bodemvochtigheidssensoren
https://wiki.dfrobot.com/Waterproof_DS18B20_Digital_Temperature_Sensor__SKU_DFR0198_ --> voorbeeld getTemp()-functie voor DS18B20
https://canvas.kdg.be/courses/49816/pages/coding-debugging-and-tracing?module_item_id=1128459 --> Hoe debugging-libraries gebruiken
https://www.youtube.com/watch?v=JHMpszgzWSg&t=550s --> ArduinoTrace video-tutorial
https://www.w3schools.com/cpp/cpp_conditions_shorthand.asp --> Ternary operator (korte schrijfwijze van if-else)
https://docs.particle.io/reference/device-os/api/time/time/ --> omdat de millis() functie een 'unsigned long' teruggeeft, gebruik ik ook 'unsigned long'. Dit om vergelijk-problemen te voorkomen 
https://www.perplexity.ai/ --> AI-tool
https://claude.ai --> AI-tool
*/

#define ARDUINOTRACE_ENABLE 1  // Disable all traces => 0

// Libraries:
#include <Arduino.h>
#include <config.h>            //Pin-definities en CONSTANTEN (grenswaarden, timing, statussen, etc.)
#include <OneWire.h>           //Nodig voor temp.sensor DS18B20
#include <DallasTemperature.h> //Nodig voor temp.sensor DS18B20
#include <ArduinoTrace.h>      //debugging-tool/-library

// Globale variabelen:
unsigned long LaatsteSensorLezing = 0;
unsigned long bewateringStartTijd = 0;
unsigned long pompDuur = 0;
int pompStatus = POMP_UIT;
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
  TRACE();
  int resMoisturePercentage = berekenBodemvochtPercentage(resSensorValue, RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN, RESISTIEVE_SENSOR_NAT_INTERVAL_MAX);
  int capMoisturePercentage = berekenBodemvochtPercentage(capSensorValue, CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN, CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX);

  // Print resistieve sensor data
  DUMP(resSensorValue);
  DUMP(resMoisturePercentage);
  
  const char* resStatusStr;
  if (resBodemStatus == STATUS_DROOG){
    resStatusStr = "DROOG";
  }
  else if (resBodemStatus == STATUS_VOCHTIG){
    resStatusStr = "VOCHTIG";
  }
  else if (resBodemStatus == STATUS_NAT){
    resStatusStr = "NAT";
  }

  DUMP(resStatusStr);
  

  // Print capacitieve sensor data
  DUMP(capSensorValue);
  DUMP(capMoisturePercentage);
  
  const char* capStatusStr;
  if (capBodemStatus == STATUS_DROOG){
    capStatusStr = "DROOG";
  }
  else if (capBodemStatus == STATUS_VOCHTIG){
    capStatusStr = "VOCHTIG";
  }
  else if (capBodemStatus == STATUS_NAT){
    capStatusStr = "NAT";
  }
  DUMP(capStatusStr);
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
    TRACE();
    DUMP("no more sensors on chain, reset search!");
    ds.reset_search();
    return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    TRACE();
    DUMP("CRC is not valid!");
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28)
  {
    TRACE();
    DUMP("Device is not recognized");
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
  TRACE();
  float temperature = getTemp();
  DUMP(temperature);
}

void zetWaterpompAan(unsigned long duurtijd) {
  TRACE();
  digitalWrite(RELAY_PIN, HIGH);
  pompStatus = POMP_AAN;
  bewateringStartTijd = millis();
  pompDuur = duurtijd;
  DUMP(pompStatus);
  DUMP(pompDuur);   //Hiermee zien we hoe lang de pomp aan gaat (in ms)
}

void zetWaterpompUit() {
  TRACE();
  digitalWrite(RELAY_PIN, LOW);
  pompStatus = POMP_UIT;
  pompDuur = 0;
  DUMP(pompStatus);
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

void controleerBewatering() {
  if (pompStatus == POMP_AAN) {
    if (millis() - bewateringStartTijd >= pompDuur) {
      zetWaterpompUit();
      knopIngedrukt = false;
    }
    return;
  }

  float temperatuur = getTemp();
  int bodemStatus = bepaalPrioriteit();
  
  if (temperatuur >= TEMP_MIN && bodemStatus == STATUS_DROOG) {
    unsigned long duurtijd = (temperatuur > TEMP_MAX) ? BEWATERING_LANG : BEWATERING_KORT; //TERNARY-OPERATOR --> variable = (condition) ? do_when_TRUE : do_when_FALSE;
    zetWaterpompAan(duurtijd);
  }
}

void controleerDrukknop() {
  if (digitalRead(BUTTON_PIN) == HIGH && pompStatus == POMP_UIT && !knopIngedrukt) {
    knopIngedrukt = true;
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

    // Check of de pomp moet stoppen
  if (pompStatus == POMP_AAN) {
    if (millis() - bewateringStartTijd >= pompDuur) {
      zetWaterpompUit();
      knopIngedrukt = false;
    }
  }

  // Voer sensormetingen uit om de 15 seconden, verwerk data en toon in seriële monitor
  if (huidigeMillis - LaatsteSensorLezing >= SENSOR_LEES_INTERVAL)
  {
    leesSensoren();
    LaatsteSensorLezing = huidigeMillis;
    controleerBewatering();
  }
}