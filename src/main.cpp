/* BRONNEN:
https://canvas.kdg.be/courses/49816/pages/basisstructuur-code-en-configuratie-van-het-plantbewateringssysteem --> basisstructuur code
https://canvas.kdg.be/courses/49816/pages/bodemvochtigheidssensoren --> voorbeeld 'value-mapping' bodemvochtigheidssensoren
https://wiki.dfrobot.com/Waterproof_DS18B20_Digital_Temperature_Sensor__SKU_DFR0198_ --> voorbeeld getTemp()-functie voor DS18B20
https://canvas.kdg.be/courses/49816/pages/coding-debugging-and-tracing?module_item_id=1128459 --> Hoe debugging-libraries gebruiken
https://www.youtube.com/watch?v=JHMpszgzWSg&t=550s --> ArduinoTrace video-tutorial
https://www.w3schools.com/cpp/cpp_conditions_shorthand.asp --> Ternary operator (korte schrijfwijze van if-else)
https://docs.particle.io/reference/device-os/api/time/time/ --> omdat de millis() functie een 'unsigned long' teruggeeft, gebruik ik ook 'unsigned long'. Dit om vergelijk-problemen te voorkomen 
https://www.perplexity.ai/ --> AI-tool voor opzoekwerk en snel andere bronnen vinden
https://claude.ai --> AI-tool (Claude Sonnet 3.7 - AnthropicAI)
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html --> ESP32 Deep Sleep documentatie
https://github.com/DFRobot/DFRobot_LIS --> Accelerometer library
https://canvas.kdg.be/courses/49815/pages/wi-fi-basisfuncties?module_item_id=1098880 --> WiFi-implementatie heb ik gedaan zoals geleerd bij Embedded Systemen Connecties, namelijk met een aparte wifi_connect.h library
https://www.learncpp.com/cpp-tutorial/header-guards/ --> Header Guards (voor libraries) uitleg waarom + hoe
*/

#define ARDUINOTRACE_ENABLE 1  // Disable all traces => 0

// Libraries:
#include <Arduino.h>
#include <config.h>            // Pin-definities en configureerbare CONSTANTEN (grenswaarden, timing, statussen, etc.)
#include <ArduinoTrace.h>      // debugging-tool/-library
#include "driver/rtc_io.h"     // Nodig voor RTC IO functies bij deep sleep
#include <Wire.h>              // I2C communicatie
#include <WiFi.h>
#include "WiFi_connect.h"      // WiFi-functionaliteit
#include "google_sheets.h"
#include <OneWire.h>           // Nodig voor temp.sensor DS18B20
#include <DallasTemperature.h> // Nodig voor temp.sensor DS18B20
#include <DFRobot_LIS2DW12.h>  // Library voor LIS2DW12 accelerometer

// OneWire object voor de temperatuursensor
OneWire ds(DS18S20_Pin);

// Accelerometer object aanmaken
DFRobot_LIS2DW12_I2C acce(&Wire, 0x18);  // I2C adres 0x18

// RTC geheugen variabelen (behouden tussen deep sleep sessies)
RTC_DATA_ATTR bool drukknopWakeUp = false;              // nodig voor EXT0 
RTC_DATA_ATTR bool beweging_gedetecteerd_flag = false;  // nog om te zien of er beweging is geweest tussen 2 sensorlezingen (in deepsleep)
RTC_DATA_ATTR bool rtc_pompAanGeweest = false;          // nodig om te loggen dat de pomp is aangeweest door drukknop (drukknop zonder sensorenlezing)
RTC_DATA_ATTR String rtc_bewateringsType = "";          // nodig om te loggen dat de pomp is aangeweest door drukknop (drukknop zonder sensorenlezing)
RTC_DATA_ATTR bool metingUitgevoerd = false;            // Bijhouden of de wake-up actie al is uitgevoerd
// Accelerometer positie opslaan in RTC geheugen --> nodig in rtc-geheugen om te kunnen vergelijken met de volgende meting
RTC_DATA_ATTR float rtc_acc_x = 0.0;                    // Vorige X-positie
RTC_DATA_ATTR float rtc_acc_y = 0.0;                    // Vorige Y-positie
RTC_DATA_ATTR float rtc_acc_z = 0.0;                    // Vorige Z-positie

// Globale variabelen voor BVHsensorwaarden en statussen
int resSensorValue;
int capSensorValue;
String resBodemStatus;
String capBodemStatus;

// Globale variabele voor de huidige temperatuur
float huidigeTemperatuur = 0.0;                       // Actuele temperatuur voor logging

// Variabelen gerelateerd aan pomp
String pompStatus = POMP_UIT;
String bewateringsType = "";
bool pompAanGeweest = false;
// Variabelen om duurtijd van water geven te kunnen regelen
unsigned long bewateringStarttijd = 0;
unsigned long pompDuur = 0;

// Variabelen voor waarden accelerometer
float huidige_acc_x = 0.0;
float huidige_acc_y = 0.0;
float huidige_acc_z = 0.0;
bool bewegingGedetecteerd = false;
// Variabele voor bewegingsdetectie timing
unsigned long laatsteBewegingsCheck = 0;

// Variabelen voor logging / wifi 
bool dataVerstuurd = false;                // Bijhouden of data al is verzonden
int wifi_retry = 0;                        // Teller voor WiFi-verbindingspogingen

int leesCapacitieveBVHSensor() { // Bepaal de juiste sensorwaarde voor de capacitieve bodemvochtigheidssensor.
  TRACE();
  int waarde = analogRead(CAP_SOILSENSOR);
  DUMP(waarde);
  return waarde;
}

int leesResistieveBVHSensor() { // Bepaal de juiste sensorwaarde voor de resistieve bodemvochtigheidssensor.
  TRACE();
  int waarde = analogRead(RES_SOILSENSOR);
  DUMP(waarde); 
  return waarde;
}

int berekenBodemvochtPercentage(int sensorValue, int sensorMinValue, int sensorMaxValue)// Zet de ruwe sensorwaarde om naar een percentage (0-100%) op basis van de min/max grenzen
{
  TRACE();
  int percentage = map(sensorValue, sensorMinValue, sensorMaxValue, 0, 100);
  DUMP(percentage);
  return percentage;
}

String berekenCategorieCapactieveBHV(int sensorwaarde) { // Bepaal de categorie (status) van de capacitieve bodemvochtigheidssensor voor de gemeten sensorwaarde.
  TRACE();
  DUMP(sensorwaarde);
  String categorie;
  
  if (sensorwaarde >= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN && sensorwaarde <= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX) {
    categorie = STATUS_DROOG;
  } 
  else if (sensorwaarde >= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && sensorwaarde <= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX) {
    categorie = STATUS_VOCHTIG;
  } 
  else if (sensorwaarde >= CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN && sensorwaarde <= CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX) {
    categorie = STATUS_NAT;
  }
  else {
    // error als de waarde buiten alle intervallen valt
    categorie = STATUS_ERROR;
  }
  
  DUMP(categorie);
  return categorie;
}

String berekenCategorieResistieveBVH(int sensorwaarde) { // Bepaal de categorie (status) van de resistieve bodemvochtigheidssensor voor de gemeten sensorwaarde.
  TRACE();
  DUMP(sensorwaarde);
  String categorie;
  
  if (sensorwaarde >= RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN && sensorwaarde <= RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX) {
    categorie = STATUS_DROOG;
  } 
  else if (sensorwaarde >= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && sensorwaarde <= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX) {
    categorie = STATUS_VOCHTIG;
  } 
  else if (sensorwaarde >= RESISTIEVE_SENSOR_NAT_INTERVAL_MIN && sensorwaarde <= RESISTIEVE_SENSOR_NAT_INTERVAL_MAX) {
    categorie = STATUS_NAT;
  }
  else {
    // error als de waarde buiten alle intervallen valt
    categorie = STATUS_ERROR;
  }
  
  DUMP(categorie);
  return categorie;
}

String berekenSamengesteldeCategorie(String categorieResistieveBVH, String categorieCapacitieveBVH) { // Bereken de samengestelde categorie voor beide bodemvochtigheidssensoren.
  TRACE();
  DUMP(categorieResistieveBVH);
  DUMP(categorieCapacitieveBVH);
  
  // Bij niet-overeenkomende status krijgt de capacitieve sensor altijd prioriteit want deze meet nauwkeuriger
  String resultaat;
  if (categorieResistieveBVH == categorieCapacitieveBVH) {
    resultaat = categorieCapacitieveBVH; // of categorieResistieveBVH (ze zijn toch gelijk)
  } else {
    resultaat = categorieCapacitieveBVH; // Capacitieve sensor krijgt prioriteit
  }
  
  DUMP(resultaat);
  return resultaat;
}

float getTemp()// Leest de temperatuur van de DS18B20 sensor en geeft deze terug in graden Celsius
{ 
  TRACE();
  byte data[12];
  byte addr[8];

  if (!ds.search(addr))
  {
    // no more sensors on chain, reset search
    Serial.println("Temperatuursensor niet gevonden, reset zoekfunctie!");
    ds.reset_search();
    return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    Serial.println("Temperatuursensor CRC verificatie mislukt!");
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28)
  {
    Serial.println("Temperatuursensor wordt niet herkend");
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
  
  DUMP(TemperatureSum);
  
  return TemperatureSum;
}

void setupAccelerometer() {
  TRACE();
  // I2C pins instellen
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  while(!acce.begin()) {
    Serial.println("Accelerometer communicatie mislukt, controleer aansluiting...");
    delay(1000); // Geef tijd om te verbinden
  }
  
  // ID tonen met DUMP
  uint8_t chipId = acce.getID();
  DUMP(chipId);
  Serial.print("Accelerometer chip ID: 0x"); 
  Serial.println(chipId, HEX);
  
  // Reset voor schone start
  acce.softReset();
  delay(100);  // Wacht kort na reset
  
  // Configureer accelerometer
  acce.setRange(DFRobot_LIS2DW12::e2_g);
  acce.setFilterPath(DFRobot_LIS2DW12::eLPF);
  acce.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
  acce.setPowerMode(DFRobot_LIS2DW12::eContLowPwr4_14bit);
  acce.setDataRate(DFRobot_LIS2DW12::eRate_100hz);
  
  Serial.println("Accelerometer is geconfigureerd voor positiemeting");
}

void leesAccelerometerWaarden() {
  TRACE();
  // Lees huidige waarden
  huidige_acc_x = acce.readAccX();
  huidige_acc_y = acce.readAccY();
  huidige_acc_z = acce.readAccZ();
  
  // Debug info
  DUMP(huidige_acc_x);
  DUMP(huidige_acc_y);
  DUMP(huidige_acc_z);
}

bool detecteerBeweging() {// Functie om beweging te detecteren door te vergelijken met vorige positie
  TRACE();
  
  // Lees huidige waarden
  leesAccelerometerWaarden();
  
  // Bereken absolute verschillen tussen huidige en opgeslagen waarden
  float delta_x = abs(huidige_acc_x - rtc_acc_x);
  float delta_y = abs(huidige_acc_y - rtc_acc_y);
  float delta_z = abs(huidige_acc_z - rtc_acc_z);
  
  DUMP(rtc_acc_x);
  DUMP(rtc_acc_y);
  DUMP(rtc_acc_z);
  DUMP(delta_x);
  DUMP(delta_y);
  DUMP(delta_z);
  
  // Check of een van de verschillen boven de drempelwaarde is
  bewegingGedetecteerd = (delta_x > BEWEGING_VERSCHIL_DREMPEL || 
                          delta_y > BEWEGING_VERSCHIL_DREMPEL || 
                          delta_z > BEWEGING_VERSCHIL_DREMPEL);
  
  if (bewegingGedetecteerd) {
    Serial.println("===================================================");
    Serial.println("BELANGRIJK: De bloempot is bewogen sinds de laatste meting!");
    Serial.println("Verschillen in acceleratie waarden:");
    Serial.print("Delta X: "); Serial.print(delta_x); Serial.print(" g, ");
    Serial.print("Delta Y: "); Serial.print(delta_y); Serial.print(" g, ");
    Serial.print("Delta Z: "); Serial.print(delta_z); Serial.println(" g");
    Serial.println("===================================================");
  }
  
  // Sla huidige waarden op in RTC geheugen voor volgende vergelijking
  rtc_acc_x = huidige_acc_x;
  rtc_acc_y = huidige_acc_y;
  rtc_acc_z = huidige_acc_z;
  
  DUMP(bewegingGedetecteerd);
  return bewegingGedetecteerd;
}

void zetWaterpompAan(int duurtijd, String type) { // Zet de waterpomp aan met een bepaalde duurtijd / type bewatering
  TRACE();
  digitalWrite(RELAY_PIN, HIGH);
  pompStatus = POMP_AAN;
  bewateringStarttijd = millis(); // Naam uit flowchart
  pompDuur = duurtijd;
  bewateringsType = type;
  
  // Markeer dat de pomp aan is geweest zodat we dit later kunnen rapporteren
  pompAanGeweest = true;  
  
  Serial.println("Waterpomp aangezet: " + type + " bewatering");
  
  DUMP(pompStatus); 
  DUMP(pompDuur);
  DUMP(bewateringsType);
  DUMP(pompAanGeweest);
}

void zetWaterpompUit() { //Zet de waterpomp uit & reset variabelen
  TRACE();
  digitalWrite(RELAY_PIN, LOW);
  pompStatus = POMP_UIT;
  pompDuur = 0;
  
  Serial.println("Waterpomp uitgezet");
  
  // Toon het afgelopen bewateringsType voordat het wordt gewist
  DUMP(bewateringsType);
  // Belangrijk: Hier NIET bewateringsType resetten, pas na loggen!
  
  DUMP(pompStatus);
  DUMP(pompDuur);
}

void leesSensorenEnGeefWaterIndienNodig() { // Deze functie bevat alle code voor het uitlezen van de sensoren en om de waterpomp indien nodig aan te zetten (KORT of LANG).
  TRACE();
  Serial.println("===================================================");
  Serial.println("=============== Nieuwe sensorlezing ===============");
  
  // Lees sensorwaarden (in volgorde van flowchart)
  resSensorValue = leesResistieveBVHSensor();
  capSensorValue = leesCapacitieveBVHSensor();
  float temperatuur = getTemp();
  
  // Sla huidige temperatuur op in globale variabele voor logging
  huidigeTemperatuur = temperatuur;
  
  // Lees accelerometer waarden en detecteer beweging door vergelijking
  bewegingGedetecteerd = detecteerBeweging();
  
  // Sla de bewegingsdetectie status op voor logging
  beweging_gedetecteerd_flag = bewegingGedetecteerd;
  
  // Bereken bodemvochtigheidscategorieën
  resBodemStatus = berekenCategorieResistieveBVH(resSensorValue);
  capBodemStatus = berekenCategorieCapactieveBHV(capSensorValue);
  
  // Bepaal finale categorie voor besluitvorming
  String finaleCategorie = berekenSamengesteldeCategorie(resBodemStatus, capBodemStatus);
  
  // Debug informatie
  Serial.println("Sensoren zijn ingelezen, water zal gegeven worden indien nodig. Sensordata:");
  DUMP(resSensorValue);
  DUMP(resBodemStatus);
  DUMP(capSensorValue);
  DUMP(capBodemStatus);
  DUMP(finaleCategorie);
  DUMP(temperatuur);
  DUMP(pompStatus);
  DUMP(huidige_acc_x);
  DUMP(huidige_acc_y);
  DUMP(huidige_acc_z);

  // Beslissingslogica, indien er water gegeven mag worden 
  if (finaleCategorie == STATUS_DROOG && temperatuur >= TEMP_MIN) {
    // Bepaal het type bewatering op basis van temperatuur
    if (temperatuur > TEMP_MAX) {
      // LANG bewateren bij hoge temperatuur
      zetWaterpompAan(BEWATERING_DUURTIJD_LANG, TYPE_LANG);
    } else {
      // KORT bewateren bij normale temperatuur
      zetWaterpompAan(BEWATERING_DUURTIJD_KORT, TYPE_KORT);
    }
    
    Serial.println("===================================================");
    Serial.println("Plant is droog en temperatuur is binnen bereik. Water wordt gegeven.");
  }

  // Markeer dat de sensoren zijn ingelezen
  metingUitgevoerd = true;

  Serial.println("===================================================");
}

void logSensorDataToGoogleSheets() { // Functie om de sensor data naar Google Sheets te loggen
  TRACE();
  
  // Bereken de vocht percentages
  int resVochtPercentage = berekenBodemvochtPercentage(resSensorValue, RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN, RESISTIEVE_SENSOR_NAT_INTERVAL_MAX);
  int capVochtPercentage = 100 - berekenBodemvochtPercentage(capSensorValue, CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN, CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX);
  
  // Bepaal de samengestelde bodemstatus
  String samengesteldeStatus = berekenSamengesteldeCategorie(resBodemStatus, capBodemStatus);
  
  Serial.println("Data versturen naar Google Sheets...");
  
  // Maak een aangepaste pompstatus-string die ook informatie bevat over of er water is gegeven
  String aangepastePompStatus = pompStatus;
  
  // Als de pomp momenteel uit staat maar aan is geweest, maken we een aangepaste status
  if (pompAanGeweest && pompStatus == POMP_UIT) {
    aangepastePompStatus = "POMP_UIT_NA_" + bewateringsType;
    Serial.println("Gewijzigde pompstatus voor logging: " + aangepastePompStatus);
  }
  
  DUMP(aangepastePompStatus);
  
  // Haal datum en tijdstip op via time.h
  String timestamp = getTimeStamp();
  
  // Bouw de URL op met alle parameters - gebruik huidige actuele waarden
  String url = String(GOOGLE_SCRIPT_URL);
  url += "?timestamp=" + timestamp;
  url += "&temperatuur=" + String(huidigeTemperatuur);
  url += "&res_waarde=" + String(resSensorValue);
  url += "&cap_waarde=" + String(capSensorValue);
  url += "&res_vochtpercentage=" + String(resVochtPercentage);
  url += "&cap_vochtpercentage=" + String(capVochtPercentage);
  url += "&res_bodemstatus=" + resBodemStatus;
  url += "&cap_bodemstatus=" + capBodemStatus;
  url += "&samengestelde_bodemstatus=" + samengesteldeStatus;
  url += "&accelero_x_waarde=" + String(huidige_acc_x);
  url += "&accelero_y_waarde=" + String(huidige_acc_y);
  url += "&accelero_z_waarde=" + String(huidige_acc_z);
  url += "&beweging_gedetecteerd=" + String(beweging_gedetecteerd_flag ? "ja" : "nee");
  url += "&pompstatus=" + aangepastePompStatus;
  
  Serial.println("URL: " + url);
  
  // Begin het HTTP verzoek
  HTTPClient http;
  http.begin(url.c_str());
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
  // Verstuur het verzoek (blokkerende versie)
  int httpCode = http.GET();
  
  if (httpCode == 200) {
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpCode));
    Serial.println("Response: " + response);
    Serial.println("Data succesvol gelogd naar Google Sheets");
    dataVerstuurd = true;
  } else {
    Serial.println("Error bij HTTP request. Code: " + String(httpCode));
    Serial.println("Response: " + http.getString());
    Serial.println("Fout bij het loggen van data naar Google Sheets");
    dataVerstuurd = true; // Toch als verzonden markeren bij fout om door te kunnen gaan
  }
  
  // Reset pompAanGeweest flag na succesvolle logging --> doe dit zodat volgende log niet "POMP_UIT_NA_" bevat, maar de echte pompstatus
  if (dataVerstuurd) {
    pompAanGeweest = false;
    bewateringsType = "";  // Voor de zekerheid ook bewateringsType resetten
  }

  http.end();
}

void print_wakeup_reason() {
  TRACE();
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  
  DUMP(wakeup_reason);

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Ontwaakt door extern signaal (drukknop)");
      drukknopWakeUp = true;
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Ontwaakt door timer (tijd voor sensorlezing)");
      drukknopWakeUp = false;
      break;
    default:
      Serial.printf("Ontwaken niet veroorzaakt door deep sleep: %d\n", wakeup_reason);
      drukknopWakeUp = false;
      break;
  }
  DUMP(drukknopWakeUp);
}

bool magNaarDeepSleepGaan() { //bij eerste opstart enkel pomp uit en geen data verzenden, anders naar deepsleep indien pomp uit is en data verstuurd. 
  TRACE();
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  // Speciale voorwaarde voor eerste opstart (geen wake-up reason)
  if (wakeup_reason == 0) {
    return (pompStatus == POMP_UIT); // Bij eerste opstart alleen controleren of pomp uit is
  }
  
  // Normale voorwaarde voor opstart uit deepsleep
  return (pompStatus == POMP_UIT && dataVerstuurd == true);
}

void gaNaarDeepSleep() {// Functie om deep sleep in te schakelen
  TRACE();
  
  // Reset de metingUitgevoerd en dataVerstuurd variabelen voor de volgende wake-up
  metingUitgevoerd = false;
  dataVerstuurd = false;
  wifi_retry = 0;
  
  // Sla de status op in RTC geheugen
  rtc_pompAanGeweest = pompAanGeweest;
  rtc_bewateringsType = bewateringsType;
  
  Serial.println("===================================================");
  Serial.println("========= Systeem gaat naar DEEP SLEEP mode ========");
  
  /* 
    BELANGRIJK: WiFi modus voor deepsleep aanpassen !!
    Om verbinding met WiFi te houden tijdens deepsleep, gebruik ik WiFi.setSleep(true) in plaats van WiFi.disconnect() --> Dit ter voorbereiding op een mogelijke uitbreiding (webserver met knop om op afstand panic button te bedienen??)
   */
  Serial.println("WiFi configureren voor deepsleep...");
  WiFi.setSleep(true); // Behoud WiFi verbinding in slaapstand (vermindert stroomverbruik maar houdt verbinding)
  
  // Isoleer de sensoren om stroomverbruik te verminderen (werkt alleen effectief wanneer res.bvhsensor niet is aangesloten (hardwarematig))
  Serial.println("Bodemvochtigheidssensoren worden geïsoleerd voor deep sleep");
  rtc_gpio_isolate(GPIO_NUM_34);  // Isoleer capacitieve sensor
  rtc_gpio_isolate(GPIO_NUM_35); // Isoleer resistieve sensor

  // Configureer timer wake-up - systeem gaat ontwaken na DEEP_SLEEP_INTERVAL
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_INTERVAL * 1000); // ms naar µs conversie = x1000
  DUMP(DEEP_SLEEP_INTERVAL);
  
  // Configureer externe wake-up voor de drukknop (GPIO 25)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1); // 1 = HIGH niveau, 0 = LOW niveau
  
  Serial.println("Systeem ontwaakt over " + String(DEEP_SLEEP_INTERVAL / 1000) + " seconden");
  Serial.println("Of eerder als de drukknop wordt ingedrukt");
  Serial.println("===================================================");
  
  Serial.flush(); // Zorg dat alle data is verzonden voordat we slapen, dit is een serial.print met ingebouwde delay
  esp_deep_sleep_start();
  // Code na esp_deep_sleep_start() wordt nooit uitgevoerd
}

void setup() {
  Serial.begin(BAUDRATE);

  Serial.println("===================================================");
  Serial.println("=================== BEGIN SETUP ===================");
  Serial.println("===================================================");
  
  // De-isoleer de GPIO pinnen van de BVHsensoren na ontwaken uit deepsleep (dit is nodig omdat de BVHsensoren geïsoleerd werden vóór deepsleep)
  rtc_gpio_init(GPIO_NUM_34);  // De-isoleer/initialiseer capacitieve sensor 
  rtc_gpio_init(GPIO_NUM_35);  // De-isoleer/initialiseer resistieve sensor 
  
  // Configureer pinnen
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(CAP_SOILSENSOR, INPUT);
  pinMode(RES_SOILSENSOR, INPUT);
  
  // Zorg dat de relais uit staat bij opstarten (veiliger)
  digitalWrite(RELAY_PIN, LOW);

  // Reset de variabelen voor een nieuwe cyclus (voorkomt looping)
  metingUitgevoerd = false;
  dataVerstuurd = false;
  wifi_retry = 0;
  
  // Haal data uit RTC geheugen
  pompAanGeweest = rtc_pompAanGeweest;
  bewateringsType = rtc_bewateringsType;

  // Toon de oorzaak van ontwaken (timer of drukknop)
  print_wakeup_reason();

  // Initialiseer de accelerometer
  setupAccelerometer();
  
  // WiFi initialiseren
  Serial.println("WiFi initialiseren...");
  wifiInit();
  WiFi.onEvent(wifiReconnect, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  
  // Initialiseer Google Sheets logging
  initGoogleSheets();
  
  Serial.println("===================================================");
  Serial.println("===== PLANTBEWATERINGSSYSTEEM GEÏNITIALISEERD =====");
  Serial.println("===================================================");
}

void loop() {
  // 0. Wat is de wakeup-reason ? 
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  
  // 1. Is de panic button ingedrukt?
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 && !metingUitgevoerd) {
    zetWaterpompAan(BEWATERING_DUURTIJD_KNOP, TYPE_KNOP);
    metingUitgevoerd = true;
  }

  // 2. Mag de waterpomp uit?
  if (pompStatus == POMP_AAN && (millis() - bewateringStarttijd > pompDuur)) {
    zetWaterpompUit();
  }

  // 3. Mogen de sensoren ingelezen worden?
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER && !metingUitgevoerd) {
    leesSensorenEnGeefWaterIndienNodig();
  }
  
  // 4. Mag de data doorgestuurd worden?
  if (pompStatus == POMP_UIT && !dataVerstuurd && wakeup_reason != 0) {
    if (WiFi.status() == WL_CONNECTED) {
      logSensorDataToGoogleSheets();
    } else {
      // Probeer WiFi opnieuw te connecteren (aantal pogingen configureerbaar in config.h)
      if (wifi_retry < MAX_WIFI_RECONNECT_ATTEMPTS) {
        wifiInit();
        wifi_retry++;
      } else {
        // Log een connectiefout
        Serial.println("WiFi verbinding mislukt na " + String(MAX_WIFI_RECONNECT_ATTEMPTS) + " herpogingen");
        dataVerstuurd = true;  // Markeer als verzonden om door te gaan
      }
    }
  }

  // 5. Mag het systeem naar deepsleep gaan?
  if (magNaarDeepSleepGaan()) {
    gaNaarDeepSleep();
    return;
  }
}