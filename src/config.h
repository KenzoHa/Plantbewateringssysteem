#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h> 

/* BRONNEN
  https://canvas.kdg.be/courses/49816/pages/oefeningen-calibratie-bodemvochtigheidssensor --> Voorbeeld kalibratiewaarden instellen
  https://canvas.kdg.be/courses/49816/pages/basisstructuur-code-en-configuratie-van-het-plantbewateringssysteem --> basisstructuur config.h
  https://docs.particle.io/reference/device-os/api/time/micros/ --> om vergelijk-problemen met tijd (millis()) te voorkomen gebruik ik const long voor intervallen, en geen const int. (millis() gebruikt unsigned long)
  */

// TIMING CONSTANTEN
const long DEEP_SLEEP_INTERVAL = 30000;       // 30 seconden tussen elke sensorlezing (in ms)
const long BEWATERING_DUURTIJD_KORT = 2000;   // 2 seconden water geven voor KORT (volgens flowchart)
const long BEWATERING_DUURTIJD_LANG = 5000;   // 5 seconden water geven voor LANG (volgens flowchart)
const long BEWATERING_DUURTIJD_KNOP = 1000;   // 1 seconde water geven bij drukknop (volgens flowchart)
const long DEBOUNCE_TIJD = 75;                // 75ms debounce-periode  
const long WACHTTIJD_VOOR_DEEPSLEEP = 1000;   // 1 seconde wachten voor debug-info voordat naar deep sleep wordt gegaan
// WiFi (tijds)instellingen
const int MAX_WIFI_RECONNECT_ATTEMPTS = 2;    // Maximum aantal pogingen voor WiFi-reconnect voordat we doorgaan
const long WIFI_RECONNECT_INTERVAL = 30000;   // 30 seconden tussen reconnect pogingen
const long WIFI_TIMEOUT = 5000;               // 5 seconden maximale wachttijd voor WiFi-verbinding
// HTTP tijdsinstellingen
const unsigned long HTTP_TIMEOUT = 10000;     // 10 seconden maximale wachttijd voor HTTP verzoeken
const unsigned long HTTP_CHECK_INTERVAL = 1000; // Check elke seconde of HTTP-verzoek is voltooid

// Drempelwaarde voor bewegingsdetectie door positievergelijking
const float BEWEGING_VERSCHIL_DREMPEL = 7;  // Drempelwaarde voor verschil in accelerometerwaarden (getal gekozen na analyse gemeten waarden in stilstand.)

// BEWATERING & PANIC-BUTTON CONSTANTEN
const float TEMP_MIN = 5.0;     // Minimum temperatuur voor bewatering
const float TEMP_MAX = 25.0;    // Maximum temperatuur voor bewatering

// STATUSSEN POMP (als String-constanten voor betere debugging met DUMP())
const String POMP_UIT = "POMP_UIT";
const String POMP_AAN = "POMP_AAN";
 
// POMP BEWATERINGSTYPE
const String TYPE_KORT = "KORT";
const String TYPE_LANG = "LANG";
const String TYPE_KNOP = "KNOP";

// STATUSSEN BODEMVOCHTIGHEID (als String-constanten voor betere debugging met DUMP())
const String STATUS_DROOG = "DROOG";
const String STATUS_VOCHTIG = "VOCHTIG";
const String STATUS_NAT = "NAT";
const String STATUS_ERROR = "ERROR";

// GRENSWAARDEN BODEMVOCHTIGHEID
// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "DROOG" te krijgen
const int RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN = 0;
const int RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX = 1157;

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "VOCHTIG" te krijgen
const int RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MIN = RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX + 1;
const int RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX = 2327;

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "NAT" te krijgen
const int RESISTIEVE_SENSOR_NAT_INTERVAL_MIN = RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX + 1;
const int RESISTIEVE_SENSOR_NAT_INTERVAL_MAX = 3440;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "DROOG" te krijgen
const int CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN = 2740;
const int CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX = 3102;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "VOCHTIG" te krijgen
const int CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN = 2336;
const int CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX = CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN - 1;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "NAT" te krijgen
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN = 1489;
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX = CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN - 1;

// PIN DEFINITIES: 
#define RES_SOILSENSOR 35
#define CAP_SOILSENSOR 34
#define DS18S20_Pin 13
#define RELAY_PIN 14 
#define BUTTON_PIN 25 
#define I2C_SDA_PIN 21   // Accelerometer - I2C data pin
#define I2C_SCL_PIN 22   // Accelerometer - I2C clock pin

// Deep Sleep configuratie
#define uS_TO_S_FACTOR 1000000  // Conversie factor van microseconden naar seconden
#define USE_EXT0_WAKEUP 1       // 1 = EXT0 wakeup, 0 = EXT1 wakeup (EXT0 is simpeler)

// BAUDRATE
#define BAUDRATE 115200

#endif // CONFIG_H