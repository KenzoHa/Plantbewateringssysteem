/* BRONNEN
  https://canvas.kdg.be/courses/49816/pages/oefeningen-calibratie-bodemvochtigheidssensor --> Voorbeeld kalibratiewaarden instellen
  https://canvas.kdg.be/courses/49816/pages/basisstructuur-code-en-configuratie-van-het-plantbewateringssysteem --> basisstructuur config.h
  https://docs.particle.io/reference/device-os/api/time/micros/ --> om vergelijk-problemen met tijd (millis()) te voorkomen gebruik ik const long voor intervallen. (en geen const int)
  */

// TIMING CONSTANTEN
const long SENSOR_LEES_INTERVAL = 10000;  // 10 seconden
const long BEWATERING_KORT = 2000;        // 2 seconden water geven voor KORT
const long BEWATERING_LANG = 5000;        // 5 seconden water geven voor LANG  
const long BEWATERING_KNOP = 5000;        // 5 seconden water geven bij drukknop

// BEWATERING & PANIC-BUTTON CONSTANTEN
const float TEMP_MIN = 5.0;     // Minimum temperatuur voor bewatering
const float TEMP_MAX = 25.0;    // Maximum temperatuur voor bewatering

// STATUSSEN BEWATERING
const int POMP_UIT = 0;
const int POMP_AAN = 1;

// STATUSSEN BODEMVOCHTIGHEID
const int STATUS_DROOG = 1;
const int STATUS_VOCHTIG = 2;
const int STATUS_NAT = 3;

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
#define RES_SOILSENSOR 12
#define CAP_SOILSENSOR 4
#define DS18S20_Pin 13
#define RELAY_PIN 14 
#define BUTTON_PIN 25 