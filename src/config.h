/* BRONNEN
  https://canvas.kdg.be/courses/49816/pages/oefeningen-calibratie-bodemvochtigheidssensor --> Voorbeeld kalibratiewaarden instellen
  https://canvas.kdg.be/courses/49816/pages/basisstructuur-code-en-configuratie-van-het-plantbewateringssysteem --> basisstructuur config.h
*/

// TIMING CONSTANTEN
const unsigned long SENSOR_LEES_INTERVAL = 15000;  // 15 seconden
const unsigned long BEWATERING_INTERVAL = 5000;    // 5 seconden tussen elke "beurt"
const unsigned long BEWATERING_KORT = 2000;     // 2 seconden water geven voor KORT
const unsigned long BEWATERING_LANG = 5000;      // 5 seconden water geven voor LANG  
const unsigned long BEWATERING_KNOP = 5000;    // 5 seconden water geven bij drukknop

// BEWATERING & PANIC-BUTTON CONSTANTEN
const float TEMP_MIN = 5.0;     // Minimum temperatuur voor bewatering
const float TEMP_MAX = 25.0;    // Maximum temperatuur voor bewatering

// STATUSSEN BEWATERING
#define BEWATERING_TYPE_KORT 1
#define BEWATERING_TYPE_LANG 2
#define BEWATERING_TYPE_KNOP 3
#define BEWATERING_UIT 0
#define BEWATERING_AAN 1

// STATUSSEN BODEMVOCHTIGHEID
#define STATUS_DROOG 1
#define STATUS_VOCHTIG 2
#define STATUS_NAT 3

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
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN = 2198;
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX = CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN - 1;

// PIN DEFINITIES: 
const int RES_SOILSENSOR = 12;
const int CAP_SOILSENSOR = 4;
const int DS18S20_Pin = 13;
const int RELAY_PIN = 14 ; 
const int BUTTON_PIN = 25 ; 