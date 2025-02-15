/* BRONNEN
  https://canvas.kdg.be/courses/49816/pages/oefeningen-calibratie-bodemvochtigheidssensor --> Voorbeeld kalibratiewaarden instellen
  https://claude.ai --> AI-tool
  https://chatgpt.com --> AI-tool
*/

// PIN DEFINITIES: 
const int RES_SOILSENSOR = 12;
const int CAP_SOILSENSOR = 4;

const int DS18S20_Pin = 13;
OneWire ds(DS18S20_Pin);

const int RELAY_PIN = 14 ; 

const int BUTTON_PIN = 25 ; 

// TIMING-VARIABELEN EN CONSTANTEN
//alle sensoren
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 10000;  // 10 seconden
//bewatering
unsigned long laatsteBewateringTijd = 0;
const unsigned long BEWATERING_INTERVAL = 5000;    // 5 seconden tussen elke "beurt"
const unsigned long BEWATERING_DUUR = 3000;         // 3 seconden water geven per "beurt"
//PANIC-BUTTON bewatering
unsigned long knopBewateringStart = 0;
const unsigned long KNOP_BEWATERING_DUUR = 3000;    // 3 seconden water geven bij drukknop

// BEWATERING & PANIC-BUTTON VARIABELEN EN CONSTANTEN
bool bewateringKanAan = true;
const float TEMP_MIN = 5.0;     // Minimum temperatuur voor bewatering
const float TEMP_MAX = 25.0;    // Maximum temperatuur voor bewatering
bool knopIngedrukt = false;

// STATUSSEN BODEMVOCHTIGHEID
#define STATUS_DROOG 1
#define STATUS_VOCHTIG 2
#define STATUS_NAT 3
#define STATUS_ERROR 0

int resSensorValue;
int capSensorValue;
int resBodemStatus;
int capBodemStatus;

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