/* SOURCES
  https://canvas.kdg.be/courses/49816/pages/oefeningen-calibratie-bodemvochtigheidssensor --> Voorbeeld kalibratiewaarden instellen
*/

// PIN DEFINITIES: 
const int RES_SOILSENSOR = 12;
const int CAP_SOILSENSOR = 4;

const int DS18S20_Pin = 4; 

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