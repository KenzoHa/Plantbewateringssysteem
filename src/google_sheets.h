#ifndef GOOGLE_SHEETS_H
#define GOOGLE_SHEETS_H

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "time.h"
#include "security.h"
#include <ArduinoTrace.h>      // debugging-tool/-library

// De Web App URL van je Google Apps Script (vervang dit met je eigen URL)
const char* GOOGLE_SCRIPT_URL = WEB_APP_URL;

// NTP Server instellingen voor de tijd
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600; // 1 uur (3600 seconden) voor CET
const int daylightOffset_sec = 3600; // 1 uur voor zomertijd

// Initialiseer tijd-synchronisatie
void initGoogleSheets() {
    TRACE();
    // Configureer NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("Google Sheets logging geïnitialiseerd");
    
    // Probeer de huidige tijd te halen om te checken of NTP werkt
    struct tm timeinfo;
    if(getLocalTime(&timeinfo)) {
        char timeStringBuff[50];
        strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
        Serial.print("Huidige tijd: ");
        Serial.println(timeStringBuff);
    } else {
        Serial.println("Kon geen tijd krijgen van NTP server");
    }
}

// Functie om een timestamp te genereren
String getTimeStamp() {
    TRACE();
    struct tm timeinfo;
    char timeStringBuff[50];
    
    if(!getLocalTime(&timeinfo)) {
        Serial.println("Kon geen tijd krijgen, gebruik fallback timestamp");
        return String(millis());
    }
    
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%S", &timeinfo);
    return String(timeStringBuff);
}

#endif // GOOGLE_SHEETS_H