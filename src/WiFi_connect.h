#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include <Arduino.h>
#include <WiFi.h>
#include "security.h"          // Hier komen je WiFi-credentials
#include <ArduinoTrace.h>      // debugging-tool/-library

// Functie om WiFi-events te loggen en af te handelen
void WiFiEvent(WiFiEvent_t event) {
  TRACE();
  Serial.printf("[WiFi-event] event: %d\n", event);

  switch (event) {
    case ARDUINO_EVENT_WIFI_READY:               Serial.println("WiFi interface ready"); break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:           Serial.println("Completed scan for access points"); break;
    case ARDUINO_EVENT_WIFI_STA_START:           Serial.println("WiFi client started"); break;
    case ARDUINO_EVENT_WIFI_STA_STOP:            Serial.println("WiFi clients stopped"); break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:       Serial.println("Connected to access point"); break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:    Serial.println("Disconnected from WiFi access point"); break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE: Serial.println("Authentication mode of access point has changed"); break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("Obtained IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:        Serial.println("Lost IP address and IP address is reset to 0"); break;
    default:                                    break;
  }
}

// Functie om WiFi te initialiseren
bool wifiInit() {
    TRACE();
    // Configureer WiFi voor betere deepsleep compatibiliteit
    WiFi.mode(WIFI_STA);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);    
    WiFi.setHostname(HOSTNAME);
    
    // Registreer event handler
    WiFi.onEvent(WiFiEvent);
    
    Serial.println("Verbinding maken met WiFi...");
    WiFi.begin(SSID, PASSWORD);  // SSID en PASSWORD zijn gedefinieerd in secrets.h
    
    // Wacht maximaal 10 seconden op verbinding
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT) {
        Serial.print(".");
        delay(500);
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nVerbonden met WiFi netwerk!");
        Serial.print("IP adres: ");
        Serial.println(WiFi.localIP());
        DUMP(WiFi.status());
        return true;
    } else {
        Serial.println("\nKon geen verbinding maken met WiFi!");
        return false;
    }
}

// Functie om WiFi te reconnecten na disconnectie
void wifiReconnect(WiFiEvent_t event, WiFiEventInfo_t info) {
  TRACE();
  Serial.println("Disconnected from WiFi, attempting to reconnect...");
  WiFi.disconnect(true);
  WiFi.begin(SSID, PASSWORD);

  // Wait for connection with timeout
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nReconnected successfully!");
    DUMP(WiFi.status());
  } else {
    Serial.println("\nFailed to reconnect. Will try again on next disconnect event.");
    DUMP(WiFi.status());
  }
}

// Functie om beschikbare WiFi-netwerken te zoeken
void searchDevices() {
    TRACE();
    Serial.println("Scanning for networks...");
    int numNetworks = WiFi.scanNetworks();
  
    if (numNetworks == 0) {
        Serial.println("No networks found");
    } else {
        Serial.print(numNetworks);
        Serial.println(" networks found");
    
        for (int element = 0; element < numNetworks; ++element) {
            Serial.print(element + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(element));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(element));
            Serial.println(")");
            delay(10);
        }
    }
  
    Serial.println("");
}

// Functie om te controleren of WiFi verbonden is en indien nodig opnieuw te verbinden
bool checkWiFiConnection() {
    TRACE();
    if (WiFi.status() != WL_CONNECTED) {
        DUMP(WiFi.status());
        Serial.println("WiFi verbinding verloren. Opnieuw verbinden...");
        return wifiInit();  // Probeer opnieuw te verbinden
    }
    return true;  // Al verbonden
}

#endif // WIFI_CONNECT_H