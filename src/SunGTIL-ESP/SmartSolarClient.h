#ifndef __SMARTSOLAR_CLIENT_H__
#define __SMARTSOLAR_CLIENT_H__

#include <Arduino.h>
#include <map>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/*
 Callbacks definitions
*/
typedef void (*OnError) (String strMsg);
typedef void (*OnMessageReceived) (void* msgData);

class SmartSolarClient {
public:        
    void setup();
    bool sendMetrics();
    bool sendLog(String strLog);   
};

#endif