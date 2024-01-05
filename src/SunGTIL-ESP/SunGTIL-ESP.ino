#include <Arduino.h>
#include <map>
#include <WiFi.h>
#include <EspMQTTClient.h>
#include <ArduinoJson.h>

#include "Configs.h"
#include "GfSun2000.h"
#include "MessageModels.h"


#define MQTT_MAX_PACKET_SIZE 2048
#define DEVICE_ID "2307060514"

#define SERIAL_LOG Serial
#define SERIAL_GTIL Serial1

#define RXPIN 6         // GPIO 6 => RX for Serial1
#define TXPIN 7         // GPIO 7 => TX for Serial1

/*---> Variables definition*/
GfSun2000 gtil_device = GfSun2000();

const char* wifi_ssid = WIFI_SSID;
const char* wifi_password = WIFI_PWD;
const char* mqtt_server = MQTT_BROKER_HOST;
const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PWD;

const char* topic_root = MQTT_ROOT_TOPIC;
const char* device_model = "SunGTIL_2000";
char mqtt_client_name[30];


EspMQTTClient* mqtt_client;
/*<---Variables definition*/


// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished() {
  char topic_subscribe[128];
  sprintf(topic_subscribe, "%s/%s/control/%s", topic_root, device_model, DEVICE_ID);
  // Subscribe to topic and display received message to Serial
  mqtt_client->subscribe(topic_subscribe, [](const String& payload) {
    SERIAL_LOG.println(payload);
  });
}

void SendMqttMessage(String msg) {
  mqtt_client->loop();

  char topic_publish[128];
  sprintf(topic_publish, "%s/%s/log/%s", topic_root, device_model, DEVICE_ID);
  // Serial.println("Publish a message to topic:");
  // SERIAL_LOG.println(topic_publish);
  // SERIAL_LOG.println("Message data:");
  // SERIAL_LOG.println(msg);

  mqtt_client->publish(topic_publish, msg.c_str());
}

void PublishCurrentMetrics(StatusLog status) {
  DynamicJsonDocument jLog(2048);
  jLog["command"] = CMD_UPDATE_LOG;
  jLog["deviceGuid"] = status.deviceGuid;

  DynamicJsonDocument jStreams(2048);

  for (DataStream& item : status.dataStreams) {
    JsonObject obj = jStreams.createNestedObject();
    obj["name"] = item.name;
    obj["value"] = item.value;
  }
  jLog["dataStreams"] = jStreams;
  // serializeJson(jLog, Serial);
  // Serial.println("");

  String jsonStr;
  serializeJson(jLog, jsonStr);
  SendMqttMessage(jsonStr);
}

// Handle error from MODBUS client:
void errorHandler(int errorId, char* errorMessage) {
  SERIAL_LOG.printf("Error response: %02X - %s\n", errorId, errorMessage);
}

// Handle error from MODBUS client:
void dataHandler(GfSun2000Data data) {
  std::map<int16_t, int16_t>::iterator itr;
  for (itr = data.modbusRegistry.begin(); itr != data.modbusRegistry.end(); ++itr) {
        Serial.printf("Registry %d: %d \n", itr->first, itr->second);
  }  

  StatusLog status;
  status.command = CMD_UPDATE_LOG;
  status.deviceGuid = data.deviceID;
  status.dataStreams.push_back(DataStream("dc_voltage", data.DCVoltage));
  status.dataStreams.push_back(DataStream("ac_voltage", data.ACVoltage));
  status.dataStreams.push_back(DataStream("output_power", data.outputPower));
  status.dataStreams.push_back(DataStream("limmiter_power", data.limmiterPower));
  status.dataStreams.push_back(DataStream("temperature", data.temperature));
  status.dataStreams.push_back(DataStream("energy_today", data.customEnergyCounter));
  status.dataStreams.push_back(DataStream("energy_total", data.totalEnergyCounter));
  PublishCurrentMetrics(status);
}

void setup() {
  SERIAL_LOG.begin(9600);
  SERIAL_GTIL.begin(9600, SERIAL_8N1, RXPIN, TXPIN);
  while (!SERIAL_LOG) {}
  gtil_device.setup(SERIAL_GTIL);
  gtil_device.setDataHandler(dataHandler);
  gtil_device.setErrorHandler(errorHandler);
  //Print a message for debug
  SERIAL_LOG.println("SUN GRID-TIE INVERTER 1000/2000");

  sprintf(mqtt_client_name, "ESP32Client_%04X", random(0xffff));
  mqtt_client = new EspMQTTClient(
    wifi_ssid,
    wifi_password,
    mqtt_server,      // MQTT Broker server ip
    mqtt_user,        // Can be omitted if not needed
    mqtt_password,    // Can be omitted if not needed
    mqtt_client_name  // Client name that uniquely identify your device
  );
  // Optional functionalities of EspMQTTClient
  mqtt_client->enableDebuggingMessages();  // Enable debugging messages sent to serial output
  mqtt_client->setMaxPacketSize(MQTT_MAX_PACKET_SIZE);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(8, OUTPUT);
}

void loop() {
  gtil_device.readData();
  digitalWrite(8, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);                      // wait for a second
  digitalWrite(8, LOW);   // turn the LED off by making the voltage LOW
  delay(500);                      // wait for a second
}
