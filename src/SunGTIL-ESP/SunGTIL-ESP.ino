#define ESP32C3_BOARD 1

#include <Arduino.h>
#include <map>
#include <WiFi.h>
#include <EspMQTTClient.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>

#include "Configs.h"
#include "GfSun2000.h"
#include "MessageModels.h"


#define MQTT_MAX_PACKET_SIZE 2048
#define DEVICE_ID "2307060514"

#define SERIAL_LOG Serial
#ifdef ESP32C3_BOARD
#define SERIAL_GTIL Serial1
#define RXPIN 6         // GPIO 6 => RX for Serial1
#define TXPIN 7         // GPIO 7 => TX for Serial1
#define LED_BUILTIN 8
#else
#define SERIAL_GTIL Serial2
#endif

/*---> Variables definition*/
WiFiManager wifiManager;

GfSun2000 gtil_device = GfSun2000();

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
  blink_led(SUCCESS_MODE);
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
  blink_led(ERROR_MODE);
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

void setup_modbus(){
#ifdef ESP32C3_BOARD
  SERIAL_GTIL.begin(9600, SERIAL_8N1, RXPIN, TXPIN);
#else
  SERIAL_GTIL.begin(9600, SERIAL_8N1);
#endif
  while (!SERIAL_LOG) {}
  gtil_device.setup(SERIAL_GTIL);
  gtil_device.setDataHandler(dataHandler);
  gtil_device.setErrorHandler(errorHandler);
}

void setup_wifi() {
  wifiManager.setDebugOutput(true);
  wifiManager.setConfigPortalTimeout(30);
  char strApName[40];
  uint32_t chipId = 0;
  for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}
  sprintf(strApName, "GTIL2_SUN_WIFI_%X", chipId);
  wifiManager.autoConnect(strApName);

  SERIAL_LOG.println("Setup WifiManager completed");
  blink_led(SUCCESS_MODE);
}

void setup_mqtt_client(){
  sprintf(mqtt_client_name, "ESP32Client_%04X", random(0xffff));
  mqtt_client = new EspMQTTClient(
    mqtt_server,      // MQTT Broker server ip
    1883,             // MQTT Broker default port
    mqtt_user,        // Can be omitted if not needed
    mqtt_password,    // Can be omitted if not needed
    mqtt_client_name  // Client name that uniquely identify your device
  );
  // Optional functionalities of EspMQTTClient
  mqtt_client->enableDebuggingMessages();  // Enable debugging messages sent to serial output
  mqtt_client->setMaxPacketSize(MQTT_MAX_PACKET_SIZE);

  SERIAL_LOG.println("Setup MQTT Client completed");
  blink_led(SUCCESS_MODE);
}

void blink_led(LED_MODE mode) {
  switch (mode) {
    case NORMAL_MODE:
      break;
    case SUCCESS_MODE:
      for (int i = 0; i < 2; i++) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);  // wait for a second
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
      }
      break;
    case ERROR_MODE:
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);  // wait for a second
      digitalWrite(LED_BUILTIN, HIGH);
      //delay(1000);
      break;
  }
}

void setup() {
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  SERIAL_LOG.begin(9600);
  //Print a message for debug
  SERIAL_LOG.println("SUN GRID-TIE INVERTER 1000/2000");
  // Setup Modbus connection
  setup_modbus();
  // Setup wifi via WifiManager
  setup_wifi();
  // Setup mqtt client
  setup_mqtt_client();
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  gtil_device.readData();
  // wait for a second
  //blink_led(NORMAL_MODE);
  delay(800);
}
