#define ESP32C3_BOARD 1

#include <Arduino.h>
#include <map>
#include <WiFi.h>
#include <PubSubClient.h>
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
#define RXPIN 7  // GPIO 6 => RX for Serial1
#define TXPIN 6  // GPIO 7 => TX for Serial1
#define LED_BUILTIN 8
#else
#define SERIAL_GTIL Serial2
#endif

#define TRIGGER_PIN   21  // GPIO 20 => Trigger for reset wifi manager
#define LOW_MODE_PIN  20 
/*---> Variables definition*/
WiFiManager wifiManager;

GfSun2000 gtil_device = GfSun2000();

const char* mqtt_server = MQTT_BROKER_HOST;
const char* topic_root = MQTT_ROOT_TOPIC;
const char* device_model = "SunGTIL_2000";

// char mqtt_user[20];
// char mqtt_password[20];
const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PWD;
char mqtt_client_name[30];

WiFiClient wifiClient;
PubSubClient* mqtt_client;
/*<---Variables definition*/


void SendMqttErrorLog(String deviceId, String msg) {
  if (!mqtt_client->connected()) {
    mqtt_reconnect();
  }
  mqtt_client->loop();

  char topic_publish[128];
  sprintf(topic_publish, "%s/%s/debug/%s", topic_root, device_model, deviceId);
  mqtt_client->publish(topic_publish, msg.c_str());
}

void SendMqttMetrics(String deviceId, String msg) {
  if (!mqtt_client->connected()) {
    mqtt_reconnect();
  }
  mqtt_client->loop();

  char topic_publish[128];
  sprintf(topic_publish, "%s/%s/log/%s", topic_root, device_model, deviceId);
  // SERIAL_LOG.println("Publish a message to topic:");
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
  // SERIAL_LOG.println("");

  String jsonStr;
  serializeJson(jLog, jsonStr);
  SendMqttMetrics(status.deviceGuid, jsonStr);
}

/////////////////////////////////////////////////////////////////////
// MODBUS SETUP
// Handle error from MODBUS client:
void errorHandler(int errorId, char* errorMessage) {
  char devId[11];
  char strLog[128];
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  sprintf(devId, "%X", chipId);
  sprintf(strLog, "Error response: %02X - %s\n", errorId, errorMessage);
  SERIAL_LOG.printf(strLog);
  SendMqttErrorLog(devId, strLog);
  blink_led(ERROR_MODE);
}

// Handle error from MODBUS client:
void dataHandler(GfSun2000Data data) {
  std::map<int16_t, int16_t>::iterator itr;
  for (itr = data.modbusRegistry.begin(); itr != data.modbusRegistry.end(); ++itr) {
    SERIAL_LOG.printf("Registry %d: %d \n", itr->first, itr->second);
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

void setup_modbus() {
#ifdef ESP32C3_BOARD
  SERIAL_GTIL.begin(9600, SERIAL_8N1, RXPIN, TXPIN);
#else
  SERIAL_GTIL.begin(9600, SERIAL_8N1);
#endif
  while (!SERIAL_GTIL) {
    delay(10);
  }
  gtil_device.setup(SERIAL_GTIL);
  gtil_device.setDataHandler(dataHandler);
  gtil_device.setErrorHandler(errorHandler);
}

/////////////////////////////////////////////////////////////////////
// WIFI SETUP

void setup_wifi() {
  digitalWrite(LED_BUILTIN, LOW);
  // Reset Wifi settings for testing
  //wifiManager.resetSettings();
  wifiManager.setDebugOutput(true);
  wifiManager.setConfigPortalTimeout(30);
  wifiManager.setTimeout(120);


  //WiFiManagerParameter custom_mqtt_user("username", "Username", mqtt_user, 20);
  //WiFiManagerParameter custom_mqtt_pass("password", "Password", mqtt_password, 20);
  //add all your parameters here
  //wifiManager.addParameter(&custom_mqtt_user);
  //wifiManager.addParameter(&custom_mqtt_pass);

  char strApName[40];
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  sprintf(strApName, "GTIL2_SUN_WIFI_%X", chipId);
  //wifiManager.autoConnect(strApName);
  if (!wifiManager.autoConnect(strApName)) {
    SERIAL_LOG.println("WifiManager: failed to connect and hit timeout");
    delay(5000);
    //reset and try again
    ESP.restart();
  }
  //read updated parameters
  //strcpy(mqtt_user, custom_mqtt_user.getValue());
  //strcpy(mqtt_password, custom_mqtt_pass.getValue());
  SERIAL_LOG.println("WifiManager: Setup WifiManager completed");
}


void check_trigger_button() {
  if (digitalRead(TRIGGER_PIN) == LOW) {
    // poor mans debounce/press-hold, code not ideal for production
    delay(50);
    if (digitalRead(TRIGGER_PIN) == LOW) {
      SERIAL_LOG.println("WifiManager: Button Pressed");
      // still holding button for 3000 ms, reset settings, code not ideaa for production
      delay(3000);  // reset delay hold
      if (digitalRead(TRIGGER_PIN) == LOW) {
        SERIAL_LOG.println("WifiManager: Button Held");
        SERIAL_LOG.println("WifiManager: Erasing Config, restarting...");
        wifiManager.resetSettings();
        ESP.restart();
        // setup again
        setup_wifi();
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////
// MQTT SETUP

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  SERIAL_LOG.print("Message arrived [");
  SERIAL_LOG.print(topic);
  SERIAL_LOG.print("] ");
  for (int i = 0; i < length; i++) {
    SERIAL_LOG.print((char)payload[i]);
  }
}

void mqtt_reconnect() {
  // Loop until we're reconnected
  while (!mqtt_client->connected()) {
    // Attempt to connect
    if (mqtt_client->connect(mqtt_client_name, mqtt_user, mqtt_password)) {
      SERIAL_LOG.println("Mqtt connected");
      // ... and resubscribe
      char topic_subscribe[128];
      sprintf(topic_subscribe, "%s/%s/control/%s", topic_root, device_model, DEVICE_ID);
      mqtt_client->subscribe(topic_subscribe);
    } else {
      SERIAL_LOG.print("Mqtt connect failed, rc=");
      SERIAL_LOG.print(mqtt_client->state());
      SERIAL_LOG.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup_mqtt_client() {
  mqtt_client = new PubSubClient(wifiClient);
  mqtt_client->setServer(mqtt_server, 1883);
  mqtt_client->setBufferSize(MQTT_MAX_PACKET_SIZE);
  mqtt_client->setCallback(mqtt_callback);

  sprintf(mqtt_client_name, "ESP8266Client_%04X", random(0xffff));
  mqtt_client->connect(mqtt_client_name, mqtt_user, mqtt_password);

  if (mqtt_client->connected()) {
    char strLog[128];
    sprintf(strLog, "MQTT client connected as name: %s; username: %s", mqtt_client_name, mqtt_user);
    SERIAL_LOG.println(strLog);
    blink_led(SUCCESS_MODE);
  }
}

void blink_led(LED_MODE mode) {
  switch (mode) {
    case NORMAL_MODE:
      break;
    case SUCCESS_MODE:
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);  // wait for a second
      digitalWrite(LED_BUILTIN, HIGH);
      //delay(50);

      break;
    case ERROR_MODE:
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);  // wait for a second
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      break;
  }
}

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LOW_MODE_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);

#ifdef ESP32C3_BOARD
  digitalWrite(LED_BUILTIN, LOW);
#else
  digitalWrite(LED_BUILTIN, HIGH);
#endif

  WiFi.mode(WIFI_STA);  // explicitly set mode, esp defaults to STA+AP
  SERIAL_LOG.begin(9600);
  //Print a message for debug
  SERIAL_LOG.println("SUN GRID-TIE INVERTER 1000/2000");
  // Setup Modbus connection
  setup_modbus();
  // Setup wifi via WifiManager
  setup_wifi();
  // Setup mqtt client
  setup_mqtt_client();
  
}

void loop() {
  digitalWrite(LOW_MODE_PIN, LOW);
  check_trigger_button();
  gtil_device.readData();
  delay(1000);
}
