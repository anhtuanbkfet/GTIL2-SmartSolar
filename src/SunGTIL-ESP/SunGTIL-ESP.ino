#define ESP32C3_BOARD 1

#include <Arduino.h>
#include <map>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include "ESP32TimerInterrupt.h"


#include "Configs.h"
#include "GfSun2000.h"
#include "MessageModels.h"

#define EEPROM_ADDRESS 0
#define EEPROM_SIZE sizeof(double)  // int64 size

#define MQTT_MAX_PACKET_SIZE 2048

#define SERIAL_LOG Serial
#ifdef ESP32C3_BOARD
#define SERIAL_GTIL Serial1
#define RXPIN 7  // GPIO 6 => RX for Serial1
#define TXPIN 6  // GPIO 7 => TX for Serial1
#define LED_BUILTIN 8
#else
#define SERIAL_GTIL Serial2
#endif

#define TRIGGER_PIN 21  // GPIO 20 => Trigger for reset wifi manager
#define LOW_MODE_PIN 20

#define TIMER0_INTERVAL_MS 1000


/*---> Variables definition*/

ESP32Timer ITimer0(0);
WiFiManager wifiManager;
GfSun2000 gtil_device = GfSun2000();
WiFiClient wifiClient;
PubSubClient* mqtt_client;

const char* mqtt_server = MQTT_BROKER_HOST;
const char* topic_root = MQTT_ROOT_TOPIC;
const char* device_model = "SunGTIL_2000";

const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PWD;
char mqtt_client_name[30];
String strDeviceId = "Unknown";


extern GfSun2000_Work_Mode gtilWorkMode;

// For count today grid consume
double g_todayGridCounter = 0.3;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

/*<---Variables definition*/

int nMinuteCount = 0;
void CheckInternetTimeTrigger() {
  //Get internet time:
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  //Send log to debug:
  char strLog[128];
  sprintf(strLog, "CheckInternetTimeTrigger: %s", timeClient.getFormattedTime().c_str());
  SERIAL_LOG.printf(strLog);
  SendMqttDeviceLog(strDeviceId, strLog);

  if (timeClient.getHours() == 0 && timeClient.getMinutes() == 0) {
    g_todayGridCounter = 0;  //Reset counter everyday
  }
  // Save to EEPROM every 15min
  nMinuteCount++;
  if (nMinuteCount >= 15) {
    nMinuteCount = 0;
    eeprom_save_data();
  }
}

void SendMqttMessage(String deviceId, String msgType, String msgData) {
  if (!mqtt_client->connected()) {
    mqtt_connect_to_server();
  }
  mqtt_client->loop();
  char topic_publish[128];
  sprintf(topic_publish, "%s/%s/%s/%s", topic_root, device_model, msgType, deviceId);
  mqtt_client->publish(topic_publish, msgData.c_str());
}

void SendMqttDeviceMetrics(String deviceId, String msg) {
  SendMqttMessage(deviceId, "log", msg);
  blink_led(SUCCESS_MODE);
}

void SendMqttDeviceLog(String deviceId, String msg) {
  SendMqttMessage(deviceId, "debug", msg);
}

void PublishCurrentMetrics(StatusLog status) {
  DynamicJsonDocument jLog(2048);
  jLog["command"] = CMD_UPDATE_METRICS;
  jLog["deviceGuid"] = status.deviceGuid;

  DynamicJsonDocument jStreams(2048);

  for (DataStream& item : status.dataStreams) {
    JsonObject obj = jStreams.createNestedObject();
    obj["name"] = item.name;
    obj["value"] = item.value;
  }
  jLog["dataStreams"] = jStreams;

  String jsonStr;
  serializeJson(jLog, jsonStr);
  SendMqttDeviceMetrics(status.deviceGuid, jsonStr);
}

void PublishRegistersDataLog(GfSun2000Data data) {
  DynamicJsonDocument jLog(2048);
  jLog["command"] = CMD_PRINT_DEBUG_LOG;
  jLog["deviceGuid"] = data.deviceID;
  jLog["gtilWorkMode"] = gtilWorkMode;

  DynamicJsonDocument jStreams(2048);
  std::map<int16_t, int16_t>::iterator itr;
  for (itr = data.modbusRegistry.begin(); itr != data.modbusRegistry.end(); ++itr) {
    JsonObject obj = jStreams.createNestedObject();
    obj["registry"] = itr->first;
    obj["value"] = itr->second;
  }
  jLog["registryValues"] = jStreams;

  String jsonStr;
  serializeJson(jLog, jsonStr);
  SendMqttDeviceLog(data.deviceID, jsonStr);
}

/////////////////////////////////////////////////////////////////////
// MODBUS SETUP
// Handle error from MODBUS client:
void errorHandler(int errorId, char* errorMessage) {
  char strLog[128];
  sprintf(strLog, "Error response: %02X - %s\n", errorId, errorMessage);
  SERIAL_LOG.printf(strLog);
  SendMqttDeviceLog(strDeviceId, strLog);
  blink_led(ERROR_MODE);
}

// Handle error from MODBUS client:
int g_secoundCounter = 0;
void dataHandler(GfSun2000Data data) {

  //Process today grid consume:
  g_todayGridCounter += (data.limmiterPower / 3600000);

  strDeviceId = String(data.deviceID);
  StatusLog status;
  status.command = CMD_UPDATE_METRICS;
  status.deviceGuid = data.deviceID;
  status.dataStreams.push_back(DataStream("dc_voltage", data.DCVoltage));
  status.dataStreams.push_back(DataStream("ac_voltage", data.ACVoltage));
  status.dataStreams.push_back(DataStream("output_power", data.outputPower));
  status.dataStreams.push_back(DataStream("limmiter_power", data.limmiterPower));
  status.dataStreams.push_back(DataStream("temperature", data.temperature));
  status.dataStreams.push_back(DataStream("energy_today", data.customEnergyCounter));
  status.dataStreams.push_back(DataStream("energy_total", data.totalEnergyCounter));
  status.dataStreams.push_back(DataStream("total_power", data.outputPower + data.limmiterPower));
  status.dataStreams.push_back(DataStream("limmiter_today", g_todayGridCounter));

  PublishCurrentMetrics(status);

  if (g_secoundCounter % 60 == 0) {
    PublishRegistersDataLog(data);
    CheckInternetTimeTrigger();
    g_secoundCounter = 0;
  }
  g_secoundCounter++;
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
void eeprom_init() {
  // initialize EEPROM with predefined size
  if (!EEPROM.begin(EEPROM_SIZE)) {
    SERIAL_LOG.println("failed to initialize EEPROM");
  }
}

char strLog[64];
void eeprom_read_data() {
  EEPROM.get(EEPROM_ADDRESS, g_todayGridCounter);
  sprintf(strLog, "EEPROM read: %f", g_todayGridCounter);
  SERIAL_LOG.println(strLog);

  if (g_todayGridCounter > 50) {
    g_todayGridCounter = 0;
  }
}


void eeprom_save_data() {
  EEPROM.put(EEPROM_ADDRESS, g_todayGridCounter);
  EEPROM.commit();
  sprintf(strLog, "EEPROM save: %f", g_todayGridCounter);
  SERIAL_LOG.println(strLog);
}

String getEspChipsetId() {
  char devId[11];
  char strLog[128];
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  sprintf(devId, "%X", chipId);
  return String(devId);
}

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
  strDeviceId = getEspChipsetId();

  sprintf(strApName, "GTIL2_SUN_WIFI_%X", strDeviceId);
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
        // Stop current timer:
        ITimer0.stopTimer();

        SERIAL_LOG.println("WifiManager: Button Held");
        SERIAL_LOG.println("WifiManager: Erasing Config, restarting...");
        wifiManager.resetSettings();
        ESP.restart();
        // setup again
        setup_wifi();

        ITimer0.restartTimer();
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

void mqtt_connect_to_server() {
  // Attempt to connect
  char topic_status[128];
  sprintf(topic_status, "%s/%s/%s/%s", topic_root, device_model, "status", strDeviceId);
  sprintf(mqtt_client_name, "ESP8266Client_%04X", random(0xffff));


  DynamicJsonDocument lastMessage(256);
  lastMessage["command"] = CMD_UPDATE_STATUS;
  lastMessage["deviceGuid"] = strDeviceId;
  lastMessage["message"] = "disconnected";

  String strLastMessage;
  serializeJson(lastMessage, strLastMessage);

  while (!mqtt_client->connected()) {
    if (mqtt_client->connect(mqtt_client_name, mqtt_user, mqtt_password,
                             topic_status, 0, true, strLastMessage.c_str())) {
      SERIAL_LOG.println("Mqtt connected");
      //Send a message to notice that i'm online
      lastMessage["message"] = "connected";
      serializeJson(lastMessage, strLastMessage);
      mqtt_client->publish(topic_status, strLastMessage.c_str());
      // ... and resubscribe
      char topic_subscribe[128];
      sprintf(topic_subscribe, "%s/%s/control/%s", topic_root, device_model, strDeviceId);
      mqtt_client->subscribe(topic_subscribe);
    } else {
      SERIAL_LOG.print("Mqtt connect failed, rc=");
      SERIAL_LOG.print(mqtt_client->state());
      SERIAL_LOG.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup_mqtt_client() {
  mqtt_client = new PubSubClient(wifiClient);
  mqtt_client->setServer(mqtt_server, 1883);
  mqtt_client->setBufferSize(MQTT_MAX_PACKET_SIZE);
  mqtt_client->setCallback(mqtt_callback);
  //mqtt_connect_to_server();
}

/////////////////////////////////////////////////////////////////////////
// Timer:
bool IRAM_ATTR TimerHandler0(void* timerNo) {
  gtil_device.readData();
  return true;
}

int secondCounter = 0;
void blink_led(LED_MODE mode) {
  switch (mode) {
    case NORMAL_MODE:
      break;
    case SUCCESS_MODE:
      secondCounter++;
      if (secondCounter == 2) {
        secondCounter = 0;
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);  // wait for a second
        digitalWrite(LED_BUILTIN, HIGH);
      }
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

  //EEPROM:
  eeprom_init();

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
  // Setup timer0:
  // Interval in microsecs
  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0)) {
    SERIAL_LOG.print(F("Starting  ITimer0 OK, millis() = "));
    SERIAL_LOG.println(millis());
  } else
    SERIAL_LOG.println(F("Can't set ITimer0. Select another freq. or timer"));

  // read configured data from eeprom:
  eeprom_read_data();

  //Start NTP client:
  timeClient.begin();
  timeClient.setTimeOffset(25200);  //Set time offset for GMT+7
}

void loop() {
  digitalWrite(LOW_MODE_PIN, LOW);
  check_trigger_button();
  delay(10);
}
