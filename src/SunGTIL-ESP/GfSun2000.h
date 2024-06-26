#ifndef GfSun2000_h
#define GfSun2000_h
#include <Arduino.h>
#include <map>
#include <ModbusClientRTU.h>

//#define BASIC_MODE true

#define MODBUS_REGISTRY_FROM  0
#define MODBUS_REGISTRY_TO    125
#define MODBUS_TIMEOUT        500
#define MODBUS_INDEX_OFFSET   3     // id server and function ID


#define REGISTRY_DEVICE_ID               3     // 10 chars; registers 3,4,5,6,7
#define REGISTRY_CUSTOM_ENERGY_COUNTER  60     // 2 bytes 
#define REGISTRY_TOTAL_ENERGY_COUNTER   63     // 2 bytes 
#define REGISTRY_AC_VOLTAGE             70     // 2 bytes
#define REGISTRY_AVERAGE_POWER          86     // 2 bytes
#define REGISTRY_DC_VOLTAGE            109     // 2 bytes
#define REGISTRY_TEMPERATURE            94     // 2 bytes (optional)
#define REGISTRY_LIMMITER_POWER         99     // 2 bytes (optional)
#define REGISTRY_OUTPUT_POWER          107     // 2 bytes (optional)

enum GfSun2000_Work_Mode {
  BASIC_MODE = 0,
  DETAIL_MODE = 1
};

struct GfSun2000Data {
    char deviceID[11];
    double ACVoltage;
    double DCVoltage;
    double outputPower;
    double limmiterPower;
    double customEnergyCounter;
    double totalEnergyCounter;
    double temperature;
    std::map<int16_t, int16_t> modbusRegistry;
};

typedef void (*GfSun2000OnData) (GfSun2000Data data);
typedef void (*GfSun2000OnError) (int errorId, char* errorMessage);

class GfSun2000 {
public:        
    void setup(HardwareSerial& serial, int8_t rtsPin = -1, int8_t remoteNumber = 1);
    bool readData();
    void setDataHandler(GfSun2000OnData handler);      
    void setErrorHandler(GfSun2000OnError handler);      
};

#endif