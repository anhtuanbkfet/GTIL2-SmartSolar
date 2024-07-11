#ifndef __MODELS_H__
#define __MODELS_H__

#define CMD_UPDATE_METRICS  "update_device_metrisc"
#define CMD_PRINT_DEBUG_LOG "print_device_log"
#define CMD_CHANGE_SETTING  "change_setting"
#define CMD_UPDATE_STATUS   "update_status"

struct DataStream{
  String name;
  double value;

  DataStream(String name, double value){
    this->name = name;
    this->value = value;
  }
};

struct StatusLog{
  String command;
  String deviceGuid;
  String firmwareVersion;
  int signalQuality;
  std::vector<DataStream> dataStreams;
};

#endif