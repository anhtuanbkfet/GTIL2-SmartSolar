#ifndef __MODELS_H__
#define __MODELS_H__

#define CMD_UPDATE_LOG      "updateDeviceLog"
#define CMD_CHANGE_SETTING  "change_setting"


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
  std::vector<DataStream> dataStreams;
};

#endif