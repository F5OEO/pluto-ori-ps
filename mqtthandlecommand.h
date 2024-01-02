#include <mosquitto.h>
extern bool HandleCommand(char *key,char *svalue);
extern void HandleCommandInit(struct mosquitto *mosq,char *sSerial);
extern  bool publish(char *mqttkey,float value);
extern bool publish(char *mqttkey,char *svalue);
bool publishstatus(size_t device,char *iio_key, char *mqttkey);
extern void PubTelemetry();