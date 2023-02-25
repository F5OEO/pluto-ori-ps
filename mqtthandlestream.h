#include <mosquitto.h>
extern bool HandleCommand(char *key,char *svalue);
extern void HandleCommandInit(struct mosquitto *mosq,char *sSerial);
extern  bool publish(char *mqttkey,float value,bool isstatus);
extern bool publish(char *mqttkey,char *svalue,bool isstatus);
extern bool publishstatus(char *iio_key,char *mqttkey);
extern bool HandleStatus(char *key,char *svalue);
extern void PubTelemetry();
void *rx_buffer_thread(void *arg);