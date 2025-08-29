#ifndef __UDPCommander_H
#define __UDPCommander_H
#include <SimpleFOC.h>
#include <WiFi.h>
#include <WiFiUdp.h>

extern const char *password;//wifi密码
extern const char *ssid ;//wifi名称

void UDPF(float send);
void UDPC(String send);

int start(void);
void WiFi_init(IPAddress ipClient, IPAddress ipServidor,unsigned int loca);
void lpf(LowPassFilter* lpf, char* user_cmd);
void pid(PIDController* pid, char* user_cmd);
void UDPmotor(FOCMotor* motor, char* user_command);
void getpackey(FOCMotor* motor,FOCMotor* motor1,IPAddress ipClient, IPAddress ipServidor,int m);
void SendWeight(IPAddress ipClient, IPAddress ipServidor,String read_1,String read_2,String read_3,String read_4,String sum_total);
void UDPmonitor(FOCMotor* motor,IPAddress ipClient, IPAddress ipServidor, char motor_id); 

#endif
