#ifndef __UDPCommander_H
#define __UDPCommander_H
#include <SimpleFOC.h>
#include <WiFi.h>
#include <WiFiUdp.h>

extern const char *password;//wifi密码
extern const char *ssid ;//wifi名称
extern float servo_L_1,servo_L_2,servo_L_3,servo_R_1,servo_R_2,servo_R_3;

void UDPF(float send);
void UDPC(String send);

int start(void);
void WiFi_init(IPAddress ipClient, IPAddress ipServidor,unsigned int loca);
void lpf(LowPassFilter* lpf, char* user_cmd);
void pid(PIDController* pid, char* user_cmd);
void UDPmotor(FOCMotor* motor, char* user_command);
float revcommand(void);
void SendAttitude(IPAddress ipClient, IPAddress ipServidor,String read_1,String read_2);
void getpackey(FOCMotor* motor,FOCMotor* motor1,IPAddress ipClient, IPAddress ipServidor,int m);
void SendWeight(IPAddress ipClient, IPAddress ipServidor,String read_1,String read_2,String read_3,String read_4,String sum_total);
void UDPmonitor(FOCMotor* motor,IPAddress ipClient, IPAddress ipServidor, char motor_id); 

#endif
