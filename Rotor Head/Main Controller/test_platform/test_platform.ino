#include "UDPCommander.h"
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <SimpleFOC.h>
#include <Adafruit_PWMServoDriver.h>

double angle_servo=0;

//PWM 硬件指定 PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

//AS5600硬件指定
MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(1);

//MPU6050 硬件指定
//TwoWire I2Ctwo = TwoWire(1);
//MPU6050 mpu6050(I2Ctwo);

//UDP 连接
unsigned int loca = 2316;//通讯端口
const char *password="Dtvycg7hhyc8";//wifi密码
const char *ssid = "VM278BC8";//wifi名称


//const char *password="12345678";//wifi密码
//const char *ssid = "TESTXUN";//wifi名称

float servo_L_1=83,servo_L_2=100,servo_L_3=180,servo_R_1=85,servo_R_2=83,servo_R_3=0;

IPAddress ipServidor(192,168,0,29);//电脑IP(要链接的IP)   //Origin
IPAddress ipClient(192,168,0,52);//自己的IP

//IPAddress ipServidor(192,168,137,1);//电脑IP(要链接的IP)   //Uni Show
//IPAddress ipClient(192,168,137,103);//自己的IP

void setup() {
  delay(3000);
  Serial.begin(115200);
  Wire.begin(18,19,100000UL);
  //启用AS5600
  I2Cone.begin(21,22, 400000UL);   //SDA0,SCL0
  sensor0.init(&I2Cone);
  //启用 MPU6050
//  I2Ctwo.begin(21,22,400000UL);
//  mpu6050.begin();
//  mpu6050.calcGyroOffsets(true);
  delay(1000);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(100);
  WiFi_init(ipClient,ipServidor,loca);
  delay(300);
  
}

void loop() {
  //mpu6050.update();
  sensor0.update();
  //Serial.println(sensor0.getAngle()); 
  //Serial.println(mpu6050.getGyroZ());
  //3Serial.println(mpu6050.getAngleZ());
  //SendAttitude(ipClient,ipServidor,String(mpu6050.getAngleZ()),String(mpu6050.getGyroZ()));   //发送姿态数据MPU6050
  SendAttitude(ipClient,ipServidor,String(sensor0.getAngle()),String(sensor0.getVelocity()));   //发送姿态数据AS5600
  revcommand();  //接受UDP角度指令
 
// Serial.print(servo_L_1);
// Serial.print("          ");
//  Serial.print(servo_L_2);
// Serial.print("          ");
//  Serial.print(servo_L_3);
// Serial.print("          ");
//  Serial.print(servo_R_1);
// Serial.print("          ");
//  Serial.print(servo_R_2);
// Serial.print("          ");
//  Serial.print(servo_R_3);
// Serial.println("");

  //左翼

  pwm.setPWM(14, 0, 2.227*(servo_L_1) + 90);    //左,Beta角60°
  pwm.setPWM(13, 0, 2.227*(servo_L_2) + 90);    //左扑动
  pwm.setPWM(12, 0, 2.227*(servo_L_3) + 90); 
  //右翼
  pwm.setPWM(1, 0, 2.227*(servo_R_1) + 90);    //左,Beta角60°
  pwm.setPWM(2, 0, 2.227*(servo_R_2) + 90);    //右扑动
  pwm.setPWM(3, 0, 2.227*(servo_R_3) + 90); 
  
}
