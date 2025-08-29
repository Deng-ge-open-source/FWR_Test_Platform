#include "UDPCommander.h"
#include "HX711.h"

//UDP 连接
unsigned int loca = 2318;//通讯端口
const char *password="Dtvycg7hhyc8";//wifi密码
const char *ssid = "VM278BC8";//wifi名称

//const char *password="12345678";//wifi密码
//const char *ssid = "TESTXUN";//wifi名称

IPAddress ipServidor(192,168,0,29);//电脑IP(要链接的IP)
IPAddress ipClient(192,168,0,51);//自己的IP  //填入Simulink接收Block的IP!

//IPAddress ipServidor(192,168,137,1);//电脑IP(要链接的IP)
//IPAddress ipClient(192,168,137,19);//自己的IP


// HX711 circuit wiring
const int DOUT_PIN_1 = 26;
const int SCK_PIN_1 = 27;

const int DOUT_PIN_2 = 25;
const int SCK_PIN_2 = 33;

const int DOUT_PIN_3 = 22;
const int SCK_PIN_3 = 23;

const int DOUT_PIN_4 = 19;
const int SCK_PIN_4 = 21;


HX711 scale1;
HX711 scale2;
HX711 scale3;
HX711 scale4;

void setup() {
  Serial.begin(115200);
  Serial.println("Hx711 FWR Plat");
  scale1.begin(DOUT_PIN_1, SCK_PIN_1);
  scale2.begin(DOUT_PIN_2, SCK_PIN_2);
  scale3.begin(DOUT_PIN_3, SCK_PIN_3);
  scale4.begin(DOUT_PIN_4, SCK_PIN_4);

  //初始化WIFI
  WiFi_init(ipClient,ipServidor,loca);
  _delay(1000);

  //Reset Scale
//  scale1.tare();
//  scale2.tare();
//  scale3.tare();
//  scale4.tare();

}

double read_1_do=0,read_2_do=0,read_3_do=0,read_4_do=0,sum_total=0;

void loop() {
   long reading1 = scale1.read();
   long reading2 = scale2.read();
   long reading3 = scale3.read();
   long reading4 = scale4.read();
   //数据前处理
   read_1_do=0.00294*(reading1-30200)-1.87;
   read_2_do=0.00294*(reading2+4500)-1.87;
   read_3_do=0.00294*(reading3-207200)-1.87;
   read_4_do=0.00294*(reading4+88700)-1.87;
   sum_total=read_1_do+read_2_do+read_3_do+read_4_do;
   SendWeight(ipClient,ipServidor,String(read_1_do),String(read_2_do),String(read_3_do),String(read_4_do),String(sum_total));
//输出初始值
//   Serial.print(reading1);
//   Serial.print("      ");
//   Serial.print(reading2);
//   Serial.print("      ");
//   Serial.print(reading3);
//   Serial.print("      ");
//   Serial.println(reading4);

   Serial.print(read_1_do);
   Serial.print("      ");
   Serial.print(read_2_do);
   Serial.print("      ");
   Serial.print(read_3_do);
   Serial.print("      ");
   Serial.print(read_4_do);
   Serial.print("      ");
   Serial.println(sum_total);
 
  
}
