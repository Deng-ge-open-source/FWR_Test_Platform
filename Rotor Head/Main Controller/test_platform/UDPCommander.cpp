#include "UDPCommander.h"

WiFiUDP Udp;//创建Udp
//接收Udp数据储存
String readUARt="";//串口数据储存
unsigned int location ;//通讯端口
IPAddress subnet(255,255,255,0);//网络掩码


int start(void)
{
  int first=0;
  while(1)
  {
    first=Udp.parsePacket();    //接收数据包
    if(first)
    {
      char rec[50];
      Serial.printf("first %d IP%s \n",first,Udp.remoteIP().toString().c_str());
      Udp.read(rec,255);
      if(rec[0]=='S')
      {
        return 1;
      }
      Serial.println(rec);
    }
  }
}
void WiFi_init(IPAddress ipClient, IPAddress ipServidor,unsigned int loca)
{
  
  location=loca;
  //ipClient=MYID;
  WiFi.mode(WIFI_STA);//WIFI设置为STA站点
  WiFi.begin(ssid,password);//连接WiFi
    // Serial.println("正在连接");
  while (WiFi.status()!=WL_CONNECTED)//等待
  {
    delay(500);
    Serial.print("..");
  }
  Serial.println("connected\n");
  WiFi.config(ipClient,ipServidor,subnet);
  Udp.begin(location);
//  if(start())
//  {
//     Serial.println("Start\n");
//  }
//  else 
//  {
//    Serial.println("fail\n");
//  }
}

bool isSentinel(char ch)
{
  if(ch == '\0')
    return true;
  else if (ch == '\r')
  {
     // printVerbose(F("Warn: \\r detected! \n"));
  }
  return false;
}

//为扑翼额外添加
//接受角度指令
char *output_d;
float revcommand(void)
{

    if(Udp.parsePacket())
    {
        char rec[80]="";
        Udp.read(rec,80);
        //Serial.println("receive");
        if(rec[0]=='T')
        {
          char *d=&rec[1];
          //char d1=rec[4];
          //output_d=d;
          //Serial.println(String(d));
            sscanf(d, "%f %f %f %f %f %f",  &servo_L_1, &servo_L_2, &servo_L_3, &servo_R_1, &servo_R_2, &servo_R_3);
        }
     }
     return 1;
}


//发送姿态数据-Z轴
void SendAttitude(IPAddress ipClient, IPAddress ipServidor,String read_1,String read_2)
{
   const char * replyPacket1 = read_1.c_str();
   const char * replyPacket2 = read_2.c_str();

   Udp.beginPacket(ipServidor,location);
   Udp.printf("65");
   Udp.printf("\t");
   Udp.printf(replyPacket1);
   Udp.printf("\t"); 
   Udp.printf(replyPacket2);
   Udp.printf("\t"); 
   Udp.printf("65");
   Udp.printf("\t");
   Udp.endPacket(); 
}


void getpackey(FOCMotor* motor,FOCMotor* motor1,IPAddress ipClient, IPAddress ipServidor,int m)
{


    if(Udp.parsePacket())
    {
        char rec[50]="";
        Udp.read(rec,50);
        //Serial.println("receive");
        if(rec[0]=='A')
        {
          char *d=&rec[1];
          if(m==1)
          {
            Udp.beginPacket(ipServidor,location);
            Udp.printf("65");
            Udp.printf("\t");
            UDPmotor(motor,d);
            Udp.endPacket(); 
          }
          else
          {
            UDPmotor(motor,d);
          }
        }
        else if(rec[0]=='B')
        {
          char *d=&rec[1];
          if(m==1)
          {
            Udp.beginPacket(ipServidor,location);
            Udp.printf("66");
            Udp.printf("\t");
            UDPmotor(motor1,d);
            Udp.endPacket();            
          }
          else
          {
            UDPmotor(motor1,d);
          }

        }
    }


}

void UDPmotor(FOCMotor* motor, char* user_command)
{
  // if target setting
  if(isDigit(user_command[0]) || user_command[0] == '-' || user_command[0] == '+')
  {
    //Udp.printf("Target:");
    motor->target = atof(user_command);
    //Udp.println(motor->target);
    return;
  }
  // parse command letter
  char cmd = user_command[0];
  char sub_cmd = user_command[1];
  // check if there is a subcommand or not
  int value_index = (sub_cmd >= 'A'  && sub_cmd <= 'Z') ?  2 :  1;
  // check if get command
  bool GET = isSentinel(user_command[value_index]);
  // parse command values
  float value = atof(&user_command[value_index]);
  // a bit of optimisation of variable memory for Arduino UNO (atmega328)
  switch(cmd){
    case CMD_C_Q_PID:      //  
      Udp.printf("PID curr q| ");
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_q, &user_command[1]);
      else pid(&motor->PID_current_q,&user_command[1]);
      break;
    case CMD_C_D_PID:      //
      Udp.printf("PID curr d| ");
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_d, &user_command[1]);
      else pid(&motor->PID_current_d, &user_command[1]);
      break;
    case CMD_V_PID:      //
      Udp.printf("PID vel| ");
      
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_velocity, &user_command[1]);
      else pid(&motor->PID_velocity, &user_command[1]);
      break;
    case CMD_A_PID:      //
     Udp.printf("PID angle| ");
      
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_angle, &user_command[1]);
      else pid(&motor->P_angle, &user_command[1]);
      break;
    case CMD_LIMITS:      //
      Udp.printf("Limits| ");
      switch (sub_cmd){
        case SCMD_LIM_VOLT:      // voltage limit change
          Udp.printf("volt: ");
          if(!GET) {
            motor->voltage_limit = value;
            motor->PID_current_d.limit = value;
            motor->PID_current_q.limit = value;
            // change velocity pid limit if in voltage mode and no phase resistance set
            if( !_isset(motor->phase_resistance) && motor->torque_controller==TorqueControlType::voltage) motor->PID_velocity.limit = value;
          }
          Udp.println(motor->voltage_limit);
          break;
        case SCMD_LIM_CURR:      // current limit
          Udp.printf("curr: ");
          if(!GET){
            motor->current_limit = value;
            // if phase resistance specified or the current control is on set the current limit to the velocity PID
            if(_isset(motor->phase_resistance) || motor->torque_controller != TorqueControlType::voltage ) motor->PID_velocity.limit = value;
          }
          Udp.println(motor->current_limit);
          break;
        case SCMD_LIM_VEL:      // velocity limit
           Udp.printf("vel: ");
          if(!GET){
            motor->velocity_limit = value;
            motor->P_angle.limit = value;
          }
          Udp.println(motor->velocity_limit);
          break;
        default:
          Udp.printf("err");
          break;
      }

      break;
    case CMD_MOTION_TYPE:
      Udp.printf("Motion:");
      switch(sub_cmd){
        case SCMD_DOWNSAMPLE:
            Udp.printf("downsample: ");
            if(!GET) motor->motion_downsample = value;
            Udp.println(motor->motion_downsample);
          break;
        default:
          // change control type
          if(!GET && value >= 0 && (int)value < 5) // if set command
            motor->controller = (MotionControlType)value;
          switch(motor->controller){
            case MotionControlType::torque:
               Udp.printf("torque");
               Serial.println("torque");
              break;
            case MotionControlType::velocity:
               Udp.printf("vel");
               Serial.println("vel");
              break;
            case MotionControlType::angle:
               Udp.printf("angle");
               Serial.println("angle");
              break;
            case MotionControlType::velocity_openloop:
               Udp.printf("vel open");
               Serial.println("vel open");
              break;
            case MotionControlType::angle_openloop:
               Udp.printf("angle open");
               Serial.println("ang open");
              break;
          }
            break;
        }
      break;
    case CMD_TORQUE_TYPE:
      // change control type
        Udp.printf("Torque: ");
      if(!GET && (int8_t)value >= 0 && (int8_t)value < 3)// if set command
        motor->torque_controller = (TorqueControlType)value;
      switch(motor->torque_controller){
        case TorqueControlType::voltage:
           Udp.printf("volt");
          break;
        case TorqueControlType::dc_current:
           Udp.printf("dc curr");
          break;
        case TorqueControlType::foc_current:
           Udp.printf("foc curr");
          break;
      }
      break;
    case CMD_STATUS:
      // enable/disable
       Udp.printf("Status:");
      if(!GET) (bool)value ? motor->enable() : motor->disable();
       Udp.println(motor->enabled);
      break;
    case CMD_PWMMOD:
       // PWM modulation change
        Udp.printf("PWM Mod | ");
       switch (sub_cmd){
        case SCMD_PWMMOD_TYPE:      // zero offset
            Udp.printf("type:");
          
          if(!GET) motor->foc_modulation = (FOCModulationType)value;
          switch(motor->foc_modulation){
            case FOCModulationType::SinePWM:
               Udp.printf("SinePWM");
              break;
            case FOCModulationType::SpaceVectorPWM:
                Udp.printf("SVPWM");
              break;
            case FOCModulationType::Trapezoid_120:
              Udp.printf("Trap 120");
              break;
            case FOCModulationType::Trapezoid_150:
              Udp.printf("Trap 150");
              break;
          }
          break;
        case SCMD_PWMMOD_CENTER:      // centered modulation
          Udp.printf("center: ");
          if(!GET) motor->modulation_centered = value;
           Udp.println(motor->modulation_centered);
          break;
        default:
           Udp.printf("err");
          break;
       }
      break;
    case CMD_RESIST:
      // enable/disable
      Udp.printf("R phase:");
      if(!GET){
        motor->phase_resistance = value;
        if(motor->torque_controller==TorqueControlType::voltage){
          motor->voltage_limit = motor->current_limit*value;
          motor->PID_velocity.limit= motor->current_limit;
        }
      }
      if(_isset(motor->phase_resistance))
      Udp.println(motor->phase_resistance);
      break;
    case CMD_SENSOR:
      // Sensor zero offset
       Udp.printf("Sensor | ");
       switch (sub_cmd){
        case SCMD_SENS_MECH_OFFSET:      // zero offset
          Udp.printf("offset: ");
          if(!GET) motor->sensor_offset = value;
          Udp.println(motor->sensor_offset);
          break;
        case SCMD_SENS_ELEC_OFFSET:      // electrical zero offset - not suggested to touch
          Udp.printf("el. offset: ");
          if(!GET) motor->zero_electric_angle = value;
          Udp.println(motor->zero_electric_angle);
          break;
        default:
         Udp.printf("err");
          break;
       }
      break;
    case CMD_MONITOR:     // get current values of the state variables
      Udp.printf("Monitor | ");
      switch (sub_cmd){
        case SCMD_GET:      // get command
          switch((uint8_t)value){
            case 0: // get target
              Udp.printf("target: ");
              Udp.println(motor->target);
              break;
            case 1: // get voltage q
                Udp.printf("Vq: ");
                Udp.println(motor->voltage.q);
              break;
            case 2: // get voltage d
               Udp.printf("Vd: ");
               Udp.println(motor->voltage.d);
              break;
            case 3: // get current q
               Udp.printf("Cq: ");
               Udp.println(motor->current.q);
              break;
            case 4: // get current d
               Udp.printf("Cd: ");
               Udp.println(motor->current.d);
              break;
            case 5: // get velocity
                Udp.printf("vel: ");
                Udp.println(motor->shaft_velocity);
              break;
            case 6: // get angle
              Udp.printf("angle: ");
              Udp.println(motor->shaft_angle);
              break;
            case 7: // get all states
             Udp.print("all: ");  
             Udp.print(motor->target);
             Udp.print(";");
             Udp.print(motor->voltage.q);
             Udp.print(";");
             Udp.print(motor->voltage.d);
             Udp.print(";");
             Udp.print(motor->current.q);
             Udp.print(";");
             Udp.print(motor->current.d);
             Udp.print(";");
             Udp.print(motor->shaft_velocity);
             Udp.print(";");
             Udp.println(motor->shaft_angle);
              break;
            default:
              Udp.printf("err");
              break;
          }
          break;
        case SCMD_DOWNSAMPLE:
          Udp.printf("downsample: ");
          if(!GET) motor->monitor_downsample = value;
          Udp.println(motor->monitor_downsample);
          break;
        case SCMD_CLEAR:
          motor->monitor_variables = (uint8_t) 0;
          Udp.printf("clear");
          break;
        case SCMD_SET:
          if(!GET) motor->monitor_variables = (uint8_t) 0;
          for(int i = 0; i < 7; i++){
            if(isSentinel(user_command[value_index+i])) break;
            if(!GET) motor->monitor_variables |=  (user_command[value_index+i] - '0') << (6-i);
             Udp.println((user_command[value_index+i] - '0'));
          }
           Udp.printf(" ");
          break;
        default:
           Udp.printf("err");
          break;
       }
      break;
    default:  // unknown cmd
     break;
    Udp.printf("err");
  }
}


void lpf(LowPassFilter* lpf, char* user_cmd){
  char cmd = user_cmd[0];
  bool GET  = isSentinel(user_cmd[1]);
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_LPF_TF:      // Tf value change
      Udp.printf("Tf: ");
      if(!GET) lpf->Tf = value;
      Udp.println(lpf->Tf);
      break;
    default:
      Udp.printf("err");
      break;
  }
}


void pid(PIDController* pid, char* user_cmd){
  char cmd = user_cmd[0];
  bool GET  = isSentinel(user_cmd[1]);
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_PID_P:      // P gain change
      Udp.printf("P:"); 
      if(!GET) pid->P = value;
      Udp.println(pid->P);
      break;
    case SCMD_PID_I:      // I gain change
      Udp.printf("I:");
      if(!GET) pid->I = value;
      Udp.println(pid->I);
      break;
    case SCMD_PID_D:      // D gain change
      Udp.printf("D:");
      if(!GET) pid->D = value;
      Udp.println(pid->D);
      break;
    case SCMD_PID_RAMP:      //  ramp change
      Udp.printf("ramp:");
      if(!GET) pid->output_ramp = value;
      Udp.println(pid->output_ramp);
      break;
    case SCMD_PID_LIM:      //  limit change
      Udp.printf("limit:");
      if(!GET) pid->limit = value;
      Udp.println(pid->limit);
      break;
    default:
      Udp.printf("err");
      break;
  }
}


unsigned int downsample=0;
unsigned int cnt = 0 ;
uint8_t variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
CurrentSense* sense;
float electrical_angle;
DQCurrent_s current;
TorqueControlType controller;

void UDPmonitor(FOCMotor* motor,IPAddress ipClient, IPAddress ipServidor, char motor_id) {
  Udp.beginPacket(Udp.remoteIP(),Udp.remotePort());
  //Udp.beginPacket(ipServidor,location);
  downsample=motor->monitor_downsample;
  variables=motor->monitor_variables;
  //Serial.println(variables);
  float toprint[50];
  int i=0;
  if( !downsample || cnt++ < downsample ) return;
  cnt = 0;
  Udp.print(motor_id,1);
  Udp.print("\t");
  if(variables & _MON_TARGET){
  Udp.print(motor->target,4);
  Udp.print("\t");
  // toprint[i]=motor->target;
  // i++;
  //  monitor_port->print("\t");
  }
  if(variables & _MON_VOLT_Q) {
   // monitor_port->print(voltage.q,4);
    Udp.print(motor->voltage.q,4);
    Udp.print("\t");
  //  toprint[i]=motor->voltage.q;
  //  i++;
   // monitor_port->print("\t");
  
  }
  if(variables & _MON_VOLT_D) {
   // monitor_port->print(voltage.d,4);
    Udp.print(motor->voltage.d,4);
    Udp.print("\t");    
  //  toprint[i]=motor->voltage.d;
  //  i++;
  //  monitor_port->print("\t");

  }
  // read currents if possible - even in voltage mode (if current_sense available)
  if(variables & _MON_CURR_Q || variables & _MON_CURR_D) {
    DQCurrent_s c{0,0};
    if(sense){
      if(controller == TorqueControlType::foc_current) c = current;
      else {
        c = sense->getFOCCurrents(electrical_angle);
        // c.q = LPF_current_q(c.q);
        // c.d = LPF_current_d(c.d);// 没有对电流进行滤波
        }
    }
    if(variables & _MON_CURR_Q) {
    //  monitor_port->print(c.q*1000,2); // mAmps
    //  UDPF(c.q*1000);
        Udp.print(motor->current.q,4);
        Udp.print("\t");
      // toprint[i]=c.q*1000;
      // i++;
    //  monitor_port->print("\t");

    }
    if(variables & _MON_CURR_D) {
    //  monitor_port->print(c.d*1000,2); // mAmps
   // UDPF(c.d*1000);
    //  monitor_port->print("\t");
        Udp.print(motor->current.d,4);
        Udp.print("\t");
    // toprint[i]=c.d*1000;
    // i++;
    } 
  }
  
  if(variables & _MON_VEL) {
   // monitor_port->print(shaft_velocity,4);
   //UDPF(motor->shaft_velocity);
        Udp.print(motor->shaft_velocity,4);
        Udp.print("\t");
  //  toprint[i]=motor->shaft_velocity;
  //  i++;
   // monitor_port->print("\t");

  }
  if(variables & _MON_ANGLE) {
  //  monitor_port->print(shaft_angle,4);
  // UDPF(motor->shaft_angle);
        Udp.print(motor->shaft_angle,4);
        Udp.print("\t");
  //  toprint[i]=motor->shaft_angle;
  //  i++;
  }
  // if(i!=0)
  // {
  //   Udp.beginPacket(ipServidor,location);
  //   for(int j=0;j<i;j++)
  //   {
  //     Udp.printf("%.4f\t",toprint[j]);  
  //   }
  //   Udp.printf("\n");    
  // }

  Udp.print(motor_id,1); //New add
  Udp.print("\t");       //New add
  
  Udp.endPacket();
}
