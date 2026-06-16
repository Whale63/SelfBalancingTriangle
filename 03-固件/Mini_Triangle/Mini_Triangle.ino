//无刷电机排线信号线
#define EncoderA        2       //编码器A相，连接到中断0，数字引脚2
#define EncoderB_DIR    3       //编码器B相，判断方向
#define Motor_EN_BIT    4       // 电机使能引脚，高电平启动
#define Motor_BK_BIT    5       // 电机刹车
#define Motor_DIR_BIT   6       // 电机正反转切换
#define Motor_PWM_BIT   10      // Digital Pin 10设置PWM输出引脚为Pin10，调整PWM占空比，控制电机转速

//3个平衡位置的机械中值角度
#define MID_ANGLE1      0
#define MID_ANGLE2      114
#define MID_ANGLE3      245

//自平衡程序和摇摆程序的划分角度，当前位置与机械中值的差值小于LIMIT_ANGLE1，进入PID自平衡程序，否则进入摇摆程序
#define LIMIT_ANGLE1    20

//引入外部函数和变量
extern void GetXAngle();        //获取当前角度
extern double kalAngleX;        //当前角度
extern double GyroX_lowout;     //当前角速度

extern int16_t PIDControl();    //PID计算
extern float Vertical_Kp,Vertical_Kd,Velocity_Kp,Velocity_Ki;     //PID参数，通过串口通讯，调整PID参数
extern float Angle_mid;         //机械中值
extern double Encoder_Sum;      
extern double Encoder_lowout;   

int32_t speedTCounter = 0;    //用于计算电机转速
int32_t speedTCounterLast;

float gyroXLowoutLast = 0;    
int16_t tempPWM_out = 0;      //PWM过渡变量
String inputString = "";      // a String to hold incoming data

char buf[8];                  //OLED显示角度

int delayTime = 0;

int sendFlag = 1;
/****************************************************************
                     LGT8F328P单片机初始化设置函数
****************************************************************/ 
void MPUInitial(){            
  pinMode(Motor_EN_BIT, OUTPUT);        //设置电机使能引脚为输出模式；
  digitalWrite(Motor_EN_BIT, LOW);       //关闭电机
  
  pinMode(Motor_DIR_BIT, OUTPUT);       //设置电机方向引脚为输出模式；
  pinMode(EncoderB_DIR, INPUT_PULLUP);  //设置编码器B相引脚为输入上拉模式；
  
  pinMode(Motor_BK_BIT, OUTPUT);        //设置电机刹车引脚为输出模式；
  digitalWrite(Motor_BK_BIT, LOW);      //关闭刹车

  /****************************************************************
    设置Timer1为PWM输出定时器，输出引脚Pin10,输出PWM控制电机转动
  ****************************************************************/ 
  cli();                                // 禁止全局中断
  pinMode(Motor_PWM_BIT, OUTPUT);       //设置PWM引脚Pin3为输出模式
  
  //设置Timer1为快速PWM模式，计数上限OCR1A；发生比较匹配时OC1B清零（低电平），计数到下限时OC1B置位（高电平）；系统时钟1分频
  TCCR1A = (1<<COM1B1) | (1<<WGM11) | (1<<WGM10);
  TCCR1B = (1<<WGM13)  | (1<<WGM12) | (1<<CS10);    // 0b00011001; 
  OCR1A = 799;                                      //设置计数器上限，输出频率20kHz
  OCR1B = 0;                                        //用作比较器，设置PWM占空比  OCR1B = 0电机转速为0
  /****************************************************************
    初始化外部中断0，下降沿触发，调用counter()函数
  ****************************************************************/
  pinMode(EncoderA, INPUT_PULLUP);      //设置编码器A相引脚为输入上拉模式；
  attachInterrupt(0, counter, FALLING); //设置编码器A中断绑定到counter()函数中；
  sei();                                // 允许全局中断
}

/****************************************************************
     控制电机转速
****************************************************************/ 
void PWM_OutPut(){

  //根据当前角度判断是进入平衡程序还是弹跳程序

  //PID平衡程序
  if(abs(kalAngleX-MID_ANGLE1)<LIMIT_ANGLE1)
  {
    //确定机械中值
    Angle_mid = MID_ANGLE1;
    //将数据输入闭环控制中，计算控制输出量
    tempPWM_out = PIDControl();
    delay(delayTime);
  }
  else if(abs(kalAngleX-MID_ANGLE2)<LIMIT_ANGLE1)
  {
    
    Angle_mid = MID_ANGLE2;
    tempPWM_out = PIDControl();
    delay(delayTime);
  }
  else if(abs(kalAngleX-MID_ANGLE3)<LIMIT_ANGLE1)
  {
    
    Angle_mid = MID_ANGLE3;
    tempPWM_out = PIDControl();
    delay(delayTime);
  }
  
  else  //弹跳程序
  {  
    int break_Flag = 0;                    //跳跃平衡失败判断标志
    tempPWM_out = 0;                       //速度设为0
    digitalWrite(Motor_BK_BIT, HIGH);      //刹车
    delay(500);
    digitalWrite(Motor_BK_BIT, LOW);      //关闭刹车  

    if( kalAngleX > 35 && kalAngleX < 48) //角度在35~45度之间，进入弹跳
    {
      digitalWrite(Motor_DIR_BIT, HIGH);
      OCR1B = 200; 
      for(int i = 0; i < 500 ; i++){
        OCR1B = OCR1B+1;
        delay(10);
      }
      delay(1000);
      digitalWrite(Motor_BK_BIT, HIGH);      //刹车
      delay(10);
      digitalWrite(Motor_DIR_BIT, LOW);
      OCR1B = 600;  
      digitalWrite(Motor_BK_BIT, LOW);      //关闭刹车   
                
      while((abs(kalAngleX-MID_ANGLE1)>LIMIT_ANGLE1)){
        GetXAngle();
        break_Flag++;
        if(break_Flag>=1000) break;//跳跃平衡失败
      }
    
    }
    else if(kalAngleX > -50 && kalAngleX < -33)
    {
      digitalWrite(Motor_DIR_BIT, LOW);
      OCR1B = 200; 
      for(int i = 0; i < 500 ; i++){
        OCR1B = OCR1B+1;
        delay(10);
      }
      delay(1000);
      digitalWrite(Motor_BK_BIT, HIGH);      //刹车
      delay(10);
      digitalWrite(Motor_DIR_BIT, HIGH);
      OCR1B = 600;  
      digitalWrite(Motor_BK_BIT, LOW);      //关闭刹车   
                
      while((abs(kalAngleX-MID_ANGLE1)>LIMIT_ANGLE1)){
        GetXAngle();
        break_Flag++;
        if(break_Flag>=1000) break;
      }   
    }

  }

  //将控制输出加载到电机上，控制电机输出
  if(tempPWM_out>0){//电机正转-顺时针
      digitalWrite(Motor_DIR_BIT, LOW);
      OCR1B = tempPWM_out; 
  }
  else{
    digitalWrite(Motor_DIR_BIT, HIGH);
    OCR1B = -tempPWM_out; 
  }
}

void setup() {
  Serial.begin(9600);                   //初始化串口   
  delay(100);

  Mpu6050init();                        //初始化MPU6050
  MPUInitial();                         //初始化配置单片机
  digitalWrite(Motor_EN_BIT, HIGH);     //使能电机
  digitalWrite(Motor_BK_BIT, LOW);      //关闭刹车
  digitalWrite(Motor_DIR_BIT, HIGH);    //设置电机方向
  
  Angle_mid = MID_ANGLE3;
  speedTCounterLast = micros();
  Serial.println("OK");
  
  delay(100);
  OLED_Init();
  OLED_ColorTurn(0);//0正常显示 1反色显示
  OLED_DisplayTurn(0);//0正常显示 1翻转180度显示
  OLED_ShowString(0,1,"MINI_Triangle",16);
  OLED_ShowString(0,3,"xAngle:",16);
}

void loop() {
//put your main code here, to run repeatedly:
  GetXAngle();  
  PWM_OutPut();//确定机械中值时注释改行
  static int ii = 0;
  if(ii >=10)//OLED显示角度耗时较长，每十次PID刷新一次位姿
  {
    ii = 0;
    dtostrf(kalAngleX,6,1,buf);
    OLED_ShowString(64,3,buf,8);
  }
  ii++;
  
  /*
   //串口返回单片机数据，用于参数调试（数据线或蓝牙模块）
  switch(1) 
  {
    case 0:
      Serial.println(tempPWM_out);      //PWM
      break;
    case 1:
      Serial.println(kalAngleX);        //输出角度，确定机械中值
      break;
    case 2:
      Serial.println(GyroX_lowout);     //低通滤波的角速度
      break;
    case 3:
      Serial.println(Velocity_Kp);      //速度环kP
      break;
    case 4:
      Serial.println(Encoder_lowout);   //低通滤波的速度偏差
      break;
    case 5:
     Serial.println(Encoder_Sum);       //速度偏差积分
    default:
      break;
  }
//  delay(30);
*/
}

//外部中断0服务程序，测量电机速度
void counter() {
   if(digitalRead(EncoderB_DIR)){//顺时针为正方向
    
    speedTCounter =  speedTCounterLast - micros() ;  //用相邻两次中断的时间差计算转速
    speedTCounterLast = micros();
    
  }
  else{//逆时针
    
    speedTCounter =  micros() - speedTCounterLast ;
    speedTCounterLast = micros();
    
  }
}
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {//串口接受数据，用于PID调试，可以使用蓝牙模块进行无线调试，上位机发送数据的格式：Axx;xx;xx;xx;xx;xx;(xx为参数)，微信搜索“平衡小车蓝牙调试助手”，使用蓝牙进行无线调试
  while (Serial.available()) {
    static bool ReceiveFlag = false;
    static int ReceiveNum = 0;
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if(inChar == 'A'){
      ReceiveFlag = true;
    }
    if((inChar == ';')&&(ReceiveFlag)){
      ReceiveNum++;
    }
    // if the incoming character is a newline, set a flag so the main loop can do something about it:
    if (ReceiveNum == 6) {
      ReceiveNum = 0;
      ReceiveFlag = false;
      
      uint8_t len = inputString.length();
      char temp[20] = "";
      char buffer[len];
      strcpy(buffer,inputString.c_str());
      uint8_t m = 0;
      uint8_t n = 0;
      
      inputString = "";    
      if(buffer[0] == 'A'){
        for (uint8_t i = 1; i < len; ++i) {
          if (buffer[i] == ';') {
            switch (n) {
              case 0:
                Vertical_Kp  = atof(temp);
                break;
              case 1:
                sendFlag = (int)atof(temp);   //根据loop()函数，更改sendFlag的值，单片机返回不同的参数
                break;
              case 2:
                Vertical_Kd  = atof(temp);
                break;
              case 3:
                Velocity_Kp  = atof(temp);
                break;
              case 4:
                Velocity_Ki = atof(temp);
                break;
              case 5:
                delayTime = atof(temp);     //相邻两次PID计算的延时参数
              default:
                break;
            }
            memset(temp,0,sizeof(temp));
            m = 0;
            n++;
          } 
          else
          {
            temp[m] = buffer[i];
            m++;
          }
        }
      }
    }
  }
}
