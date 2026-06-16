extern double GyroX_lowout;         //角速度
extern double kalAngleX;
//直立环参数100 14
float Vertical_Kp = 100.0,Vertical_Kd = 14.0;
float Angle_mid = 0;//机械中值
int16_t PWM_out,Vertical_PWM,Velocity_PWM;

//速度环参数250 0.01
float Velocity_Kp = 250.0, Velocity_Ki=0.01;
double Encoder_Err=0, Encoder_Sum=0;
double Encoder_lowout=0;


int16_t PIDControl(){
  /****************************************************************
     速度环PI控制器：
****************************************************************/ 
  //1.计算速度偏差 Encoder_Err
  Encoder_Err  = 16666.6/speedTCounter;
  //Stimer = micros();
  //2.对速度偏差进行低通滤波//low_out = (1-a)*Ek+a*low_out_last;
  Encoder_lowout = 0.3*Encoder_Err + 0.7*Encoder_lowout;
  //Encoder_lowout = Encoder_Err;
  //3.对速度偏差,积分出位移
  Encoder_Sum += Encoder_Err;
  if(Encoder_Sum>10000){
    Encoder_Sum = 10000;
  }
  else if(Encoder_Sum<-10000){
    Encoder_Sum = -10000;
  }
  //4.速度环控制输出——期望角度
  Velocity_PWM = (int16_t)Velocity_Kp*Encoder_lowout+Velocity_Ki*Encoder_Sum;

/****************************************************************
     直立环PD控制器：
****************************************************************/ 

  Vertical_PWM = (int16_t)Vertical_Kp*(kalAngleX - Angle_mid) + Vertical_Kd * GyroX_lowout;

  PWM_out = Vertical_PWM + Velocity_PWM;

  if(PWM_out > 790){
      PWM_out = 790;
  }
  else if(PWM_out < -790){
      PWM_out = -790;
  }
  return PWM_out;
}
