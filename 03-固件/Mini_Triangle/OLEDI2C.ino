
//////////////////////////////////////////////////////////////////////////////////   
//  功能描述   : arduino UNO OLED显示屏例程
//              说明: 
//              ----------------------------------------------------------------
//              GND  电源地
//              VCC  5v电源
//              D0   A5（SCL）
//              D1   A4（SDA）
//              RES  A3 注：SPI接口显示屏改成IIC接口时需要接RES引脚
//                           IIC接口显示屏用户请忽略       
//******************************************************************************/
#include "oledfont.h"
#include "Wire.h"

#define OLED_CMD  0  //写命令
#define OLED_DATA 1 //写数据

//反显函数
void OLED_ColorTurn(u8 i)
{
  if(!i) 
    OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
  else  
    OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
}

//屏幕旋转180度
void OLED_DisplayTurn(u8 i)
{
  if(i==0)
    {
      OLED_WR_Byte(0xC8,OLED_CMD);//正常显示
      OLED_WR_Byte(0xA1,OLED_CMD);
    }
else
    {
      OLED_WR_Byte(0xC0,OLED_CMD);//反转显示
      OLED_WR_Byte(0xA0,OLED_CMD);
    }
}

//发送一个字节
//向SSD1306写入一个字节。
//mode:数据/命令标志 0,表示命令;1,表示数据;
void OLED_WR_Byte(u8 dat,u8 mode)
{
  Wire.beginTransmission(0x3c);
  if(mode){Wire.write(0x40);}
  else{Wire.write(0x00);}
  Wire.write(dat); // sends one byte
  Wire.endTransmission();    // stop transmitting
}

//坐标设置
void OLED_Set_Pos(u8 x, u8 y) 
{ 
  OLED_WR_Byte(0xb0+y,OLED_CMD);
  OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
  OLED_WR_Byte((x&0x0f),OLED_CMD);
}       
//开启OLED显示    
void OLED_Display_On(void)
{
  OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
  OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
  OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
  OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
  OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
  OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}            
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!   
void OLED_Clear(void)  
{  
  u8 i,n;       
  for(i=0;i<8;i++)  
  {  
    OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
    OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
    OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
    for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
  } //更新显示
}
//在指定位置显示一个字符
//x:0~127
//y:0~63         
//sizey:选择字体 6x8  8x16
void OLED_ShowChar(u8 x,u8 y,const u8 chr,u8 sizey)
{       
  u8 c=0,sizex=sizey/2,temp;
  u16 i=0,size1;
  if(sizey==8)size1=6;
  else size1=(sizey/8+((sizey%8)?1:0))*(sizey/2);
  c=chr-' ';//得到偏移后的值
  OLED_Set_Pos(x,y);
  for(i=0;i<size1;i++)
  {
    if(i%sizex==0&&sizey!=8) OLED_Set_Pos(x,y++);
    if(sizey==8)
    {
      temp=pgm_read_byte(&asc2_0806[c][i]);
      OLED_WR_Byte(temp,OLED_DATA);//6X8字号
    }
    else if(sizey==16) 
    {
      temp=pgm_read_byte(&asc2_1608[c][i]);
      OLED_WR_Byte(temp,OLED_DATA);//8x16字号
    }
    else return;
  }
}
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
  u32 result=1;  
  while(n--)result*=m;    
  return result;
}         
//显示数字
//x,y :起点坐标
//num:要显示的数字
//len :数字的位数
//sizey:字体大小      
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 sizey)
{           
  u8 t,temp,m=0;
  u8 enshow=0;
  if(sizey==8)m=2;
  for(t=0;t<len;t++)
  {
    temp=(num/oled_pow(10,len-t-1))%10;
    if(enshow==0&&t<(len-1))
    {
      if(temp==0)
      {
        OLED_ShowChar(x+(sizey/2+m)*t,y,' ',sizey);
        continue;
      }else enshow=1;
    }
    OLED_ShowChar(x+(sizey/2+m)*t,y,temp+'0',sizey);
  }
}
//显示一个字符号串
void OLED_ShowString(u8 x,u8 y,const char *chr,u8 sizey)
{
  u8 j=0;
  while (chr[j]!='\0')
  {   
    OLED_ShowChar(x,y,chr[j++],sizey);
    if(sizey==8)x+=6;
    else x+=sizey/2;
  }
}
//OLED的初始化
void OLED_Init(void)
{
  Wire.begin(0x3c); // join i2c bus (address optional for master)
  delay(5);OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
  delay(5);OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
  delay(5);OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
  delay(5);OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  delay(5);OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
  delay(5);OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
  delay(5);OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
  delay(5);OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
  delay(5);OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
  delay(5);OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
  delay(5);OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
  delay(5);OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset Shift Mapping RAM Counter (0x00~0x3F)
  delay(5);OLED_WR_Byte(0x00,OLED_CMD);//-not offset
  delay(5);OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
  delay(5);OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
  delay(5);OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
  delay(5);OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  delay(5);OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
  delay(5);OLED_WR_Byte(0x12,OLED_CMD);
  delay(5);OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
  delay(5);OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
  delay(5);OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
  delay(5);OLED_WR_Byte(0x02,OLED_CMD);//
  delay(5);OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
  delay(5);OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
  delay(5);OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
  delay(5);OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
  delay(5);OLED_Clear();
  delay(5);OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
}
