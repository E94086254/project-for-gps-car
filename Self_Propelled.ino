#define REMOTEXY_MODE__ESP8266_HARDSERIAL_CLOUD

#include <RemoteXY.h>
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "Steven’s iPhone"
#define REMOTEXY_WIFI_PASSWORD "00000000"
#define REMOTEXY_CLOUD_SERVER "cloud.remotexy.com"
#define REMOTEXY_CLOUD_PORT 6376
#define REMOTEXY_CLOUD_TOKEN "63e38f71000775c1d3b3f2c6026f3e2a"

#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 77 bytes
  { 255,3,0,8,0,70,0,16,31,1,68,17,3,53,27,33,8,36,68,17,
  33,53,27,33,8,36,10,48,23,4,15,15,4,26,31,79,78,0,31,79,
  70,70,0,10,48,8,20,15,15,4,26,31,79,78,0,31,79,70,70,0,
  10,48,39,20,15,15,4,26,31,79,78,0,31,79,70,70,0 };

struct {

    // input variables
  uint8_t forward=0; // =1 if state is ON, else =0 
  uint8_t left=0; // =1 if state is ON, else =0 
  uint8_t right=0; // =1 if state is ON, else =0 

    // output variables
  float left_speed;
  float right_speed;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)
int Left = 3;//左輪為3腳位
int Right = 5;//右輪為5腳位
int speed_dect2 = A1;//沒用
float val_2;//沒用
int beforeval_2=0;//沒用
float rpm_2;//沒用
int flag=0;//判斷障礙物旗標 if flag=0 no item , if flag=1 there are item
int tmp[10] = { };//暫存陣列
float min_distance;
int avoid=0;//計數
unsigned long int count_2=0;//沒用
unsigned long int timer=0;
unsigned long pulseWidth;
void setup() 
{
  delay(3000);
  RemoteXY_Init (); 
  pinMode(10, OUTPUT); // Set pin 2 as trigger pin
  digitalWrite(10, LOW); // Set trigger LOW for continuous read
  pinMode(11, INPUT); //   Set pin 3 as monitor pin
  pinMode(Left,OUTPUT);
  pinMode(Right,OUTPUT);
  pinMode(speed_dect2, INPUT);
  analogWrite(Left,0);
  analogWrite(Right,0);
  // TODO you setup code  
}

void loop() 
{ 
  RemoteXY_Handler ();
  val_2=(analogRead(speed_dect2))*5/1024;//ADC
  if(val_2>2)
  {
    val_2=1;
  }
  else if(val_2<=2)
  {
    val_2=0;
  }
  if(val_2==1 and beforeval_2==0)//Positive edge trigger
  {
    counter_2();
  }
  beforeval_2=val_2;
  if((millis()-timer)>=1000)
  {
      rpm_2=count_2;
      rpm_2=(rpm_2/24)*60;
      Serial.print(rpm_2);
      RemoteXY.right_speed=rpm_2;
      timer=millis();
      count_2=0;
  }
  if(RemoteXY.forward==1&&RemoteXY.right==0&&RemoteXY.left==0)
  {
    if(flag==0)//沒有遇到障礙物
    {
      analogWrite(Left,25);
      analogWrite(Right,27);
    }
    Lidar_scan();//掃描一次
    if (avoid>4 && flag==0)
    {
      while(1)
      {
        analogWrite(Left,0);
        analogWrite(Right,0);
        if((millis()-timer)>=500)
        {
          timer=millis();
          break;
        }
      }
      turn_left();
      flag=1;
    }
    if (flag==1)//左轉後
    {
      for(int i=0;i<10;i++)//經過一秒得到十個數字
      {
         Lidar_scan();
         tmp[i]=pulseWidth;//掃到的距離
         delay(100);
         if (RemoteXY.forward==0)
         {
          break;
         }
      }
      min_distance=tmp[0];
      for(int i=0;i<10;i++)//找到掃一圈最小的距離
      {
        if(tmp[i]<min_distance)
        min_distance=tmp[i];
      }
      if(min_distance>=50)//已經躲開障礙物就右轉
      {
        turn_right();
        flag=0;
      }
      else if(min_distance<50)//沒有躲開就直走
      {
        analogWrite(Left,25);
        analogWrite(Right,27);
        timer = millis()+800;
      }
      else if(min_distance<20)//如果距離太近再往左靠
      {
        analogWrite(Left,23);
        analogWrite(Right,27);
      }
    }
  }
  else if(RemoteXY.forward==0&&RemoteXY.right==0&&RemoteXY.left==0)
  {
    analogWrite(Left,0);
    analogWrite(Right,0);
  }
}
void counter_2() 
{
   count_2++;
}
void Lidar_scan()
{
  pulseWidth = pulseIn(11, HIGH); // Count how long the pulse is high in microsecond
  pulseWidth = pulseWidth / 10; // 10usec = 1 cm of distance
  if(pulseWidth<100)
  {
    avoid++;
  }
  else
  {
    avoid=0;
  }
}
void turn_left()
{
  while(1)
  {
    analogWrite(Left,0);
    analogWrite(Right,27);
    if((millis()-timer)>=800)
    {
      timer=millis();
      break;
    }
    if (RemoteXY.forward==0)
    {
      break;
    }
  }
}
void turn_right()
{
  while(1)
  {
    analogWrite(Left,25);
    analogWrite(Right,0);
    if((millis()-timer)>=2000)
    {
      timer=millis();
      break;
    }
    if (RemoteXY.forward==0)
    {
      break;
    }
  }
}
