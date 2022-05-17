#define REMOTEXY_MODE__ESP8266_HARDSERIAL_CLOUD

#include <RemoteXY.h>
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "Henry"
#define REMOTEXY_WIFI_PASSWORD "Henry0530"
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
  uint8_t forward; // =1 if state is ON, else =0 
  uint8_t left; // =1 if state is ON, else =0 
  uint8_t right; // =1 if state is ON, else =0 

    // output variables
  float left_speed;
  float right_speed;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)
int left = 3;
int right = 5;
int speed_dect1 = A0;//left
int speed_dect2 = A1;//right
float val_1;
int beforeval_1=0;
float rpm_1;
float val_2;
int beforeval_2=0;
float rpm_2;
unsigned long int count_1=0;
unsigned long int count_2=0;
unsigned long int timer=0;
void setup() 
{
  delay(5000);
  RemoteXY_Init (); 
  pinMode(left,OUTPUT);
  pinMode(right,OUTPUT);
  pinMode(speed_dect1, INPUT);
  pinMode(speed_dect2, INPUT);
  analogWrite(left,0);
  analogWrite(right,0);
  // TODO you setup code  
}

void loop() 
{ 
  val_1=(analogRead(speed_dect1))*5/1024;//ADC
  val_2=(analogRead(speed_dect2))*5/1024;//ADC
  //left signal convert to digital
  if(val_1>2)
  {
    val_1=1;
  }
  else if(val_1<=2)
  {
    val_1=0;
  }
  //left signal convert to digital
  if(val_2>2)
  {
    val_2=1;
  }
  else if(val_2<=2)
  {
    val_2=0;
  }
  if(val_1==1 and beforeval_1==0)//Positive edge trigger
  {
    counter_1();
  }
  if(val_2==1 and beforeval_2==0)//Positive edge trigger
  {
    counter_2();
  }
  beforeval_1=val_1;
  beforeval_2=val_2;
  RemoteXY_Handler ();
  //forward judge
  if(RemoteXY.forward==1&&RemoteXY.right==0&&RemoteXY.left==0)
  {
    analogWrite(left,16);
    analogWrite(right,15);
    if (rpm_1>rpm_2)//left>right
    {
      analogWrite(left,16);
      analogWrite(right,17);
    }
    else if (rpm_1<rpm_2)//left<right
    {
      analogWrite(left,16);
      analogWrite(right,15);
    }
  }
  else if(RemoteXY.forward==0&&RemoteXY.right==0&&RemoteXY.left==0)
  {
    analogWrite(left,0);
    analogWrite(right,0);
  }
  //left judge
  if(RemoteXY.left==1)
  {
    analogWrite(left,20);
    analogWrite(right,15);
  }

  //right judge  
  if(RemoteXY.right==1)
  {
    analogWrite(left,15);
    analogWrite(right,20);
  }
    
  //print speed on phone
  if((millis()-timer)>=1000)
  {
      rpm_1=count_1;
      rpm_1=(rpm_1/24)*60;
      rpm_2=count_2;
      rpm_2=(rpm_2/24)*60;
      RemoteXY.left_speed=rpm_1;
      RemoteXY.right_speed=rpm_2;
      timer=millis();
      count_1=0;
      count_2=0;
  }


}
void counter_1() {
   count_1++;
}
void counter_2() {
   count_2++;
}
