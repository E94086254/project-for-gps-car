#include <SoftwareSerial.h>
#define REMOTEXY_MODE__ESP8266_HARDSERIAL_CLOUD

#include <RemoteXY.h>
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "Hartmann_Lo"
#define REMOTEXY_WIFI_PASSWORD "72m214j0yyc8i"
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

SoftwareSerial mySerial(11,10);//建立軟體串列埠腳位 (RX, TX)

 

void setup() {
  delay(3000);
  RemoteXY_Init (); 
  mySerial.begin(115200);  //設定軟體通訊速率

}

 

void loop() {

  RemoteXY_Handler ();
  while (!mySerial.available()) {} //直到暫存器出現訊號才跳出迴圈

  Serial.write(mySerial.read());  //傳輸讀取的訊號

  while (mySerial.available()>0) {//如果暫存器有訊號則不斷讀取直到沒有

    mySerial.read();

  }  

}
