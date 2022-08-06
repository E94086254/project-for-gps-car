#include <SoftwareSerial.h>

void setup() {

  Serial.begin(115200);   //設定軟體串列埠速率

}

 

void loop() {

  Serial.write("1"); //讀取一號機傳送之字元並顯示

  delay(2000);

}
