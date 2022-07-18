

/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  v3/GetDistancePwm

  This example shows how to read distance from a LIDAR-Lite connected over the
  PWM interface.

  Connections:
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite Ground (black) to Arduino GND
  LIDAR-Lite Mode control (yellow) to Arduino digital input (pin 3)
  LIDAR-Lite Mode control (yellow) to 1 kOhm resistor lead 1
  1 kOhm resistor lead 2 to Arduino digital output (pin 2)

  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information:
  http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/
#include <Servo.h>   //載入函式庫，這是內建的，不用安裝

Servo myservo;  // 建立SERVO物件

unsigned long pulseWidth;


void setup()
{
  //Serial.begin(115200); // Start serial communications

  //pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  //digitalWrite(2, LOW); // Set trigger LOW for continuous read

  //pinMode(3, INPUT); // Set pin 3 as monitor pin
  
  myservo.attach(9);  // 設定要將伺服馬達接到哪一個PIN腳

}
/*
void Lidar_scan()
{
  pulseWidth = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds

  // If we get a reading that isn't zero, let's print it
  if(pulseWidth != 0)
  {
    pulseWidth = pulseWidth / 10; // 10usec = 1 cm of distance
    Serial.println(pulseWidth); // Print the distance
  }
}
*/

void loop()
{
  
  myservo.write(0);  //旋轉到0度，就是一般所說的歸零
  
  delay(770);
    
  myservo.write(180);  //旋轉到0度，就是一般所說的
   
  delay(763);

}
