/** Arduino I2C objects example.
 * Settings: Object detector, I2C, addr 51, Dynamixel API, 5V.
 * Wiring:
 *       Camera         Arduino Camera
 * 1-VF >|O O|  2-+5      SCL  -  IC0
 * 3-Gnd |O O|  4-Gnd     SDA  -  ID1
 * 5-TX   O O|  6-RX      5V   -  +5
 * 7-SCK |O O|  8-SNS     Gnd  -  Gnd
 * 9-IC0 |O O| 10-ID1     
 */ 
 
#include "TrackingCamI2C.h"

TrackingCamI2C trackingCam;
unsigned long previousMillis = 0; // stores last time cam was updated

void setup() {
  /* TrackingCamI2C::init(uint8_t cam_id, uint32_t speed);
   *   cam_id - 1..127, default 51
   *   speed - 100000/400000, cam enables auto detection of master clock 
   */
  trackingCam.init(51, 400000);
  Serial.begin(115200);
  delay(5000);
}

void loop() {
  uint8_t n = trackingCam.readObjects(5); // read data about 5 first objects
  Serial.println("All objects"); // print information about all objects
  Serial.println(n); // print numbers of objects
  for(int i = 0; i < n; i++)
  {
    Serial.print(trackingCam.obj[i].type, DEC);
    Serial.print(" ");
    Serial.print(trackingCam.obj[i].cx, DEC);
    Serial.print(" ");
    Serial.print(trackingCam.obj[i].cy, DEC);
    Serial.print(" ");
    Serial.print(trackingCam.obj[i].angle, DEC);
    Serial.print(" ");
    Serial.print(trackingCam.obj[i].obj_size, DEC);
    Serial.println("");
  }    
    
  // wait for the next frame
  while(millis() - previousMillis < 33) 
  {};
  previousMillis = millis();
}