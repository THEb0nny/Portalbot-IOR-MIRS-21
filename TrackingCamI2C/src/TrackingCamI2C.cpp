#include <TrackingCamI2c.h>
#include <Wire.h>

#define nTRACKINGCAMI2C_DEBUG
#define TRACKINGCAMI2C_DEBUG_BAUD 115200

static uint8_t TrackingCam_ReadData(uint8_t cam_id, uint8_t addr, uint8_t len, uint8_t* resp);

void TrackingCamI2C::init(uint8_t cam_id, uint32_t speed)
{
  this->cam_id = cam_id;
  Wire.begin(); // join i2c bus (address optional for master)
  Wire.setClock(speed);
  delay(100);
#ifdef TRACKINGCAMI2C_DEBUG
  Serial.begin(TRACKINGCAMI2C_DEBUG_BAUD);  // start serial for output
#endif
}

uint8_t TrackingCamI2C::readBlobs(uint8_t max_blob_n)
{  
  uint8_t resp[255];
  uint8_t n = 0;
  uint8_t idx = 0;
  bool is_bulk_reading = false;
  uint8_t lines = max_blob_n > 10? 10: max_blob_n;
 
  if(lines == 0)
    lines = 1;
    
  if(is_bulk_reading)
  {
    if(TrackingCam_ReadData(cam_id, 16, (max_blob_n? lines * 16: 6), resp)) //Bulk reading
      return 0;
  }
 
  for(int i = 0; i < lines; i++)
  {
    if(!is_bulk_reading)
    {
      if(TrackingCam_ReadData(cam_id, 16 + i*16, (max_blob_n? 16: 6), resp)) //Line by line reading
        return n; 
		idx = 0;
    }
    
    blob[i].type = resp[idx++];
    if(blob[i].type == 0xFF)
      return n;
    else
      n++;
      
    blob[i].dummy = resp[idx++];
    blob[i].cx = ((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8);
    idx += 2;
    blob[i].cy = ((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8);
    idx += 2;
    if(max_blob_n == 0)
      break;
    blob[i].area = (uint32_t)((((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8))) * 4;
    idx += 2;
    blob[i].left = ((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8);
    idx += 2;
    blob[i].right = ((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8);
    idx += 2;
    blob[i].top = ((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8);
    idx += 2;
    blob[i].bottom = ((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8);
    idx += 2;
  }
  return n;
}
/*
uint8_t TrackingCamI2C::readObjects(uint8_t max_obj_n)
{
  uint8_t resp[255];
  uint8_t n = 0;
  uint8_t idx = 0;
  bool is_bulk_reading = false;
  uint8_t lines = max_obj_n > 10? 10: max_obj_n;
 
  if(lines == 0)
    lines = 1;
    
  if(is_bulk_reading)
  {
    if(TrackingCam_ReadData(cam_id, 16, (max_obj_n? lines * 10: 6), resp)) //Bulk reading
      return 0;
  }
 
  for(int i = 0; i < lines; i++)
  {
    if(!is_bulk_reading)
    {
      if(TrackingCam_ReadData(cam_id, 16 + i*10, (max_obj_n? 10: 6), resp)) //Line by line reading
        return n; 
      idx = 0;
    }
    obj[i].type = resp[idx++];
    if(obj[i].type == 0xFF)
      return n;
    else
      n++;
    obj[i].dummy = resp[idx++];
    obj[i].cx = ((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8);
    idx += 2;
    obj[i].cy = ((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8);
    idx += 2;
    if(max_obj_n == 0)
      break;
    obj[i].angle = ((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8);
    idx += 2;
    obj[i].obj_size = (uint32_t)((((uint16_t)resp[idx]) + (((uint16_t)resp[idx + 1]) << 8))) * 4;
    idx += 2;
  }
  return n;
}
*/
/* Reads specified number of bytes from camera control registers (dxl API)
 * Do not forget to disable debug prints!!!
 */
static uint8_t TrackingCam_ReadData(uint8_t cam_id, uint8_t addr, uint8_t len, uint8_t* resp)
{
  Wire.beginTransmission(cam_id); // transmit to device #1
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(cam_id, len);    // request len bytes from slave device cam_id
  uint8_t idx = 0;
  while (Wire.available())
  { // slave may send less than requested
      resp[idx] = Wire.read(); // receive a byte as character
#ifdef TRACKINGCAMI2C_DEBUG 
      Serial.print(resp[idx], HEX);         // print the character
	  Serial.print(" ");
#endif
      idx++;
  }
#ifdef TRACKINGCAMI2C_DEBUG 
  Serial.print("\n");
#endif
  return 0;
}


