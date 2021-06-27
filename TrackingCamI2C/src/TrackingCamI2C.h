#ifndef TrackingCamI2C_h
#define TrackingCamI2C_h

#include "Arduino.h"

/*struct TrackingCamObjInfo_t
{
  uint8_t type;
  uint8_t dummy;
  uint16_t cx;
  uint16_t cy;
  uint16_t angle;
  uint32_t obj_size;
};*/

struct TrackingCamBlobInfo_t
{
  uint8_t type;
  uint8_t dummy;
  uint16_t cx;
  uint16_t cy;
  uint8_t area;
  uint8_t left;
  uint8_t right;
  uint8_t top;
  uint8_t bottom;
  //uint32_t area;
  //uint16_t left;
  //uint16_t right;
  //uint16_t top;
  //uint16_t bottom;
};

class TrackingCamI2C
{
  uint8_t cam_id;
public:
  void init(uint8_t cam_id, uint32_t speed);
  uint8_t readBlobs(uint8_t max_blob_n = 5);
  //uint8_t readObjects(uint8_t max_obj_n = 5);
  TrackingCamBlobInfo_t blob[16];
  //TrackingCamObjInfo_t obj[16];
};

#endif
