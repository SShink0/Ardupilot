#pragma once
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_config.h>

/*
  return the number of bytes to send for a packetised connection
*/
uint16_t mavlink_packetise(ByteBuffer &writebuf, uint16_t n);

