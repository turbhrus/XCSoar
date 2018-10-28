// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS_MAVLink.hpp
/// @brief	One size fits all header for MAVLink integration.

#ifndef MAVLink_hpp
#define MAVLink_hpp

/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

#include "Mavlink/include/PIXHAWK/version.h"

#define MAVLINK_MAX_PAYLOAD_LEN 255

#include "Mavlink/include/mavlink_types.h"

void mavlink_xcsoar_send_buffer(mavlink_channel_t chan, const uint8_t *line, int len);
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) mavlink_xcsoar_send_buffer(chan, buf, len)

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "Mavlink/include/PIXHAWK/mavlink.h"

#ifndef MAVLINK_CHAN
#define MAVLINK_CHAN MAVLINK_COMM_1
#endif

#pragma GCC diagnostic pop

#endif // MAVLink_hpp
