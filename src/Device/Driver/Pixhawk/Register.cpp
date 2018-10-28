/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2013 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Device/Driver/Pixhawk.hpp"
#include "Device.hpp"
#include "Operation/Operation.hpp"
#include "Device/Port/Port.hpp"



class Port;
static Port *mavlink_send_port;

static Device *
PixhawkCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  mavlink_send_port = &com_port;
  return new PixhawkDevice(com_port);
}

const struct DeviceRegister pixhawk_driver = {
  _T("Pixhawk"),
  _T("Pixhawk"),
  DeviceRegister::RAW_GPS_DATA | DeviceRegister::NO_TIMEOUT,
  PixhawkCreateOnPort,
};


/* This function is required by Mavlink for message sending.
 * It's located here to access the com port by exploiting file scope,
 * which is a rude hack in need of a smarter implementation.
 */
void mavlink_xcsoar_send_buffer(mavlink_channel_t chan, const uint8_t *line, int len)
{
  const unsigned timeout_ms = 1000;
  NullOperationEnvironment env;

  // ignoring return value
  mavlink_send_port->FullWrite(line, len, env, timeout_ms);
}
