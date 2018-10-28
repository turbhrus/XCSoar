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

#ifndef XCSOAR_PIXHAWK_DEVICE_HPP
#define XCSOAR_PIXHAWK_DEVICE_HPP

#include "Device/Driver.hpp"
#include "Volatile.hpp"
#include "Util/StaticFifoBuffer.hpp"

class PixhawkDevice : public AbstractDevice
{
private:
  Port &port;
  mavlink_message_t msg;
  mavlink_status_t status;
  StaticFifoBuffer<char, 4192> fifo;

  Pixhawk::VolatileData volatile_data;

public:
  PixhawkDevice(Port &_port)
    :port(_port){}

public:
  virtual void LinkTimeout() override;

  virtual void OnCalculatedUpdate(const MoreData &basic,
                                  const DerivedInfo &calculated) override;

  virtual bool DataReceived(const void *data, size_t length,
                            struct NMEAInfo &info) override;
};

#endif
