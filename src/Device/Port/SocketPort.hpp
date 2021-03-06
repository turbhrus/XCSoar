/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
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

#ifndef XCSOAR_DEVICE_SOCKET_PORT_HPP
#define XCSOAR_DEVICE_SOCKET_PORT_HPP

#include "BufferedPort.hpp"
#include "Net/SocketDescriptor.hpp"
#include "IO/Async/SocketEventHandler.hpp"

#ifndef HAVE_POSIX
#include "IO/Async/SocketThread.hpp"
#endif

/**
 * A UDP listener port class.
 */
class SocketPort : public BufferedPort, protected SocketEventHandler {
  SocketDescriptor socket;

#ifndef HAVE_POSIX
  /* on WIN32, the IOThread class is not available; SocketThread
     emulates it */
  SocketThread thread;
#endif

public:
  /**
   * Creates a new SocketPort object, but does not open it yet.
   *
   * @param handler the callback object for input received on the
   * port
   */
  SocketPort(PortListener *_listener, DataHandler &_handler)
    :BufferedPort(_listener, _handler),
     socket(SocketDescriptor::Undefined())
#ifndef HAVE_POSIX
    , thread(*this)
#endif
  {}

  /**
   * Closes the serial port (Destructor)
   */
  virtual ~SocketPort();

  /**
   * Make the object use the specified socket.
   */
  void Set(SocketDescriptor &&_socket);

protected:
  /**
   * Close the socket.  This object can be reused afterwards by
   * calling Set().
   */
  void Close();

public:
  /**
   * Opens an UDP listener port
   * @return True on success, False on failure
   */
  bool OpenUDPListener(unsigned port);

  /* virtual methods from class Port */
  virtual PortState GetState() const override;
  virtual bool Drain() override;
  virtual bool SetBaudrate(unsigned baud_rate) override;
  virtual unsigned GetBaudrate() const override;
  virtual size_t Write(const void *data, size_t length) override;

protected:
  /* virtual methods from class SocketEventHandler */
  bool OnSocketEvent(SocketDescriptor s, unsigned mask) override;
};

#endif
