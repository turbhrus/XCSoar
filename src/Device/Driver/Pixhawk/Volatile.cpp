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

// TODO: find somewhere more appropriate for this
//#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "Volatile.hpp"
#include "NMEA/Derived.hpp"
#include "Device/Util/NMEAWriter.hpp"
#include "Operation/Operation.hpp"


void
Pixhawk::VolatileData::CopyFrom(const DerivedInfo &calculated)
{
  static Validity previous_airspace_warning;
  static bool final_leg=false, task_active=true;

  speed_to_fly = calculated.V_stf;
  flying = calculated.flight.flying;
  circling = calculated.circling;

  if(task_active){
    waypoint_id = calculated.common_stats.ordered_summary.active;
    // If this is the last leg, then "active" will not increment
    // when goal is reached. Deal with this case manually.
    if(final_leg){
      if(calculated.task_stats.task_finished){ // check if goal reached
        waypoint_id++; // increment for the last time
        task_active = false;
      }
    }else{
      final_leg = calculated.common_stats.next_is_last;
    }
  }

  if (calculated.airspace_warnings.latest != previous_airspace_warning) {
    // There's a new airspace warning
    // Increment warning ID and let receiver check for changes
    airspace_warning_id++;
  }
  previous_airspace_warning = calculated.airspace_warnings.latest;

  terrain_warning = calculated.terrain_warning;
}


void
Pixhawk::VolatileData::SendTo() const
{
  mavlink_msg_xcsoar_calculated_send(
      MAVLINK_CHAN,
      (float)speed_to_fly,
      flying,
      circling,
      waypoint_id,
      airspace_warning_id,
      terrain_warning);
}
