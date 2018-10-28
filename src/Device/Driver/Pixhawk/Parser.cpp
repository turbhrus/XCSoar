#include "Device.hpp"
#include "NMEA/Info.hpp"
#include "Device/Port/Port.hpp"
#include "Util/ConvertString.hpp"
#include "Atmosphere/Temperature.hpp"
#include "Time/PeriodClock.hpp"

/*
 *
 */
static void ReceiveHeartbeat(mavlink_message_t msg, NMEAInfo &info)
{
  mavlink_heartbeat_t packet;

  mavlink_msg_heartbeat_decode(&msg, &packet);

  info.alive.Update(info.clock);
}

/*
 *
 */
static void ReceiveFastData(mavlink_message_t msg, NMEAInfo &info)
{
  mavlink_pixhawk_hg_fast_t packet;

  mavlink_msg_pixhawk_hg_fast_decode(&msg, &packet);

  // GPS time
  info.ProvideTime((double)packet.gps_time * 1e-3);
  // G-force
  double xacc = (double)packet.xacc;
  double yacc = (double)packet.yacc;
  double zacc = (double)packet.zacc;
  double load = sqrt(xacc*xacc + yacc*yacc + zacc*zacc) / 9.80665; // in "G"s
  info.acceleration.ProvideGLoad( load, true);
  // pitch
  Angle pitch = Angle::Radians((double)packet.pitch);
  info.attitude.pitch_angle = pitch;
  info.attitude.pitch_angle_available.Update(info.clock);
  // roll
  Angle roll  = Angle::Radians((double)packet.roll);
  info.attitude.bank_angle = roll;
  info.attitude.bank_angle_available.Update(info.clock);
  // heading
  Angle hdg = Angle::Degrees((double)packet.hdg * 0.01);
  info.attitude.heading = hdg;
  info.attitude.heading_available.Update(info.clock);
  info.heading = hdg;
  info.heading_available.Update(info.clock);

  info.alive.Update(info.clock);
}

/*
 *
 */
static void ReceiveMedData(mavlink_message_t msg, NMEAInfo &info)
{
  mavlink_pixhawk_hg_med_t packet;

  mavlink_msg_pixhawk_hg_med_decode(&msg, &packet);
  // position
  Angle lat = Angle::Degrees((double)packet.lat * 1e-7);
  Angle lon = Angle::Degrees((double)packet.lon * 1e-7);
  GeoPoint loc(lon, lat);
  info.location = loc;
  info.location_available.Update(info.clock);
  // altitude
  double alt = (double)packet.alt * 1e-3;
  info.ProvideBaroAltitudeTrue(alt);

  //ground speed
  SpeedVector v = SpeedVector((double)packet.vy*0.01, (double)packet.vx*0.01);
  info.ground_speed = v.norm;  // in m/s
  info.ground_speed_available.Update(info.clock);
  // ground track
  info.track = v.bearing;
  info.track_available.Update(info.clock);

  // vario
  info.ProvideNoncompVario((double)packet.vz * 0.01);
  // energy compensated vario
  info.ProvideTotalEnergyVario((double)packet.energy_vario * 0.01);
  // Airspeed
  info.ProvideBothAirspeeds((double)packet.indicated_airspeed,
                            (double)packet.true_airspeed);

  info.alive.Update(info.clock);
}

/*
 *
 */
static void ReceiveSlowData(mavlink_message_t msg, NMEAInfo &info)
{
  mavlink_pixhawk_hg_slow_t packet;

  mavlink_msg_pixhawk_hg_slow_decode(&msg, &packet);

  /* Hard coded UTC offset. Obviously not ideal but will have to do for now.
   * NZ offset is 13 hours during daylight savings.
   * It's close enough for date unless we plan to fly around midnight.
   */
  unsigned utc_offset_seconds = 13 * 3600;
  // Date
  BrokenDateTime dt = BrokenDateTime::FromUnixTimeUTC(packet.gps_time_sec + utc_offset_seconds);
  info.ProvideDate(BrokenDate(dt.year,dt.month,dt.day));
  // GPS altitude
  info.gps_altitude = (double)packet.gps_alt * 1e-3;
  info.gps_altitude_available.Update(info.clock);
  // horizontal dilution of precision (m)
  info.gps.hdop = (double)packet.eph * 0.01;
  // num satellites
  info.gps.satellites_used = (int)packet.satellites_visible;
  info.gps.satellites_used_available.Update(info.clock);
  // GPS signal OK?
  info.gps.fix_quality = packet.gps_fix_type >= 2 ? FixQuality::GPS : FixQuality::NO_FIX;
  info.gps.fix_quality_available.Update(info.clock);
  // temperature (deg K)
  info.temperature = CelsiusToKelvin((double)packet.temperature);
  info.temperature_available = true;
  // humidity (%)
  info.humidity = (double)packet.humidity;
  //  info.humidity_available = true;
  // wind (m/s)
  info.ProvideExternalWind( SpeedVector( (double)packet.wind_e, (double)packet.wind_n));

  info.gps.real = true;
  info.alive.Update(info.clock);
}

/*
 *
 */
static void ReceiveText(mavlink_message_t msg, NMEAInfo &info)
{
  mavlink_statustext_t packet;
  int i;
  char char_buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];

  mavlink_msg_statustext_decode(&msg, &packet);

  for (i=0; i<MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN; i++){
    char_buf[i] = packet.text[i];
    if( char_buf[i] == '\0') break;
  }
  char_buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = '\0';
  size_t len = (size_t)i+1;

  // convert to TCHAR
  UTF8ToWideConverter wide_buf(char_buf);
  if (wide_buf.IsValid()){
    info.ProvideDebugMsgExt( wide_buf, len);
  }
  info.alive.Update(info.clock);
}

/*
 *
 */
static void DebugTextInternal(const char *msg, NMEAInfo &info)
{
  char char_buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
  int i;
  for (i=0; i<MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN; i++){
    char_buf[i] = msg[i];
    if( char_buf[i] == '\0') break;
  }
  char_buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = '\0';
  size_t len = (size_t)i+1;

  // convert to TCHAR
  UTF8ToWideConverter wide_buf(char_buf);
  if (wide_buf.IsValid()){
    info.ProvideDebugMsgInt( wide_buf, len);
  }
}

/*
 *
 */
static void HandleMessage(mavlink_message_t msg, NMEAInfo &info)
{
  switch(msg.msgid)
  {
  case MAVLINK_MSG_ID_HEARTBEAT:
    ReceiveHeartbeat(msg, info);
    break;

  case MAVLINK_MSG_ID_PIXHAWK_HG_FAST:
    ReceiveFastData(msg, info);
    break;

  case MAVLINK_MSG_ID_PIXHAWK_HG_MED:
    ReceiveMedData(msg, info);
    break;

  case MAVLINK_MSG_ID_PIXHAWK_HG_SLOW:
    ReceiveSlowData(msg, info);
    break;

  case MAVLINK_MSG_ID_STATUSTEXT:
    ReceiveText(msg, info);
    break;

  default:
    //Do nothing
    break;
  }
}

/*
 *
 */
bool PixhawkDevice::DataReceived(const void *_data, size_t _length,
                            struct NMEAInfo &info)
{
  size_t length = _length;

  assert(_data != NULL);
  assert(length > 0);

  fifo.Shift();
  auto range = fifo.Write();
  if (range.size < length){ // Full - wipe the entire buffer
    fifo.Clear();
  }else{
    memcpy(range.data, (const uint8_t *)_data, length);
    fifo.Append(length);
  }

  // extract Mavlink messages from buffer
  while(true){

    // Read data from buffer
    range = fifo.Read();

    // look for a valid message
    unsigned i;
    for(i=0; i<range.size; ++i){
      char c = (char)range.data[i];
      if(mavlink_parse_char(MAVLINK_CHAN, c, &msg, &status)) {
        HandleMessage(msg, info);
        fifo.Consume(i+1);
        fifo.Shift();
        break;
      }
    }
    if(i==range.size) break;
  }
  return true;
}






/*
 *
 */
//bool PixhawkDevice::DataReceived(const void *_data, size_t length,
//                            struct NMEAInfo &info)
//{
//  assert(_data != NULL);
//  assert(length > 0);
//
//  const uint8_t *data = (const uint8_t *)_data, *end = data + length;
//  bool result=false;
//
//  int ds = MonotonicClockMS() / 100; // deciseconds
//  char s[150];
//  n_noluck++;
//  sprintf(s, "%d hb%d f%d m%d s%d t%d n%d nl%d gp%d",ds, n_heartbeat,n_fast,n_med,n_slow,n_text,n_nothing,n_noluck, n_gp);
//  DebugTextInternal(s, info);
//
//
//  for(; data < end; data++)
//  {
//    // Try to get a new message
//    uint8_t c = *data;
//    if(mavlink_parse_char(MAVLINK_CHAN, c, &msg, &status)) {
//
//      HandleMessage(msg, info);
//      result=true;
//    }
//  }
//
//  return result;
////  return true; // I think this is correct, otherwise the same data may be passed back in. TODO check!
//}
