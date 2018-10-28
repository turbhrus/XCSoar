// MESSAGE PIXHAWK_HG_SLOW PACKING

#define MAVLINK_MSG_ID_PIXHAWK_HG_SLOW 232

MAVPACKED(
typedef struct __mavlink_pixhawk_hg_slow_t {
 uint64_t gps_time_sec; /*< time since 1st Jan 1970 in seconds*/
 int32_t gps_alt; /*< Altitude (WGS84), in mm (positive for up)*/
 float temperature; /*< Temperature in degrees celsius*/
 float wind_n; /*< Wind North +ve (m/s)*/
 float wind_e; /*< Wind East +ve (m/s)*/
 float wind_d; /*< Wind Down +ve (m/s)*/
 uint16_t eph; /*< PS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX*/
 uint8_t gps_fix_type; /*< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
 uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/
 uint8_t humidity; /*< Relative humidity in percent*/
}) mavlink_pixhawk_hg_slow_t;

#define MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN 33
#define MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_MIN_LEN 33
#define MAVLINK_MSG_ID_232_LEN 33
#define MAVLINK_MSG_ID_232_MIN_LEN 33

#define MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_CRC 188
#define MAVLINK_MSG_ID_232_CRC 188



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PIXHAWK_HG_SLOW { \
	232, \
	"PIXHAWK_HG_SLOW", \
	10, \
	{  { "gps_time_sec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_pixhawk_hg_slow_t, gps_time_sec) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_pixhawk_hg_slow_t, gps_alt) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pixhawk_hg_slow_t, temperature) }, \
         { "wind_n", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pixhawk_hg_slow_t, wind_n) }, \
         { "wind_e", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pixhawk_hg_slow_t, wind_e) }, \
         { "wind_d", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_pixhawk_hg_slow_t, wind_d) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_pixhawk_hg_slow_t, eph) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_pixhawk_hg_slow_t, gps_fix_type) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_pixhawk_hg_slow_t, satellites_visible) }, \
         { "humidity", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_pixhawk_hg_slow_t, humidity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PIXHAWK_HG_SLOW { \
	"PIXHAWK_HG_SLOW", \
	10, \
	{  { "gps_time_sec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_pixhawk_hg_slow_t, gps_time_sec) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_pixhawk_hg_slow_t, gps_alt) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pixhawk_hg_slow_t, temperature) }, \
         { "wind_n", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pixhawk_hg_slow_t, wind_n) }, \
         { "wind_e", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pixhawk_hg_slow_t, wind_e) }, \
         { "wind_d", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_pixhawk_hg_slow_t, wind_d) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_pixhawk_hg_slow_t, eph) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_pixhawk_hg_slow_t, gps_fix_type) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_pixhawk_hg_slow_t, satellites_visible) }, \
         { "humidity", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_pixhawk_hg_slow_t, humidity) }, \
         } \
}
#endif

/**
 * @brief Pack a pixhawk_hg_slow message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gps_time_sec time since 1st Jan 1970 in seconds
 * @param eph PS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param gps_alt Altitude (WGS84), in mm (positive for up)
 * @param gps_fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @param temperature Temperature in degrees celsius
 * @param humidity Relative humidity in percent
 * @param wind_n Wind North +ve (m/s)
 * @param wind_e Wind East +ve (m/s)
 * @param wind_d Wind Down +ve (m/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pixhawk_hg_slow_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t gps_time_sec, uint16_t eph, int32_t gps_alt, uint8_t gps_fix_type, uint8_t satellites_visible, float temperature, uint8_t humidity, float wind_n, float wind_e, float wind_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN];
	_mav_put_uint64_t(buf, 0, gps_time_sec);
	_mav_put_int32_t(buf, 8, gps_alt);
	_mav_put_float(buf, 12, temperature);
	_mav_put_float(buf, 16, wind_n);
	_mav_put_float(buf, 20, wind_e);
	_mav_put_float(buf, 24, wind_d);
	_mav_put_uint16_t(buf, 28, eph);
	_mav_put_uint8_t(buf, 30, gps_fix_type);
	_mav_put_uint8_t(buf, 31, satellites_visible);
	_mav_put_uint8_t(buf, 32, humidity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN);
#else
	mavlink_pixhawk_hg_slow_t packet;
	packet.gps_time_sec = gps_time_sec;
	packet.gps_alt = gps_alt;
	packet.temperature = temperature;
	packet.wind_n = wind_n;
	packet.wind_e = wind_e;
	packet.wind_d = wind_d;
	packet.eph = eph;
	packet.gps_fix_type = gps_fix_type;
	packet.satellites_visible = satellites_visible;
	packet.humidity = humidity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PIXHAWK_HG_SLOW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_CRC);
}

/**
 * @brief Pack a pixhawk_hg_slow message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_time_sec time since 1st Jan 1970 in seconds
 * @param eph PS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param gps_alt Altitude (WGS84), in mm (positive for up)
 * @param gps_fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @param temperature Temperature in degrees celsius
 * @param humidity Relative humidity in percent
 * @param wind_n Wind North +ve (m/s)
 * @param wind_e Wind East +ve (m/s)
 * @param wind_d Wind Down +ve (m/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pixhawk_hg_slow_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t gps_time_sec,uint16_t eph,int32_t gps_alt,uint8_t gps_fix_type,uint8_t satellites_visible,float temperature,uint8_t humidity,float wind_n,float wind_e,float wind_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN];
	_mav_put_uint64_t(buf, 0, gps_time_sec);
	_mav_put_int32_t(buf, 8, gps_alt);
	_mav_put_float(buf, 12, temperature);
	_mav_put_float(buf, 16, wind_n);
	_mav_put_float(buf, 20, wind_e);
	_mav_put_float(buf, 24, wind_d);
	_mav_put_uint16_t(buf, 28, eph);
	_mav_put_uint8_t(buf, 30, gps_fix_type);
	_mav_put_uint8_t(buf, 31, satellites_visible);
	_mav_put_uint8_t(buf, 32, humidity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN);
#else
	mavlink_pixhawk_hg_slow_t packet;
	packet.gps_time_sec = gps_time_sec;
	packet.gps_alt = gps_alt;
	packet.temperature = temperature;
	packet.wind_n = wind_n;
	packet.wind_e = wind_e;
	packet.wind_d = wind_d;
	packet.eph = eph;
	packet.gps_fix_type = gps_fix_type;
	packet.satellites_visible = satellites_visible;
	packet.humidity = humidity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PIXHAWK_HG_SLOW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_CRC);
}

/**
 * @brief Encode a pixhawk_hg_slow struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pixhawk_hg_slow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pixhawk_hg_slow_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pixhawk_hg_slow_t* pixhawk_hg_slow)
{
	return mavlink_msg_pixhawk_hg_slow_pack(system_id, component_id, msg, pixhawk_hg_slow->gps_time_sec, pixhawk_hg_slow->eph, pixhawk_hg_slow->gps_alt, pixhawk_hg_slow->gps_fix_type, pixhawk_hg_slow->satellites_visible, pixhawk_hg_slow->temperature, pixhawk_hg_slow->humidity, pixhawk_hg_slow->wind_n, pixhawk_hg_slow->wind_e, pixhawk_hg_slow->wind_d);
}

/**
 * @brief Encode a pixhawk_hg_slow struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pixhawk_hg_slow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pixhawk_hg_slow_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pixhawk_hg_slow_t* pixhawk_hg_slow)
{
	return mavlink_msg_pixhawk_hg_slow_pack_chan(system_id, component_id, chan, msg, pixhawk_hg_slow->gps_time_sec, pixhawk_hg_slow->eph, pixhawk_hg_slow->gps_alt, pixhawk_hg_slow->gps_fix_type, pixhawk_hg_slow->satellites_visible, pixhawk_hg_slow->temperature, pixhawk_hg_slow->humidity, pixhawk_hg_slow->wind_n, pixhawk_hg_slow->wind_e, pixhawk_hg_slow->wind_d);
}

/**
 * @brief Send a pixhawk_hg_slow message
 * @param chan MAVLink channel to send the message
 *
 * @param gps_time_sec time since 1st Jan 1970 in seconds
 * @param eph PS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param gps_alt Altitude (WGS84), in mm (positive for up)
 * @param gps_fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @param temperature Temperature in degrees celsius
 * @param humidity Relative humidity in percent
 * @param wind_n Wind North +ve (m/s)
 * @param wind_e Wind East +ve (m/s)
 * @param wind_d Wind Down +ve (m/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pixhawk_hg_slow_send(mavlink_channel_t chan, uint64_t gps_time_sec, uint16_t eph, int32_t gps_alt, uint8_t gps_fix_type, uint8_t satellites_visible, float temperature, uint8_t humidity, float wind_n, float wind_e, float wind_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN];
	_mav_put_uint64_t(buf, 0, gps_time_sec);
	_mav_put_int32_t(buf, 8, gps_alt);
	_mav_put_float(buf, 12, temperature);
	_mav_put_float(buf, 16, wind_n);
	_mav_put_float(buf, 20, wind_e);
	_mav_put_float(buf, 24, wind_d);
	_mav_put_uint16_t(buf, 28, eph);
	_mav_put_uint8_t(buf, 30, gps_fix_type);
	_mav_put_uint8_t(buf, 31, satellites_visible);
	_mav_put_uint8_t(buf, 32, humidity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW, buf, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_CRC);
#else
	mavlink_pixhawk_hg_slow_t packet;
	packet.gps_time_sec = gps_time_sec;
	packet.gps_alt = gps_alt;
	packet.temperature = temperature;
	packet.wind_n = wind_n;
	packet.wind_e = wind_e;
	packet.wind_d = wind_d;
	packet.eph = eph;
	packet.gps_fix_type = gps_fix_type;
	packet.satellites_visible = satellites_visible;
	packet.humidity = humidity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW, (const char *)&packet, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_CRC);
#endif
}

/**
 * @brief Send a pixhawk_hg_slow message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pixhawk_hg_slow_send_struct(mavlink_channel_t chan, const mavlink_pixhawk_hg_slow_t* pixhawk_hg_slow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pixhawk_hg_slow_send(chan, pixhawk_hg_slow->gps_time_sec, pixhawk_hg_slow->eph, pixhawk_hg_slow->gps_alt, pixhawk_hg_slow->gps_fix_type, pixhawk_hg_slow->satellites_visible, pixhawk_hg_slow->temperature, pixhawk_hg_slow->humidity, pixhawk_hg_slow->wind_n, pixhawk_hg_slow->wind_e, pixhawk_hg_slow->wind_d);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW, (const char *)pixhawk_hg_slow, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_CRC);
#endif
}

#if MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pixhawk_hg_slow_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t gps_time_sec, uint16_t eph, int32_t gps_alt, uint8_t gps_fix_type, uint8_t satellites_visible, float temperature, uint8_t humidity, float wind_n, float wind_e, float wind_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, gps_time_sec);
	_mav_put_int32_t(buf, 8, gps_alt);
	_mav_put_float(buf, 12, temperature);
	_mav_put_float(buf, 16, wind_n);
	_mav_put_float(buf, 20, wind_e);
	_mav_put_float(buf, 24, wind_d);
	_mav_put_uint16_t(buf, 28, eph);
	_mav_put_uint8_t(buf, 30, gps_fix_type);
	_mav_put_uint8_t(buf, 31, satellites_visible);
	_mav_put_uint8_t(buf, 32, humidity);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW, buf, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_CRC);
#else
	mavlink_pixhawk_hg_slow_t *packet = (mavlink_pixhawk_hg_slow_t *)msgbuf;
	packet->gps_time_sec = gps_time_sec;
	packet->gps_alt = gps_alt;
	packet->temperature = temperature;
	packet->wind_n = wind_n;
	packet->wind_e = wind_e;
	packet->wind_d = wind_d;
	packet->eph = eph;
	packet->gps_fix_type = gps_fix_type;
	packet->satellites_visible = satellites_visible;
	packet->humidity = humidity;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW, (const char *)packet, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_CRC);
#endif
}
#endif

#endif

// MESSAGE PIXHAWK_HG_SLOW UNPACKING


/**
 * @brief Get field gps_time_sec from pixhawk_hg_slow message
 *
 * @return time since 1st Jan 1970 in seconds
 */
static inline uint64_t mavlink_msg_pixhawk_hg_slow_get_gps_time_sec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field eph from pixhawk_hg_slow message
 *
 * @return PS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_pixhawk_hg_slow_get_eph(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field gps_alt from pixhawk_hg_slow message
 *
 * @return Altitude (WGS84), in mm (positive for up)
 */
static inline int32_t mavlink_msg_pixhawk_hg_slow_get_gps_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field gps_fix_type from pixhawk_hg_slow message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 */
static inline uint8_t mavlink_msg_pixhawk_hg_slow_get_gps_fix_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field satellites_visible from pixhawk_hg_slow message
 *
 * @return Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_pixhawk_hg_slow_get_satellites_visible(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field temperature from pixhawk_hg_slow message
 *
 * @return Temperature in degrees celsius
 */
static inline float mavlink_msg_pixhawk_hg_slow_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field humidity from pixhawk_hg_slow message
 *
 * @return Relative humidity in percent
 */
static inline uint8_t mavlink_msg_pixhawk_hg_slow_get_humidity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field wind_n from pixhawk_hg_slow message
 *
 * @return Wind North +ve (m/s)
 */
static inline float mavlink_msg_pixhawk_hg_slow_get_wind_n(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field wind_e from pixhawk_hg_slow message
 *
 * @return Wind East +ve (m/s)
 */
static inline float mavlink_msg_pixhawk_hg_slow_get_wind_e(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field wind_d from pixhawk_hg_slow message
 *
 * @return Wind Down +ve (m/s)
 */
static inline float mavlink_msg_pixhawk_hg_slow_get_wind_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a pixhawk_hg_slow message into a struct
 *
 * @param msg The message to decode
 * @param pixhawk_hg_slow C-struct to decode the message contents into
 */
static inline void mavlink_msg_pixhawk_hg_slow_decode(const mavlink_message_t* msg, mavlink_pixhawk_hg_slow_t* pixhawk_hg_slow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	pixhawk_hg_slow->gps_time_sec = mavlink_msg_pixhawk_hg_slow_get_gps_time_sec(msg);
	pixhawk_hg_slow->gps_alt = mavlink_msg_pixhawk_hg_slow_get_gps_alt(msg);
	pixhawk_hg_slow->temperature = mavlink_msg_pixhawk_hg_slow_get_temperature(msg);
	pixhawk_hg_slow->wind_n = mavlink_msg_pixhawk_hg_slow_get_wind_n(msg);
	pixhawk_hg_slow->wind_e = mavlink_msg_pixhawk_hg_slow_get_wind_e(msg);
	pixhawk_hg_slow->wind_d = mavlink_msg_pixhawk_hg_slow_get_wind_d(msg);
	pixhawk_hg_slow->eph = mavlink_msg_pixhawk_hg_slow_get_eph(msg);
	pixhawk_hg_slow->gps_fix_type = mavlink_msg_pixhawk_hg_slow_get_gps_fix_type(msg);
	pixhawk_hg_slow->satellites_visible = mavlink_msg_pixhawk_hg_slow_get_satellites_visible(msg);
	pixhawk_hg_slow->humidity = mavlink_msg_pixhawk_hg_slow_get_humidity(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN? msg->len : MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN;
        memset(pixhawk_hg_slow, 0, MAVLINK_MSG_ID_PIXHAWK_HG_SLOW_LEN);
	memcpy(pixhawk_hg_slow, _MAV_PAYLOAD(msg), len);
#endif
}
