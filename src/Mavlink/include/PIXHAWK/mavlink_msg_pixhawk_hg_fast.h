// MESSAGE PIXHAWK_HG_FAST PACKING

#define MAVLINK_MSG_ID_PIXHAWK_HG_FAST 230

MAVPACKED(
typedef struct __mavlink_pixhawk_hg_fast_t {
 uint32_t gps_time; /*< time since UTC midnight in milliseconds*/
 float xacc; /*< X acceleration (m/s^2)*/
 float yacc; /*< Y acceleration (m/s^2)*/
 float zacc; /*< Z acceleration (m/s^2)*/
 float roll; /*< Roll angle (rad, -pi..+pi)*/
 float pitch; /*< Pitch angle (rad, -pi..+pi)*/
 uint16_t hdg; /*< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
}) mavlink_pixhawk_hg_fast_t;

#define MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN 26
#define MAVLINK_MSG_ID_PIXHAWK_HG_FAST_MIN_LEN 26
#define MAVLINK_MSG_ID_230_LEN 26
#define MAVLINK_MSG_ID_230_MIN_LEN 26

#define MAVLINK_MSG_ID_PIXHAWK_HG_FAST_CRC 186
#define MAVLINK_MSG_ID_230_CRC 186



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PIXHAWK_HG_FAST { \
	230, \
	"PIXHAWK_HG_FAST", \
	7, \
	{  { "gps_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_pixhawk_hg_fast_t, gps_time) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pixhawk_hg_fast_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pixhawk_hg_fast_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pixhawk_hg_fast_t, zacc) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pixhawk_hg_fast_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pixhawk_hg_fast_t, pitch) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_pixhawk_hg_fast_t, hdg) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PIXHAWK_HG_FAST { \
	"PIXHAWK_HG_FAST", \
	7, \
	{  { "gps_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_pixhawk_hg_fast_t, gps_time) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pixhawk_hg_fast_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pixhawk_hg_fast_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pixhawk_hg_fast_t, zacc) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pixhawk_hg_fast_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pixhawk_hg_fast_t, pitch) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_pixhawk_hg_fast_t, hdg) }, \
         } \
}
#endif

/**
 * @brief Pack a pixhawk_hg_fast message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gps_time time since UTC midnight in milliseconds
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pixhawk_hg_fast_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t gps_time, float xacc, float yacc, float zacc, float roll, float pitch, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN];
	_mav_put_uint32_t(buf, 0, gps_time);
	_mav_put_float(buf, 4, xacc);
	_mav_put_float(buf, 8, yacc);
	_mav_put_float(buf, 12, zacc);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_uint16_t(buf, 24, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN);
#else
	mavlink_pixhawk_hg_fast_t packet;
	packet.gps_time = gps_time;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PIXHAWK_HG_FAST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_CRC);
}

/**
 * @brief Pack a pixhawk_hg_fast message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_time time since UTC midnight in milliseconds
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pixhawk_hg_fast_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t gps_time,float xacc,float yacc,float zacc,float roll,float pitch,uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN];
	_mav_put_uint32_t(buf, 0, gps_time);
	_mav_put_float(buf, 4, xacc);
	_mav_put_float(buf, 8, yacc);
	_mav_put_float(buf, 12, zacc);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_uint16_t(buf, 24, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN);
#else
	mavlink_pixhawk_hg_fast_t packet;
	packet.gps_time = gps_time;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PIXHAWK_HG_FAST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_CRC);
}

/**
 * @brief Encode a pixhawk_hg_fast struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pixhawk_hg_fast C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pixhawk_hg_fast_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pixhawk_hg_fast_t* pixhawk_hg_fast)
{
	return mavlink_msg_pixhawk_hg_fast_pack(system_id, component_id, msg, pixhawk_hg_fast->gps_time, pixhawk_hg_fast->xacc, pixhawk_hg_fast->yacc, pixhawk_hg_fast->zacc, pixhawk_hg_fast->roll, pixhawk_hg_fast->pitch, pixhawk_hg_fast->hdg);
}

/**
 * @brief Encode a pixhawk_hg_fast struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pixhawk_hg_fast C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pixhawk_hg_fast_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pixhawk_hg_fast_t* pixhawk_hg_fast)
{
	return mavlink_msg_pixhawk_hg_fast_pack_chan(system_id, component_id, chan, msg, pixhawk_hg_fast->gps_time, pixhawk_hg_fast->xacc, pixhawk_hg_fast->yacc, pixhawk_hg_fast->zacc, pixhawk_hg_fast->roll, pixhawk_hg_fast->pitch, pixhawk_hg_fast->hdg);
}

/**
 * @brief Send a pixhawk_hg_fast message
 * @param chan MAVLink channel to send the message
 *
 * @param gps_time time since UTC midnight in milliseconds
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pixhawk_hg_fast_send(mavlink_channel_t chan, uint32_t gps_time, float xacc, float yacc, float zacc, float roll, float pitch, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN];
	_mav_put_uint32_t(buf, 0, gps_time);
	_mav_put_float(buf, 4, xacc);
	_mav_put_float(buf, 8, yacc);
	_mav_put_float(buf, 12, zacc);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_uint16_t(buf, 24, hdg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_FAST, buf, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_CRC);
#else
	mavlink_pixhawk_hg_fast_t packet;
	packet.gps_time = gps_time;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.hdg = hdg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_FAST, (const char *)&packet, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_CRC);
#endif
}

/**
 * @brief Send a pixhawk_hg_fast message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pixhawk_hg_fast_send_struct(mavlink_channel_t chan, const mavlink_pixhawk_hg_fast_t* pixhawk_hg_fast)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pixhawk_hg_fast_send(chan, pixhawk_hg_fast->gps_time, pixhawk_hg_fast->xacc, pixhawk_hg_fast->yacc, pixhawk_hg_fast->zacc, pixhawk_hg_fast->roll, pixhawk_hg_fast->pitch, pixhawk_hg_fast->hdg);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_FAST, (const char *)pixhawk_hg_fast, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_CRC);
#endif
}

#if MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pixhawk_hg_fast_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t gps_time, float xacc, float yacc, float zacc, float roll, float pitch, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, gps_time);
	_mav_put_float(buf, 4, xacc);
	_mav_put_float(buf, 8, yacc);
	_mav_put_float(buf, 12, zacc);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_uint16_t(buf, 24, hdg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_FAST, buf, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_CRC);
#else
	mavlink_pixhawk_hg_fast_t *packet = (mavlink_pixhawk_hg_fast_t *)msgbuf;
	packet->gps_time = gps_time;
	packet->xacc = xacc;
	packet->yacc = yacc;
	packet->zacc = zacc;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->hdg = hdg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_FAST, (const char *)packet, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_CRC);
#endif
}
#endif

#endif

// MESSAGE PIXHAWK_HG_FAST UNPACKING


/**
 * @brief Get field gps_time from pixhawk_hg_fast message
 *
 * @return time since UTC midnight in milliseconds
 */
static inline uint32_t mavlink_msg_pixhawk_hg_fast_get_gps_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field xacc from pixhawk_hg_fast message
 *
 * @return X acceleration (m/s^2)
 */
static inline float mavlink_msg_pixhawk_hg_fast_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yacc from pixhawk_hg_fast message
 *
 * @return Y acceleration (m/s^2)
 */
static inline float mavlink_msg_pixhawk_hg_fast_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field zacc from pixhawk_hg_fast message
 *
 * @return Z acceleration (m/s^2)
 */
static inline float mavlink_msg_pixhawk_hg_fast_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll from pixhawk_hg_fast message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_pixhawk_hg_fast_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitch from pixhawk_hg_fast message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_pixhawk_hg_fast_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field hdg from pixhawk_hg_fast message
 *
 * @return Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_pixhawk_hg_fast_get_hdg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Decode a pixhawk_hg_fast message into a struct
 *
 * @param msg The message to decode
 * @param pixhawk_hg_fast C-struct to decode the message contents into
 */
static inline void mavlink_msg_pixhawk_hg_fast_decode(const mavlink_message_t* msg, mavlink_pixhawk_hg_fast_t* pixhawk_hg_fast)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	pixhawk_hg_fast->gps_time = mavlink_msg_pixhawk_hg_fast_get_gps_time(msg);
	pixhawk_hg_fast->xacc = mavlink_msg_pixhawk_hg_fast_get_xacc(msg);
	pixhawk_hg_fast->yacc = mavlink_msg_pixhawk_hg_fast_get_yacc(msg);
	pixhawk_hg_fast->zacc = mavlink_msg_pixhawk_hg_fast_get_zacc(msg);
	pixhawk_hg_fast->roll = mavlink_msg_pixhawk_hg_fast_get_roll(msg);
	pixhawk_hg_fast->pitch = mavlink_msg_pixhawk_hg_fast_get_pitch(msg);
	pixhawk_hg_fast->hdg = mavlink_msg_pixhawk_hg_fast_get_hdg(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN? msg->len : MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN;
        memset(pixhawk_hg_fast, 0, MAVLINK_MSG_ID_PIXHAWK_HG_FAST_LEN);
	memcpy(pixhawk_hg_fast, _MAV_PAYLOAD(msg), len);
#endif
}
