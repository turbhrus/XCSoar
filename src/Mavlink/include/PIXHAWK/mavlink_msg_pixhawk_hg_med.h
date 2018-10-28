// MESSAGE PIXHAWK_HG_MED PACKING

#define MAVLINK_MSG_ID_PIXHAWK_HG_MED 231

MAVPACKED(
typedef struct __mavlink_pixhawk_hg_med_t {
 int32_t lat; /*< Latitude, expressed as * 1E7*/
 int32_t lon; /*< Longitude, expressed as * 1E7*/
 int32_t alt; /*< Altitude, expressed as mm AMSL*/
 float indicated_airspeed; /*< Current IAS in m/s*/
 float true_airspeed; /*< Current TAS in m/s*/
 int16_t vx; /*< Ground X Speed (Latitude), expressed as cm/s*/
 int16_t vy; /*< Ground Y Speed (Longitude), expressed as cm/s*/
 int16_t vz; /*< Vario, expressed as cm/s*/
 int16_t energy_vario; /*< Energy compensated vario, expressed as cm/s*/
}) mavlink_pixhawk_hg_med_t;

#define MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN 28
#define MAVLINK_MSG_ID_PIXHAWK_HG_MED_MIN_LEN 28
#define MAVLINK_MSG_ID_231_LEN 28
#define MAVLINK_MSG_ID_231_MIN_LEN 28

#define MAVLINK_MSG_ID_PIXHAWK_HG_MED_CRC 44
#define MAVLINK_MSG_ID_231_CRC 44



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PIXHAWK_HG_MED { \
	231, \
	"PIXHAWK_HG_MED", \
	9, \
	{  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_pixhawk_hg_med_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_pixhawk_hg_med_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_pixhawk_hg_med_t, alt) }, \
         { "indicated_airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pixhawk_hg_med_t, indicated_airspeed) }, \
         { "true_airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pixhawk_hg_med_t, true_airspeed) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_pixhawk_hg_med_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_pixhawk_hg_med_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_pixhawk_hg_med_t, vz) }, \
         { "energy_vario", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_pixhawk_hg_med_t, energy_vario) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PIXHAWK_HG_MED { \
	"PIXHAWK_HG_MED", \
	9, \
	{  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_pixhawk_hg_med_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_pixhawk_hg_med_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_pixhawk_hg_med_t, alt) }, \
         { "indicated_airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pixhawk_hg_med_t, indicated_airspeed) }, \
         { "true_airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pixhawk_hg_med_t, true_airspeed) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_pixhawk_hg_med_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_pixhawk_hg_med_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_pixhawk_hg_med_t, vz) }, \
         { "energy_vario", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_pixhawk_hg_med_t, energy_vario) }, \
         } \
}
#endif

/**
 * @brief Pack a pixhawk_hg_med message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude, expressed as mm AMSL
 * @param vx Ground X Speed (Latitude), expressed as cm/s
 * @param vy Ground Y Speed (Longitude), expressed as cm/s
 * @param vz Vario, expressed as cm/s
 * @param energy_vario Energy compensated vario, expressed as cm/s
 * @param indicated_airspeed Current IAS in m/s
 * @param true_airspeed Current TAS in m/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pixhawk_hg_med_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t energy_vario, float indicated_airspeed, float true_airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_float(buf, 12, indicated_airspeed);
	_mav_put_float(buf, 16, true_airspeed);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_int16_t(buf, 26, energy_vario);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN);
#else
	mavlink_pixhawk_hg_med_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.indicated_airspeed = indicated_airspeed;
	packet.true_airspeed = true_airspeed;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.energy_vario = energy_vario;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PIXHAWK_HG_MED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PIXHAWK_HG_MED_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_CRC);
}

/**
 * @brief Pack a pixhawk_hg_med message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude, expressed as mm AMSL
 * @param vx Ground X Speed (Latitude), expressed as cm/s
 * @param vy Ground Y Speed (Longitude), expressed as cm/s
 * @param vz Vario, expressed as cm/s
 * @param energy_vario Energy compensated vario, expressed as cm/s
 * @param indicated_airspeed Current IAS in m/s
 * @param true_airspeed Current TAS in m/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pixhawk_hg_med_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t lat,int32_t lon,int32_t alt,int16_t vx,int16_t vy,int16_t vz,int16_t energy_vario,float indicated_airspeed,float true_airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_float(buf, 12, indicated_airspeed);
	_mav_put_float(buf, 16, true_airspeed);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_int16_t(buf, 26, energy_vario);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN);
#else
	mavlink_pixhawk_hg_med_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.indicated_airspeed = indicated_airspeed;
	packet.true_airspeed = true_airspeed;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.energy_vario = energy_vario;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PIXHAWK_HG_MED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PIXHAWK_HG_MED_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_CRC);
}

/**
 * @brief Encode a pixhawk_hg_med struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pixhawk_hg_med C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pixhawk_hg_med_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pixhawk_hg_med_t* pixhawk_hg_med)
{
	return mavlink_msg_pixhawk_hg_med_pack(system_id, component_id, msg, pixhawk_hg_med->lat, pixhawk_hg_med->lon, pixhawk_hg_med->alt, pixhawk_hg_med->vx, pixhawk_hg_med->vy, pixhawk_hg_med->vz, pixhawk_hg_med->energy_vario, pixhawk_hg_med->indicated_airspeed, pixhawk_hg_med->true_airspeed);
}

/**
 * @brief Encode a pixhawk_hg_med struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pixhawk_hg_med C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pixhawk_hg_med_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pixhawk_hg_med_t* pixhawk_hg_med)
{
	return mavlink_msg_pixhawk_hg_med_pack_chan(system_id, component_id, chan, msg, pixhawk_hg_med->lat, pixhawk_hg_med->lon, pixhawk_hg_med->alt, pixhawk_hg_med->vx, pixhawk_hg_med->vy, pixhawk_hg_med->vz, pixhawk_hg_med->energy_vario, pixhawk_hg_med->indicated_airspeed, pixhawk_hg_med->true_airspeed);
}

/**
 * @brief Send a pixhawk_hg_med message
 * @param chan MAVLink channel to send the message
 *
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude, expressed as mm AMSL
 * @param vx Ground X Speed (Latitude), expressed as cm/s
 * @param vy Ground Y Speed (Longitude), expressed as cm/s
 * @param vz Vario, expressed as cm/s
 * @param energy_vario Energy compensated vario, expressed as cm/s
 * @param indicated_airspeed Current IAS in m/s
 * @param true_airspeed Current TAS in m/s
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pixhawk_hg_med_send(mavlink_channel_t chan, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t energy_vario, float indicated_airspeed, float true_airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_float(buf, 12, indicated_airspeed);
	_mav_put_float(buf, 16, true_airspeed);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_int16_t(buf, 26, energy_vario);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_MED, buf, MAVLINK_MSG_ID_PIXHAWK_HG_MED_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_CRC);
#else
	mavlink_pixhawk_hg_med_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.indicated_airspeed = indicated_airspeed;
	packet.true_airspeed = true_airspeed;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.energy_vario = energy_vario;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_MED, (const char *)&packet, MAVLINK_MSG_ID_PIXHAWK_HG_MED_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_CRC);
#endif
}

/**
 * @brief Send a pixhawk_hg_med message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pixhawk_hg_med_send_struct(mavlink_channel_t chan, const mavlink_pixhawk_hg_med_t* pixhawk_hg_med)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pixhawk_hg_med_send(chan, pixhawk_hg_med->lat, pixhawk_hg_med->lon, pixhawk_hg_med->alt, pixhawk_hg_med->vx, pixhawk_hg_med->vy, pixhawk_hg_med->vz, pixhawk_hg_med->energy_vario, pixhawk_hg_med->indicated_airspeed, pixhawk_hg_med->true_airspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_MED, (const char *)pixhawk_hg_med, MAVLINK_MSG_ID_PIXHAWK_HG_MED_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_CRC);
#endif
}

#if MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pixhawk_hg_med_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t energy_vario, float indicated_airspeed, float true_airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_float(buf, 12, indicated_airspeed);
	_mav_put_float(buf, 16, true_airspeed);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_int16_t(buf, 26, energy_vario);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_MED, buf, MAVLINK_MSG_ID_PIXHAWK_HG_MED_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_CRC);
#else
	mavlink_pixhawk_hg_med_t *packet = (mavlink_pixhawk_hg_med_t *)msgbuf;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->indicated_airspeed = indicated_airspeed;
	packet->true_airspeed = true_airspeed;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	packet->energy_vario = energy_vario;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PIXHAWK_HG_MED, (const char *)packet, MAVLINK_MSG_ID_PIXHAWK_HG_MED_MIN_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN, MAVLINK_MSG_ID_PIXHAWK_HG_MED_CRC);
#endif
}
#endif

#endif

// MESSAGE PIXHAWK_HG_MED UNPACKING


/**
 * @brief Get field lat from pixhawk_hg_med message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_pixhawk_hg_med_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from pixhawk_hg_med message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_pixhawk_hg_med_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from pixhawk_hg_med message
 *
 * @return Altitude, expressed as mm AMSL
 */
static inline int32_t mavlink_msg_pixhawk_hg_med_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field vx from pixhawk_hg_med message
 *
 * @return Ground X Speed (Latitude), expressed as cm/s
 */
static inline int16_t mavlink_msg_pixhawk_hg_med_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field vy from pixhawk_hg_med message
 *
 * @return Ground Y Speed (Longitude), expressed as cm/s
 */
static inline int16_t mavlink_msg_pixhawk_hg_med_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field vz from pixhawk_hg_med message
 *
 * @return Vario, expressed as cm/s
 */
static inline int16_t mavlink_msg_pixhawk_hg_med_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field energy_vario from pixhawk_hg_med message
 *
 * @return Energy compensated vario, expressed as cm/s
 */
static inline int16_t mavlink_msg_pixhawk_hg_med_get_energy_vario(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field indicated_airspeed from pixhawk_hg_med message
 *
 * @return Current IAS in m/s
 */
static inline float mavlink_msg_pixhawk_hg_med_get_indicated_airspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field true_airspeed from pixhawk_hg_med message
 *
 * @return Current TAS in m/s
 */
static inline float mavlink_msg_pixhawk_hg_med_get_true_airspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a pixhawk_hg_med message into a struct
 *
 * @param msg The message to decode
 * @param pixhawk_hg_med C-struct to decode the message contents into
 */
static inline void mavlink_msg_pixhawk_hg_med_decode(const mavlink_message_t* msg, mavlink_pixhawk_hg_med_t* pixhawk_hg_med)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	pixhawk_hg_med->lat = mavlink_msg_pixhawk_hg_med_get_lat(msg);
	pixhawk_hg_med->lon = mavlink_msg_pixhawk_hg_med_get_lon(msg);
	pixhawk_hg_med->alt = mavlink_msg_pixhawk_hg_med_get_alt(msg);
	pixhawk_hg_med->indicated_airspeed = mavlink_msg_pixhawk_hg_med_get_indicated_airspeed(msg);
	pixhawk_hg_med->true_airspeed = mavlink_msg_pixhawk_hg_med_get_true_airspeed(msg);
	pixhawk_hg_med->vx = mavlink_msg_pixhawk_hg_med_get_vx(msg);
	pixhawk_hg_med->vy = mavlink_msg_pixhawk_hg_med_get_vy(msg);
	pixhawk_hg_med->vz = mavlink_msg_pixhawk_hg_med_get_vz(msg);
	pixhawk_hg_med->energy_vario = mavlink_msg_pixhawk_hg_med_get_energy_vario(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN? msg->len : MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN;
        memset(pixhawk_hg_med, 0, MAVLINK_MSG_ID_PIXHAWK_HG_MED_LEN);
	memcpy(pixhawk_hg_med, _MAV_PAYLOAD(msg), len);
#endif
}
