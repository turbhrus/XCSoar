// MESSAGE XCSOAR_CALCULATED PACKING

#define MAVLINK_MSG_ID_XCSOAR_CALCULATED 233

MAVPACKED(
typedef struct __mavlink_xcsoar_calculated_t {
 float speed_to_fly; /*< Optimal IAS (m/s)*/
 uint16_t current_turnpoint; /*< ID of target waypoint*/
 uint8_t flying; /*< Non zero if flying detected*/
 uint8_t circling; /*< Non zero if thermalling detected*/
 uint8_t airspace_warning_idx; /*< Unique warning identifier. Wraps around after 255*/
 uint8_t terrain_warning; /*< */
}) mavlink_xcsoar_calculated_t;

#define MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN 10
#define MAVLINK_MSG_ID_XCSOAR_CALCULATED_MIN_LEN 10
#define MAVLINK_MSG_ID_233_LEN 10
#define MAVLINK_MSG_ID_233_MIN_LEN 10

#define MAVLINK_MSG_ID_XCSOAR_CALCULATED_CRC 31
#define MAVLINK_MSG_ID_233_CRC 31



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_XCSOAR_CALCULATED { \
	233, \
	"XCSOAR_CALCULATED", \
	6, \
	{  { "speed_to_fly", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_xcsoar_calculated_t, speed_to_fly) }, \
         { "current_turnpoint", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_xcsoar_calculated_t, current_turnpoint) }, \
         { "flying", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_xcsoar_calculated_t, flying) }, \
         { "circling", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_xcsoar_calculated_t, circling) }, \
         { "airspace_warning_idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_xcsoar_calculated_t, airspace_warning_idx) }, \
         { "terrain_warning", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_xcsoar_calculated_t, terrain_warning) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_XCSOAR_CALCULATED { \
	"XCSOAR_CALCULATED", \
	6, \
	{  { "speed_to_fly", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_xcsoar_calculated_t, speed_to_fly) }, \
         { "current_turnpoint", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_xcsoar_calculated_t, current_turnpoint) }, \
         { "flying", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_xcsoar_calculated_t, flying) }, \
         { "circling", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_xcsoar_calculated_t, circling) }, \
         { "airspace_warning_idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_xcsoar_calculated_t, airspace_warning_idx) }, \
         { "terrain_warning", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_xcsoar_calculated_t, terrain_warning) }, \
         } \
}
#endif

/**
 * @brief Pack a xcsoar_calculated message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param speed_to_fly Optimal IAS (m/s)
 * @param flying Non zero if flying detected
 * @param circling Non zero if thermalling detected
 * @param current_turnpoint ID of target waypoint
 * @param airspace_warning_idx Unique warning identifier. Wraps around after 255
 * @param terrain_warning 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xcsoar_calculated_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float speed_to_fly, uint8_t flying, uint8_t circling, uint16_t current_turnpoint, uint8_t airspace_warning_idx, uint8_t terrain_warning)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN];
	_mav_put_float(buf, 0, speed_to_fly);
	_mav_put_uint16_t(buf, 4, current_turnpoint);
	_mav_put_uint8_t(buf, 6, flying);
	_mav_put_uint8_t(buf, 7, circling);
	_mav_put_uint8_t(buf, 8, airspace_warning_idx);
	_mav_put_uint8_t(buf, 9, terrain_warning);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN);
#else
	mavlink_xcsoar_calculated_t packet;
	packet.speed_to_fly = speed_to_fly;
	packet.current_turnpoint = current_turnpoint;
	packet.flying = flying;
	packet.circling = circling;
	packet.airspace_warning_idx = airspace_warning_idx;
	packet.terrain_warning = terrain_warning;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XCSOAR_CALCULATED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_XCSOAR_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_CRC);
}

/**
 * @brief Pack a xcsoar_calculated message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param speed_to_fly Optimal IAS (m/s)
 * @param flying Non zero if flying detected
 * @param circling Non zero if thermalling detected
 * @param current_turnpoint ID of target waypoint
 * @param airspace_warning_idx Unique warning identifier. Wraps around after 255
 * @param terrain_warning 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_xcsoar_calculated_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float speed_to_fly,uint8_t flying,uint8_t circling,uint16_t current_turnpoint,uint8_t airspace_warning_idx,uint8_t terrain_warning)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN];
	_mav_put_float(buf, 0, speed_to_fly);
	_mav_put_uint16_t(buf, 4, current_turnpoint);
	_mav_put_uint8_t(buf, 6, flying);
	_mav_put_uint8_t(buf, 7, circling);
	_mav_put_uint8_t(buf, 8, airspace_warning_idx);
	_mav_put_uint8_t(buf, 9, terrain_warning);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN);
#else
	mavlink_xcsoar_calculated_t packet;
	packet.speed_to_fly = speed_to_fly;
	packet.current_turnpoint = current_turnpoint;
	packet.flying = flying;
	packet.circling = circling;
	packet.airspace_warning_idx = airspace_warning_idx;
	packet.terrain_warning = terrain_warning;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_XCSOAR_CALCULATED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_XCSOAR_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_CRC);
}

/**
 * @brief Encode a xcsoar_calculated struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param xcsoar_calculated C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xcsoar_calculated_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_xcsoar_calculated_t* xcsoar_calculated)
{
	return mavlink_msg_xcsoar_calculated_pack(system_id, component_id, msg, xcsoar_calculated->speed_to_fly, xcsoar_calculated->flying, xcsoar_calculated->circling, xcsoar_calculated->current_turnpoint, xcsoar_calculated->airspace_warning_idx, xcsoar_calculated->terrain_warning);
}

/**
 * @brief Encode a xcsoar_calculated struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param xcsoar_calculated C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_xcsoar_calculated_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_xcsoar_calculated_t* xcsoar_calculated)
{
	return mavlink_msg_xcsoar_calculated_pack_chan(system_id, component_id, chan, msg, xcsoar_calculated->speed_to_fly, xcsoar_calculated->flying, xcsoar_calculated->circling, xcsoar_calculated->current_turnpoint, xcsoar_calculated->airspace_warning_idx, xcsoar_calculated->terrain_warning);
}

/**
 * @brief Send a xcsoar_calculated message
 * @param chan MAVLink channel to send the message
 *
 * @param speed_to_fly Optimal IAS (m/s)
 * @param flying Non zero if flying detected
 * @param circling Non zero if thermalling detected
 * @param current_turnpoint ID of target waypoint
 * @param airspace_warning_idx Unique warning identifier. Wraps around after 255
 * @param terrain_warning 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_xcsoar_calculated_send(mavlink_channel_t chan, float speed_to_fly, uint8_t flying, uint8_t circling, uint16_t current_turnpoint, uint8_t airspace_warning_idx, uint8_t terrain_warning)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN];
	_mav_put_float(buf, 0, speed_to_fly);
	_mav_put_uint16_t(buf, 4, current_turnpoint);
	_mav_put_uint8_t(buf, 6, flying);
	_mav_put_uint8_t(buf, 7, circling);
	_mav_put_uint8_t(buf, 8, airspace_warning_idx);
	_mav_put_uint8_t(buf, 9, terrain_warning);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XCSOAR_CALCULATED, buf, MAVLINK_MSG_ID_XCSOAR_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_CRC);
#else
	mavlink_xcsoar_calculated_t packet;
	packet.speed_to_fly = speed_to_fly;
	packet.current_turnpoint = current_turnpoint;
	packet.flying = flying;
	packet.circling = circling;
	packet.airspace_warning_idx = airspace_warning_idx;
	packet.terrain_warning = terrain_warning;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XCSOAR_CALCULATED, (const char *)&packet, MAVLINK_MSG_ID_XCSOAR_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_CRC);
#endif
}

/**
 * @brief Send a xcsoar_calculated message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_xcsoar_calculated_send_struct(mavlink_channel_t chan, const mavlink_xcsoar_calculated_t* xcsoar_calculated)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_xcsoar_calculated_send(chan, xcsoar_calculated->speed_to_fly, xcsoar_calculated->flying, xcsoar_calculated->circling, xcsoar_calculated->current_turnpoint, xcsoar_calculated->airspace_warning_idx, xcsoar_calculated->terrain_warning);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XCSOAR_CALCULATED, (const char *)xcsoar_calculated, MAVLINK_MSG_ID_XCSOAR_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_CRC);
#endif
}

#if MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_xcsoar_calculated_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float speed_to_fly, uint8_t flying, uint8_t circling, uint16_t current_turnpoint, uint8_t airspace_warning_idx, uint8_t terrain_warning)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, speed_to_fly);
	_mav_put_uint16_t(buf, 4, current_turnpoint);
	_mav_put_uint8_t(buf, 6, flying);
	_mav_put_uint8_t(buf, 7, circling);
	_mav_put_uint8_t(buf, 8, airspace_warning_idx);
	_mav_put_uint8_t(buf, 9, terrain_warning);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XCSOAR_CALCULATED, buf, MAVLINK_MSG_ID_XCSOAR_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_CRC);
#else
	mavlink_xcsoar_calculated_t *packet = (mavlink_xcsoar_calculated_t *)msgbuf;
	packet->speed_to_fly = speed_to_fly;
	packet->current_turnpoint = current_turnpoint;
	packet->flying = flying;
	packet->circling = circling;
	packet->airspace_warning_idx = airspace_warning_idx;
	packet->terrain_warning = terrain_warning;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_XCSOAR_CALCULATED, (const char *)packet, MAVLINK_MSG_ID_XCSOAR_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN, MAVLINK_MSG_ID_XCSOAR_CALCULATED_CRC);
#endif
}
#endif

#endif

// MESSAGE XCSOAR_CALCULATED UNPACKING


/**
 * @brief Get field speed_to_fly from xcsoar_calculated message
 *
 * @return Optimal IAS (m/s)
 */
static inline float mavlink_msg_xcsoar_calculated_get_speed_to_fly(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field flying from xcsoar_calculated message
 *
 * @return Non zero if flying detected
 */
static inline uint8_t mavlink_msg_xcsoar_calculated_get_flying(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field circling from xcsoar_calculated message
 *
 * @return Non zero if thermalling detected
 */
static inline uint8_t mavlink_msg_xcsoar_calculated_get_circling(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field current_turnpoint from xcsoar_calculated message
 *
 * @return ID of target waypoint
 */
static inline uint16_t mavlink_msg_xcsoar_calculated_get_current_turnpoint(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field airspace_warning_idx from xcsoar_calculated message
 *
 * @return Unique warning identifier. Wraps around after 255
 */
static inline uint8_t mavlink_msg_xcsoar_calculated_get_airspace_warning_idx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field terrain_warning from xcsoar_calculated message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_xcsoar_calculated_get_terrain_warning(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Decode a xcsoar_calculated message into a struct
 *
 * @param msg The message to decode
 * @param xcsoar_calculated C-struct to decode the message contents into
 */
static inline void mavlink_msg_xcsoar_calculated_decode(const mavlink_message_t* msg, mavlink_xcsoar_calculated_t* xcsoar_calculated)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	xcsoar_calculated->speed_to_fly = mavlink_msg_xcsoar_calculated_get_speed_to_fly(msg);
	xcsoar_calculated->current_turnpoint = mavlink_msg_xcsoar_calculated_get_current_turnpoint(msg);
	xcsoar_calculated->flying = mavlink_msg_xcsoar_calculated_get_flying(msg);
	xcsoar_calculated->circling = mavlink_msg_xcsoar_calculated_get_circling(msg);
	xcsoar_calculated->airspace_warning_idx = mavlink_msg_xcsoar_calculated_get_airspace_warning_idx(msg);
	xcsoar_calculated->terrain_warning = mavlink_msg_xcsoar_calculated_get_terrain_warning(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN? msg->len : MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN;
        memset(xcsoar_calculated, 0, MAVLINK_MSG_ID_XCSOAR_CALCULATED_LEN);
	memcpy(xcsoar_calculated, _MAV_PAYLOAD(msg), len);
#endif
}
