#pragma once
// MESSAGE DAHUA_UPGRADE_REQUEST PACKING

#define MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST 228

MAVPACKED(
typedef struct __mavlink_dahua_upgrade_request_t {
 uint8_t id; /*< Component ID*/
 uint8_t reserved[7]; /*< Place Holder.*/
}) mavlink_dahua_upgrade_request_t;

#define MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN 8
#define MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_MIN_LEN 8
#define MAVLINK_MSG_ID_228_LEN 8
#define MAVLINK_MSG_ID_228_MIN_LEN 8

#define MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_CRC 62
#define MAVLINK_MSG_ID_228_CRC 62

#define MAVLINK_MSG_DAHUA_UPGRADE_REQUEST_FIELD_RESERVED_LEN 7

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DAHUA_UPGRADE_REQUEST { \
    228, \
    "DAHUA_UPGRADE_REQUEST", \
    2, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_dahua_upgrade_request_t, id) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 7, 1, offsetof(mavlink_dahua_upgrade_request_t, reserved) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DAHUA_UPGRADE_REQUEST { \
    "DAHUA_UPGRADE_REQUEST", \
    2, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_dahua_upgrade_request_t, id) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 7, 1, offsetof(mavlink_dahua_upgrade_request_t, reserved) }, \
         } \
}
#endif

/**
 * @brief Pack a dahua_upgrade_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id Component ID
 * @param reserved Place Holder.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dahua_upgrade_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, id);
    _mav_put_uint8_t_array(buf, 1, reserved, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN);
#else
    mavlink_dahua_upgrade_request_t packet;
    packet.id = id;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_CRC);
}

/**
 * @brief Pack a dahua_upgrade_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id Component ID
 * @param reserved Place Holder.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dahua_upgrade_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, id);
    _mav_put_uint8_t_array(buf, 1, reserved, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN);
#else
    mavlink_dahua_upgrade_request_t packet;
    packet.id = id;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_CRC);
}

/**
 * @brief Encode a dahua_upgrade_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dahua_upgrade_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dahua_upgrade_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dahua_upgrade_request_t* dahua_upgrade_request)
{
    return mavlink_msg_dahua_upgrade_request_pack(system_id, component_id, msg, dahua_upgrade_request->id, dahua_upgrade_request->reserved);
}

/**
 * @brief Encode a dahua_upgrade_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dahua_upgrade_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dahua_upgrade_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dahua_upgrade_request_t* dahua_upgrade_request)
{
    return mavlink_msg_dahua_upgrade_request_pack_chan(system_id, component_id, chan, msg, dahua_upgrade_request->id, dahua_upgrade_request->reserved);
}

/**
 * @brief Send a dahua_upgrade_request message
 * @param chan MAVLink channel to send the message
 *
 * @param id Component ID
 * @param reserved Place Holder.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dahua_upgrade_request_send(mavlink_channel_t chan, uint8_t id, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, id);
    _mav_put_uint8_t_array(buf, 1, reserved, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST, buf, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_CRC);
#else
    mavlink_dahua_upgrade_request_t packet;
    packet.id = id;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_CRC);
#endif
}

/**
 * @brief Send a dahua_upgrade_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_dahua_upgrade_request_send_struct(mavlink_channel_t chan, const mavlink_dahua_upgrade_request_t* dahua_upgrade_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dahua_upgrade_request_send(chan, dahua_upgrade_request->id, dahua_upgrade_request->reserved);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST, (const char *)dahua_upgrade_request, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_dahua_upgrade_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, id);
    _mav_put_uint8_t_array(buf, 1, reserved, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST, buf, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_CRC);
#else
    mavlink_dahua_upgrade_request_t *packet = (mavlink_dahua_upgrade_request_t *)msgbuf;
    packet->id = id;
    mav_array_memcpy(packet->reserved, reserved, sizeof(uint8_t)*7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST, (const char *)packet, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE DAHUA_UPGRADE_REQUEST UNPACKING


/**
 * @brief Get field id from dahua_upgrade_request message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_dahua_upgrade_request_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field reserved from dahua_upgrade_request message
 *
 * @return Place Holder.
 */
static inline uint16_t mavlink_msg_dahua_upgrade_request_get_reserved(const mavlink_message_t* msg, uint8_t *reserved)
{
    return _MAV_RETURN_uint8_t_array(msg, reserved, 7,  1);
}

/**
 * @brief Decode a dahua_upgrade_request message into a struct
 *
 * @param msg The message to decode
 * @param dahua_upgrade_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_dahua_upgrade_request_decode(const mavlink_message_t* msg, mavlink_dahua_upgrade_request_t* dahua_upgrade_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    dahua_upgrade_request->id = mavlink_msg_dahua_upgrade_request_get_id(msg);
    mavlink_msg_dahua_upgrade_request_get_reserved(msg, dahua_upgrade_request->reserved);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN;
        memset(dahua_upgrade_request, 0, MAVLINK_MSG_ID_DAHUA_UPGRADE_REQUEST_LEN);
    memcpy(dahua_upgrade_request, _MAV_PAYLOAD(msg), len);
#endif
}
