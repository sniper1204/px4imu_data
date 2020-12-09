#pragma once
// MESSAGE DAHUA_ENUM_COMPONENT PACKING

#define MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT 230

MAVPACKED(
typedef struct __mavlink_dahua_enum_component_t {
 uint8_t dummy; /*< Dummy data*/
 uint8_t reserved[7]; /*< Place Holder.*/
}) mavlink_dahua_enum_component_t;

#define MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN 8
#define MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_MIN_LEN 8
#define MAVLINK_MSG_ID_230_LEN 8
#define MAVLINK_MSG_ID_230_MIN_LEN 8

#define MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_CRC 125
#define MAVLINK_MSG_ID_230_CRC 125

#define MAVLINK_MSG_DAHUA_ENUM_COMPONENT_FIELD_RESERVED_LEN 7

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DAHUA_ENUM_COMPONENT { \
    230, \
    "DAHUA_ENUM_COMPONENT", \
    2, \
    {  { "dummy", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_dahua_enum_component_t, dummy) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 7, 1, offsetof(mavlink_dahua_enum_component_t, reserved) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DAHUA_ENUM_COMPONENT { \
    "DAHUA_ENUM_COMPONENT", \
    2, \
    {  { "dummy", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_dahua_enum_component_t, dummy) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 7, 1, offsetof(mavlink_dahua_enum_component_t, reserved) }, \
         } \
}
#endif

/**
 * @brief Pack a dahua_enum_component message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param dummy Dummy data
 * @param reserved Place Holder.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dahua_enum_component_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t dummy, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN];
    _mav_put_uint8_t(buf, 0, dummy);
    _mav_put_uint8_t_array(buf, 1, reserved, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN);
#else
    mavlink_dahua_enum_component_t packet;
    packet.dummy = dummy;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_MIN_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_CRC);
}

/**
 * @brief Pack a dahua_enum_component message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dummy Dummy data
 * @param reserved Place Holder.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dahua_enum_component_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t dummy,const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN];
    _mav_put_uint8_t(buf, 0, dummy);
    _mav_put_uint8_t_array(buf, 1, reserved, 7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN);
#else
    mavlink_dahua_enum_component_t packet;
    packet.dummy = dummy;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*7);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_MIN_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_CRC);
}

/**
 * @brief Encode a dahua_enum_component struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dahua_enum_component C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dahua_enum_component_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dahua_enum_component_t* dahua_enum_component)
{
    return mavlink_msg_dahua_enum_component_pack(system_id, component_id, msg, dahua_enum_component->dummy, dahua_enum_component->reserved);
}

/**
 * @brief Encode a dahua_enum_component struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dahua_enum_component C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dahua_enum_component_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dahua_enum_component_t* dahua_enum_component)
{
    return mavlink_msg_dahua_enum_component_pack_chan(system_id, component_id, chan, msg, dahua_enum_component->dummy, dahua_enum_component->reserved);
}

/**
 * @brief Send a dahua_enum_component message
 * @param chan MAVLink channel to send the message
 *
 * @param dummy Dummy data
 * @param reserved Place Holder.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dahua_enum_component_send(mavlink_channel_t chan, uint8_t dummy, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN];
    _mav_put_uint8_t(buf, 0, dummy);
    _mav_put_uint8_t_array(buf, 1, reserved, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT, buf, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_MIN_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_CRC);
#else
    mavlink_dahua_enum_component_t packet;
    packet.dummy = dummy;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT, (const char *)&packet, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_MIN_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_CRC);
#endif
}

/**
 * @brief Send a dahua_enum_component message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_dahua_enum_component_send_struct(mavlink_channel_t chan, const mavlink_dahua_enum_component_t* dahua_enum_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dahua_enum_component_send(chan, dahua_enum_component->dummy, dahua_enum_component->reserved);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT, (const char *)dahua_enum_component, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_MIN_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_CRC);
#endif
}

#if MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_dahua_enum_component_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t dummy, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, dummy);
    _mav_put_uint8_t_array(buf, 1, reserved, 7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT, buf, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_MIN_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_CRC);
#else
    mavlink_dahua_enum_component_t *packet = (mavlink_dahua_enum_component_t *)msgbuf;
    packet->dummy = dummy;
    mav_array_memcpy(packet->reserved, reserved, sizeof(uint8_t)*7);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT, (const char *)packet, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_MIN_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_CRC);
#endif
}
#endif

#endif

// MESSAGE DAHUA_ENUM_COMPONENT UNPACKING


/**
 * @brief Get field dummy from dahua_enum_component message
 *
 * @return Dummy data
 */
static inline uint8_t mavlink_msg_dahua_enum_component_get_dummy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field reserved from dahua_enum_component message
 *
 * @return Place Holder.
 */
static inline uint16_t mavlink_msg_dahua_enum_component_get_reserved(const mavlink_message_t* msg, uint8_t *reserved)
{
    return _MAV_RETURN_uint8_t_array(msg, reserved, 7,  1);
}

/**
 * @brief Decode a dahua_enum_component message into a struct
 *
 * @param msg The message to decode
 * @param dahua_enum_component C-struct to decode the message contents into
 */
static inline void mavlink_msg_dahua_enum_component_decode(const mavlink_message_t* msg, mavlink_dahua_enum_component_t* dahua_enum_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    dahua_enum_component->dummy = mavlink_msg_dahua_enum_component_get_dummy(msg);
    mavlink_msg_dahua_enum_component_get_reserved(msg, dahua_enum_component->reserved);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN? msg->len : MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN;
        memset(dahua_enum_component, 0, MAVLINK_MSG_ID_DAHUA_ENUM_COMPONENT_LEN);
    memcpy(dahua_enum_component, _MAV_PAYLOAD(msg), len);
#endif
}
