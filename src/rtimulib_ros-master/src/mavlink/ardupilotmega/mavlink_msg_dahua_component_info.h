#pragma once
// MESSAGE DAHUA_COMPONENT_INFO PACKING

#define MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO 231

MAVPACKED(
typedef struct __mavlink_dahua_component_info_t {
 uint32_t patch; /*< Firmware patch level.*/
 uint8_t id; /*< Component ID*/
 uint8_t mode; /*< Current system mode.*/
 uint8_t major; /*< Firmware major version.*/
 uint8_t minor; /*< Firmware minor version.*/
 uint8_t reserved[4]; /*< Place Holder.*/
}) mavlink_dahua_component_info_t;

#define MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN 12
#define MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_MIN_LEN 12
#define MAVLINK_MSG_ID_231_LEN 12
#define MAVLINK_MSG_ID_231_MIN_LEN 12

#define MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_CRC 247
#define MAVLINK_MSG_ID_231_CRC 247

#define MAVLINK_MSG_DAHUA_COMPONENT_INFO_FIELD_RESERVED_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DAHUA_COMPONENT_INFO { \
    231, \
    "DAHUA_COMPONENT_INFO", \
    6, \
    {  { "patch", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_dahua_component_info_t, patch) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_dahua_component_info_t, id) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_dahua_component_info_t, mode) }, \
         { "major", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_dahua_component_info_t, major) }, \
         { "minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_dahua_component_info_t, minor) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 4, 8, offsetof(mavlink_dahua_component_info_t, reserved) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DAHUA_COMPONENT_INFO { \
    "DAHUA_COMPONENT_INFO", \
    6, \
    {  { "patch", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_dahua_component_info_t, patch) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_dahua_component_info_t, id) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_dahua_component_info_t, mode) }, \
         { "major", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_dahua_component_info_t, major) }, \
         { "minor", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_dahua_component_info_t, minor) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 4, 8, offsetof(mavlink_dahua_component_info_t, reserved) }, \
         } \
}
#endif

/**
 * @brief Pack a dahua_component_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id Component ID
 * @param mode Current system mode.
 * @param major Firmware major version.
 * @param minor Firmware minor version.
 * @param patch Firmware patch level.
 * @param reserved Place Holder.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dahua_component_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint8_t mode, uint8_t major, uint8_t minor, uint32_t patch, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN];
    _mav_put_uint32_t(buf, 0, patch);
    _mav_put_uint8_t(buf, 4, id);
    _mav_put_uint8_t(buf, 5, mode);
    _mav_put_uint8_t(buf, 6, major);
    _mav_put_uint8_t(buf, 7, minor);
    _mav_put_uint8_t_array(buf, 8, reserved, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN);
#else
    mavlink_dahua_component_info_t packet;
    packet.patch = patch;
    packet.id = id;
    packet.mode = mode;
    packet.major = major;
    packet.minor = minor;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_MIN_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_CRC);
}

/**
 * @brief Pack a dahua_component_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id Component ID
 * @param mode Current system mode.
 * @param major Firmware major version.
 * @param minor Firmware minor version.
 * @param patch Firmware patch level.
 * @param reserved Place Holder.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dahua_component_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint8_t mode,uint8_t major,uint8_t minor,uint32_t patch,const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN];
    _mav_put_uint32_t(buf, 0, patch);
    _mav_put_uint8_t(buf, 4, id);
    _mav_put_uint8_t(buf, 5, mode);
    _mav_put_uint8_t(buf, 6, major);
    _mav_put_uint8_t(buf, 7, minor);
    _mav_put_uint8_t_array(buf, 8, reserved, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN);
#else
    mavlink_dahua_component_info_t packet;
    packet.patch = patch;
    packet.id = id;
    packet.mode = mode;
    packet.major = major;
    packet.minor = minor;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_MIN_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_CRC);
}

/**
 * @brief Encode a dahua_component_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dahua_component_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dahua_component_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dahua_component_info_t* dahua_component_info)
{
    return mavlink_msg_dahua_component_info_pack(system_id, component_id, msg, dahua_component_info->id, dahua_component_info->mode, dahua_component_info->major, dahua_component_info->minor, dahua_component_info->patch, dahua_component_info->reserved);
}

/**
 * @brief Encode a dahua_component_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dahua_component_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dahua_component_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dahua_component_info_t* dahua_component_info)
{
    return mavlink_msg_dahua_component_info_pack_chan(system_id, component_id, chan, msg, dahua_component_info->id, dahua_component_info->mode, dahua_component_info->major, dahua_component_info->minor, dahua_component_info->patch, dahua_component_info->reserved);
}

/**
 * @brief Send a dahua_component_info message
 * @param chan MAVLink channel to send the message
 *
 * @param id Component ID
 * @param mode Current system mode.
 * @param major Firmware major version.
 * @param minor Firmware minor version.
 * @param patch Firmware patch level.
 * @param reserved Place Holder.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dahua_component_info_send(mavlink_channel_t chan, uint8_t id, uint8_t mode, uint8_t major, uint8_t minor, uint32_t patch, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN];
    _mav_put_uint32_t(buf, 0, patch);
    _mav_put_uint8_t(buf, 4, id);
    _mav_put_uint8_t(buf, 5, mode);
    _mav_put_uint8_t(buf, 6, major);
    _mav_put_uint8_t(buf, 7, minor);
    _mav_put_uint8_t_array(buf, 8, reserved, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO, buf, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_MIN_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_CRC);
#else
    mavlink_dahua_component_info_t packet;
    packet.patch = patch;
    packet.id = id;
    packet.mode = mode;
    packet.major = major;
    packet.minor = minor;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO, (const char *)&packet, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_MIN_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_CRC);
#endif
}

/**
 * @brief Send a dahua_component_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_dahua_component_info_send_struct(mavlink_channel_t chan, const mavlink_dahua_component_info_t* dahua_component_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dahua_component_info_send(chan, dahua_component_info->id, dahua_component_info->mode, dahua_component_info->major, dahua_component_info->minor, dahua_component_info->patch, dahua_component_info->reserved);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO, (const char *)dahua_component_info, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_MIN_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_dahua_component_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint8_t mode, uint8_t major, uint8_t minor, uint32_t patch, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, patch);
    _mav_put_uint8_t(buf, 4, id);
    _mav_put_uint8_t(buf, 5, mode);
    _mav_put_uint8_t(buf, 6, major);
    _mav_put_uint8_t(buf, 7, minor);
    _mav_put_uint8_t_array(buf, 8, reserved, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO, buf, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_MIN_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_CRC);
#else
    mavlink_dahua_component_info_t *packet = (mavlink_dahua_component_info_t *)msgbuf;
    packet->patch = patch;
    packet->id = id;
    packet->mode = mode;
    packet->major = major;
    packet->minor = minor;
    mav_array_memcpy(packet->reserved, reserved, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO, (const char *)packet, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_MIN_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE DAHUA_COMPONENT_INFO UNPACKING


/**
 * @brief Get field id from dahua_component_info message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_dahua_component_info_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field mode from dahua_component_info message
 *
 * @return Current system mode.
 */
static inline uint8_t mavlink_msg_dahua_component_info_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field major from dahua_component_info message
 *
 * @return Firmware major version.
 */
static inline uint8_t mavlink_msg_dahua_component_info_get_major(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field minor from dahua_component_info message
 *
 * @return Firmware minor version.
 */
static inline uint8_t mavlink_msg_dahua_component_info_get_minor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field patch from dahua_component_info message
 *
 * @return Firmware patch level.
 */
static inline uint32_t mavlink_msg_dahua_component_info_get_patch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field reserved from dahua_component_info message
 *
 * @return Place Holder.
 */
static inline uint16_t mavlink_msg_dahua_component_info_get_reserved(const mavlink_message_t* msg, uint8_t *reserved)
{
    return _MAV_RETURN_uint8_t_array(msg, reserved, 4,  8);
}

/**
 * @brief Decode a dahua_component_info message into a struct
 *
 * @param msg The message to decode
 * @param dahua_component_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_dahua_component_info_decode(const mavlink_message_t* msg, mavlink_dahua_component_info_t* dahua_component_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    dahua_component_info->patch = mavlink_msg_dahua_component_info_get_patch(msg);
    dahua_component_info->id = mavlink_msg_dahua_component_info_get_id(msg);
    dahua_component_info->mode = mavlink_msg_dahua_component_info_get_mode(msg);
    dahua_component_info->major = mavlink_msg_dahua_component_info_get_major(msg);
    dahua_component_info->minor = mavlink_msg_dahua_component_info_get_minor(msg);
    mavlink_msg_dahua_component_info_get_reserved(msg, dahua_component_info->reserved);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN? msg->len : MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN;
        memset(dahua_component_info, 0, MAVLINK_MSG_ID_DAHUA_COMPONENT_INFO_LEN);
    memcpy(dahua_component_info, _MAV_PAYLOAD(msg), len);
#endif
}
