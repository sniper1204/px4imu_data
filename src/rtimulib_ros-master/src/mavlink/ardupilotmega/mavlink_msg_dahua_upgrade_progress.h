#pragma once
// MESSAGE DAHUA_UPGRADE_PROGRESS PACKING

#define MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS 229

MAVPACKED(
typedef struct __mavlink_dahua_upgrade_progress_t {
 uint8_t id; /*< Component ID*/
 uint8_t state; /*< Upgrade State, 0: note start 1: upgrading 2: finish with success 3: finish with error. */
 uint8_t progress; /*< Upgrade progress, 0-100, 101-255 means failed and with different reason.*/
 uint8_t reserved[6]; /*< Place Holder.*/
}) mavlink_dahua_upgrade_progress_t;

#define MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN 9
#define MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_MIN_LEN 9
#define MAVLINK_MSG_ID_229_LEN 9
#define MAVLINK_MSG_ID_229_MIN_LEN 9

#define MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_CRC 1
#define MAVLINK_MSG_ID_229_CRC 1

#define MAVLINK_MSG_DAHUA_UPGRADE_PROGRESS_FIELD_RESERVED_LEN 6

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DAHUA_UPGRADE_PROGRESS { \
    229, \
    "DAHUA_UPGRADE_PROGRESS", \
    4, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_dahua_upgrade_progress_t, id) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_dahua_upgrade_progress_t, state) }, \
         { "progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_dahua_upgrade_progress_t, progress) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 6, 3, offsetof(mavlink_dahua_upgrade_progress_t, reserved) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DAHUA_UPGRADE_PROGRESS { \
    "DAHUA_UPGRADE_PROGRESS", \
    4, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_dahua_upgrade_progress_t, id) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_dahua_upgrade_progress_t, state) }, \
         { "progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_dahua_upgrade_progress_t, progress) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 6, 3, offsetof(mavlink_dahua_upgrade_progress_t, reserved) }, \
         } \
}
#endif

/**
 * @brief Pack a dahua_upgrade_progress message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id Component ID
 * @param state Upgrade State, 0: note start 1: upgrading 2: finish with success 3: finish with error. 
 * @param progress Upgrade progress, 0-100, 101-255 means failed and with different reason.
 * @param reserved Place Holder.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dahua_upgrade_progress_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint8_t state, uint8_t progress, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN];
    _mav_put_uint8_t(buf, 0, id);
    _mav_put_uint8_t(buf, 1, state);
    _mav_put_uint8_t(buf, 2, progress);
    _mav_put_uint8_t_array(buf, 3, reserved, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN);
#else
    mavlink_dahua_upgrade_progress_t packet;
    packet.id = id;
    packet.state = state;
    packet.progress = progress;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_CRC);
}

/**
 * @brief Pack a dahua_upgrade_progress message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id Component ID
 * @param state Upgrade State, 0: note start 1: upgrading 2: finish with success 3: finish with error. 
 * @param progress Upgrade progress, 0-100, 101-255 means failed and with different reason.
 * @param reserved Place Holder.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dahua_upgrade_progress_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint8_t state,uint8_t progress,const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN];
    _mav_put_uint8_t(buf, 0, id);
    _mav_put_uint8_t(buf, 1, state);
    _mav_put_uint8_t(buf, 2, progress);
    _mav_put_uint8_t_array(buf, 3, reserved, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN);
#else
    mavlink_dahua_upgrade_progress_t packet;
    packet.id = id;
    packet.state = state;
    packet.progress = progress;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_CRC);
}

/**
 * @brief Encode a dahua_upgrade_progress struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dahua_upgrade_progress C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dahua_upgrade_progress_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dahua_upgrade_progress_t* dahua_upgrade_progress)
{
    return mavlink_msg_dahua_upgrade_progress_pack(system_id, component_id, msg, dahua_upgrade_progress->id, dahua_upgrade_progress->state, dahua_upgrade_progress->progress, dahua_upgrade_progress->reserved);
}

/**
 * @brief Encode a dahua_upgrade_progress struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dahua_upgrade_progress C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dahua_upgrade_progress_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dahua_upgrade_progress_t* dahua_upgrade_progress)
{
    return mavlink_msg_dahua_upgrade_progress_pack_chan(system_id, component_id, chan, msg, dahua_upgrade_progress->id, dahua_upgrade_progress->state, dahua_upgrade_progress->progress, dahua_upgrade_progress->reserved);
}

/**
 * @brief Send a dahua_upgrade_progress message
 * @param chan MAVLink channel to send the message
 *
 * @param id Component ID
 * @param state Upgrade State, 0: note start 1: upgrading 2: finish with success 3: finish with error. 
 * @param progress Upgrade progress, 0-100, 101-255 means failed and with different reason.
 * @param reserved Place Holder.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dahua_upgrade_progress_send(mavlink_channel_t chan, uint8_t id, uint8_t state, uint8_t progress, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN];
    _mav_put_uint8_t(buf, 0, id);
    _mav_put_uint8_t(buf, 1, state);
    _mav_put_uint8_t(buf, 2, progress);
    _mav_put_uint8_t_array(buf, 3, reserved, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS, buf, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_CRC);
#else
    mavlink_dahua_upgrade_progress_t packet;
    packet.id = id;
    packet.state = state;
    packet.progress = progress;
    mav_array_memcpy(packet.reserved, reserved, sizeof(uint8_t)*6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS, (const char *)&packet, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_CRC);
#endif
}

/**
 * @brief Send a dahua_upgrade_progress message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_dahua_upgrade_progress_send_struct(mavlink_channel_t chan, const mavlink_dahua_upgrade_progress_t* dahua_upgrade_progress)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dahua_upgrade_progress_send(chan, dahua_upgrade_progress->id, dahua_upgrade_progress->state, dahua_upgrade_progress->progress, dahua_upgrade_progress->reserved);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS, (const char *)dahua_upgrade_progress, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_CRC);
#endif
}

#if MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_dahua_upgrade_progress_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint8_t state, uint8_t progress, const uint8_t *reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, id);
    _mav_put_uint8_t(buf, 1, state);
    _mav_put_uint8_t(buf, 2, progress);
    _mav_put_uint8_t_array(buf, 3, reserved, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS, buf, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_CRC);
#else
    mavlink_dahua_upgrade_progress_t *packet = (mavlink_dahua_upgrade_progress_t *)msgbuf;
    packet->id = id;
    packet->state = state;
    packet->progress = progress;
    mav_array_memcpy(packet->reserved, reserved, sizeof(uint8_t)*6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS, (const char *)packet, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_MIN_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_CRC);
#endif
}
#endif

#endif

// MESSAGE DAHUA_UPGRADE_PROGRESS UNPACKING


/**
 * @brief Get field id from dahua_upgrade_progress message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_dahua_upgrade_progress_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field state from dahua_upgrade_progress message
 *
 * @return Upgrade State, 0: note start 1: upgrading 2: finish with success 3: finish with error. 
 */
static inline uint8_t mavlink_msg_dahua_upgrade_progress_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field progress from dahua_upgrade_progress message
 *
 * @return Upgrade progress, 0-100, 101-255 means failed and with different reason.
 */
static inline uint8_t mavlink_msg_dahua_upgrade_progress_get_progress(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field reserved from dahua_upgrade_progress message
 *
 * @return Place Holder.
 */
static inline uint16_t mavlink_msg_dahua_upgrade_progress_get_reserved(const mavlink_message_t* msg, uint8_t *reserved)
{
    return _MAV_RETURN_uint8_t_array(msg, reserved, 6,  3);
}

/**
 * @brief Decode a dahua_upgrade_progress message into a struct
 *
 * @param msg The message to decode
 * @param dahua_upgrade_progress C-struct to decode the message contents into
 */
static inline void mavlink_msg_dahua_upgrade_progress_decode(const mavlink_message_t* msg, mavlink_dahua_upgrade_progress_t* dahua_upgrade_progress)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    dahua_upgrade_progress->id = mavlink_msg_dahua_upgrade_progress_get_id(msg);
    dahua_upgrade_progress->state = mavlink_msg_dahua_upgrade_progress_get_state(msg);
    dahua_upgrade_progress->progress = mavlink_msg_dahua_upgrade_progress_get_progress(msg);
    mavlink_msg_dahua_upgrade_progress_get_reserved(msg, dahua_upgrade_progress->reserved);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN? msg->len : MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN;
        memset(dahua_upgrade_progress, 0, MAVLINK_MSG_ID_DAHUA_UPGRADE_PROGRESS_LEN);
    memcpy(dahua_upgrade_progress, _MAV_PAYLOAD(msg), len);
#endif
}
