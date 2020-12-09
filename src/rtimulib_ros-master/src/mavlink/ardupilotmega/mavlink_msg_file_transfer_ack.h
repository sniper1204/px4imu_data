#pragma once
// MESSAGE FILE_TRANSFER_ACK PACKING

#define MAVLINK_MSG_ID_FILE_TRANSFER_ACK 227

MAVPACKED(
typedef struct __mavlink_file_transfer_ack_t {
 uint16_t seq; /*< Packet sequence*/
 uint8_t status; /*< File transfer packet receive status, success/failed*/
}) mavlink_file_transfer_ack_t;

#define MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN 3
#define MAVLINK_MSG_ID_FILE_TRANSFER_ACK_MIN_LEN 3
#define MAVLINK_MSG_ID_227_LEN 3
#define MAVLINK_MSG_ID_227_MIN_LEN 3

#define MAVLINK_MSG_ID_FILE_TRANSFER_ACK_CRC 71
#define MAVLINK_MSG_ID_227_CRC 71



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FILE_TRANSFER_ACK { \
    227, \
    "FILE_TRANSFER_ACK", \
    2, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_file_transfer_ack_t, seq) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_file_transfer_ack_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FILE_TRANSFER_ACK { \
    "FILE_TRANSFER_ACK", \
    2, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_file_transfer_ack_t, seq) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_file_transfer_ack_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a file_transfer_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq Packet sequence
 * @param status File transfer packet receive status, success/failed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_transfer_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t seq, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN);
#else
    mavlink_file_transfer_ack_t packet;
    packet.seq = seq;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FILE_TRANSFER_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_CRC);
}

/**
 * @brief Pack a file_transfer_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq Packet sequence
 * @param status File transfer packet receive status, success/failed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_transfer_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t seq,uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN);
#else
    mavlink_file_transfer_ack_t packet;
    packet.seq = seq;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FILE_TRANSFER_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_CRC);
}

/**
 * @brief Encode a file_transfer_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param file_transfer_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_file_transfer_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_file_transfer_ack_t* file_transfer_ack)
{
    return mavlink_msg_file_transfer_ack_pack(system_id, component_id, msg, file_transfer_ack->seq, file_transfer_ack->status);
}

/**
 * @brief Encode a file_transfer_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param file_transfer_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_file_transfer_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_file_transfer_ack_t* file_transfer_ack)
{
    return mavlink_msg_file_transfer_ack_pack_chan(system_id, component_id, chan, msg, file_transfer_ack->seq, file_transfer_ack->status);
}

/**
 * @brief Send a file_transfer_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param seq Packet sequence
 * @param status File transfer packet receive status, success/failed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_file_transfer_ack_send(mavlink_channel_t chan, uint16_t seq, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_ACK, buf, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_CRC);
#else
    mavlink_file_transfer_ack_t packet;
    packet.seq = seq;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_ACK, (const char *)&packet, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_CRC);
#endif
}

/**
 * @brief Send a file_transfer_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_file_transfer_ack_send_struct(mavlink_channel_t chan, const mavlink_file_transfer_ack_t* file_transfer_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_file_transfer_ack_send(chan, file_transfer_ack->seq, file_transfer_ack->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_ACK, (const char *)file_transfer_ack, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_file_transfer_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t seq, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint8_t(buf, 2, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_ACK, buf, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_CRC);
#else
    mavlink_file_transfer_ack_t *packet = (mavlink_file_transfer_ack_t *)msgbuf;
    packet->seq = seq;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_ACK, (const char *)packet, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_MIN_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE FILE_TRANSFER_ACK UNPACKING


/**
 * @brief Get field seq from file_transfer_ack message
 *
 * @return Packet sequence
 */
static inline uint16_t mavlink_msg_file_transfer_ack_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field status from file_transfer_ack message
 *
 * @return File transfer packet receive status, success/failed
 */
static inline uint8_t mavlink_msg_file_transfer_ack_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a file_transfer_ack message into a struct
 *
 * @param msg The message to decode
 * @param file_transfer_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_file_transfer_ack_decode(const mavlink_message_t* msg, mavlink_file_transfer_ack_t* file_transfer_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    file_transfer_ack->seq = mavlink_msg_file_transfer_ack_get_seq(msg);
    file_transfer_ack->status = mavlink_msg_file_transfer_ack_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN? msg->len : MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN;
        memset(file_transfer_ack, 0, MAVLINK_MSG_ID_FILE_TRANSFER_ACK_LEN);
    memcpy(file_transfer_ack, _MAV_PAYLOAD(msg), len);
#endif
}
