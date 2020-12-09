#include <zmq.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <mutex>
#include <chrono>
#include <thread>

#include "../mavlink/common/mavlink.h"
#include "../mavlink/mavlink_types.h"


#if (defined (WIN32))
#include <zhelpers.hpp>
#endif


int main () {

    //  Prepare our context, each process has one context.
    zmq::context_t context (1);

    // If you need multi-thread work, do not try to use the same socket from
    // multiple threads.
    zmq::socket_t publisher (context, ZMQ_PUB);

    /*
     In order to make it compatible with MavLing, make an appointment about the
     socket ID: "ipc://system_id.component_id.channel_id". One OS one system_id;
     one program or process one component_id; one component may have multiple
     channel, the channel number is the same with mavlink's. In each component,
     we also assume that: the first channel #0 is for server, the second channel
     #1 is for heartbeat, and the left channels are for data. Make sure to
     transform same kind data in one channel, since the total number of channels
     is 16, and use less channel as far as you can. There might be many
     subscribers to each publisher, and a stocked subscriber will NOT stock the
     others.
     */
    //publisher.bind("ipc://255.0.0");
	publisher.bind("tcp://192.168.1.4:30002");

    // Sample optiflow message:
    mavlink_distance_sensor_t depth;
    depth.covariance = 0;
    depth.current_distance = 0;
    depth.id = 0;
    depth.max_distance = 18;
    depth.min_distance = 2;
    depth.orientation = 0;
    depth.time_boot_ms = 30000;
    depth.type = 0;


    mavlink_optical_flow_t optiflow;
	optiflow.flow_comp_m_x = 1;
	optiflow.flow_comp_m_y = 2;
	optiflow.flow_x = 0;
	optiflow.flow_y = 0;
	optiflow.ground_distance = 31;
	optiflow.quality = 253;
	optiflow.time_usec = 130;
	optiflow.sensor_id = 101;

	// Sample attitude message:
	mavlink_attitude_t attitude;
	attitude.roll = 10;
	attitude.pitch = 11;
	attitude.yaw = 12;
	attitude.rollspeed = 13;
	attitude.pitchspeed = 14;
	attitude.yawspeed = 15;

	// Sample gps message:
	mavlink_gps_global_origin_t gps;
	gps.altitude = 16;
	gps.latitude = 17;
	gps.longitude = 18;

	/*
	Sample mavlink message for each of above data messages. Please DO NOT use
	one mavlink message for multiple data messages. Send the different data
	message by one mavlink message continuously will cause error. Also it will
	be better to use different 0mq message for different mavlink message,
	although it's OK to use the same 0mq message for ease.
	*/
	mavlink_message_t msg[3];
	int system_id    = 253; // system id
	int component_id = 23; // companion computer component id

	// Be careful the subscriber will take a little time to connect the
	// publisher at the beginning.
    sleep(3);
    while (1)
    {
    	depth.current_distance++;
//    	optiflow.flow_x++;
//    	optiflow.flow_y--;
//    	mavlink_msg_optical_flow_encode(system_id, component_id, &msg[0], &optiflow);
    	mavlink_msg_distance_sensor_encode(system_id, component_id, &msg[0], &depth);
    	// Use the zero copy way to create 0mq message, this can improve the
    	// performance when you need to transform lots data message.
    	zmq::message_t message((void *)&msg[0].magic, MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)msg[0].len, NULL);
        publisher.send(message);

//        attitude.roll += 2.0;
//        attitude.yawspeed -=2.0;
//        mavlink_msg_attitude_encode(system_id, component_id, &msg[1], &attitude);
//        message.rebuild((void *)&msg[1].magic, MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)msg[1].len, NULL);
//        publisher.send(message);
//
//        gps.latitude++;
//        gps.altitude--;
//        mavlink_msg_gps_global_origin_encode(system_id, component_id, &msg[2], &gps);
//        message.rebuild((void *)&msg[2].magic, MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)msg[2].len, NULL);
//        publisher.send(message);

//        std::cout << optiflow.flow_x << std::endl;
//        std::cout << optiflow.flow_y << std::endl;
        std::cout << depth.current_distance << std::endl;
        usleep(30000);
    }
    return 0;
}
