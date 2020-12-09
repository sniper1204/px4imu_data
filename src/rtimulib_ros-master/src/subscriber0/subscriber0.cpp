#include <iostream>
#include <sstream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <zmq.hpp>

#include "../mavlink/common/mavlink.h"
#include "../mavlink/mavlink_types.h"

class Subscriber0
{
public:
	Subscriber0(){};
	~Subscriber0(){};
	void init(zmq::context_t& context_)
	{

	};
private:
    mavlink_message_t msg_;
    mavlink_status_t status_;
    mavlink_optical_flow_t optiflow_;
};


void subscriber0(zmq::context_t& context_)
{
	zmq::socket_t sub(context_, ZMQ_SUB);
	//sub.connect("ipc://255.0.0");
	sub.connect("tcp://192.168.1.4:30000");
	//sub.connect("tcp://10.35.16.177:50000");
	sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    mavlink_message_t msg;
    mavlink_status_t status;
    mavlink_optical_flow_t optiflow;
    mavlink_distance_sensor_t depth;
    mavlink_global_position_int_t global;
	mavlink_gps_global_origin_t gpss;
	mavlink_attitude_quaternion_t attitude;
	mavlink_attitude_t att;
	mavlink_local_position_ned_t pos_ned;


    while(true)
    {
        zmq::message_t update;
		//std::cout << "1111111111111111 " << std::endl;

        bool statue = sub.recv(&update);

        // process received bytes
        uint8_t *ck = (uint8_t *)update.data();
        size_t nbytes = update.size();

		//std::cout << "2222222222222222222 " << std::endl;
		//std::cout << "statue = " << statue << std::endl;

        for(size_t i=0; i<nbytes; i++)
        {
        	// We are receiving the data via channel 0 here.
        	mavlink_parse_char(0, ck[i], &msg, &status);
        }
 
        /*
        Because the mavlink message format is fixed, we can not use 0mq's filter
        scheme, although that way is more efficient. Receive all messages and filter
        them by yourself locally.
        */
        switch (msg.msgid)
        {
		/*
        case MAVLINK_MSG_ID_OPTICAL_FLOW:
        	mavlink_msg_optical_flow_decode(&msg, &optiflow);
        	std::cout << "time_usec: " << optiflow.time_usec << std::endl;
        	std::cout << "flow_comp_m_x: " << optiflow.flow_comp_m_x << std::endl;
        	std::cout << "flow_comp_m_y: " << optiflow.flow_comp_m_y << std::endl;
        	std::cout << "flow_x: " << optiflow.flow_x << std::endl;
        	std::cout << "flow_y: " << optiflow.flow_y << std::endl;
        	std::cout << "ground_distance: " << optiflow.ground_distance << std::endl;
        	std::cout << "quality: " << unsigned(optiflow.quality) << std::endl;
        	std::cout << "sensor_id: " << unsigned(optiflow.sensor_id) << std::endl;
        	std::cout << "========" <<  std::endl;
        	break;
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        	mavlink_msg_distance_sensor_decode(&msg, &depth);
        	std::cout << "covariance: " << depth.covariance << std::endl;
        	std::cout << "current_distance: " << depth.current_distance << std::endl;
        	std::cout << "id: " << depth.id << std::endl;
        	std::cout << "max_distance: " << depth.max_distance << std::endl;
        	std::cout << "min_distance: " << depth.min_distance << std::endl;
        	std::cout << "orientation: " << depth.orientation << std::endl;
        	std::cout << "time_boot_ms: " << depth.time_boot_ms << std::endl;
        	std::cout << "type: " << depth.type << std::endl;
        	std::cout << "========" <<  std::endl;
        	break;
		case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
        	mavlink_msg_gps_global_origin_decode(&msg, &gpss);
			std::cout << "=============gpss" <<  std::endl;
        	std::cout << "Lat: " << gpss.latitude << std::endl;
        	std::cout << "Lon: " << gpss.longitude << std::endl;
        	std::cout << "Alt: " << gpss.altitude << std::endl;
        	break;
		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
        	mavlink_msg_attitude_quaternion_decode(&msg, &attitude);
			std::cout << "=============attitude" <<  std::endl;
			std::cout << "time_boot_ms: " << attitude.time_boot_ms << std::endl;
			std::cout << "q1: " << attitude.q1 << std::endl;
        	std::cout << "q2: " << attitude.q2 << std::endl;
        	std::cout << "q3: " << attitude.q3 << std::endl;
        	std::cout << "q4: " << attitude.q4 << std::endl;
			std::cout << "rollspeed: " << attitude.rollspeed << std::endl;
			std::cout << "pitchspeed: " << attitude.pitchspeed << std::endl;
			std::cout << "yawspeed: " << attitude.yawspeed << std::endl;
        	break;
		*/
			
		case MAVLINK_MSG_ID_ATTITUDE:
        	mavlink_msg_attitude_decode(&msg, &att);
			std::cout << "=============att" <<  std::endl;
			std::cout << "time_boot_ms: " << att.time_boot_ms << std::endl;
			std::cout << "roll: " << att.roll*57.3 << std::endl;
        	std::cout << "pitch: " << att.pitch*57.3 << std::endl;
        	std::cout << "yaw: " << att.yaw*57.3 << std::endl;
			std::cout << "rollspeed: " << att.rollspeed*57.3  << std::endl;
			std::cout << "pitchspeed: " << att.pitchspeed*57.3 << std::endl;
			std::cout << "yawspeed: " << att.yawspeed*57.3 << std::endl;
        	break;
		
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        	mavlink_msg_local_position_ned_decode(&msg, &pos_ned);
			std::cout << "=============pos_ned" <<  std::endl;
			std::cout << "time_boot_ms: " << pos_ned.time_boot_ms << std::endl;
			std::cout << "px: " << pos_ned.x << std::endl;
        	std::cout << "py: " << pos_ned.y << std::endl;
        	std::cout << "pz: " << pos_ned.z << std::endl;
			std::cout << "vx: " << pos_ned.vx << std::endl;
			std::cout << "vy: " << pos_ned.vy << std::endl;
			std::cout << "vz: " << pos_ned.vz << std::endl;
        	break;
		
			/*
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        	mavlink_msg_global_position_int_decode(&msg, &global);
			std::cout << "=============global" <<  std::endl;
        	std::cout << "Lat: " << global.lat << std::endl;
        	std::cout << "Lon: " << global.lat << std::endl;
        	std::cout << "Alt: " << global.alt << std::endl;
        	std::cout << "relative_alt: " << global.relative_alt << std::endl;
        	std::cout << "Vx: " << global.vx << std::endl;
        	std::cout << "Vy: " << global.vy << std::endl;
        	std::cout << "Vz: " << global.vz << std::endl;
        	std::cout << "hdg: " << global.hdg << std::endl;
        	break;
			*/
        default:
        	break;
        }
    }
}


void subscriber1(zmq::context_t& context_)
{
	zmq::socket_t sub (context_, ZMQ_SUB);
	sub.connect("tcp://10.35.16.159:30000");
	sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    mavlink_message_t msg;
    mavlink_status_t status;
    mavlink_attitude_t attitude;
    mavlink_gps_global_origin_t gps;

    while(true)
    {
        zmq::message_t update;
        sub.recv(&update);

        // process received bytes
        uint8_t *ck = (uint8_t *)update.data();
        size_t nbytes = update.size();
        for(size_t i=0; i<nbytes; i++)
        {
        	// We are receiving the data via channel 0 here.
        	if(mavlink_parse_char(0, ck[i], &msg, &status))
        	{
				// Filter the mavlink data message locally.
				switch (msg.msgid)
				{
				case MAVLINK_MSG_ID_ATTITUDE:
					mavlink_msg_attitude_decode(&msg, &attitude);
					std::cout << "roll: " << attitude.roll << std::endl;
					std::cout << "yawspeed: " << attitude.yawspeed << std::endl;
					std::cout << "++++++++" <<  std::endl;
					break;
				default:
					break;
				}
        	}
        }
    }
}


int main (int argc, char *argv[])
{
    zmq::context_t context(1);
    std::cout << "Collecting updates from weather server…\n" << std::endl;

    std::thread th0(subscriber0, std::ref(context));
//    std::thread th1(subscriber1, std::ref(context));

    th0.join();
//    th1.join();

    return 0;
}
