#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <time.h>

#include <iostream>
#include <sstream>
#include <memory>
#include <chrono>

#include <zmq.hpp>

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/opencv.hpp>
#include <zmq.hpp>

#include "mavlink/common/mavlink.h"
#include "mavlink/mavlink_types.h"

#include<stdlib.h>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h> 
#include<sys/stat.h>   
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#include<string.h>


//宏定义
#define FALSE  -1
#define TRUE   0

static const double G_TO_MPSS = 9.80665;

float uav_px;
float uav_py;
float uav_pz;
float uav_vx;
float uav_vy;
float uav_vz;
float uav_roll;
float uav_pitch;
float uav_yaw;
double uav_time;
double tx1_time;
double delta_time;
float uav_accx;
float uav_accy;
float uav_accz;
float uav_gyrox;
float uav_gyroy;
float uav_gyroz;
bool time_flg = 0;

ros::Publisher pub_imu_px4;

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


int UART0_Open(int fd,char* port)
{
    //fd = open( port, O_RDWR|O_NOCTTY|O_NONBLOCK | O_NDELAY);
    //fd = open( port, O_RDWR|O_NOCTTY|O_NONBLOCK );
    fd = open( port, O_RDWR|O_NOCTTY );
    if (FALSE == fd)
    {
        perror("Can't Open Serial Port");
        return(FALSE);
    }
    return fd;
}
 
void UART0_Close(int fd)
{
    close(fd);
}

int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int   i;
    int   status;
    int   speed_arr[] = { B115200, B57600, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200, 57600, 19200,  9600,  4800,  2400,  1200,  300};
         
    struct termios options;
   
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");    
        return(FALSE); 
    }
    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if(speed == name_arr[i])
        {             
            cfsetispeed(&options, speed_arr[i]); 
            cfsetospeed(&options, speed_arr[i]);  
        }
    }     
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
    //设置数据流控制
    switch(flow_ctrl)
    {
        case 0://不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;   
      
        case 1://使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2 ://使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {  
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:    
            options.c_cflag |= CS7;
            break;
        case 8:    
            options.c_cflag |= CS8;
            break;  
        default:   
            fprintf(stderr,"Unsupported data size\n");
            return (FALSE); 
    }
    //设置校验位
    switch (parity)
    {  
       case 'n':
       case 'N': //无奇偶校验位。
            options.c_cflag &= ~PARENB; 
            options.c_iflag &= ~INPCK;    
            break; 
       case 'o':  
       case 'O'://设置为奇校验    
            options.c_cflag |= (PARODD | PARENB); 
            options.c_iflag |= INPCK;             
            break; 
       case 'e': 
       case 'E'://设置为偶校验  
            options.c_cflag |= PARENB;       
            options.c_cflag &= ~PARODD;       
            options.c_iflag |= INPCK;      
            break;
       case 's':
       case 'S': //设置为空格 
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break; 
        default:  
            fprintf(stderr,"Unsupported parity\n");    
            return (FALSE); 
    } 
    // 设置停止位 
    switch (stopbits)
    {  
       case 1:   
            options.c_cflag &= ~CSTOPB; break; 
       case 2:   
            options.c_cflag |= CSTOPB; break;
       default:   
            fprintf(stderr,"Unsupported stop bits\n"); 
            return (FALSE);
    }
  //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(INLCR | ICRNL | IXON);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的
    //options.c_lflag &= ~(ISIG | ICANON);
    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 0; /* 读取一个字符等待1*(1/10)s */  
    options.c_cc[VMIN] = 0; /* 读取字符的最少个数为34 */
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);
    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)  
    {
        perror("com set error!\n");  
        return (FALSE); 
    }
    return (TRUE); 
}

int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err;
    //设置串口数据帧格式
    if (UART0_Set(fd,115200,0,8,1,'N') == FALSE)
    {                                                         
        return FALSE;
    }
    else
    {
        return  TRUE;
    }
}

int UART0_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;
   
    struct timeval time;
   
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
   
    time.tv_sec = 10;
    time.tv_usec = 0;
   
    //使用select实现串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
   // if(fs_sel)
    if(1)
    {
        len = read(fd,rcv_buf,data_len);
	    //printf("I am right!(version1.2) len = %d fs_sel = %d\n",len,fs_sel);
        return len;
    }
    else
    {
	    printf("Sorry,I am wrong!");
        return FALSE;
    }     
}

int UART0_Send(int fd, char *send_buf,int data_len)
{
    int len = 0;
   
    len = write(fd,send_buf,data_len);
    if (len == data_len )
    {
        return len;
    }     
    else   
    {          
        tcflush(fd,TCOFLUSH);
        return FALSE;
    }
   
}

void subscriber0()
{
    //zmq::context_t context_(1);
    //zmq::socket_t sub(context_, ZMQ_SUB);
	//sub.connect("ipc://255.0.0");
	//sub.connect("tcp://192.168.1.4:30000");
	//sub.connect("tcp://10.35.16.177:50000");
	//sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    mavlink_message_t msg;
    mavlink_status_t status;
    //mavlink_optical_flow_t optiflow;
    //mavlink_distance_sensor_t depth;
    //mavlink_global_position_int_t global;
	//mavlink_gps_global_origin_t gpss;
	//mavlink_attitude_quaternion_t attitude;
	mavlink_attitude_t att;
	mavlink_raw_imu_t raw_imu;
    mavlink_local_position_ned_t pos_ned;
    //Eigen::Quaterniond uav_q;
    int fd;                            //文件描述符
    int err;                           //返回调用函数的状态
    int len;                        
    int i;
    char rcv_buf[1024];
    char temp_buf[1024];      

    fd = UART0_Open(fd,"/dev/ttyUSB0"); //打开串口，返回文件描述符
    //printf("fd = %d\n",fd);
    //printf("---------------\n");
    do{
        err = UART0_Init(fd,115200,0,8,1,'N');
        printf("Set Port Exactly!\n");
    }while(FALSE == err || FALSE == fd);
   
    while (1) //串口循环读取数据
    {  
        bzero(rcv_buf,sizeof(rcv_buf));
        len = UART0_Recv(fd, rcv_buf,1024);
        //printf("\nlen0 = %d\n",len);
        if(len > 0)
        {
			//rcv_buf[len] = '\0';
            //printf("receive data is %s\n",rcv_buf);
		    //printf("len = %d\n",len);
        }
        else
        {
            printf("cannot receive data\n");
        }

        uint8_t *ck = (uint8_t *)rcv_buf;
        size_t nbytes = len;

        //printf("recv-%d:", len);
        for(size_t i=0; i<nbytes; i++)
        {
            //printf("%d-%02x ",i,ck[i]);
           
            if(mavlink_parse_char(0, ck[i], &msg, &status))
            {
                switch (msg.msgid)
                {
                    case MAVLINK_MSG_ID_RAW_IMU:
                        mavlink_msg_raw_imu_decode(&msg, &raw_imu);

                        uav_time = raw_imu.time_usec;
                        tx1_time = ros::Time::now().toSec() * 1e6;
                        //printf("time_uav11 = %f\n",uav_time);
                        if(time_flg == 0)
                        {
                            if(uav_time > 1)
                            {
                                time_flg = 1;
                                delta_time = tx1_time - uav_time;
                            }
                        }

                        uav_time = uav_time + delta_time;

                        uav_accx = (float)raw_imu.xacc * 9.8/1000;
                        uav_accy = (float)raw_imu.yacc * 9.8/1000;
                        uav_accz = (float)raw_imu.zacc * 9.8/1000;
                        uav_gyrox = (float)raw_imu.xgyro/1000;
                        uav_gyroy = (float)raw_imu.ygyro/1000;
                        uav_gyroz = (float)raw_imu.zgyro/1000;

                        //std::cout << "=============imu" << std::endl;

                        //printf("time_delta = %f\n",delta_time);
                        //std::cout << "time_old" << tx1_time << std::endl;
                        //std::cout << "time_boot_ms: " << uav_time << std::endl;
                        //std::cout << "xacc: " << uav_accx << std::endl;
                        //std::cout << "yacc: " << uav_accy << std::endl;
                        //std::cout << "zacc: " << uav_accz << std::endl;
                        //std::cout << "xgyro: " << uav_gyrox << std::endl;
                        //std::cout << "ygyro: " << uav_gyroy << std::endl;
                        //std::cout << "zgyro: " << uav_gyroz << std::endl;

                        break;
                    default:
                        break;
                }
                std::string frame_id = "imu";
                sensor_msgs::Imu px4_imu_msg;
                px4_imu_msg.header.frame_id = frame_id;
                ros::Time temp_time(uav_time * 1e-6);
                px4_imu_msg.header.stamp = temp_time;
                px4_imu_msg.angular_velocity.x = uav_gyrox;
                px4_imu_msg.angular_velocity.y = uav_gyroy;
                px4_imu_msg.angular_velocity.z = uav_gyroz;
                px4_imu_msg.linear_acceleration.x = uav_accx;
                px4_imu_msg.linear_acceleration.y = uav_accy;
                px4_imu_msg.linear_acceleration.z = uav_accz;

                pub_imu_px4.publish(px4_imu_msg);
               
           }
           
        }
        //printf("recv-end\n");
        //bzero(rcv_buf,sizeof(rcv_buf));
       
        //printf("ID = %d\n",msg.msgid);
        //printf("time_now = %f\n",ros::Time::now().toSec() * 1e9);
    }            
    UART0_Close(fd);
/*
    while(0)
    {
        zmq::message_t update;

        bool statue = sub.recv(&update);
        if (statue == false)
        {
            return;
        } 

        // process received bytes
        uint8_t *ck = (uint8_t *)update.data();
        size_t nbytes = update.size();

        for(size_t i=0; i<nbytes; i++)
        {
        	// We are receiving the data via channel 0 here.
        	mavlink_parse_char(0, ck[i], &msg, &status);
            //printf("ck[%d] = %d\n",i,ck[i]);
        }

        switch (msg.msgid)
        {		
            case MAVLINK_MSG_ID_RAW_IMU:
                mavlink_msg_raw_imu_decode(&msg, &raw_imu);

                uav_time = raw_imu.time_usec;
                uav_accx = raw_imu.xacc * 9.8/1000;
                uav_accy = raw_imu.yacc * 9.8/1000;
                uav_accz = raw_imu.zacc * 9.8/1000;
                uav_gyrox = raw_imu.xgyro ;
                uav_gyroy = raw_imu.ygyro ;
                uav_gyroz = raw_imu.zgyro ;

                std::cout << "=============imu" <<  std::endl;
                std::cout << "time_boot_ms: " << raw_imu.time_usec << std::endl;
                std::cout << "xacc: " << raw_imu.xacc * 9.8/1000<< std::endl;
                std::cout << "yacc: " << raw_imu.yacc * 9.8/1000<< std::endl;
                std::cout << "zacc: " << raw_imu.zacc * 9.8/1000<< std::endl;
                std::cout << "xgyro: " << uav_gyrox << std::endl;
                std::cout << "ygyro: " << uav_gyroy << std::endl;
                std::cout << "zgyro: " << uav_gyroz << std::endl;
                break;
            default:
                break;
        }
    }
*/
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4imu_node");
    ROS_INFO("Imu driver is now running");
    ros::NodeHandle nh("~");
    
    //zmq::context_t context(1);
    //registerPub(nh);
    
    pub_imu_px4 = nh.advertise<sensor_msgs::Imu>("", 2000);
 

    std::thread th0{subscriber0};

    ros::spin();
 
    return 0;
}
