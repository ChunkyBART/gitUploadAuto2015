#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>

#define REPLY_SIZE 1024
#define TIMEOUT 1000

double X;
double Y;
double Heading;
char * pRoll;
char * pPitch;
char * pYaw;
char * pGyroX;
char * pGyroY;
char * pGyroZ;
char * pAccX;
char * pAccY;
char * pAccZ;

    ros::Publisher imu_pub;
ros::Publisher attitude_pub;
ros::Publisher ahrs_pub;
ros::Publisher Mag_pub;
double Roll;
double Pitch;
double Yaw;
double GyroX;
double GyroY;
double GyroZ;
double AccX;
double AccY;
double AccZ;
double Mx, My , Mz;
double LinearX,LinearY,LinearZ;
double oR,oP,oY;
const double degree = M_PI/180;
  ros::Time current_time,last_time;
  serial::Serial device("/dev/ttyACM1", 115200, serial::Timeout::simpleTimeout(50));



int main(int argc, char** argv)
{
    ros::init(argc, argv, "IMU_Node2");
    ros::NodeHandle n;


     imu_pub = n.advertise<sensor_msgs::Imu>("/imu/data",100);


    // Change the next line according to your port name and baud rate
    try
    {

        if(device.isOpen())
           {
               ROS_INFO("Port is opened");
          }
    }
    catch(serial::SerialException& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }

       ros::Rate r(50);
          while(ros::ok())
          {


       // Get the reply, the last value is the timeout in ms
       try{
     std::string reply;

        // ros::Duration(0.005).sleep();

           device.readline(reply);
           Roll = strtod(reply.c_str(),&pRoll)/2;
           Pitch = strtod(pRoll,&pRoll)/2;
           Yaw  = strtod(pRoll,&pRoll)/2;
           GyroX = strtod(pRoll,&pRoll);
           GyroY = strtod(pRoll,&pRoll);
           GyroZ = strtod(pRoll,&pRoll);
           AccX = strtod(pRoll,&pRoll);
           AccY = strtod(pRoll,&pRoll);
           AccZ = strtod(pRoll,&pRoll);
           Mx = strtod(pRoll,&pRoll);
           My = strtod(pRoll,&pRoll);
           Mz = strtod(pRoll,NULL);

           ROS_INFO("Euler %.2f %.2f %.2f",Roll,Pitch,Yaw);

	   sensor_msgs::Imu imu;

           geometry_msgs::Quaternion Q;
           Q = tf::createQuaternionMsgFromRollPitchYaw(Roll*-1,Pitch*-1,Yaw*-1);

           imu.orientation.x = 0;
           imu.orientation.y = 0;
           imu.orientation.z = 0;
           imu.orientation.w = 1;


           LinearX =  (AccX/256)*9.806;
           LinearY = (AccY/256)*9.806;
           LinearZ = (AccZ/256)*9.806;

           imu.angular_velocity.x = GyroX*-1*0.07*0.01745329252;;
           imu.angular_velocity.y = GyroY*0.07*0.01745329252;;
           imu.angular_velocity.z = GyroZ*-1*0.07*0.01745329252;;

           imu.linear_acceleration.x =  LinearX*-1;
           imu.linear_acceleration.y =  LinearY;
           imu.linear_acceleration.z =  LinearZ*-1;
           imu.header.stamp = ros::Time::now();
           imu.header.frame_id = "imu";
           imu_pub.publish(imu);


       }
       catch(serial::SerialException& e)
       {
           ROS_INFO("Error %s",e.what());
       }

              r.sleep();
    }

}
