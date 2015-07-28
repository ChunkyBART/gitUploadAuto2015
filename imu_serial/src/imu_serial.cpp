#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16.h>
#include <tf/tf.h>
#include <math.h>
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
ros::Publisher CO2_pub;
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
double CO2;
double LinearX,LinearY,LinearZ;
double oR,oP,oY;
const double degree = M_PI/180;
  ros::Time current_time,last_time;
  serial::Serial device("/dev/imu", 115200, serial::Timeout::simpleTimeout(50));
  void setRollPitch(const geometry_msgs::Quaternion& q)
  {
    double roll, pitch;
    roll  = atan2(2*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    pitch = -asin(2*(q.x*q.z - q.w*q.y));
    ROS_INFO("roll: %f, pitch: %f",roll,pitch);
  }
void imucallback(const ros::TimerEvent& event)
{
    // Get the reply, the last value is the timeout in ms
    try{
  std::string reply;

     // ros::Duration(0.005).sleep();

        device.readline(reply);
     // ROS_INFO("imu %s",reply.c_str());
        Roll = strtod(reply.c_str(),&pRoll);
        Pitch = strtod(pRoll,&pRoll);
        Yaw  = strtod(pRoll,&pRoll);
        GyroX = strtod(pRoll,&pRoll);
        GyroY = strtod(pRoll,&pRoll);
        GyroZ = strtod(pRoll,&pRoll);
        AccX = strtod(pRoll,&pRoll);
        AccY = strtod(pRoll,&pRoll);
        AccZ = strtod(pRoll,&pRoll);
        Mx = strtod(pRoll,&pRoll);
        My = strtod(pRoll,&pRoll);
        Mz = strtod(pRoll,&pRoll);
        CO2 = strtod(pRoll,NULL);
        std_msgs::Int16 data;
        data.data=CO2;
        CO2_pub.publish(data);
/*if(abs(Roll-oR) > 0.5  )
{
    Roll = oR;
}
if(abs(Pitch-oP) > 0.5 )
{
    Pitch = oP;
}
if(abs(Yaw-oY) > 0.5  )
{
    Yaw = oY;
}
*/
geometry_msgs::Vector3Stamped Mag;
geometry_msgs::Quaternion Q;
Q = tf::createQuaternionMsgFromRollPitchYaw(Roll*degree,Pitch*degree,Yaw*degree);
        LinearX =  (AccX/256)*9.806;
        LinearY = (AccY/256)*9.806;
        LinearZ = (AccZ/256)*9.806;
        sensor_msgs::Imu imu;
        sensor_msgs::Imu ahrs;
        sensor_msgs::Imu attitude;

//setRollPitch(Q);
        imu.orientation.x = 0;
        imu.orientation.y = 0;
        imu.orientation.z = 0;
        imu.orientation.w = 1;


        imu.angular_velocity.x = GyroX*-1;
        imu.angular_velocity.y = GyroY;
        imu.angular_velocity.z = GyroZ*-1;

        imu.linear_acceleration.x =  LinearX*-1;
        imu.linear_acceleration.y =  LinearY;
        imu.linear_acceleration.z =  LinearZ*-1;
current_time = ros::Time::now();
        imu.header.stamp = current_time;
        imu.header.frame_id = "/imu";

        ahrs.orientation.x = Roll;
        ahrs.orientation.y = -Pitch;
        ahrs.orientation.z = Yaw;
       ahrs.orientation.w = Q.w;


        ahrs.angular_velocity.x = GyroX;
        ahrs.angular_velocity.y = GyroY;
        ahrs.angular_velocity.z = GyroZ;

        ahrs.linear_acceleration.x = LinearX;
        ahrs.linear_acceleration.y = LinearY;
        ahrs.linear_acceleration.z = LinearZ;
current_time = ros::Time::now();
        ahrs.header.stamp = current_time;
        ahrs.header.frame_id = "raw_imu";

        attitude = imu;

        imu_pub.publish(imu);
        ahrs_pub.publish(ahrs);
    //    attitude_pub.publish(attitude);
        Mag.vector.x = Mx;
        Mag.vector.y = My;
        Mag.vector.z = Mz;
        Mag.header.frame_id = "/magnetometer";
        Mag.header.stamp = current_time;
        //Mag_pub.publish(Mag);
//ROS_INFO("Roll %f Pitch %f Yaw %f",ahrs.orientation.x,ahrs.orientation.y,ahrs.orientation.z);
//ROS_INFO("%0.2f  %0.2f %0.2f %0.2f %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %.2f %.2f %.2f",Roll,Pitch,Yaw,GyroX,GyroY,GyroZ,LinearX,LinearY,LinearZ,Mx,My,Mz);
   oR = Roll;
   oP = Pitch;
   oY = Yaw;
        //r.sleep();
        //ROS_INFO("Gyro : %.2f %.2f %.2f  Acc : %.2f %.2f %.2f",GyroX,GyroY,GyroZ,AccX,AccY,AccZ);
    //   ROS_INFO("%0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %.2f  %.2f  %.2f",imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w,imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z,imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z,Mx,My,Mz);
    }
    catch(serial::SerialException& e)
    {
        ROS_INFO("Error %s",e.what());
    }






}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "IMU_Node");
    ros::NodeHandle n;


     imu_pub = n.advertise<sensor_msgs::Imu>("/imu/data_raw",100);
      ahrs_pub = n.advertise<sensor_msgs::Imu>("imu",100);
       attitude_pub = n.advertise<sensor_msgs::Imu>("ahrs",100);
       Mag_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/mag",100);
       CO2_pub = n.advertise<std_msgs::Int16>("co2",1);

    ros::Timer timer = n.createTimer(ros::Duration(0.005), imucallback);







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

ros::spin();


}
