#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#define REPLY_SIZE 1024
#define TIMEOUT 1000
float speedL,speedR,errorL,errorR,cL,cR,aL;
float psL =0;
ros::Publisher speedL_pub;
ros::Publisher speedR_pub;
ros::Publisher errorL_pub;
ros::Publisher errorR_pub;
ros::Publisher cL_pub;
ros::Publisher cR_pub;
ros::Publisher aL_pub;
std_msgs::Float32 sl ,sr,el,er,cl,cr,al;
  serial::Serial device1("/dev/control", 115200, serial::Timeout::simpleTimeout(50));
//  serial::Serial device2("/dev/sensor2", 115200, serial::Timeout::simpleTimeout(50));
char *A;
char *B;
void speedcallback(const ros::TimerEvent& event)
{
    // Get the reply, the last value is the timeout in ms
    try{
  std::string reply1;
  std::string reply2;
     // ros::Duration(0.005).sleep();
        device1.readline(reply1);
  //     device2.readline(reply2);
    speedL = strtod(reply1.c_str(),&A);
    errorL = strtod(A,&A);
    aL = strtod(A,NULL);
    cL = psL+errorL;
    //    speedR = strtod(reply2.c_str(),&B);
   // errorR = strtod(B,NULL);
 ROS_INFO("speedL %f speedR %f errorL: %f errorR: %f",speedL,speedR,errorL,errorR);
    sl.data = speedL;
    psL = speedL;
   //       sr.data = speedR;
	el.data = errorL;
    cl.data = cL;
    al.data = aL;
   // er.data = errorR;
          speedL_pub.publish(sl);
     //     speedR_pub.publish(sr);
	 errorL_pub.publish(el);
  //  errorR_pub.publish(er);
     cL_pub.publish(cl);
     aL_pub.publish(al);
    }
    catch(serial::SerialException& e)
    {
        ROS_INFO("Error %s",e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plot_node");
    ros::NodeHandle n;


     speedL_pub = n.advertise<std_msgs::Float32>("Speed_L",100);
    //  speedR_pub = n.advertise<std_msgs::Float32>("Speed_R",100);
 errorL_pub = n.advertise<std_msgs::Float32>("Error_L",100);
   //   errorR_pub = n.advertise<std_msgs::Float32>("Error_R",100);
    cL_pub = n.advertise<std_msgs::Float32>("C_L",100);
    aL_pub = n.advertise<std_msgs::Float32>("A_L",100);
    ros::Timer timer = n.createTimer(ros::Duration(0.01), speedcallback);


    // Change the next line according to your port name and baud rate
    try
    {

        if(device1.isOpen() )
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
