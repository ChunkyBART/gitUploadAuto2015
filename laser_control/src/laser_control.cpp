#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <math.h>
#include <std_msgs/Int16.h>
geometry_msgs::PoseStamped Robot_pose;
geometry_msgs::Quaternion Q;
ros::Publisher laser_pub;
float degtorad(float deg);
float distancevic;
///////// if 180 degree change here //////
double s1out = 0; //135>121.5//-120>-114
double s2out = 0; //-114>-108
double s3out = 0; //-108>-102
double s4out = 0; //-102>-96
double s5out = 0; //-96>-90
double s6out = 0; //-90>-84
double s7out = 0; //-84>-78
double s8out = 0; //-78>-72
double s9out = 0; //-72>-66
double s10out = 0; //-66>-60
double s11out = 0; //-60>-54
double s12out = 0; //-54>-48
double s13out = 0; //-48>-42
double s14out = 0; //-42>-36
double s15out = 0; //-36>-30
double s16out = 0; //-30>-24
double s17out = 0; //-24>-18
double s18out = 0; //-18>-12
double s19out =0; //-12>6
double s20out =0; //-6>0
double s21out =0; // 0>6
double s22out =0; // 6>12
double s23out =0; // 12>18
double s24out =0; // 18>24
double s25out =0; // 24>30
double s26out =0; // 30>36
double s27out =0; // 36>42
double s28out =0; // 42>48
double s29out =0; // 48>54
double s30out =0; // 54>60
double s31out =0; // 60>66
double s32out =0; // 66>72
double s33out =0; // 72>78
double s34out =0; // 78>84
double s35out =0; // 84>90
double s36out =0; // 90>96
double s37out =0; // 96>102
double s38out =0; // 102>108
double s39out =0; // 108>114
double s40out =0; // 114>120

double angle =0;
double basetiltcommand = 0;
float commandjoy_1 = 0;
float commandjoy_2 = 0;
float x,y;
float radsum;
void lasercal_Callback(const ros::TimerEvent& e)
{
      std_msgs::Int16MultiArray laser_msg;
      laser_msg.data.push_back(s1out);
      laser_msg.data.push_back(s2out);
      laser_msg.data.push_back(s3out);
      laser_msg.data.push_back(s4out);
      laser_msg.data.push_back(s5out);
      laser_msg.data.push_back(s6out);
      laser_msg.data.push_back(s7out);
      laser_msg.data.push_back(s8out);
      laser_msg.data.push_back(s9out);
      laser_msg.data.push_back(s10out);
      laser_msg.data.push_back(s11out);
      laser_msg.data.push_back(s12out);
      laser_msg.data.push_back(s13out);
      laser_msg.data.push_back(s14out);
      laser_msg.data.push_back(s15out);
      laser_msg.data.push_back(s16out);
      laser_msg.data.push_back(s17out);
      laser_msg.data.push_back(s18out);
      laser_msg.data.push_back(s19out);
      laser_msg.data.push_back(s20out);
      laser_msg.data.push_back(s21out);
      laser_msg.data.push_back(s22out);
      laser_msg.data.push_back(s23out);
      laser_msg.data.push_back(s24out);
      laser_msg.data.push_back(s25out);
      laser_msg.data.push_back(s26out);
      laser_msg.data.push_back(s27out);
      laser_msg.data.push_back(s28out);
      laser_msg.data.push_back(s29out);
      laser_msg.data.push_back(s30out);
      laser_msg.data.push_back(s31out);
      laser_msg.data.push_back(s32out);
      laser_msg.data.push_back(s33out);
      laser_msg.data.push_back(s34out);
      laser_msg.data.push_back(s35out);
      laser_msg.data.push_back(s36out);
      laser_msg.data.push_back(s37out);
      laser_msg.data.push_back(s38out);
      laser_msg.data.push_back(s39out);
      laser_msg.data.push_back(s40out);
      laser_pub.publish(laser_msg);
}
void LaserCallback(const sensor_msgs::LaserScanConstPtr& data)
{
   std::vector<float> listDataScan;
   listDataScan = data->ranges;
   //////robot scan status////
   #define full_step 27// >>>max round = 720/10 >>180 degree
   //#define full_step 17 //max round = 1080/20 >>270 degree
       double s1 = 0;
       double s2 = 0;
       double s3 = 0;
       double s4 = 0;
       double s5 = 0;
       double s6 = 0;
       double s7 = 0;
       double s8 = 0;
       double s9 = 0;
       double s10 =0;
       double s11 =0;
       double s12 =0;
       double s13 =0;
       double s14 =0;
       double s15 =0;
       double s16 =0;
       double s17 =0;
       double s18 =0;
       double s19 =0;
       double s20 =0;
       double s21 =0;
       double s22 =0;
       double s23 =0;
       double s24 =0;
       double s25 =0;
       double s26 =0;
       double s27 =0;
       double s28 =0;
       double s29 =0;
       double s30 =0;
       double s31 =0;
       double s32 =0;
       double s33 =0;
       double s34 =0;
       double s35 =0;
       double s36 =0;
       double s37 =0;
       double s38 =0;
       double s39 =0;
       double s40 =0;
   ////////////////////////// Divide laser //////////////////////

       for(int i = 0 ; i<full_step*40;i++)
       {
           if(listDataScan[i]> 5.0)
               listDataScan[i] = 5.0;
           if(isnan(listDataScan[i]))
                listDataScan[i] = 0;
           if(i>=0 &&i <full_step)
           {
                s1 = (listDataScan[i])+s1;
           }
           else if (i >= full_step && i < full_step * 2)
           {
               s2 = (listDataScan[i])+s2;
           }
           else if (i >= full_step * 2 && i < full_step * 3)
           {
               s3 = (listDataScan[i])+s3;
           }
           else if (i >= full_step * 3 && i < full_step * 4)
           {
               s4 = (listDataScan[i])+s4;
           }
           else if (i >= full_step * 4 && i < full_step * 5)
           {
               s5 = (listDataScan[i])+s5;
           }
           else if (i >= full_step * 5 && i < full_step * 6)
           {
               s6 = (listDataScan[i])+s6;
           }
           else if (i >= full_step * 6 && i < full_step * 7)
           {
               s7 = (listDataScan[i])+s7;
           }
           else if (i >= full_step * 7 && i < full_step * 8)
           {
               s8 = (listDataScan[i])+s8;
           }
           else if (i >= full_step * 8 && i < full_step * 9)
           {
               s9 = (listDataScan[i])+s9;
           }
           else if (i >= full_step * 9 && i < full_step * 10)
           {
               s10 = (listDataScan[i])+s10;
           }
           else if (i >= full_step * 10 && i < full_step * 11)
           {
               s11 = (listDataScan[i])+s11;
           }
           else if (i >= full_step * 11 && i < full_step * 12)
           {
               s12 = (listDataScan[i])+s12;
           }
           else if (i >= full_step * 12 && i < full_step * 13)
           {
               s13 = (listDataScan[i])+s13;
           }
           else if (i >= full_step * 13 && i < full_step * 14)
           {
               s14 = (listDataScan[i])+s14;
           }
           else if (i >= full_step * 14 && i < full_step * 15)
           {
               s15 = (listDataScan[i])+s15;
           }
           else if (i >= full_step * 15 && i < full_step * 16)
           {
               s16 = (listDataScan[i])+s16;
           }
           else if (i >= full_step * 16 && i < full_step * 17)
           {
               s17 = (listDataScan[i])+s17;
           }
           else if (i >= full_step * 17 && i < full_step * 18)
           {
               s18 = (listDataScan[i])+s18;
           }
           else if (i >= full_step * 18 && i < full_step * 19)
           {
               s19 = (listDataScan[i])+s19;
           }
           else if (i >= full_step * 19 && i < full_step * 20)
           {
               s20 = (listDataScan[i])+s20;
           }
           else if (i >= full_step * 20 && i < full_step * 21)
           {
               s21 = (listDataScan[i])+s21;
           }
           else if (i >= full_step * 21 && i < full_step * 22)
           {
               s22 = (listDataScan[i])+s22;
           }
           else if (i >= full_step * 22 && i < full_step * 23)
           {
               s23 = (listDataScan[i])+s23;
           }
           else if (i >= full_step * 23 && i < full_step * 24)
           {
               s24 = (listDataScan[i])+s24;
           }
           else if (i >= full_step * 24 && i < full_step * 25)
           {
               s25 = (listDataScan[i])+s25;
           }
           else if (i >= full_step * 25 && i < full_step * 26)
           {
               s26 = (listDataScan[i])+s26;
           }
           else if (i >= full_step * 26 && i < full_step * 27)
           {
               s27 = (listDataScan[i])+s27;
           }
           else if (i >= full_step * 27 && i < full_step * 28)
           {
               s28 = (listDataScan[i])+s28;
           }
           else if (i >= full_step * 28 && i < full_step * 29)
           {
               s29 = (listDataScan[i])+s29;
           }
           else if (i >= full_step * 29 && i < full_step * 30)
           {
               s30 = (listDataScan[i])+s30;
           }
           else if (i >= full_step * 30 && i < full_step * 31)
           {
               s31 = (listDataScan[i])+s31;
           }
           else if (i >= full_step * 31 && i < full_step * 32)
           {
               s32 = (listDataScan[i])+s32;
           }
           else if (i >= full_step * 32 && i < full_step * 33)
           {
               s33 = (listDataScan[i])+s33;
           }
           else if (i >= full_step * 33 && i < full_step * 34)
           {
               s34 = (listDataScan[i])+s34;
           }
           else if (i >= full_step * 34 && i < full_step * 35)
           {
               s35 = (listDataScan[i])+s35;
           }
           else if (i >= full_step * 35 && i < full_step * 36)
           {
               s36 = (listDataScan[i])+s36;
           }
           else if (i >= full_step * 36 && i < full_step * 37)
           {
               s37 = (listDataScan[i])+s37;
           }
           else if (i >= full_step * 37 && i < full_step * 38)
           {
               s38 = (listDataScan[i])+s38;
           }
           else if (i >= full_step * 38 && i < full_step * 39)
           {
               s39 = (listDataScan[i])+s39;
           }
           else if (i >= full_step * 39 && i < full_step * 40)
           {
               s40 = (listDataScan[i])+s40;
           }
       }
       s1out = (s1/(full_step))*100;
       s2out = (s2/full_step)*100;
       s3out = (s3/full_step)*100;
       s4out = (s4/full_step)*100;
       s5out = (s5/full_step)*100;
       s6out = (s6/full_step)*100;
       s7out = (s7/full_step)*100;
       s8out = (s8/full_step)*100;
       s9out = (s9/full_step)*100;
       s10out = (s10/full_step)*100;
       s11out = (s11/full_step)*100;
       s12out = (s12/full_step)*100;
       s13out = (s13/full_step)*100;
       s14out = (s14/full_step)*100;
       s15out = (s15/full_step)*100;
       s16out = (s16/full_step)*100;
       s17out = (s17/full_step)*100;
       s18out = (s18/full_step)*100;
       s19out = (s19/full_step)*100;
       s20out = (s20/full_step)*100;
       s21out = (s21/full_step)*100;
       s22out = (s22/full_step)*100;
       s23out = (s23/full_step)*100;
       s24out = (s24/full_step)*100;
       s25out = (s25/full_step)*100;
       s26out = (s26/full_step)*100;
       s27out = (s27/full_step)*100;
       s28out = (s28/full_step)*100;
       s29out = (s29/full_step)*100;
       s30out = (s30/full_step)*100;
       s31out = (s31/full_step)*100;
       s32out = (s32/full_step)*100;
       s33out = (s33/full_step)*100;
       s34out = (s34/full_step)*100;
       s35out = (s35/full_step)*100;
       s36out = (s36/full_step)*100;
       s37out = (s37/full_step)*100;
       s38out = (s38/full_step)*100;
       s39out = (s39/full_step)*100;
       s40out = (s40/full_step)*100;
       ros::Duration(0.05).sleep();
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_control");
    ros::NodeHandle n;
    ros::Timer jointmove_timer_;
    ROS_INFO("Laser_Online");
    ros::Subscriber scan_sub = n.subscribe("scan", 1, LaserCallback);
    laser_pub =  n.advertise<std_msgs::Int16MultiArray>("laser_data",50);
    jointmove_timer_ = n.createTimer(ros::Duration(0.31),lasercal_Callback);
    ros::spin();
}
