﻿#include <string>
#include <iostream>
#include <cstdio>
#include <ros/ros.h>
#include "serial/serial.h"
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16MultiArray.h>
double Time_begin;
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
double start_angle;
double yaw,ang;
double pitch =0;
int robotsize = 0.55;
//int countlaser_display =0;
bool is_slope = false;
bool is_slopeH= false;
bool is_roll30 = false;
bool is_clear = false;
bool is_stop = false;
bool is_start = false;
bool is_manual = false;
bool is_planB = false;
bool is_apporach = false;
int cntr = 0;
int cntl = 0;
serial::Serial my_serial("/dev/control", 115200, serial::Timeout::simpleTimeout(50));
int dl, dr=1;
int flo,fro,bro,blo;
int speed = 30;             //robot speed 0-99
int step_move = 1;          // step size of flipper
int step_mani = 5;          // step size of Manipulator
int step_wrist = 50;        // step size of AX12
int FL_angle = 0;           // initial flipper FL 0-360
int FR_angle = 0;           // initial flipper FR 0-360
int BL_angle =0;            // initial flipper BL 0-360
int BR_angle =0;            // initial flipper BR 0-360
int j1_angle =90;           // initial Manipulator joint 1 --> j1_min - j1_max  0-180
int j2_angle =0;            // initial Manipulator joint 2 --> j2_min - j2_max
int j3_angle =0;            // initial Manipulator joint 3 --> j3_min - j3_max
int j4_angle =0;            // initial Manipulator joint 4 --> j4_min - j4_max
int j5_angle =512;         // initial servo wrist
int j6_angle = 512;         // initial servo tilt
int j7_angle = 0;
int en_FL = 0;              // read angle value encoder
int en_FR = 0;              // read angle value encoder
int en_BL = 0;              // read angle value encoder
int en_BR =0;               // read angle value encoder
int en_j1 =0, en_j2=0,en_j3=0,en_j4=0, en_j5=0,en_j6=0;
int j1_max = 180;           // maximum angle Joint 1 manipulator
int j1_min = 0;             // minimum angle Joint 1 manipulator
int j2_max = 85;
int j2_min = 0;
int j3_max = 130;
int j3_min = 0;
int j4_max = 200;
int j4_min = 0;
int j5_max = 999;          // maximum angle servo wrist
int j5_min = 0;             // minimum angle servo wrist
int j6_max = 750;
int j6_min = 320;
//----------------send communcition parameter to master----------//
string FL = "A";
string FR = "B";
string FL_L;
string FL_R;
string BR_R;
string BL_L;
string j_1;
string j_2;
string j_3;
string j_4;
string j_5;
string j_6;
string j_7;
int distancelaser[40];
char reply[50];         // read encoder from master to this buffer
char Mreply[50];
std::string re;// read encoder from master to this buffer
char* A;                // pointer for extract each flipper
char* J;
ros::Publisher Leg_pos; // publish flipper angle system (fl fr bl br)
std_msgs::Int16MultiArray fliper_pos;  // parameter for Leg_pos publisher
std::vector<short> pos; // change paremeter to ROS system parameter
ros::Publisher command_pub;
ros::Publisher status_robot;
ros::Publisher speed_robot;
ros::Publisher goal_pub;
void mani_pos(int a1,int a2, int a3, int a4)
{
    if(a1>j1_max)
    {
        a1 = j1_max;
    }
    if(a1<j1_min)
    {
        a1 = j1_min;
    }
    if(a2>j2_max)
    {
        a2 = j2_max;
    }
    if(a2<j1_min)
    {
        a2 = j2_min;
    }
    if(a3>j3_max)
    {
        a3 = j3_max;
    }
    if(a3<j3_min)
    {
        a3 = j3_min;
    }
    if(a4>j4_max)
    {
        a4 = j4_max;
    }
    if(a4<j4_min)
    {
        a4 = j4_min;
    }

    //Format to position control "Address direction spd1 spd2 pos1 pos2 pos3 j"  8 letters//
    // G=J1 H=J2 I=J3 J=J4 K=J5 L=J6
    j_1 = "Gl";
    string s = boost::lexical_cast<string>( a1 );   // "lexical_cast" change int (a1) to string (s)
    if(a1 >=100)
    {
    j_1.append("90");               // add string to
    j_1.append(s);
    j_1.append("j");
      }
    else if(a1 < 100&& a1 >= 10 )               // chk pos not more than 3 base
    {
        j_1.append("900");
        j_1.append(s);
        j_1.append("j");
    }
    else if(a1< 10)
    {
        j_1.append("9000");
        j_1.append(s);
        j_1.append("j");
    }
    //ROS_INFO("%s",j_1.c_str());

        ros::Duration(0.01).sleep(); my_serial.write(j_1);  // Ros sleep in this node only
       j1_angle = a1;                           // update pos to monitor

       if(a2 > 180)
       {
           a2 = 180;
       }
       j_2 = "Hl";
        s = boost::lexical_cast<string>( a2 );
       if(a2 >=100)
       {
       j_2.append("90");
       j_2.append(s);
       j_2.append("j");
         }
       else if(a2 < 100&& a2 >= 10 )
       {
           j_2.append("900");
           j_2.append(s);
           j_2.append("j");
       }
       else if(a2 < 10)
       {
           j_2.append("9000");
           j_2.append(s);
           j_2.append("j");
       }
       //ROS_INFO("%s",j_2.c_str());

           ros::Duration(0.01).sleep(); my_serial.write(j_2);
          j2_angle = a2;


          j_3 = "Il";
          s = boost::lexical_cast<string>( a3 );
          if(a3 >=100)
          {
          j_3.append("99");
          j_3.append(s);
          j_3.append("j");
            }
          else if(a3 < 100&& a3 >= 10 )
          {
              j_3.append("990");
              j_3.append(s);
              j_3.append("j");
          }
          else if(a3 < 10)
          {
              j_3.append("9900");
              j_3.append(s);
              j_3.append("j");
          }
          //ROS_INFO("%s",j_3.c_str());

              ros::Duration(0.5).sleep(); my_serial.write(j_3);
             j3_angle = a3;

             j_4 = "Jl";
             s = boost::lexical_cast<string>( a4 );
             if(a4 >=100)
             {
             j_4.append("99");
             j_4.append(s);
             j_4.append("j");
               }
             else if(a4 < 100&& a4 >= 10 )
             {
                 j_4.append("990");
                 j_4.append(s);
                 j_4.append("j");
             }
             else if(a4 < 10)
             {
                 j_4.append("9900");
                 j_4.append(s);
                 j_4.append("j");
             }
             //ROS_INFO("%s",j_4.c_str());

                 ros::Duration(0.01).sleep(); my_serial.write(j_4);
                j4_angle = a4;


}
//------------------Callback from Robot control node------//
//--------------to cmd master to move---------------------//

void teleop_callback(const std_msgs::StringConstPtr& msg)
{   string s;                       // robot spd string
    int old_FL = FL_angle;          // to chk last position to know next direction
    int old_FR = FR_angle;          // if current pos - old pos >0 so then go +direction
    int old_BL = BL_angle;          // if current pos - old pos <0 so then go -direction
    int old_BR = BR_angle;

   // //ROS_INFO("cFL: %d, FR: %d, BL: %d, BR: %d",FL_angle,FR_angle,BL_angle,BR_angle);  // SHOW value in window
   // //ROS_INFO("j1: %d, j2: %d, j3: %d, j4: %d",j1_angle,j2_angle,j3_angle,j4_angle);


    if(msg->data == "forward")      // msg is from robot control to move "forward"
    {
        // speed control "Address direction spd1 spd2 j"
        // A = ML B=MR C=FL D=FR E=BL F=BR


        if(speed > 99)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("r");
           FL.append(s);
            FL.append("j");
          FR.append("r");
           FR.append(s);
            FR.append("j");

        }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("r0");

           FL.append(s);
            FL.append("j");
          FR.append("r0");

           FR.append(s);
            FR.append("j");
        }
                else
        {
             s = boost::lexical_cast<string>( speed );

          FL.append("r");

           FL.append(s);
            FL.append("j");

          FR.append("r");
           FR.append(s);
            FR.append("j");
        }
        //ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
        my_serial.write(FL);
        ros::Duration(0.02).sleep();
        my_serial.write(FR);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
        dl = 1;
    }
    else if(msg->data == "backward")
    {
        if(speed > 99)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("l");
           FL.append(s);
            FL.append("j");

          FR.append("l");
           FR.append(s);
            FR.append("j");

        }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("l0");

           FL.append(s);
            FL.append("j");

          FR.append("l0");
           FR.append(s);
            FR.append("j");
        }
                else
        {
             s = boost::lexical_cast<string>( speed );

          FL.append("l");
           FL.append(s);
            FL.append("j");

          FR.append("l");
           FR.append(s);
            FR.append("j");
        }
        //ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
        my_serial.write(FL);
            ros::Duration(0.02).sleep();
        my_serial.write(FR);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
        dl = 2;
    }
    else if(msg->data == "left")
    {
        if(speed > 99)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("r");
           FL.append(s);
            FL.append("j");

          FR.append("l");
           FR.append(s);
            FR.append("j");

        }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("r0");
           FL.append(s);
            FL.append("j");

          FR.append("l0");
           FR.append(s);
            FR.append("j");
        }
                else
        {
             s = boost::lexical_cast<string>( speed );
         //    s = boost::lexical_cast<string>(10);
          FL.append("r");
           FL.append(s);
            FL.append("j");
       // s = boost::lexical_cast<string>(45);

          FR.append("l");
           FR.append(s);
            FR.append("j");
        }
        //ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
        my_serial.write(FL);
            ros::Duration(0.02).sleep();
        my_serial.write(FR);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
        dl = 3;
    }
    else if(msg->data == "right")
    {
        if(speed > 99)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("l");
           FL.append(s);
            FL.append("j");

          FR.append("r");
           FR.append(s);
            FR.append("j");

        }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("l0");
           FL.append(s);
            FL.append("j");

          FR.append("r0");
           FR.append(s);
            FR.append("j");
        }
                else
        {
             s = boost::lexical_cast<string>( speed );

          FL.append("l");
           FL.append(s);
            FL.append("j");


          FR.append("r");
           FR.append(s);
            FR.append("j");
        }
        //ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
        my_serial.write(FL);
            ros::Duration(0.02).sleep();
        my_serial.write(FR);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
        dl = 4;
    }
    else if(msg->data == "right+")                  // MOVE WHEEL R ONLY
    {
        if(speed > 99)
        {

             s = boost::lexical_cast<string>( 99 );


          FR.append("l");
           FR.append(s);
            FR.append("j");

        }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );



          FR.append("l0");
           FR.append(s);
            FR.append("j");
        }
                else
        {
             s = boost::lexical_cast<string>( speed );




          FR.append("l");
           FR.append(s);
            FR.append("j");
        }

        //ROS_INFO("%s" , FR.c_str());
        //my_serial.write(FL.c_str(),5);
          //  ros::Duration(0.02).sleep();
        my_serial.write(FR);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
    }
    else if(msg->data == "left+")                   // mOVE wHEEL l ONly
    {
        if(speed > 99)
        {

             s = boost::lexical_cast<string>( 99 );

          FL.append("l");
           FL.append(s);
            FL.append("j");

         }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("l0");
           FL.append(s);
            FL.append("j");


        }
                else
        {
             s = boost::lexical_cast<string>( speed );

          FL.append("l");
           FL.append(s);
            FL.append("j");


        }
      //  FR.append("s");
       // FR.append("00");
        //  FR.append("j");





        //ROS_INFO(" %s",FL.c_str());
        my_serial.write(FL);
            ros::Duration(0.02).sleep();

        FL = "A";
        FR = "B";
    }
   else if(msg->data == "stop")
        {
      
                if(dl == 1 && dr !=1)
                {
              FL.append("r");
              FL.append("00");
                FL.append("j");

              FR.append("r");
              FR.append("00");
                FR.append("j");
                }
                else if(dl == 2&& dr !=1)
                {
                    FL.append("l");
                    FL.append("00");
                      FL.append("j");

                    FR.append("l");
                    FR.append("00");
                    FR.append("j");
                }
                else if(dl == 3&& dr !=1)
                {
                    FL.append("r");
                    FL.append("00");
                      FL.append("j");

                    FR.append("l");
                    FR.append("00");
                    FR.append("j");
                }
                else if(dl == 4&& dr !=1)
                {
                    FL.append("l");
                    FL.append("00");
                      FL.append("j");

                    FR.append("r");
                    FR.append("00");
                    FR.append("j");
                }


  if(dr == 1)
        {
            FL.append("s");
            FL.append("00");
              FL.append("j");

            FR.append("s");
            FR.append("00");
              FR.append("j");
        }

            //ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
            my_serial.write(FL);
                ros::Duration(0.02).sleep();
            my_serial.write(FR);
            ros::Duration(0.02).sleep();
            FL = "A";
            FR = "B";
        }
    else if(msg->data == "FLFRup")
    { old_FR = FR_angle;
        FR_angle = FR_angle + step_move;






        if(FR_angle == 0)
        {
            FL_R = "Dr";
        }
       else if(FR_angle - old_FR > 0)
        {
            FL_R = "Dr";
           //   //ROS_INFO("UPPPP");

        }
        else if(FR_angle - old_FR < 0)
        {
         FL_R = "Dl";
         // //ROS_INFO("downnnnn");
        }
        else if(FR_angle - old_FR == 0)
        {
             //  //ROS_INFO("000000000000");
            FL_R = "Dl";
        }
        if(FR_angle >= 360)
        {
          FR_angle = 1;
           FL_R = "Dr";
        }
              string s = boost::lexical_cast<string>( FR_angle );
              if(FR_angle >=100)
              {
              FL_R.append("99");
              FL_R.append(s);
              FL_R.append("j");
                }
              else if(FR_angle < 100 && FR_angle >= 10)
              {
                  FL_R.append("990");
                  FL_R.append(s);
                  FL_R.append("j");
              }
              else if(FR_angle < 10)
              {
                  FL_R.append("9900");
                  FL_R.append(s);
                  FL_R.append("j");
              }
              //ROS_INFO("%s UP",FL_R.c_str());
              if(FR_angle!=360&&FR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(FL_R);

              }
        old_FL = FL_angle;



       FL_angle = FL_angle + step_move;
       //ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);


       if(FL_angle == 0)
       {
           FL_L = "Cr";
       }
      else if(FL_angle - old_FL > 0)
       {
           FL_L = "Cr";

       }
       else if(FL_angle - old_FL < 0)
       {
        FL_L = "Cl";
       }
       else if(FL_angle - old_FL == 0)
       {
           FL_L = "Cl";
       }
       if(FL_angle >= 360)
       {
         FL_angle = 1;
          FL_L = "Cr";
       }
             s = boost::lexical_cast<string>( FL_angle );


             if(FL_angle >=100)
             {

             FL_L.append("99");
             FL_L.append(s);
             FL_L.append("j");
                 }

             else if(FL_angle < 100 && FL_angle >= 10)
             {
                 FL_L.append("990");
                 FL_L.append(s);
                 FL_L.append("j");
             }
             else if(FL_angle < 10)
             {
                 FL_L.append("9900");
                 FL_L.append(s);
                 FL_L.append("j");
             }


             if(FL_angle != 360 && FL_angle != 0)
             {
                // if(FL_angle == 0)
                 //{
                 ros::Duration(0.01).sleep(); my_serial.write(FL_L);
//ROS_INFO("%s UP",FL_L.c_str());
                 //}
                 //else if(en_FL==old_FL)
                 //{
                  //   ros::Duration(0.01).sleep(); my_serial.write(FL_L.c_str(),8);
                 //}
             }

    }
    else if(msg->data == "FLFRdown")
    {
        old_FL = FL_angle;

         FL_angle = FL_angle - step_move;

        //ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);

        if(FL_angle == 360)
        {
            FL_L ="Cl";
        }
        else if(FL_angle - old_FL > 0)
        {
            FL_L = "Cr";

        }
        else if(FL_angle - old_FL < 0)
        {
         FL_L = "Cl";
        }
        else if(FL_angle - old_FL == 0)
        {
            FL_L = "Cl";
        }
        if(FL_angle <= 0)
        {
          FL_angle = 359+FL_angle;
          FL_L="Cl";
        }
              string s = boost::lexical_cast<string>( FL_angle );
              if(FL_angle >=100)
              {
              FL_L.append("99");
              FL_L.append(s);
              FL_L.append("j");
                }
              else if(FL_angle < 100&& FL_angle >= 10 )
              {
                  FL_L.append("990");
                  FL_L.append(s);
                  FL_L.append("j");
              }
              else if(FL_angle < 10)
              {
                              FL_L.append("9900");
                  FL_L.append(s);
                  FL_L.append("j");

              }
              //ROS_INFO("%s down",FL_L.c_str());
              if(FL_angle!=360&&FL_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(FL_L);

              }

    }
    else if(msg->data == "FLup" )                    // Flipper FL go +direction
    {
        old_FL = FL_angle;
        FL_angle = FL_angle + step_move;
        //ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);
        if(FL_angle == 0)
        {
            FL_L = "Cr";
        }
       else if(FL_angle - old_FL > 0)
        {
            FL_L = "Cr";

        }
        else if(FL_angle - old_FL < 0)
        {
         FL_L = "Cl";
        }
        else if(FL_angle - old_FL == 0)
        {
            FL_L = "Cl";
        }
        if(FL_angle >= 360)
        {
          FL_angle = 1;
           FL_L = "Cr";
        }
              string s = boost::lexical_cast<string>( FL_angle );


              if(FL_angle >=100)
              {

              FL_L.append("99");
              FL_L.append(s);
              FL_L.append("j");
                  }

              else if(FL_angle < 100 && FL_angle >= 10)
              {
                  FL_L.append("990");
                  FL_L.append(s);
                  FL_L.append("j");
              }
              else if(FL_angle < 10)
              {
                  FL_L.append("9900");
                  FL_L.append(s);
                  FL_L.append("j");
              }


              if(FL_angle != 360 && FL_angle != 0)
              {
                 // if(FL_angle == 0)
                  //{
                  ros::Duration(0.01).sleep(); my_serial.write(FL_L);
//ROS_INFO("%s UP",FL_L.c_str());
                  //}
                  //else if(en_FL==old_FL)
                  //{
                   //   ros::Duration(0.01).sleep(); my_serial.write(FL_L.c_str(),8);
                  //}
              }
    }
    else if(msg->data == "FLdown" )
    {

        old_FL = FL_angle;

         FL_angle = FL_angle - step_move;

        //ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);

        if(FL_angle == 360)
        {
            FL_L ="Cl";
        }
        else if(FL_angle - old_FL > 0)
        {
            FL_L = "Cr";

        }
        else if(FL_angle - old_FL < 0)
        {
         FL_L = "Cl";
        }
        else if(FL_angle - old_FL == 0)
        {
            FL_L = "Cl";
        }
        if(FL_angle <= 0)
        {
          FL_angle = 359+FL_angle;
          FL_L="Cl";
        }
              string s = boost::lexical_cast<string>( FL_angle );
              if(FL_angle >=100)
              {
              FL_L.append("99");
              FL_L.append(s);
              FL_L.append("j");
                }
              else if(FL_angle < 100&& FL_angle >= 10 )
              {
                  FL_L.append("990");
                  FL_L.append(s);
                  FL_L.append("j");
              }
              else if(FL_angle < 10)
              {
                              FL_L.append("9900");
                  FL_L.append(s);
                  FL_L.append("j");

              }
              //ROS_INFO("%s down",FL_L.c_str());
              if(FL_angle!=360&&FL_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(FL_L);

              }
    }
}

//------------------subscribe value to this node------------//
void SpeedCallback(const std_msgs::Int16ConstPtr& msg)
{
speed = msg->data;
}
void leg_Callback(const ros::TimerEvent& event)  // timer of this node for read leg pos
{
    try         // if in "try" can do then do, but if can't do go to "catch"
    {
        std::string enleg;
        std::string enm;
    my_serial.write("?Aj");
    ros::Duration(0.005).sleep();
     my_serial.readline(enleg);
       en_FL = strtod(enleg.c_str(),&A);        // 'strtod' change str to double
        ROS_INFO("%d",en_FL);
       en_FR = strtod(A,&A);
       en_BL = strtod(A,&A);
       en_BR = strtod(A,NULL);
       if(en_FL!=0)
              {
                  FL_angle = en_FL;
              }
              if(en_FR!=0)
              {
                 FR_angle = en_FR;
              }
              if(en_BR!=0)
              {
                  BR_angle = en_BR;
              }
              if(en_BL!=0)
              {
                  BL_angle = en_BL;
              }
            pos.push_back(en_FL);            // put the read angle to pos vector

      }
       catch(serial::SerialException& e)
       {
        ROS_ERROR("read Timeout");
       }
}
void angle_callback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    int a1 = (int)msg->data[0];
    int a2 = (int)msg->data[1];
    int a3 = (int)msg->data[2];
    int a4 = (int)msg->data[3];
    //int a5 = (int)msg->data[4];
    //int a6 = (int)msg->data[5];
    mani_pos(a1,a2,a3,a4);
}
void speed_control(int R_direction,int L_direction,int R_speed,int L_speed)
{
    string s;
    //int R_direction = msg->data[0];
   // int L_direction = msg->data[1];
  //  int R_speed = msg->data[2];
  //  int L_speed = msg->data[3];
 //   int FL_Pos = msg->data[4];
   // int FR_Pos = msg->data[5];
    string Rd;
    string Ld;

 //INFO("FL_POS %d : FR_POS %d",FL_Pos,FR_Pos);
    if( R_direction == 0) // backward
    {
        Rd = "r";
    }
    else
    {
        Rd = "l";
    }

    if(L_direction == 0)
    {
        Ld = "l";
    }
    else
    {
        Ld = "r";
    }


        if(L_speed > 99)
        {

             s = boost::lexical_cast<string>( L_speed );

          FL.append(Ld);
           FL.append(s);


        }
        else if(L_speed < 10)
        {

             s = boost::lexical_cast<string>( L_speed );

          FL.append(Ld);
          FL.append("0");
           FL.append(s);
            FL.append("j");



        }
                else
        {
             s = boost::lexical_cast<string>( L_speed );

          FL.append(Ld);

           FL.append(s);
            FL.append("j");


        }

        if(R_speed > 99)
        {

             s = boost::lexical_cast<string>( L_speed );

          FR.append(Rd);
           FR.append(s);


        }
        else if(R_speed < 10)
        {

             s = boost::lexical_cast<string>( R_speed );

          FR.append(Rd);
          FR.append("0");
           FR.append(s);
            FR.append("j");



        }
                else
        {
             s = boost::lexical_cast<string>( R_speed );

          FR.append(Rd);

           FR.append(s);
            FR.append("j");


        }

            if(R_speed == 0 && L_speed == 0)
            {
                my_serial.write("As00j");
                ros::Duration(0.02).sleep();
                my_serial.write("Bs00j");
               //  //ROS_INFO("Stop");
            }
            else
            {
   //     //ROS_INFO("CMd_vel::::::::::::%s : %s",FL.c_str(),FR.c_str());
        ros::Duration(0.02).sleep();
        my_serial.write(FL);
        ros::Duration(0.02).sleep();
        my_serial.write(FR);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
            }
   //leg_control(FL_Pos,FR_Pos);



}
void imucallback(const sensor_msgs::ImuPtr& im)
{   
    
   double angle;
angle = im->orientation.y;
////ROS_INFO("%d",abs(ang-pitch));
if(abs(pitch-ang)>8)
{
//ROS_WARN("slope down");
        my_serial.write("As00j");
        ros::Duration(0.01).sleep();
        my_serial.write("Bs00j");
        ros::Duration(0.01).sleep();
	ros::Duration(0.1).sleep();
}
     /// check high slope

  if(im->orientation.y > 15)
  {
     // is_slopeH = true;
  }
  else
  {
     is_slopeH = false;
  }

   /// check low slope
   if(im->orientation.x <-7.0)
      {
          is_slope = false;
        //  ROS_WARN("slope");
      }
      else
      {
          is_slope = false;
      }
/// check roll slope30 //
    if(abs(im->orientation.x)>10)
	{
        is_roll30 = false;
	}
	else 
	{
		is_roll30 = false;
	}
pitch = im->orientation.x;
yaw = im->orientation.z;
}

void navigationCallback(const geometry_msgs::TwistPtr& msg)
{
    std_msgs::String status;
    if(is_start==true&&is_stop == false&&is_manual == false)
    {
    double speedL, speedR;
    double linear, angular;
    int dL,dR;
    int maxspeed = 40;
    int minspeed = 0;
    linear = msg->linear.x;
    angular = msg->angular.z;
   // speedL = msg->linear.x*10 - msg->angular.z*(robotsize/2);
    //speedR = msg->linear.x*10 + msg->angular.z*(robotsize/2);
              // robot speed in m/s
        if(is_slope == false)
        {
            speedL = (linear*1 - angular*0.30 ); //0.65
            speedR = (linear*1 + angular*0.30 );//0.8
            // m/s to m/min
            speedR = speedR*60;
            speedL = speedL*60;
            // 1 track rpm = 1.18 m/min
            speedR = speedR/1.3; // 1.18
            speedL = speedL/1.3;
            //  4 motor rpm = 1 track rpm
            speedR = speedR*4.1;
            speedL = speedL*4.1;
            minspeed = 13; //8
            maxspeed = 40;
        }
        else
        {
          //  //ROS_INFO("SLOPE MODEEEEEEEEEEEEEEEEEEEEEEEEEE");
            speedL = (linear*1 - angular*0.28 );
            speedR = (linear*1 + angular*0.28 );
            // m/s to m/min
            speedR = speedR*60;
            speedL = speedL*60;
            // 1 track rpm = 1.18 m/min
            speedR = speedR/1.3; // 1.18
            speedL = speedL/1.3;
            //  4 motor rpm = 1 track rpm
            speedR = speedR*4.1;
            speedL = speedL*4.1;
            minspeed = 10;
        }
        if(speedL>0)
        {
         dL = 1;  // forward direction
            if(speedL > maxspeed)
            {
            speedL = maxspeed;
            }
            else if(speedL < minspeed )
            {
            speedL = minspeed;
            }
        }
        else if(speedL<0)
        {
            dL = 0; //forward direction
            if(speedL < -maxspeed)
            {
            speedL = -maxspeed;
            }
            else if(speedL > -minspeed)
            {
            speedL = -minspeed;
            }
        }
        else if(speedL==0)
        {
             speedL = 0;
        }
        if(speedR>0)
        {
             dR = 1;
            if(speedR>maxspeed)
            {
                speedR = maxspeed;
            }
            else if(speedR<minspeed && speedR!=0)
            {
                speedR = minspeed;
            }
        }
        else if(speedR <0)
        {
            dR = 0;
            if(speedR< -maxspeed)
            {
                speedR = -maxspeed;
            }
            else if(speedR > -minspeed && speedR!=0)
            {
                speedR = -minspeed;
            }
        }
        else if(speedR == 0)
        {
            speedR =0;
        }
        //if(is_start == true && is_stop == false)
        //{
       // //ROS_INFO("L %f:a %f: speedL %f: speedR %f",linear,angular,speedL,speedR);
        std_msgs::String speedtext;
   // char* buffspeed;
    //sprintf(buffspeed, "L %f      R %f", speedL,speedR);
    //std::string s;
       string sp = boost::lexical_cast<string>( (int)speedL );
       speedtext.data = sp;
       sp = boost::lexical_cast<string>( (int)speedR );
       speedtext.data.append("     ");
       speedtext.data.append(sp);
     // speedtext.data.c_str() = buffspeed;
       speed_robot.publish(speedtext);
       speed_control(dR,dL,abs(speedR),abs(speedL));
       status.data = "Start";
       status_robot.publish(status);
    //}

  //    Time_begin = ros::Time::now().toSec();
    }
    else if(is_stop == true||is_manual == false)
    {
       int speedL =0;
       int speedR =0;
       int dR = 0,dL=0;
    //   //ROS_INFO("speedL %f: speedR %f",speedL,speedR);
       std_msgs::String speedtext;
      // char* buffspeed;
       //sprintf(buffspeed, "L %f      R %f", speedL,speedR);
       //std::string s;
       string sp = boost::lexical_cast<string>( (int)speedL );
       speedtext.data = sp;
       sp = boost::lexical_cast<string>( (int)speedR );
       speedtext.data.append("     ");
       speedtext.data.append(sp);
        // speedtext.data.c_str() = buffspeed;
       speed_robot.publish(speedtext);
       speed_control(dR,dL,abs(speedR),abs(speedL));
       status.data = "Start";
       status_robot.publish(status);
    }
}
void LaserCallback(const std_msgs::Int16MultiArray& data)
{
    for(int i = 0;i<40;i++)
    {
    distancelaser[i] = data.data[i];
    }
    if(is_start==true&&is_stop==false&&is_apporach ==false)
    {
    if(distancelaser[16]<35&&distancelaser[18]<32&&distancelaser[20]<30&&distancelaser[22]<27)
    {
        is_stop = true;
        ROS_WARN("COLLISION WARING");
        //ROS_INFO("walk back");
        my_serial.write("Al20j");
        ros::Duration(0.01).sleep();
        my_serial.write("Br20j");
        ros::Duration(0.01).sleep();
        ros::Duration(0.03).sleep();
        std_msgs::String ss;
         ss.data = "path";
         command_pub.publish(ss);
        is_stop = false;
    }
    else if(distancelaser[20]<27&&distancelaser[22]<30&&distancelaser[24]<32&&distancelaser[26]<35)
    {
        is_stop = true;
        ROS_WARN("COLLISION WARING");
        //ROS_INFO("walk back");
        my_serial.write("Al20j");
        ros::Duration(0.01).sleep();
        my_serial.write("Br20j");
        ros::Duration(0.01).sleep();
        ros::Duration(0.03).sleep();
        std_msgs::String ss;
        ss.data = "path";
        command_pub.publish(ss);
        is_stop = false;
    }
    else if(distancelaser[17]<25&&distancelaser[18]<25&&distancelaser[19]<25&&distancelaser[20]<25)
    {
        is_stop = true;
        ROS_WARN("COLLISION WARING");
        //ROS_INFO("walk back");
        my_serial.write("Al20j");
        ros::Duration(0.01).sleep();
        my_serial.write("Br20j");
        ros::Duration(0.01).sleep();
        ros::Duration(0.03).sleep();
        std_msgs::String ss;
        ss.data = "path";
        command_pub.publish(ss);
        is_stop = false;
    }
    else if(distancelaser[21]<25&&distancelaser[22]<25&&distancelaser[23]<25&&distancelaser[24]<25)
    {
        is_stop = true;
        ROS_WARN("COLLISION WARING");
        //ROS_INFO("walk back");
        my_serial.write("Al20j");
        ros::Duration(0.01).sleep();
        my_serial.write("Br20j");
        ros::Duration(0.01).sleep();
        ros::Duration(0.03).sleep();
        std_msgs::String ss;
        ss.data = "path";
        command_pub.publish(ss);
        is_stop = false;
    }
    }
    else if(is_start==true&&is_stop==false&&is_apporach ==true)
    {
    if(distancelaser[16]<15&&distancelaser[18]<15&&distancelaser[20]<15&&distancelaser[22]<17)
    {
        is_stop = true;
        ROS_WARN("COLLISION WARING");
        //ROS_INFO("walk back");
        my_serial.write("Al20j");
        ros::Duration(0.01).sleep();
        my_serial.write("Br20j");
        ros::Duration(0.01).sleep();
        ros::Duration(0.03).sleep();
        std_msgs::String ss;
         ss.data = "path";
         command_pub.publish(ss);
        is_stop = false;
    }
    else if(distancelaser[20]<17&&distancelaser[22]<15&&distancelaser[24]<15&&distancelaser[26]<15)
    {
        is_stop = true;
        ROS_WARN("COLLISION WARING");
        //ROS_INFO("walk back");
        my_serial.write("Al20j");
        ros::Duration(0.01).sleep();
        my_serial.write("Br20j");
        ros::Duration(0.01).sleep();
        ros::Duration(0.03).sleep();
        std_msgs::String ss;
        ss.data = "path";
        command_pub.publish(ss);
        is_stop = false;
    }
    }
}
void commandCallback(const std_msgs::StringPtr& msg)
{
    std_msgs::String status;
    ROS_INFO("%s",msg->data.c_str());
    if( msg->data == "start_walk")
    {
        std_msgs::String ss;
        ss.data = "start_navi";
        command_pub.publish(ss);
        ros::Duration(0.01).sleep();
        ss.data = "path";
        command_pub.publish(ss);
        ros::Duration(0.01).sleep();
        status.data = "Start";
        status_robot.publish(status);
        ros::Duration(0.01).sleep();
        ROS_INFO("GO");
        //////// Do what ever u want when robot is going to start ///////
        is_start = true;
        is_stop = false;
    }
    else if(msg->data == "left")
    {
        geometry_msgs::PoseStamped goalleft;
           // goalleft.header = "test";
            goalleft.pose.position.x = 1;
            goalleft.pose.position.y = 0.3;
    }
    else if(msg->data == "moveback")
    {
        ROS_WARN("Cannot generate path move back");
        is_planB = true;
        status.data = "Plan B";
        status_robot.publish(status);
        if(is_planB == true&&is_manual == false)
            {
            ROS_ERROR("planB ON");
            ros::Duration(1).sleep();
                if(distancelaser[34] > distancelaser[6] && distancelaser[34] > 80 && distancelaser[38] > distancelaser[2] && distancelaser[38] > 100 )
                    {
                    // Left turn
                    ROS_INFO("rotate Left");
                    my_serial.write("Al40j");
                    ros::Duration(0.01).sleep();
                    my_serial.write("Bl40j");
                    ros::Duration(0.01).sleep();
                    }
                else if(distancelaser[34] < distancelaser[6] && distancelaser[6] > 80 && distancelaser[38] < distancelaser[2] && distancelaser[2] > 100)
                    {
                    // right trun
                    ROS_INFO("rotate left");
                    my_serial.write("Ar40j");
                    ros::Duration(0.01).sleep();
                    my_serial.write("Br40j");
                    ros::Duration(0.01).sleep();
                    }
                else if(distancelaser[18] < 14 && distancelaser[20] < 14 && distancelaser[22] < 14)
                    {
                    ROS_INFO("plan B moveback hard");
                    ros::Duration(4).sleep();
                    my_serial.write("Al20j");
                    ros::Duration(0.01).sleep();
                    my_serial.write("Br20j");
                    ros::Duration(0.01).sleep();
                    ros::Duration(0.5).sleep();
                    }

                if(distancelaser[16] < 37 && distancelaser[22] < 35 && distancelaser[18] < 35 && distancelaser[20] < 37)
                    {
                    ROS_INFO("plan B moveback soft");
                    ros::Duration(4).sleep();
                    my_serial.write("Al20j");
                    ros::Duration(0.01).sleep();
                    my_serial.write("Br20j");
                    ros::Duration(0.01).sleep();
                    }
                else if(distancelaser[18] > 60 && distancelaser[20] > 60 &&distancelaser[16] > 60&&distancelaser[22] > 60)
                    {
                    ROS_INFO("move forward");
                    is_stop = true;
                    my_serial.write("Ar30j");
                    ros::Duration(0.01).sleep();
                    my_serial.write("Bl30j");
                    ros::Duration(0.01).sleep();
                    }
                else
                    {
                    ROS_INFO("move back don't sure anything");
                    is_stop = true;
                    my_serial.write("Al15j");
                    ros::Duration(0.01).sleep();
                    my_serial.write("Br15j");
                    ros::Duration(0.01).sleep();
                    }
                    ros::Duration(0.3).sleep();
                    my_serial.write("As00j");
                    ros::Duration(0.01).sleep();
                    my_serial.write("Bs00j");
                    ros::Duration(0.01).sleep();
                    ros::Duration(0.2).sleep();
                    is_planB = false;
                    is_stop = false;
                    std_msgs::String ss;
                    ss.data = "path";
                    command_pub.publish(ss);
            }
    }
    else if(msg->data == "stop")
    {
        is_stop = true;
        status.data = "Stop";
        status_robot.publish(status);
    }
    else if(msg->data == "getposition")
    {
     //       is_start = true;
     //       is_stop = false;
    }
    else if(msg->data == "reach_goal")
    {
        is_apporach = false;
    }
    else if(msg->data == "teleop_mode") // switch teleop mode
    {   ROS_INFO("MANUAL");
        is_manual = true;
        status.data = "MANUAL";
        status_robot.publish(status);
    }
    else if(msg->data == "auto_mode") // switch autonomous mode
    {   ROS_INFO("AUTO");
        is_manual = false;
        status.data = "AUTO";
        status_robot.publish(status);
    }
    else if(msg->data == "go") // switch autonomous mode
    {   
        ROS_INFO("GO to next victim");
        is_start = true;
        is_stop = false;
	    std_msgs::String ss;
        ss.data = "path";
     	command_pub.publish(ss);
        status.data = "Next Go Go";
        status_robot.publish(status);
    }
    else if(msg->data == "timeout_search")
    {
        ROS_INFO("Search agian");
            is_start =true;
            is_stop  = false;
            //std_msgs::String ss;
    }
    else if(msg->data == "approach_victim")
    {
        is_start = true;
        is_stop = false;
        status.data = "Approach Victim";
        status_robot.publish(status);

    }
    else if(msg->data == "reset")
    {
        is_start = false;
        is_stop = true;
        status.data = "RESET MAP";
        status_robot.publish(status);
    }
    else if(msg->data == "check_hole")
    {
        is_start = false;
        is_stop = true;
        status.data = "check hole";
        status_robot.publish(status);
    }
}
void sloptime_Callback(const ros::TimerEvent& event)
{    
      ang=pitch;
}
void apporach_Callback(const geometry_msgs::PoseStamped &s)
{
    is_apporach = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dsPIC_Node");            // start ros node name"dsPIC_Node"
    ros::NodeHandle n;                              // handle the information
    ros::Subscriber sub = n.subscribe("teleop_control", 1, teleop_callback);    //subscribe cmd from robot control
    ros::Subscriber sub2 = n.subscribe("speed", 1, SpeedCallback);              //sbscribe spd from robot control
    ros::Subscriber sub3 = n.subscribe("angle",10,angle_callback);              //subscribe angle from robot control
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",10,navigationCallback);
    ros::Subscriber imu_sub = n.subscribe("imu",10,imucallback);
    ros::Subscriber command_sub = n.subscribe("syscommand",10,commandCallback);
    ros::Timer imu_time = n.createTimer(ros::Duration(0.25),sloptime_Callback);
    ros::Timer leg_time = n.createTimer(ros::Duration(0.75),leg_Callback);
    ros::Subscriber apporach_sub = n.subscribe("move_base_simple/goal",10,apporach_Callback);
    ros::Subscriber laser_sub = n.subscribe("laser_data", 1, LaserCallback);
    Time_begin = ros::Time::now().toSec();
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",10);
    command_pub = n.advertise<std_msgs::String>("syscommand",10);
    status_robot = n.advertise<std_msgs::String>("status_robot",10);
    speed_robot = n.advertise<std_msgs::String>("speedrobot",10);
    std::string str = "/dev/ttyUSB0";
    n.getParam("my_serial", str);
    try
    {
      // ROS_INFO("Try to open port %s.",str.c_str());


    if(my_serial.isOpen())
    {
        ROS_INFO("Port is opened");
        std::string enleg;
        std::string enm;
        /*
    my_serial.write("?Aj");
    ros::Duration(0.01).sleep();
     my_serial.readline(enleg);
      //int result_io =  device.read(reply,18,1000);   // buffer,length,timeout
  //    ROS_INFO("%d",result_io);         // chk number of msgs
  // ROS_INFO("leg::::::%s",enleg.c_str());
       en_FL = strtod(enleg.c_str(),&A);        // 'strtod' change str to double
       en_FR = strtod(A,&A);
       en_BL = strtod(A,&A);
       en_BR = strtod(A,NULL);


           FL_angle = en_FL;
           FR_angle = en_FR;
          BR_angle = en_BR;
           BL_angle = en_BL;

      my_serial.write("?Mj");
      ros::Duration(0.01).sleep();
      my_serial.readline(enm);
        // result_io =  device.read(Mreply,18,1000);   // buffer,length,timeout
         //   ROS_INFO("%d",result_io);         // chk number of msgs
     ROS_INFO("%d",FL_angle);
            en_j1 = strtod(enm.c_str(),&J);        // 'strtod' change str to double
            en_j2 = strtod(J,&J);
            en_j3 = strtod(J,&J);
            en_j4 = strtod(J,NULL);
         //   en_j5 = j5_angle*0.29;
           // en_j6 = j6_angle*0.29;
            j1_angle = en_j1;
            j2_angle = en_j2;
            j3_angle = en_j3;
            */
        my_serial.write("As00j");
        ros::Duration(0.01).sleep();
        my_serial.write("Bs00j");
        ros::Duration(0.01).sleep();
    }
    else
    {
        ROS_FATAL("Failed to open the serial port!!!");
       // ROS_BREAK();
    }

  //   ros::Duration(0.01).sleep();

    }

    catch(serial::SerialException& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        //ROS_BREAK();
    }
 ros::spin();
}