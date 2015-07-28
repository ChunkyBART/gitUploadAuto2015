#include <string>
#include <iostream>
#include <cstdio>
#include <ros/ros.h>
#include "serial/serial.h"
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
int robotsize = 0.55;
bool is_slope = false;
bool is_clear = false;
bool is_stop = false;
bool is_start = true;
bool is_manual = false;
int cntr = 0;
int cntl = 0;
serial::Serial my_serial("/dev/control", 115200, serial::Timeout::simpleTimeout(50));
int dl, dr=1;
int flo,fro,bro,blo;
int speed = 60;             //robot speed 0-99
int step_move = 3;          // step size of flipper
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
char reply[50];         // read encoder from master to this buffer
char Mreply[50];
std::string re;// read encoder from master to this buffer
char* A;                // pointer for extract each flipper
char* J;
ros::Publisher Leg_pos; // publish flipper angle system (fl fr bl br)
std_msgs::Int16MultiArray fliper_pos;  // parameter for Leg_pos publisher
std::vector<short> pos; // change paremeter to ROS system parameter

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
    ROS_INFO("%s",j_1.c_str());

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
       ROS_INFO("%s",j_2.c_str());

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
          ROS_INFO("%s",j_3.c_str());

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
             ROS_INFO("%s",j_4.c_str());

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

   // ROS_INFO("cFL: %d, FR: %d, BL: %d, BR: %d",FL_angle,FR_angle,BL_angle,BR_angle);  // SHOW value in window
   // ROS_INFO("j1: %d, j2: %d, j3: %d, j4: %d",j1_angle,j2_angle,j3_angle,j4_angle);


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
        ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
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
        ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
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

          FL.append("r");
           FL.append(s);
            FL.append("j");


          FR.append("l");
           FR.append(s);
            FR.append("j");
        }
        ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
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
        ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
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

        ROS_INFO("%s" , FR.c_str());
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





        ROS_INFO(" %s",FL.c_str());
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

            ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
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
           //   ROS_INFO("UPPPP");

        }
        else if(FR_angle - old_FR < 0)
        {
         FL_R = "Dl";
         // ROS_INFO("downnnnn");
        }
        else if(FR_angle - old_FR == 0)
        {
             //  ROS_INFO("000000000000");
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
              ROS_INFO("%s UP",FL_R.c_str());
              if(FR_angle!=360&&FR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(FL_R);

              }
        old_FL = FL_angle;



       FL_angle = FL_angle + step_move;
       ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);


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
ROS_INFO("%s UP",FL_L.c_str());
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

        ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);

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
              ROS_INFO("%s down",FL_L.c_str());
              if(FL_angle!=360&&FL_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(FL_L);

              }
              old_FR = FR_angle;
              FR_angle = FR_angle -step_move;


              if(FR_angle == 360)
              {
                  FL_R ="Dl";
              }
              else if(FR_angle - old_FR > 0)
              {
                  FL_R = "Dr";

              }
              else if(FR_angle - old_FR < 0)
              {
               FL_R = "Dl";
              }
              else if(FR_angle - old_FR == 0)
              {
                  FL_R = "Dl";
              }
              if(FR_angle <= 0)
              {
                FR_angle = 359+FR_angle;
                FL_L="Dl";
              }
                     s = boost::lexical_cast<string>( FR_angle );
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
                    ROS_INFO("%s DOWN",FL_R.c_str());
                    if(FR_angle!=360&&FR_angle != 0)
                    {
                        ros::Duration(0.01).sleep(); my_serial.write(FL_R);

                    }
    }
    else if(msg->data == "FLup" )                    // Flipper FL go +direction
    {
         old_FL = FL_angle;



        FL_angle = FL_angle + step_move;
        ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);


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
ROS_INFO("%s UP",FL_L.c_str());
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

        ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);

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
              ROS_INFO("%s down",FL_L.c_str());
              if(FL_angle!=360&&FL_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(FL_L);

              }

    }

    else if(msg->data == "FRup")
    {

        old_FR = FR_angle;
        FR_angle = FR_angle + step_move;






        if(FR_angle == 0)
        {
            FL_R = "Dr";
        }
       else if(FR_angle - old_FR > 0)
        {
            FL_R = "Dr";
           //   ROS_INFO("UPPPP");

        }
        else if(FR_angle - old_FR < 0)
        {
         FL_R = "Dl";
         // ROS_INFO("downnnnn");
        }
        else if(FR_angle - old_FR == 0)
        {
             //  ROS_INFO("000000000000");
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
              ROS_INFO("%s UP",FL_R.c_str());
              if(FR_angle!=360&&FR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(FL_R);

              }

    }
    else if(msg->data == "FRdown")
    {

        old_FR = FR_angle;
        FR_angle = FR_angle -step_move;


        if(FR_angle == 360)
        {
            FL_R ="Dl";
        }
        else if(FR_angle - old_FR > 0)
        {
            FL_R = "Dr";

        }
        else if(FR_angle - old_FR < 0)
        {
         FL_R = "Dl";
        }
        else if(FR_angle - old_FR == 0)
        {
            FL_R = "Dl";
        }
        if(FR_angle <= 0)
        {
          FR_angle = 359+FR_angle;
          FL_L="Dl";
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
              ROS_INFO("%s DOWN",FL_R.c_str());
              if(FR_angle!=360&&FR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(FL_R);

              }


    }
    else if(msg->data == "BLBRup")
    { old_BR = BR_angle;
        BR_angle = BR_angle +step_move;


        if(BR_angle == 0)
        {
            BR_R = "Fr";
        }
        else if(BR_angle - old_BR > 0)
        {
            BR_R = "Fr";

        }
        else if(BR_angle - old_BR < 0)
        {
         BR_R = "Fl";
        }
        else if(BR_angle - old_BR == 0)
        {
            BR_R = "Fl";
        }
        if(BR_angle >= 360)
        {
          BR_angle = 1;
           BR_R = "Fr";
        }
              string s = boost::lexical_cast<string>( BR_angle );
              if(BR_angle >=100)
              {
              BR_R.append("99");
              BR_R.append(s);
              BR_R.append("j");
                }
              else if(BR_angle < 100 && BR_angle >= 10 )
              {
                  BR_R.append("990");
                  BR_R.append(s);
                  BR_R.append("j");
              }
              else if(BR_angle < 10)
              {
                  BR_R.append("9900");
                  BR_R.append(s);
                  BR_R.append("j");
              }

              ROS_INFO("%s",BR_R.c_str());
              if(BR_angle!=360&&BR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(BR_R);

              }
        old_BL = BL_angle;

        BL_angle = BL_angle +step_move;

        ROS_INFO("OLD %d, NEW %d", old_BL,BL_angle);

        if(BL_angle == 0)
        {
            BL_L = "Er";
        }
        else if(BL_angle - old_BL > 0)
        {
            BL_L = "Er";

        }
        else if(BL_angle - old_BL < 0)
        {
         BL_L = "El";
        }
        else if(BL_angle - old_BL == 0)
        {
            BL_L = "El";
        }
        if(BL_angle >= 360)
        {
          BL_angle = 1;
           BL_L = "Er";
        }
               s = boost::lexical_cast<string>( BL_angle );
              if(BL_angle >=100)
              {
              BL_L.append("99");
              BL_L.append(s);
              BL_L.append("j");
                }
              else if(BL_angle < 100&& BL_angle >= 10 )
              {
                  BL_L.append("990");
                  BL_L.append(s);
                  BL_L.append("j");
              }
              else if(BL_angle < 10)
              {
                  BL_L.append("9900");
                  BL_L.append(s);
                  BL_L.append("j");
              }
              ROS_INFO("%s",BL_L.c_str());
              if(BL_angle!=360&&BL_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(BL_L);

              }
    }
    else if(msg->data == "BLBRdown")
    {
        old_BL = BL_angle;
        BL_angle = BL_angle - step_move;
         ROS_INFO("OLD %d, NEW %d", old_BL,BL_angle);

        if(BL_angle == 360)
        {
            BL_L = "El";
        }
      else if(BL_angle - old_BL > 0)
        {
            BL_L = "Er";

        }
        else if(BL_angle - old_BL < 0)
        {
         BL_L = "El";
        }
        else if(BL_angle - old_BL == 0)
        {
            BL_L = "El";
        }
        if(BL_angle <= 0)
        {
          BL_angle = 359+BL_angle;
          BL_L="El";
        }
             string s = boost::lexical_cast<string>( BL_angle );
             if(BL_angle >=100)
             {
             BL_L.append("99");
             BL_L.append(s);
             BL_L.append("j");
               }
             else if(BL_angle < 100 && BL_angle >= 10)
             {
                 BL_L.append("990");
                 BL_L.append(s);
                 BL_L.append("j");
             }
             else if(BL_angle < 10)
             {
                 BL_L.append("9900");
                 BL_L.append(s);
                 BL_L.append("j");
             }
             ROS_INFO("%s",BL_L.c_str());
             if(BL_angle!=360&&BL_angle != 0)
             {
                 ros::Duration(0.01).sleep(); my_serial.write(BL_L);

             }
             old_BR = BR_angle;
             BR_angle = BR_angle -step_move;

             if(BR_angle == 360)
             {
                 BR_R = "Fl";
             }
             else if(BR_angle - old_BR > 0)
             {
                 BR_R = "Fr";

             }
             else if(BR_angle - old_BR < 0)
             {
              BR_R = "Fl";
             }
             else if(BR_angle - old_BR == 0)
             {
                 BR_R = "Fl";
             }
             if(BR_angle <= 0)
             {
               BR_angle = 359+BR_angle;
               BR_R="Fl";
             }
                    s = boost::lexical_cast<string>( BR_angle );
                   if(BR_angle >=100)
                   {
                   BR_R.append("99");
                   BR_R.append(s);
                   BR_R.append("j");
                     }
                   else if(BR_angle < 100 && BR_angle >= 10)
                   {
                       BR_R.append("990");
                       BR_R.append(s);
                       BR_R.append("j");
                   }
                   else if(BR_angle < 10)
                   {
                       BR_R.append("9900");
                       BR_R.append(s);
                       BR_R.append("j");
                   }
                   ROS_INFO("%s",BR_R.c_str());
                   if(BR_angle!=360&&BR_angle != 0)
                   {
                       ros::Duration(0.01).sleep(); my_serial.write(BR_R);

                   }
    }
    else if(msg->data == "alldown")
    {
        old_BL = BL_angle;
        BL_angle = BL_angle - step_move;
         ROS_INFO("OLD %d, NEW %d", old_BL,BL_angle);

        if(BL_angle == 360)
        {
            BL_L = "El";
        }
      else if(BL_angle - old_BL > 0)
        {
            BL_L = "Er";

        }
        else if(BL_angle - old_BL < 0)
        {
         BL_L = "El";
        }
        else if(BL_angle - old_BL == 0)
        {
            BL_L = "El";
        }
        if(BL_angle <= 0)
        {
          BL_angle = 359+BL_angle;
          BL_L="El";
        }
             string s = boost::lexical_cast<string>( BL_angle );
             if(BL_angle >=100)
             {
             BL_L.append("99");
             BL_L.append(s);
             BL_L.append("j");
               }
             else if(BL_angle < 100 && BL_angle >= 10)
             {
                 BL_L.append("990");
                 BL_L.append(s);
                 BL_L.append("j");
             }
             else if(BL_angle < 10)
             {
                 BL_L.append("9900");
                 BL_L.append(s);
                 BL_L.append("j");
             }
             ROS_INFO("%s",BL_L.c_str());
             if(BL_angle!=360&&BL_angle != 0)
             {
                 ros::Duration(0.01).sleep(); my_serial.write(BL_L);

             }
             old_BR = BR_angle;
             BR_angle = BR_angle -step_move;

             if(BR_angle == 360)
             {
                 BR_R = "Fl";
             }
             else if(BR_angle - old_BR > 0)
             {
                 BR_R = "Fr";

             }
             else if(BR_angle - old_BR < 0)
             {
              BR_R = "Fl";
             }
             else if(BR_angle - old_BR == 0)
             {
                 BR_R = "Fl";
             }
             if(BR_angle <= 0)
             {
               BR_angle = 359+BR_angle;
               BR_R="Fl";
             }
                    s = boost::lexical_cast<string>( BR_angle );
                   if(BR_angle >=100)
                   {
                   BR_R.append("99");
                   BR_R.append(s);
                   BR_R.append("j");
                     }
                   else if(BR_angle < 100 && BR_angle >= 10)
                   {
                       BR_R.append("990");
                       BR_R.append(s);
                       BR_R.append("j");
                   }
                   else if(BR_angle < 10)
                   {
                       BR_R.append("9900");
                       BR_R.append(s);
                       BR_R.append("j");
                   }
                   ROS_INFO("%s",BR_R.c_str());
                   if(BR_angle!=360&&BR_angle != 0)
                   {
                       ros::Duration(0.01).sleep(); my_serial.write(BR_R);

                   }
                   old_FL = FL_angle;

                    FL_angle = FL_angle - step_move;

                   ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);

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
                          s = boost::lexical_cast<string>( FL_angle );
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
                         ROS_INFO("%s down",FL_L.c_str());
                         if(FL_angle!=360&&FL_angle != 0)
                         {
                             ros::Duration(0.01).sleep(); my_serial.write(FL_L);

                         }
                         old_FR = FR_angle;
                         FR_angle = FR_angle -step_move;


                         if(FR_angle == 360)
                         {
                             FL_R ="Dl";
                         }
                         else if(FR_angle - old_FR > 0)
                         {
                             FL_R = "Dr";

                         }
                         else if(FR_angle - old_FR < 0)
                         {
                          FL_R = "Dl";
                         }
                         else if(FR_angle - old_FR == 0)
                         {
                             FL_R = "Dl";
                         }
                         if(FR_angle <= 0)
                         {
                           FR_angle = 359+FR_angle;
                           FL_L="Dl";
                         }
                                s = boost::lexical_cast<string>( FR_angle );
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
                               ROS_INFO("%s DOWN",FL_R.c_str());
                               if(FR_angle!=360&&FR_angle != 0)
                               {
                                   ros::Duration(0.01).sleep(); my_serial.write(FL_R);

                               }
    }
    else if(msg->data =="allup")
    {
        old_BR = BR_angle;
                BR_angle = BR_angle +step_move;


                if(BR_angle == 0)
                {
                    BR_R = "Fr";
                }
                else if(BR_angle - old_BR > 0)
                {
                    BR_R = "Fr";

                }
                else if(BR_angle - old_BR < 0)
                {
                 BR_R = "Fl";
                }
                else if(BR_angle - old_BR == 0)
                {
                    BR_R = "Fl";
                }
                if(BR_angle >= 360)
                {
                  BR_angle = 1;
                   BR_R = "Fr";
                }
                      string s = boost::lexical_cast<string>( BR_angle );
                      if(BR_angle >=100)
                      {
                      BR_R.append("99");
                      BR_R.append(s);
                      BR_R.append("j");
                        }
                      else if(BR_angle < 100 && BR_angle >= 10 )
                      {
                          BR_R.append("990");
                          BR_R.append(s);
                          BR_R.append("j");
                      }
                      else if(BR_angle < 10)
                      {
                          BR_R.append("9900");
                          BR_R.append(s);
                          BR_R.append("j");
                      }

                      ROS_INFO("%s",BR_R.c_str());
                      if(BR_angle!=360&&BR_angle != 0)
                      {
                          ros::Duration(0.01).sleep(); my_serial.write(BR_R);

                      }
                old_BL = BL_angle;

                BL_angle = BL_angle +step_move;

                ROS_INFO("OLD %d, NEW %d", old_BL,BL_angle);

                if(BL_angle == 0)
                {
                    BL_L = "Er";
                }
                else if(BL_angle - old_BL > 0)
                {
                    BL_L = "Er";

                }
                else if(BL_angle - old_BL < 0)
                {
                 BL_L = "El";
                }
                else if(BL_angle - old_BL == 0)
                {
                    BL_L = "El";
                }
                if(BL_angle >= 360)
                {
                  BL_angle = 1;
                   BL_L = "Er";
                }
                       s = boost::lexical_cast<string>( BL_angle );
                      if(BL_angle >=100)
                      {
                      BL_L.append("99");
                      BL_L.append(s);
                      BL_L.append("j");
                        }
                      else if(BL_angle < 100&& BL_angle >= 10 )
                      {
                          BL_L.append("990");
                          BL_L.append(s);
                          BL_L.append("j");
                      }
                      else if(BL_angle < 10)
                      {
                          BL_L.append("9900");
                          BL_L.append(s);
                          BL_L.append("j");
                      }
                      ROS_INFO("%s",BL_L.c_str());
                      if(BL_angle!=360&&BL_angle != 0)
                      {
                          ros::Duration(0.01).sleep(); my_serial.write(BL_L);

                      }
                      old_FR = FR_angle;
                              FR_angle = FR_angle + step_move;






                              if(FR_angle == 0)
                              {
                                  FL_R = "Dr";
                              }
                             else if(FR_angle - old_FR > 0)
                              {
                                  FL_R = "Dr";
                                 //   ROS_INFO("UPPPP");

                              }
                              else if(FR_angle - old_FR < 0)
                              {
                               FL_R = "Dl";
                               // ROS_INFO("downnnnn");
                              }
                              else if(FR_angle - old_FR == 0)
                              {
                                   //  ROS_INFO("000000000000");
                                  FL_R = "Dl";
                              }
                              if(FR_angle >= 360)
                              {
                                FR_angle = 1;
                                 FL_R = "Dr";
                              }
                                     s = boost::lexical_cast<string>( FR_angle );
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
                                    ROS_INFO("%s UP",FL_R.c_str());
                                    if(FR_angle!=360&&FR_angle != 0)
                                    {
                                        ros::Duration(0.01).sleep(); my_serial.write(FL_R);

                                    }
                              old_FL = FL_angle;



                             FL_angle = FL_angle + step_move;
                             ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);


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
                      ROS_INFO("%s UP",FL_L.c_str());
                                       //}
                                       //else if(en_FL==old_FL)
                                       //{
                                        //   ros::Duration(0.01).sleep(); my_serial.write(FL_L.c_str(),8);
                                       //}
                                   }
    }
    else if(msg->data == "BLup")
    {

        old_BL = BL_angle;

        BL_angle = BL_angle +step_move;

        ROS_INFO("OLD %d, NEW %d", old_BL,BL_angle);

        if(BL_angle == 0)
        {
            BL_L = "Er";
        }
        else if(BL_angle - old_BL > 0)
        {
            BL_L = "Er";

        }
        else if(BL_angle - old_BL < 0)
        {
         BL_L = "El";
        }
        else if(BL_angle - old_BL == 0)
        {
            BL_L = "El";
        }
        if(BL_angle >= 360)
        {
          BL_angle = 1;
           BL_L = "Er";
        }
              string s = boost::lexical_cast<string>( BL_angle );
              if(BL_angle >=100)
              {
              BL_L.append("99");
              BL_L.append(s);
              BL_L.append("j");
                }
              else if(BL_angle < 100&& BL_angle >= 10 )
              {
                  BL_L.append("990");
                  BL_L.append(s);
                  BL_L.append("j");
              }
              else if(BL_angle < 10)
              {
                  BL_L.append("9900");
                  BL_L.append(s);
                  BL_L.append("j");
              }
              ROS_INFO("%s",BL_L.c_str());
              if(BL_angle!=360&&BL_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(BL_L);

              }

    }
    else if(msg->data == "BLdown")
    {

         old_BL = BL_angle;
         BL_angle = BL_angle - step_move;
          ROS_INFO("OLD %d, NEW %d", old_BL,BL_angle);

         if(BL_angle == 360)
         {
             BL_L = "El";
         }
       else if(BL_angle - old_BL > 0)
         {
             BL_L = "Er";

         }
         else if(BL_angle - old_BL < 0)
         {
          BL_L = "El";
         }
         else if(BL_angle - old_BL == 0)
         {
             BL_L = "El";
         }
         if(BL_angle <= 0)
         {
           BL_angle = 359+BL_angle;
           BL_L="El";
         }
              string s = boost::lexical_cast<string>( BL_angle );
              if(BL_angle >=100)
              {
              BL_L.append("99");
              BL_L.append(s);
              BL_L.append("j");
                }
              else if(BL_angle < 100 && BL_angle >= 10)
              {
                  BL_L.append("990");
                  BL_L.append(s);
                  BL_L.append("j");
              }
              else if(BL_angle < 10)
              {
                  BL_L.append("9900");
                  BL_L.append(s);
                  BL_L.append("j");
              }
              ROS_INFO("%s",BL_L.c_str());
              if(BL_angle!=360&&BL_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(BL_L);

              }
    }
    else if(msg->data == "BRup")
    {

        old_BR = BR_angle;
        BR_angle = BR_angle +step_move;


        if(BR_angle == 0)
        {
            BR_R = "Fr";
        }
        else if(BR_angle - old_BR > 0)
        {
            BR_R = "Fr";

        }
        else if(BR_angle - old_BR < 0)
        {
         BR_R = "Fl";
        }
        else if(BR_angle - old_BR == 0)
        {
            BR_R = "Fl";
        }
        if(BR_angle >= 360)
        {
          BR_angle = 1;
           BR_R = "Fr";
        }
              string s = boost::lexical_cast<string>( BR_angle );
              if(BR_angle >=100)
              {
              BR_R.append("99");
              BR_R.append(s);
              BR_R.append("j");
                }
              else if(BR_angle < 100 && BR_angle >= 10 )
              {
                  BR_R.append("990");
                  BR_R.append(s);
                  BR_R.append("j");
              }
              else if(BR_angle < 10)
              {
                  BR_R.append("9900");
                  BR_R.append(s);
                  BR_R.append("j");
              }

              ROS_INFO("%s",BR_R.c_str());
              if(BR_angle!=360&&BR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(BR_R);

              }

    }
    else if(msg->data == "BRdown")
    {

        old_BR = BR_angle;
        BR_angle = BR_angle -step_move;

        if(BR_angle == 360)
        {
            BR_R = "Fl";
        }
        else if(BR_angle - old_BR > 0)
        {
            BR_R = "Fr";

        }
        else if(BR_angle - old_BR < 0)
        {
         BR_R = "Fl";
        }
        else if(BR_angle - old_BR == 0)
        {
            BR_R = "Fl";
        }
        if(BR_angle <= 0)
        {
          BR_angle = 359+BR_angle;
          BR_R="Fl";
        }
              string s = boost::lexical_cast<string>( BR_angle );
              if(BR_angle >=100)
              {
              BR_R.append("99");
              BR_R.append(s);
              BR_R.append("j");
                }
              else if(BR_angle < 100 && BR_angle >= 10)
              {
                  BR_R.append("990");
                  BR_R.append(s);
                  BR_R.append("j");
              }
              else if(BR_angle < 10)
              {
                  BR_R.append("9900");
                  BR_R.append(s);
                  BR_R.append("j");
              }
              ROS_INFO("%s",BR_R.c_str());
              if(BR_angle!=360&&BR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); my_serial.write(BR_R);

              }
    }

    //-------------Special Posture ---------------//
    else if(msg->data == "down45down0")
    {
         if(FL_angle < 135)
        {
             FL_L = "Cr99135j";
             ROS_INFO("cUP %s",FL_L.c_str());
             my_serial.write(FL_L);
             FL_angle = 135;
        }
        else if((FL_angle-135) > (360-FL_angle)+135 )
        {
            FL_L = "Cr99135j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 135;

        }
        else if((FL_angle)-135 <(360-FL_angle)+135)
        {
            FL_L = "Cl99135j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 135;
        }

         ros::Duration(0.01).sleep();
         if(FR_angle < 145)
        {
             FL_R = "Dr99145j";
             ROS_INFO("cUP %s",FL_R.c_str());
             my_serial.write(FL_R);
             FR_angle = 145;
        }
        else if((FR_angle-145) > (360-FR_angle)+145 )
        {
            FL_R = "Dr99145j";
            ROS_INFO("cUP %s",FL_R.c_str());
            my_serial.write(FL_R);
            FR_angle = 145;

        }
        else if((FR_angle)-145 <(360-FR_angle)+145)
        {
            FL_R = "Dl99145j";
            ROS_INFO("cUP %s",FL_R.c_str());
            my_serial.write(FL_R);
            FR_angle = 145;
        }
         ros::Duration(0.01).sleep();
         if(BL_angle < 70)
        {
             BL_L = "Er99070j";
             ROS_INFO("cUP %s",BL_L.c_str());
             my_serial.write(BL_L);
             BL_angle = 70;
        }
        else if((BL_angle-70) > (360-BL_angle)+70 )
        {
            BL_L = "Er99070j";
            ROS_INFO("cUP %s",BL_L.c_str());
            my_serial.write(BL_L);
            BL_angle = 70;

        }
        else if((BL_angle)-70 <(360-BL_angle)+70)
        {
            BL_L = "El99070j";
            ROS_INFO("cUP %s",BL_L.c_str());
            my_serial.write(BL_L);
            BL_angle = 70;
        }

         ros::Duration(0.01).sleep();

         if(BR_angle < 80)
        {
             BR_R = "Fr99080j";
             ROS_INFO("cUP %s",BR_R.c_str());
             my_serial.write(BR_R);
             BR_angle = 80;
        }
        else if((BR_angle-80) > (360-BR_angle)+80 )
        {
            BR_R = "Fr99080j";
            ROS_INFO("cUP %s",BR_R.c_str());
            my_serial.write(BR_R);
            BR_angle = 80;

        }
        else if((BR_angle)-80 <(360-BR_angle)+80)
        {
            BR_R = "Fl99080j";
            ROS_INFO("cUP %s",BR_R.c_str());
            my_serial.write(BR_R);
            BR_angle = 80;
        }

    }

    else if(msg->data == "down45down45")
    {
         if(FL_angle < 135)
        {
             FL_L = "Cr99135j";
             ROS_INFO("cUP %s",FL_L.c_str());
             my_serial.write(FL_L);
             FL_angle = 135;
        }
        else if((FL_angle-135) > (360-FL_angle)+135 )
        {
            FL_L = "Cr99135j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 135;

        }
        else if((FL_angle)-135 <(360-FL_angle)+135)
        {
            FL_L = "Cl99135j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 135;
        }

         ros::Duration(0.01).sleep();
         if(FR_angle < 145)
        {
             FL_R = "Dr99145j";
             ROS_INFO("cUP %s",FL_R.c_str());
             my_serial.write(FL_R);
             FR_angle = 145;
        }
        else if((FR_angle-145) > (360-FR_angle)+145 )
        {
            FL_R = "Dr99145j";
            ROS_INFO("cUP %s",FL_R.c_str());
            my_serial.write(FL_R);
            FR_angle = 145;

        }
        else if((FR_angle)-145 <(360-FR_angle)+145)
        {
            FL_R = "Dl99145j";
            ROS_INFO("cUP %s",FL_R.c_str());
            my_serial.write(FL_R);
            FR_angle = 145;
        }
         ros::Duration(0.01).sleep();
         if(BL_angle < 115)
        {
             BL_L = "Er99115j";
             ROS_INFO("cUP %s",BL_L.c_str());
             my_serial.write(BL_L);
             BL_angle = 115;
        }
        else if((BL_angle-115) > (360-BL_angle)+115 )
        {
            BL_L = "Er99115j";
            ROS_INFO("cUP %s",BL_L.c_str());
            my_serial.write(BL_L);
            BL_angle = 115;

        }
        else if((BL_angle)-115 <(360-BL_angle)+115)
        {
            BL_L = "El99115j";
            ROS_INFO("cUP %s",BL_L.c_str());
            my_serial.write(BL_L);
            BL_angle = 115;
        }

         ros::Duration(0.01).sleep();

         if(BR_angle < 125)
        {
             BR_R = "Fr99125j";
             ROS_INFO("cUP %s",BR_R.c_str());
             my_serial.write(BR_R);
             BR_angle = 125;
        }
        else if((BR_angle-125) > (360-BR_angle)+125 )
        {
            BR_R = "Fr99125j";
            ROS_INFO("cUP %s",BR_R.c_str());
            my_serial.write(BR_R);
            BR_angle = 125;

        }
        else if((BR_angle)-125 <(360-BR_angle)+125)
        {
            BR_R = "Fl99125j";
            ROS_INFO("cUP %s",BR_R.c_str());
            my_serial.write(BR_R);
            BR_angle = 125;
        }

    }
    else if(msg->data == "up0up45")
    {
        if(FL_angle < 90)
       {
            FL_L = "Cr99090j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 90;
       }
       else if((FL_angle-90) > (360-FL_angle)+90 )
       {
           FL_L = "Cr99090j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 90;

       }
       else if((FL_angle)-90 <(360-FL_angle)+90)
       {
           FL_L = "Cl99090j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 90;
       }

             ros::Duration(0.01).sleep();
        if(FR_angle < 95)
       {
            FL_R = "Dr99095j";
            ROS_INFO("cUP %s",FL_R.c_str());
            my_serial.write(FL_R);
            FR_angle = 95;
       }
       else if((FR_angle-95) > (360-FR_angle)+95 )
       {
           FL_R = "Dr99095j";
           ROS_INFO("cUP %s",FL_R.c_str());
           my_serial.write(FL_R);
           FR_angle = 95;

       }
       else if((FR_angle)-95 <(360-FR_angle)+95)
       {
           FL_R = "Dl99095j";
           ROS_INFO("cUP %s",FL_R.c_str());
           my_serial.write(FL_R);
           FR_angle = 95;
       }
          ros::Duration(0.01).sleep();
         if(BL_angle < 40)
        {
             BL_L = "Er99040j";
             ROS_INFO("cUP %s",BL_L.c_str());
             my_serial.write(BL_L);
             BL_angle = 40;
        }
        else if((BL_angle-40) > (360-BL_angle)+40 )
        {
            BL_L = "Er99040j";
            ROS_INFO("cUP %s",BL_L.c_str());
            my_serial.write(BL_L);
            BL_angle = 40;

        }
        else if((BL_angle)-40 <(360-BL_angle)+40)
        {
            BL_L = "El99040j";
            ROS_INFO("cUP %s",BL_L.c_str());
            my_serial.write(BL_L);
            BL_angle = 40;
        }

         ros::Duration(0.01).sleep();

         if(BR_angle < 45)
        {
             BR_R = "Fr99045j";
             ROS_INFO("cUP %s",BR_R.c_str());
             my_serial.write(BR_R);
             BR_angle = 45;
        }
        else if((BR_angle-45) > (360-BR_angle)+45 )
        {
            BR_R = "Fr99045j";
            ROS_INFO("cUP %s",BR_R.c_str());
            my_serial.write(BR_R);
            BR_angle = 45;

        }
        else if((BR_angle)-45 <(360-BR_angle)+45)
        {
            BR_R = "Fl99045j";
            ROS_INFO("cUP %s",BR_R.c_str());
            my_serial.write(BR_R);
            BR_angle = 45;
        }

    }
    else if(msg->data == "up0down45")
    {
        if(FL_angle < 90)
       {
            FL_L = "Cr99090j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 90;
       }
       else if((FL_angle-90) > (360-FL_angle)+90 )
       {
           FL_L = "Cr99090j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 90;

       }
       else if((FL_angle)-90 <(360-FL_angle)+90)
       {
           FL_L = "Cl99090j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 90;
       }

        ros::Duration(0.01).sleep();
   if(FR_angle < 95)
  {
       FL_R = "Dr99095j";
       ROS_INFO("cUP %s",FL_R.c_str());
       my_serial.write(FL_R);
       FR_angle = 95;
  }
  else if((FR_angle-95) > (360-FR_angle)+95 )
  {
      FL_R = "Dr99095j";
      ROS_INFO("cUP %s",FL_R.c_str());
      my_serial.write(FL_R);
      FR_angle = 95;

  }
  else if((FR_angle)-95 <(360-FR_angle)+95)
  {
      FL_R = "Dl99095j";
      ROS_INFO("cUP %s",FL_R.c_str());
      my_serial.write(FL_R);
      FR_angle = 95;
  }
   ros::Duration(0.01).sleep();
   if(BL_angle < 115)
  {
       BL_L = "Er99115j";
       ROS_INFO("cUP %s",BL_L.c_str());
       my_serial.write(BL_L);
       BL_angle = 115;
  }
  else if((BL_angle-115) > (360-BL_angle)+115 )
  {
      BL_L = "Er99115j";
      ROS_INFO("cUP %s",BL_L.c_str());
      my_serial.write(BL_L);
      BL_angle = 115;

  }
  else if((BL_angle)-115 <(360-BL_angle)+115)
  {
      BL_L = "El99115j";
      ROS_INFO("cUP %s",BL_L.c_str());
      my_serial.write(BL_L);
      BL_angle = 115;
  }

   ros::Duration(0.01).sleep();

   if(BR_angle < 125)
  {
       BR_R = "Fr99125j";
       ROS_INFO("cUP %s",BR_R.c_str());
       my_serial.write(BR_R);
       BR_angle = 125;
  }
  else if((BR_angle-125) > (360-BR_angle)+125 )
  {
      BR_R = "Fr99125j";
      ROS_INFO("cUP %s",BR_R.c_str());
      my_serial.write(BR_R);
      BR_angle = 125;

  }
  else if((BR_angle)-125 <(360-BR_angle)+125)
  {
      BR_R = "Fl99125j";
      ROS_INFO("cUP %s",BR_R.c_str());
      my_serial.write(BR_R);
      BR_angle = 125;
  }

    }
    else if(msg->data == "cup")
    {
         if(FL_angle < 40)
        {
             FL_L = "Cr99040j";
             ROS_INFO("cUP %s",FL_L.c_str());
             my_serial.write(FL_L);
             FL_angle = 40;
        }
        else if((FL_angle-40) > (360-FL_angle)+40 )
        {
            FL_L = "Cr99040j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 40;

        }
        else if((FL_angle)-40 <(360-FL_angle)+40)
        {
            FL_L = "Cl99040j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 40;
        }

         ros::Duration(0.01).sleep();
         if(FR_angle < 50)
        {
             FL_R = "Dr99050j";
             ROS_INFO("cUP %s",FL_R.c_str());
             my_serial.write(FL_R);
             FR_angle = 50;
        }
        else if((FR_angle-50) > (360-FR_angle)+50 )
        {
            FL_R = "Dr99050j";
            ROS_INFO("cUP %s",FL_R.c_str());
            my_serial.write(FL_R);
            FR_angle = 45;

        }
        else if((FR_angle)-50 <(360-FR_angle)+50)
        {
            FL_R = "Dl99050j";
            ROS_INFO("cUP %s",FL_R.c_str());
            my_serial.write(FL_R);
            FR_angle = 50;
        }
         ros::Duration(0.01).sleep();
        if(BL_angle < 40)
       {
            BL_L = "Er99040j";
            ROS_INFO("cUP %s",BL_L.c_str());
            my_serial.write(BL_L);
            BL_angle = 40;
       }
       else if((BL_angle-40) > (360-BL_angle)+40 )
       {
           BL_L = "Er99040j";
           ROS_INFO("cUP %s",BL_L.c_str());
           my_serial.write(BL_L);
           BL_angle = 40;

       }
       else if((BL_angle)-40 <(360-BL_angle)+40)
       {
           BL_L = "El99040j";
           ROS_INFO("cUP %s",BL_L.c_str());
           my_serial.write(BL_L);
           BL_angle = 40;
       }

        ros::Duration(0.01).sleep();

        if(BR_angle < 45)
       {
            BR_R = "Fr99045j";
            ROS_INFO("cUP %s",BR_R.c_str());
            my_serial.write(BR_R);
            BR_angle = 45;
       }
       else if((BR_angle-45) > (360-BR_angle)+45 )
       {
           BR_R = "Fr99045j";
           ROS_INFO("cUP %s",BR_R.c_str());
           my_serial.write(BR_R);
           BR_angle = 45;

       }
       else if((BR_angle)-45 <(360-BR_angle)+45)
       {
           BR_R = "Fl99045j";
           ROS_INFO("cUP %s",BR_R.c_str());
           my_serial.write(BR_R);
           BR_angle = 45;
       }

    }
         else if(msg->data == "superman")
         {
        if(FL_angle < 90)
       {
            FL_L = "Cr99090j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 90;
       }
       else if((FL_angle-90) > (360-FL_angle)+90 )
       {
           FL_L = "Cr99090j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 90;

       }
       else if((FL_angle)-90 <(360-FL_angle)+90)
       {
           FL_L = "Cl99090j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 90;
       }

        ros::Duration(0.01).sleep();
   if(FR_angle < 95)
  {
       FL_R = "Dr99095j";
       ROS_INFO("cUP %s",FL_R.c_str());
       my_serial.write(FL_R);
       FR_angle = 95;
  }
  else if((FR_angle-95) > (360-FR_angle)+95 )
  {
      FL_R = "Dr99095j";
      ROS_INFO("cUP %s",FL_R.c_str());
      my_serial.write(FL_R);
      FR_angle = 95;

  }
  else if((FR_angle)-95 <(360-FR_angle)+95)
  {
      FL_R = "Dl99095j";
      ROS_INFO("cUP %s",FL_R.c_str());
      my_serial.write(FL_R);
      FR_angle = 95;
  }
   ros::Duration(0.01).sleep();
   if(BL_angle < 70)
  {
       BL_L = "Er99070j";
       ROS_INFO("cUP %s",BL_L.c_str());
       my_serial.write(BL_L);
       BL_angle = 70;
  }
  else if((BL_angle-70) > (360-BL_angle)+70 )
  {
      BL_L = "Er99070j";
      ROS_INFO("cUP %s",BL_L.c_str());
      my_serial.write(BL_L);
      BL_angle = 70;

  }
  else if((BL_angle)-70 <(360-BL_angle)+70)
  {
      BL_L = "El99070j";
      ROS_INFO("cUP %s",BL_L.c_str());
      my_serial.write(BL_L);
      BL_angle = 70;
  }

   ros::Duration(0.01).sleep();

   if(BR_angle < 80)
  {
       BR_R = "Fr99080j";
       ROS_INFO("cUP %s",BR_R.c_str());
       my_serial.write(BR_R);
       BR_angle = 80;
  }
  else if((BR_angle-80) > (360-BR_angle)+80 )
  {
      BR_R = "Fr99080j";
      ROS_INFO("cUP %s",BR_R.c_str());
      my_serial.write(BR_R);
      BR_angle = 80;

  }
  else if((BR_angle)-80 <(360-BR_angle)+80)
  {
      BR_R = "Fl99080j";
      ROS_INFO("cUP %s",BR_R.c_str());
      my_serial.write(BR_R);
      BR_angle = 80;
  }
    }
    else if(msg->data == "up45down45")
    {
        if(FL_angle < 40)
       {
            FL_L = "Cr99040j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 40;
       }
       else if((FL_angle-40) > (360-FL_angle)+40 )
       {
           FL_L = "Cr99040j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 40;

       }
       else if((FL_angle)-40 <(360-FL_angle)+40)
       {
           FL_L = "Cl99040j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 40;
       }

        ros::Duration(0.01).sleep();
        if(FR_angle < 50)
       {
            FL_R = "Dr99050j";
            ROS_INFO("cUP %s",FL_R.c_str());
            my_serial.write(FL_R);
            FR_angle = 50;
       }
       else if((FR_angle-50) > (360-FR_angle)+50 )
       {
           FL_R = "Dr99050j";
           ROS_INFO("cUP %s",FL_R.c_str());
           my_serial.write(FL_R);
           FR_angle = 45;

       }
       else if((FR_angle)-50 <(360-FR_angle)+50)
       {
           FL_R = "Dl99050j";
           ROS_INFO("cUP %s",FL_R.c_str());
           my_serial.write(FL_R);
           FR_angle = 50;
       }

        ros::Duration(0.01).sleep();
        if(BL_angle < 115)
       {
            BL_L = "Er99115j";
            ROS_INFO("cUP %s",BL_L.c_str());
            my_serial.write(BL_L);
            BL_angle = 115;
       }
       else if((BL_angle-115) > (360-BL_angle)+115 )
       {
           BL_L = "Er99115j";
           ROS_INFO("cUP %s",BL_L.c_str());
           my_serial.write(BL_L);
           BL_angle = 115;

       }
       else if((BL_angle)-115 <(360-BL_angle)+115)
       {
           BL_L = "El99115j";
           ROS_INFO("cUP %s",BL_L.c_str());
           my_serial.write(BL_L);
           BL_angle = 115;
       }

        ros::Duration(0.01).sleep();

        if(BR_angle < 125)
       {
            BR_R = "Fr99125j";
            ROS_INFO("cUP %s",BR_R.c_str());
            my_serial.write(BR_R);
            BR_angle = 125;
       }
       else if((BR_angle-125) > (360-BR_angle)+125 )
       {
           BR_R = "Fr99125j";
           ROS_INFO("cUP %s",BR_R.c_str());
           my_serial.write(BR_R);
           BR_angle = 125;

       }
       else if((BR_angle)-125 <(360-BR_angle)+125)
       {
           BR_R = "Fl99125j";
           ROS_INFO("cUP %s",BR_R.c_str());
           my_serial.write(BR_R);
           BR_angle = 125;
       }
    }
    else if(msg->data == "up45down0")
    {
        if(FL_angle < 40)
       {
            FL_L = "Cr99040j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 40;
       }
       else if((FL_angle-40) > (360-FL_angle)+40 )
       {
           FL_L = "Cr99040j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 40;

       }
       else if((FL_angle)-40 <(360-FL_angle)+40)
       {
           FL_L = "Cl99040j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 40;
       }

        ros::Duration(0.01).sleep();
        if(FR_angle < 50)
       {
            FL_R = "Dr99050j";
            ROS_INFO("cUP %s",FL_R.c_str());
            my_serial.write(FL_R);
            FR_angle = 50;
       }
       else if((FR_angle-50) > (360-FR_angle)+50 )
       {
           FL_R = "Dr99050j";
           ROS_INFO("cUP %s",FL_R.c_str());
           my_serial.write(FL_R);
           FR_angle = 45;

       }
       else if((FR_angle)-50 <(360-FR_angle)+50)
       {
           FL_R = "Dl99050j";
           ROS_INFO("cUP %s",FL_R.c_str());
           my_serial.write(FL_R);
           FR_angle = 50;
       }

          ros::Duration(0.01).sleep();
          if(BL_angle < 70)
         {
              BL_L = "Er99070j";
              ROS_INFO("cUP %s",BL_L.c_str());
              my_serial.write(BL_L);
              BL_angle = 70;
         }
         else if((BL_angle-70) > (360-BL_angle)+70 )
         {
             BL_L = "Er99070j";
             ROS_INFO("cUP %s",BL_L.c_str());
             my_serial.write(BL_L);
             BL_angle = 70;

         }
         else if((BL_angle)-70 <(360-BL_angle)+70)
         {
             BL_L = "El99070j";
             ROS_INFO("cUP %s",BL_L.c_str());
             my_serial.write(BL_L);
             BL_angle = 70;
         }

          ros::Duration(0.01).sleep();

          if(BR_angle < 80)
         {
              BR_R = "Fr99080j";
              ROS_INFO("cUP %s",BR_R.c_str());
              my_serial.write(BR_R);
              BR_angle = 80;
         }
         else if((BR_angle-80) > (360-BR_angle)+80 )
         {
             BR_R = "Fr99080j";
             ROS_INFO("cUP %s",BR_R.c_str());
             my_serial.write(BR_R);
             BR_angle = 80;

         }
         else if((BR_angle)-80 <(360-BR_angle)+80)
         {
             BR_R = "Fl99080j";
             ROS_INFO("cUP %s",BR_R.c_str());
             my_serial.write(BR_R);
             BR_angle = 80;
         }
    }
    else if(msg->data == "leghome")
    {
        if(FL_angle < 0)
       {
            FL_L = "Cr99000j";
            ROS_INFO("cUP %s",FL_L.c_str());
            my_serial.write(FL_L);
            FL_angle = 0;
       }
       else if((FL_angle-0) > (360-FL_angle)+0 )
       {
           FL_L = "Cr99000j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 0;

       }
       else if((FL_angle)-0 <(360-FL_angle)+0)
       {
           FL_L = "Cl99000j";
           ROS_INFO("cUP %s",FL_L.c_str());
           my_serial.write(FL_L);
           FL_angle = 0;
       }

          ros::Duration(0.01).sleep();
        if(FR_angle < 0)
       {
            FL_R = "Dr99000j";
            ROS_INFO("cUP %s",FL_R.c_str());
            my_serial.write(FL_R);
            FR_angle = 0;
       }
       else if((FR_angle-0) > (360-FR_angle)+0 )
       {
           FL_R = "Dr99000j";
           ROS_INFO("cUP %s",FL_R.c_str());
           my_serial.write(FL_R);
           FR_angle = 0;

       }
       else if((FR_angle)-0 <(360-FR_angle)+0)
       {
           FL_R = "Dl99000j";
           ROS_INFO("cUP %s",FL_R.c_str());
           my_serial.write(FL_R);
           FR_angle = 0;
       }

          ros::Duration(0.01).sleep();
        if(BL_angle < 0)
       {
            BL_L = "Er99000j";
            ROS_INFO("cUP %s",BL_L.c_str());
            my_serial.write(BL_L);
            BL_angle = 0;
       }
       else if((BL_angle-0) > (360-BL_angle)+0 )
       {
           BL_L = "Er99000j";
           ROS_INFO("cUP %s",BL_L.c_str());
           my_serial.write(BL_L);
           BL_angle = 0;

       }
       else if((BL_angle)-0 <(360-BL_angle)+0)
       {
           BL_L = "El99000j";
           ROS_INFO("cUP %s",BL_L.c_str());
           my_serial.write(BL_L);
           BL_angle = 0;
       }

          ros::Duration(0.01).sleep();
        if(BR_angle < 0)
       {
            BR_R = "Fr99000j";
            ROS_INFO("cUP %s",BR_R.c_str());
            my_serial.write(BR_R);
            BR_angle = 0;
       }
       else if((BR_angle-0) > (360-BR_angle)+0 )
       {
           BR_R = "Fr99000j";
           ROS_INFO("cUP %s",BR_R.c_str());
           my_serial.write(BR_R);
           BR_angle = 0;

       }
       else if((BR_angle)-0 <(360-BR_angle)+0)
       {
           BR_R = "Fl99000j";
           ROS_INFO("cUP %s",BR_R.c_str());
           my_serial.write(BR_R);
           BR_angle = 0;
       }
    }
    //-------------------Manipulator posture------------//
    else if(msg->data == "j1up")
    {
        j1_angle = j1_angle+step_mani;
        if(j1_angle > j1_max)
        {
            j1_angle = j1_max;
        }
        j_1 = "Gl";
        string s = boost::lexical_cast<string>( j1_angle );
        if(j1_angle >=100)
        {
        j_1.append("80");
        j_1.append(s);
        j_1.append("j");
          }
        else if(j1_angle < 100&& j1_angle >= 10 )
        {
            j_1.append("800");
            j_1.append(s);
            j_1.append("j");
        }
        else if(j1_angle < 10)
        {
            j_1.append("8000");
            j_1.append(s);
            j_1.append("j");
        }
        ROS_INFO("%s",j_1.c_str());
        if(j1_angle!=360)
        {
            ros::Duration(0.01).sleep(); my_serial.write(j_1);

        }
    }
    else if(msg->data == "j1down")
    {
        j1_angle = j1_angle-step_mani;
        if(j1_angle < j1_min)
        {
            j1_angle = j1_min;
        }
        j_1 = "Gl";
        string s = boost::lexical_cast<string>( j1_angle );
        if(j1_angle >=100)
        {
        j_1.append("80");
        j_1.append(s);
        j_1.append("j");
          }
        else if(j1_angle < 100&& j1_angle >= 10 )
        {
            j_1.append("800");
            j_1.append(s);
            j_1.append("j");
        }
        else if(j1_angle < 10)
        {
            j_1.append("8000");
            j_1.append(s);
            j_1.append("j");
        }
        ROS_INFO("%s",j_1.c_str());
        if(j1_angle!=360)
        {
            ros::Duration(0.01).sleep(); my_serial.write(j_1);

        }
    }
    else if(msg->data == "j2up")
    {
        j2_angle = j2_angle+step_mani;
        if(j2_angle > j2_max)
        {
            j2_angle = j2_max;
        }
        j_2 = "Hl";
        string s = boost::lexical_cast<string>( j2_angle );
        if(j2_angle >=100)
        {
        j_2.append("90");
        j_2.append(s);
        j_2.append("j");
          }
        else if(j2_angle < 100&& j2_angle >= 10 )
        {
            j_2.append("900");
            j_2.append(s);
            j_2.append("j");
        }
        else if(j2_angle < 10)
        {
            j_2.append("9000");
            j_2.append(s);
            j_2.append("j");
        }
        ROS_INFO("%s",j_2.c_str());
        if(j2_angle!=360)
        {
            ros::Duration(0.01).sleep(); my_serial.write(j_2);

        }
    }
    else if(msg->data == "j2down")
    {
        j2_angle = j2_angle-step_mani;
        if(j2_angle < j2_min)
        {
            j2_angle = j2_min;
        }
        j_2 = "Hl";
        string s = boost::lexical_cast<string>( j2_angle );
        if(j2_angle >=100)
        {
        j_2.append("90");
        j_2.append(s);
        j_2.append("j");
          }
        else if(j2_angle < 100&& j2_angle >= 10 )
        {
            j_2.append("900");
            j_2.append(s);
            j_2.append("j");
        }
        else if(j2_angle < 10)
        {
            j_2.append("9000");
            j_2.append(s);
            j_2.append("j");
        }
        ROS_INFO("%s",j_2.c_str());
        if(j2_angle!=360)
        {
            ros::Duration(0.01).sleep(); my_serial.write(j_2);

        }
    }
    else if(msg->data == "j3up")
    {
        j3_angle = j3_angle+step_mani;
        if(j3_angle > j3_max)
        {
            j3_angle = j3_max;
        }
        j_3 = "Il";
        string s = boost::lexical_cast<string>( j3_angle );
        if(j3_angle >=100)
        {
        j_3.append("90");
        j_3.append(s);
        j_3.append("j");
          }
        else if(j3_angle < 100&& j3_angle >= 10 )
        {
            j_3.append("900");
            j_3.append(s);
            j_3.append("j");
        }
        else if(j3_angle < 10)
        {
            j_3.append("9000");
            j_3.append(s);
            j_3.append("j");
        }
        ROS_INFO("%s",j_3.c_str());
        if(j3_angle!=360)
        {
            ros::Duration(0.01).sleep(); my_serial.write(j_3);

        }
    }
    else if(msg->data == "j3down")
    {

        j3_angle = j3_angle-step_mani;
        if(j3_angle < j3_min)
        {
            j3_angle = j3_min;
        }
        j_3 = "Il";
        string s = boost::lexical_cast<string>( j3_angle );
        if(j3_angle >=100)
        {
        j_3.append("90");
        j_3.append(s);
        j_3.append("j");
          }
        else if(j3_angle < 100&& j3_angle >= 10 )
        {
            j_3.append("900");
            j_3.append(s);
            j_3.append("j");
        }
        else if(j3_angle < 10)
        {
            j_3.append("9000");
            j_3.append(s);
            j_3.append("j");
        }
        ROS_INFO("%s",j_3.c_str());
        if(j3_angle!=360)
        {
            ros::Duration(0.01).sleep(); my_serial.write(j_3);

        }
    }
    else if(msg->data == "j4up")
    {
        j4_angle = j4_angle+step_mani;
        if(j4_angle > j4_max)
        {
            j4_angle = j4_max;
        }
        j_4 = "Jl";
        string s = boost::lexical_cast<string>( j4_angle );
        if(j4_angle >=100)
        {
        j_4.append("90");
        j_4.append(s);
        j_4.append("j");
          }
        else if(j4_angle < 100&& j4_angle >= 10 )
        {
            j_4.append("900");
            j_4.append(s);
            j_4.append("j");
        }
        else if(j4_angle < 10)
        {
            j_4.append("9000");
            j_4.append(s);
            j_4.append("j");
        }
        ROS_INFO("%s",j_4.c_str());
        if(j4_angle!=360)
        {
            ros::Duration(0.01).sleep(); my_serial.write(j_4);

        }
    }
    else if(msg->data == "j4down")
    {
        j4_angle = j4_angle-step_mani;
        if(j4_angle <j4_min)
        {
            j4_angle = j4_min;
        }
        j_4 = "Jl";
        string s = boost::lexical_cast<string>( j4_angle );
        if(j4_angle >=100)
        {
        j_4.append("90");
        j_4.append(s);
        j_4.append("j");
          }
        else if(j4_angle < 100&& j4_angle >= 10 )
        {
            j_4.append("900");
            j_4.append(s);
            j_4.append("j");
        }
        else if(j4_angle < 10)
        {
            j_4.append("9000");
            j_4.append(s);
            j_4.append("j");
        }
        ROS_INFO("%s",j_4.c_str());
        if(j4_angle!=360)
        {
            ros::Duration(0.01).sleep(); my_serial.write(j_4);

        }
    }
    else if(msg->data == "j5up")
    {
        j5_angle = j5_angle+step_wrist;
        if(j5_angle > j5_max)
        {
            j5_angle = j5_max;
        }
        j_5 = "Kl";
        string s = boost::lexical_cast<string>( j5_angle );
        if(j5_angle >=100)
        {
        j_5.append("55");
        j_5.append(s);
        j_5.append("j");
          }
        else if(j5_angle < 100&& j5_angle >= 10 )
        {
            j_5.append("550");
            j_5.append(s);
            j_5.append("j");
        }
        else if(j5_angle < 10)
        {
            j_5.append("5500");
            j_5.append(s);
            j_5.append("j");
        }
        ROS_INFO("%s",j_5.c_str());

            ros::Duration(0.03).sleep(); my_serial.write(j_5); ros::Duration(0.03).sleep();


    }
    else if(msg->data == "j5down")
    {
        j5_angle = j5_angle-step_wrist;
        if(j5_angle < j5_min)
        {
            j5_angle = j5_min;
        }
        j_5 = "Kl";
        string s = boost::lexical_cast<string>( j5_angle );
        if(j5_angle >=100)
        {
        j_5.append("55");
        j_5.append(s);
        j_5.append("j");
          }
        else if(j5_angle < 100&& j5_angle >= 10 )
        {
            j_5.append("550");
            j_5.append(s);
            j_5.append("j");
        }
        else if(j5_angle < 10)
        {
            j_5.append("5500");
            j_5.append(s);
            j_5.append("j");
        }
        ROS_INFO("%s",j_5.c_str());

           ros::Duration(0.03).sleep(); my_serial.write(j_5); ros::Duration(0.03).sleep();


    }
    else if(msg->data == "j6down")
    {
        j6_angle = j6_angle+step_wrist;
        if(j6_angle > j6_max)
        {
            j6_angle = j6_max;
        }
        j_6 = "Ll";
        string s = boost::lexical_cast<string>( j6_angle );
        if(j6_angle >=100)
        {
        j_6.append("50");
        j_6.append(s);
        j_6.append("j");
          }
        else if(j6_angle < 100&& j6_angle >= 10 )
        {
            j_6.append("500");
            j_6.append(s);
            j_6.append("j");
        }
        else if(j6_angle < 10)
        {
            j_6.append("5000");
            j_6.append(s);
            j_6.append("j");
        }
        ROS_INFO("%s",j_6.c_str());

            ros::Duration(0.03).sleep(); my_serial.write(j_6); ros::Duration(0.03).sleep();


    }
    else if(msg->data == "j6up")
    {
        j6_angle = j6_angle-step_wrist;
        if(j6_angle < j6_min)
        {
            j6_angle = j6_min;
        }
        j_6 = "Ll";
        string s = boost::lexical_cast<string>( j6_angle );
        if(j6_angle >=100)
        {
        j_6.append("50");
        j_6.append(s);
        j_6.append("j");
          }
        else if(j6_angle < 100&& j6_angle >= 10 )
        {
            j_6.append("500");
            j_6.append(s);
            j_6.append("j");
        }
        else if(j6_angle < 10)
        {
            j_6.append("5000");
            j_6.append(s);
            j_6.append("j");
        }
        ROS_INFO("%s",j_6.c_str());

            ros::Duration(0.03).sleep(); my_serial.write(j_6); ros::Duration(0.03).sleep();


    }
    else if(msg->data == "j7up")
    {
        j7_angle = j7_angle+step_wrist;
        if(j7_angle > 180)
        {
            j7_angle = 180;
        }
        j_7 = "Ml";
        string s = boost::lexical_cast<string>( j7_angle );
        if(j7_angle >=100)
        {
        j_7.append("50");
        j_7.append(s);
        j_7.append("j");
          }
        else if(j7_angle < 100&& j7_angle >= 10 )
        {
            j_7.append("500");
            j_7.append(s);
            j_7.append("j");
        }
        else if(j7_angle < 10)
        {
            j_7.append("5000");
            j_7.append(s);
            j_7.append("j");
        }
        ROS_INFO("%s",j_7.c_str());

            ros::Duration(0.01).sleep(); my_serial.write(j_7);


    }
    else if(msg->data == "j7down")
    {
        j7_angle = j7_angle-step_wrist;
        if(j7_angle < 0)
        {
            j7_angle = 0;
        }
        j_7 = "Ml";
        string s = boost::lexical_cast<string>( j7_angle );
        if(j7_angle >=100)
        {
        j_7.append("70");
        j_7.append(s);
        j_7.append("j");
          }
        else if(j7_angle < 100&& j7_angle >= 10 )
        {
            j_7.append("700");
            j_7.append(s);
            j_7.append("j");
        }
        else if(j7_angle < 10)
        {
            j_7.append("7000");
            j_7.append(s);
            j_7.append("j");
        }
        ROS_INFO("%s",j_7.c_str());

            ros::Duration(0.01).sleep(); my_serial.write(j_7);


    }
    //----------------Special Manipulator posture -------------//
    else if(msg->data == "manireset")
    {
        mani_pos(90,0,5,0);
    }
    else if(msg->data == "manimiddle")
    {
        mani_pos(90,45,90,0);
    }
    else if(msg->data == "manihigh")
    {
        mani_pos(90,60,130,0);
    }
    else if(msg->data == "manidoor")
    {
        mani_pos(110,70,120,0);
    }
    else if(msg->data == "maniup")
    {
        mani_pos(90,85,120,0);
    }
    else if(msg->data == "camreset")
    {
        j_5 =  "Kl99512j";
        ros::Duration(0.01).sleep(); my_serial.write(j_5);
        j_6 =  "Ll10415j";
        ros::Duration(0.01).sleep(); my_serial.write(j_6);
        j_6 =  "Ll10315j";
        ros::Duration(0.01).sleep(); my_serial.write(j_6);
        j5_angle = 512;
        j6_angle = 315;
    }
    else

    {
       my_serial.write(msg->data.c_str());
    }

}

//------------------subscribe value to this node------------//
void SpeedCallback(const std_msgs::Int16ConstPtr& msg)
{

speed = msg->data;
}
void legcallback(const ros::TimerEvent& event)  // timer of this node for read leg pos
{
    try         // if in "try" can do then do, but if can't do go to "catch"
    {

        std::string enleg;
        std::string enm;
    my_serial.write("?Aj");
    ros::Duration(0.005).sleep();
     my_serial.readline(enleg);
      //int result_io =  device.read(reply,18,1000);   // buffer,length,timeout
  //    ROS_INFO("%d",result_io);         // chk number of msgs
  // ROS_INFO("leg::::::%s",enleg.c_str());
       en_FL = strtod(enleg.c_str(),&A);        // 'strtod' change str to double
       en_FR = strtod(A,&A);
       en_BL = strtod(A,&A);
       en_BR = strtod(A,NULL);
    //   ROS_INFO("FL %d : FR %d : BL %d : BR %d",abs(FL_angle-en_FL),abs(FR_angle-en_FR),abs(BL_angle-en_BL),abs(BR_angle-en_BR));
     /*  if(abs(FL_angle-en_FL)<10||abs(FL_angle-en_FL)>355)
       {
           FL_angle = en_FL;
       }
       if(abs(FR_angle-en_FR)<10||abs(FR_angle-en_FR)>355)
       {
          FR_angle = en_FR;
       }
       if(abs(BR_angle-en_BR)<10||abs(BR_angle-en_BR)>355)
       {
           BR_angle = en_BR;
       }
       if(abs(BL_angle-en_BL)<10||abs(BL_angle-en_BL)>355)
       {
           BL_angle = en_BL;
       }*/
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

         ros::Duration(0.005).sleep();
      my_serial.write("?Mj");
      ros::Duration(0.005).sleep();
      my_serial.readline(enm);
        // result_io =  device.read(Mreply,18,1000);   // buffer,length,timeout
         //   ROS_INFO("%d",result_io);         // chk number of msgs
  //    ROS_INFO("mani:::::::::::%s",enm.c_str());
            en_j1 = strtod(enm.c_str(),&J);        // 'strtod' change str to double
            en_j2 = strtod(J,&J);
            en_j3 = strtod(J,&J);
            en_j4 = strtod(J,NULL);
            en_j5 = j5_angle*0.29;
            en_j6 = j6_angle*0.29;
            pos.push_back(en_FL);            // put the read angle to pos vector
            pos.push_back(en_FR);
            pos.push_back(en_BL);
            pos.push_back(en_BR);
            pos.push_back(en_j1);
            pos.push_back(en_j2);
            pos.push_back(en_j3);
            pos.push_back(en_j4);
            pos.push_back(en_j5);
            pos.push_back(en_j6);
            pos.resize(10);
            fliper_pos.data = pos;           // change to ros std_msgs
            fliper_pos.data.resize(10);
            Leg_pos.publish(fliper_pos);     // publish angle
       //     ROS_INFO(":FL %d, FR %d, BL %d, BR %d, J1 %d, J2 %d, J3 %d, J4 %d, J5 %d, J6 %d",fliper_pos.data[0],fliper_pos.data[1],fliper_pos.data[2],fliper_pos.data[3],fliper_pos.data[4],fliper_pos.data[5],fliper_pos.data[6],fliper_pos.data[7],fliper_pos.data[8],fliper_pos.data[9]);
            fliper_pos.data.capacity();
            pos.clear();

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
    
 //ROS_INFO("FL_POS %d : FR_POS %d",FL_Pos,FR_Pos);
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
                 ROS_INFO("Stop");
            }
            else
            {
        ROS_INFO("CMd_vel::::::::::::%s : %s",FL.c_str(),FR.c_str());
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
void imucallback(const sensor_msgs::ImuConstPtr& imu)
{
      if(imu->orientation.y < -4)
      {
          is_slope = true;
      }
      else
      {
          is_slope = false;
      }

}

void navigationCallback(const geometry_msgs::TwistPtr& msg)
{
    double speedL, speedR;
    double linear, angular;
    int dL,dR;
    int maxspeed = 80;
    int minspeed = 10;
    linear = msg->linear.x;
    angular = msg->angular.z;
   // speedL = msg->linear.x*10 - msg->angular.z*(robotsize/2);
    //speedR = msg->linear.x*10 + msg->angular.z*(robotsize/2);
              // robot speed in m/s
if(is_slope == false)
{
            speedL = (linear*0.3 - angular*0.20 );
            speedR = (linear*0.3 + angular*0.20 );
 // ROS_INFO("speedL %f: speedR %f",speedL,speedR);
            // m/s to m/min
            speedR = speedR*60;
            speedL = speedL*60;
            // 1 track rpm = 1.18 m/min
            speedR = speedR/1.28; // 1.18
            speedL = speedL/1.28;
            //  4 motor rpm = 1 track rpm
            speedR = speedR*3;
            speedL = speedL*3;
}
else
{
ROS_INFO("SLOPE MODEEEEEEEEEEEEEEEEEEEEEEEEEE");
 speedL = (linear*0.3 - angular*0.20 );
            speedR = (linear*0.3 + angular*0.20 );
 // ROS_INFO("speedL %f: speedR %f",speedL,speedR);
            // m/s to m/min
            speedR = speedR*60;
            speedL = speedL*60;
            // 1 track rpm = 1.18 m/min
            speedR = speedR/1.28;
            speedL = speedL/1.28;
            //  4 motor rpm = 1 track rpm
            speedR = speedR*3;
            speedL = speedL*3;

	if(speedR < 0)
	{
       speedR = -10;
       speedL = speedL+10;
	}
    if(speedL < 0)
	{
        speedL = -10;
        speedR = speedR+10;
	}
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
    if(is_start == true && is_stop == false && is_manual == false)
    {
    ROS_INFO("L %f:a %f: speedL %f: speedR %f",linear,angular,speedL,speedR);

    speed_control(dR,dL,abs(speedR),abs(speedL));

    }
      
/*

    double w = 0.5;
    double Lx, Az;
    int sf = 30 , st =30;
    string s;
    Az = msg->angular.z;
    Lx = msg->linear.x;
    ROS_INFO("L: %f : R: %f",Lx,Az);
    if(Az == 0 && Lx==0)
    {
        ROS_INFO("stop");
        my_serial.write("As00j");
        ros::Duration(0.01).sleep();
        my_serial.write("Bs00j");
        ros::Duration(0.01).sleep();
    }
    else if(Az > -w && Az < w && Az !=0 )
    {
        // go forward or backward
        if(Lx > 0)
        {    if(is_clear == true)
            {
                ROS_INFO("stoppp");
                my_serial.write("As00j");
                ros::Duration(0.01).sleep();
                my_serial.write("Bs00j");
                ros::Duration(0.01).sleep();
                ROS_INFO("stoppp");
                my_serial.write("As00j");
                ros::Duration(0.01).sleep();
                my_serial.write("Bs00j");
                ros::Duration(0.01).sleep();
                is_clear =false;
            }
            ROS_INFO("Forward");
            string A = "Ar";
             s = boost::lexical_cast<string>( sf );
             A.append(s);
             A.append("j");
            my_serial.write(A);
            ros::Duration(0.01).sleep();
            string B = "Bl";
             s = boost::lexical_cast<string>( sf );
             B.append(s);
             B.append("j");
            my_serial.write(B);
            ros::Duration(0.01).sleep();
               ROS_INFO("%s : %s",A.c_str(),B.c_str());
           // forward
        }
        else
        {
            if(is_clear == true)
                        {
                            ROS_INFO("stoppp");
                            my_serial.write("As00j");
                            ros::Duration(0.01).sleep();
                            my_serial.write("Bs00j");
                            ros::Duration(0.01).sleep();
                            ROS_INFO("stoppp");
                            my_serial.write("As00j");
                            ros::Duration(0.01).sleep();
                            my_serial.write("Bs00j");
                            ros::Duration(0.01).sleep();
                            is_clear =false;
                        }
            ROS_INFO("backward");
            string A = "Al";
             s = boost::lexical_cast<string>( sf );
             A.append(s);
             A.append("j");
            my_serial.write(A);
            ros::Duration(0.01).sleep();
            string B = "Br";
             s = boost::lexical_cast<string>( sf );
             B.append(s);
             B.append("j");
            my_serial.write(B);
            ros::Duration(0.01).sleep();
                  ROS_INFO("%s : %s",A.c_str(),B.c_str());
            //backward
        }
    }
    else if(Az > w&&Az !=0)
    {
        // Left
        if(is_slope == false)
        {
        ROS_INFO("left");

        string A = "Al";
         s = boost::lexical_cast<string>( st );
         A.append(s);
         A.append("j");
        my_serial.write(A);
        ros::Duration(0.01).sleep();
        string B = "Bl";
         s = boost::lexical_cast<string>( st );
         B.append(s);
         B.append("j");
        my_serial.write(B);
        ros::Duration(0.01).sleep();
          ROS_INFO("%s : %s",A.c_str(),B.c_str());
        cntl++;
        if(cntl > 10)
        {
        is_clear = true;
        cntl = 0;
        }
        }
        else
        {
            ROS_INFO("LEFT SLOPE");
            string A = "Al";
             s = boost::lexical_cast<string>( st/2 );
             A.append(s);
             A.append("j");
            my_serial.write(A);
            ros::Duration(0.01).sleep();
            string B = "Bl";
             s = boost::lexical_cast<string>( st);
             B.append(s);
             B.append("j");
            my_serial.write(B);
            ros::Duration(0.01).sleep();
                ROS_INFO("%s : %s",A.c_str(),B.c_str());
            cntl++;
            if(cntl > 10)
            {
            is_clear = true;
            cntl = 0;
            }
        }
    }
    else if(Az<-w&&Az !=0)
    {
        if(is_slope == false)
        {
        // right
        ROS_INFO("right");
        string A = "Ar";
         s = boost::lexical_cast<string>( st );
         A.append(s);
         A.append("j");
        my_serial.write(A);
        ros::Duration(0.01).sleep();
        string B = "Br";
         s = boost::lexical_cast<string>( st );
         B.append(s);
         B.append("j");
        my_serial.write(B);
        ros::Duration(0.01).sleep();
            ROS_INFO("%s : %s",A.c_str(),B.c_str());
        cntr++;
                if(cntr>10)
        {
        is_clear =true;
        cntr=0;
        }
        }
      else
        {
            ROS_INFO("RIGHT SLOPE");
            string A = "Ar";
             s = boost::lexical_cast<string>( st/2 );
             A.append(s);
             A.append("j");
            my_serial.write(A);
            ros::Duration(0.01).sleep();
            string B = "Br";
             s = boost::lexical_cast<string>( st);
             B.append(s);
             B.append("j");
            my_serial.write(B);
            ros::Duration(0.01).sleep();
                ROS_INFO("%s : %s",A.c_str(),B.c_str());
            cntl++;
            if(cntl > 10)
            {
            is_clear = true;
            cntl = 0;
            }
        }
    }
*/
}
void commandCallback(const std_msgs::StringPtr& msg)
{
    if(msg->data == "start_walk" || "go")
    {
        is_start = true;
        is_stop = false;
    }
    else if(msg->data =="stop")
    {
        is_start = false;
        is_stop = true;
    }
    else if(msg->data == "manual") // switch teleop mode
    {
        is_manual = true;
    }
    else if(msg->data == "auto") // switch autonomous mode
    {
        is_manual = false;
    }
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
  ros::Subscriber command_sub = n.subscribe("autocommand",10,commandCallback);
 Leg_pos = n.advertise<std_msgs::Int16MultiArray>("leg_pos",10);               //advertised leg pos to publish
 //ros::Timer timer = n.createTimer(ros::Duration(0.05), legcallback);             //define timer function
// ros::Timer en_update = n.createTimer(ros::Duration(20),update_callback);
ros::Timer timer = n.createTimer(ros::Duration(0.05), legcallback); 

 std::string str = "/dev/ttyUSB0";
 n.getParam("my_serial", str);
    //char reply[REPLY_SIZE

    // Change the next line according to your port name and baud rate
    try
    {
      // ROS_INFO("Try to open port %s.",str.c_str());


    if(my_serial.isOpen())
    {
        ROS_INFO("Port is opened");
        std::string enleg;
        std::string enm;
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
