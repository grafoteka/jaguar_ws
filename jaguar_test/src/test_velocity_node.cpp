#include <ros/ros.h>
#include <std_msgs/String.h>
#include <jaguar_test/BaseMotorCmd.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "jaguar_test_velocity");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<jaguar_test::BaseMotorCmd>("/drrobot_jaguar_v6_basemotor_cmd", 1);

  // Create message
  jaguar_test::BaseMotorCmd message;

  ros::Rate loop_rate(10);

  double time = 0.0;
  double time_old = 0.0;

  float leftCmd, rightCmd;
  float vel_command = 0.0;

  double time_initial = ros::Time::now().toSec();

  ROS_INFO("Estoy aqui");

  while (ros::ok())
  {
    time = ros::Time::now().toSec();
    //ROS_INFO("El tiempo es: %.2f", time);

    if ((time - time_initial) < 30.0)
    {
      vel_command = 200;
      static int i = 0;
      if(i == 0)
      {
        ROS_INFO("vel_command = 200");
        i = 1;
      }
    }

    else if ((30 <= (time - time_initial)) && ((time - time_initial) < 60))
    {
      vel_command = 250;
      static int i = 0;
      if(i == 0)
      {
        ROS_INFO("vel_command = 250");
        i = 1;
      }
    }

    else if ((60 <= (time - time_initial)) && ((time - time_initial) < 90))
    {
      vel_command = 300;
      static int i = 0;
      if(i == 0)
      {
        ROS_INFO("vel_command = 300");
        i = 1;
      }
    }

    else if ((90 <= (time - time_initial)) && ((time - time_initial) < 120))
    {
      vel_command = 350;
      static int i = 0;
      if(i == 0)
      {
        ROS_INFO("vel_command = 350");
        i = 1;
      }
    }

    else if ((120 <= (time - time_initial)) && ((time - time_initial) < 150))
    {
      vel_command = 400;
      static int i = 0;
      if(i == 0)
      {
        ROS_INFO("vel_command = 400");
        i = 1;
      }
    }

    else
      return 0;

    leftCmd =  vel_command;
    rightCmd = -vel_command;

    message.leftCmd = leftCmd;
    message.rightCmd = rightCmd;

    chatter_pub.publish(message);

    ros::spinOnce();

    loop_rate.sleep();


  }

  return 0;

}
