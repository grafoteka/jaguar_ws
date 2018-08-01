#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <joy_node/BaseMotorCmd.h>
#include <joy_node/FlipMotorCmd.h>

int vel_command = 0.0; // m/s
int leftCmd = 0;
int rightCmd = 0;
float joyL = 0;
float joyA = 0;
float wheel_base = 0.47;
float wheel_radius= 0.165;
int z = 0;
int m=0;
int rightFrontCmd = 0;
int leftFrontCmd = 0;
int leftRearCmd = 0;
int rightRearCmd = 0;


void lectura_joystick(const sensor_msgs::Joy::ConstPtr& joy)
{


  if(joy->buttons[1] == 1 and joy->buttons[5] == 1)  //FLIPS
    {

        rightFrontCmd =  250;
        ROS_INFO("Pos Flip Right Front: %i", rightFrontCmd );
    }

  if(joy->buttons[2] == 1 and joy->buttons[5] == 1)
        {

            leftFrontCmd= 200;
            ROS_INFO("Pos Flip Left Front: %i", leftFrontCmd );


  }

   if(joy->buttons[3] == 1 and joy->buttons[5] == 1)
                  {

               leftFrontCmd = -200;
               ROS_INFO("Pos Flip Right Front: %i", leftFrontCmd );

   }
   if(joy->buttons[0] == 1 and joy->buttons[5] == 1)
         {
            rightFrontCmd = -250;
            ROS_INFO("Pos Flip Right Front: %i", rightFrontCmd );
   }



  // ORUGAS


  if(joy->axes[7] == 1 and joy->buttons[5] == 1 and joy->axes[6] == 0 ) //AVANCE
  {
    if(vel_command < 500){
      vel_command = vel_command + 10;
      leftCmd = -vel_command;
      rightCmd = vel_command;
      ROS_INFO("velocidad en m/s: %i", vel_command);
    }
  }

  if(joy->axes[7] == 1 and joy->axes[6] == 1 and joy->buttons[5] == 1  ) //GIRO 45 GRADOS
  {
    if(vel_command < 500){

      leftCmd = -vel_command + 50 ;
      rightCmd = vel_command + 50;
      ROS_INFO("velocidad en m/s: %i", vel_command);
    }
  }

  if(joy->axes[7] == 1 and joy->axes[6] == -1 and joy->buttons[5] == 1  ) //GIRO 45 GRADOS
  {
    if(vel_command < 500){

      leftCmd = -vel_command - 50;
      rightCmd = vel_command - 50;
      ROS_INFO("velocidad en m/s: %i", vel_command);
    }
  }


  if(joy->axes[7] == -1 and joy->buttons[5] == 1 and joy->axes[6] == 0 ) //RETROCESO
  {
    if(vel_command > -500){
      vel_command = vel_command - 10;
      leftCmd = -vel_command;
      rightCmd = vel_command;
      ROS_INFO("velocidad en m/s: %i", vel_command);
    }
  }

  if(joy->axes[0] == 1 and joy->buttons[5] == 1)   //GIRO INCREMENTAL EN RUEDAS
  {
      m=m+10;
      leftCmd =  -vel_command+m;
      rightCmd =  vel_command+m;
      ROS_INFO("velocidad en m/s: %i", vel_command );
    }

  if(joy->axes[0] == -1 and joy->buttons[5] == 1)
  {
      m=m+10;
      leftCmd =  -vel_command-m;
      rightCmd =  vel_command-m;
      ROS_INFO("velocidad en m/s: %i", vel_command );
    }

  if(joy->axes[1] == 1 and joy->buttons[5] == 1)  //IGUALAR AMBAS RUEDAS A LA VELOCIDAD PREDEFINIDA
  {
      m=0;
      leftCmd =  -vel_command;
      rightCmd =  vel_command;
      ROS_INFO("velocidad en m/s: %i", vel_command );
    }

  if(joy->axes[1] == -1 and joy->buttons[5] == 1)
  {
      m=0;
      leftCmd =  vel_command;
      rightCmd = -vel_command;
      ROS_INFO("velocidad en m/s: %i", vel_command );
    }

  if(joy->axes[6] == 1 and joy->buttons[5] == 1 and joy->axes[7] == 0 ) // Giro en parado
  {
      z = 300;
     // joyL = joy->axes[1];
      //joyA = joy->axes[0];
      //vel_command = ((joyL*z - joyA*z*wheel_base/2)/wheel_radius);
      vel_command = z;
      leftCmd =  vel_command;
      rightCmd = vel_command;
      ROS_INFO("velocidad en m/s: %i", vel_command );
      //ROS_INFO("velocidad en m/s: %.2f", joyL );
      //ROS_INFO("velocidad en m/s: %.2f", joyA );
    }


  if(joy->axes[6] == -1 and joy->buttons[5] == 1 and joy->axes[7] == 0 ) //Giro en parado
  {
      z = 300;
      //joyL = joy->axes[1];
      //joyA = joy->axes[0];
      vel_command = z;
      leftCmd =  -vel_command;
      rightCmd = -vel_command;
      ROS_INFO("velocidad en m/s: %i", vel_command );
      //ROS_INFO("velocidad en m/s: %.2f", joyL );
      //ROS_INFO("velocidad en m/s: %.2f", joyA );
    }

  if(joy->buttons[5] == 0)   // RESETEO
  {
    
      vel_command = 0;
      leftCmd = vel_command;
      rightCmd = vel_command;
      rightFrontCmd = 0;
      leftFrontCmd = 0;
      leftRearCmd = 0;
      rightRearCmd = 0;
      m = 0;

      ROS_INFO("velocidad en m/s: %i", vel_command);
    
  }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_node_patch");
    ros::NodeHandle nh;

    ros::Subscriber joystick = nh.subscribe("/joy", 1, lectura_joystick);
    ros::Publisher chatter_pub = nh.advertise<joy_node::BaseMotorCmd>("/drrobot_jaguar_v6_basemotor_cmd", 1);
    ros::Publisher chatter_pub2 = nh.advertise<joy_node::FlipMotorCmd>("drrobot_jaguar_v6_flipmotor_cmd", 1);

    // Create message
    joy_node::BaseMotorCmd message;
    joy_node::FlipMotorCmd message2;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
     // message.leftCmd = -vel_command;
      //message.rightCmd = vel_command;
      message.leftCmd = leftCmd;
      message.rightCmd = rightCmd;
      message2.leftFrontCmd = leftFrontCmd;
      message2.rightFrontCmd = rightFrontCmd;
      message2.leftRearCmd = leftRearCmd;
      message2.rightRearCmd = rightRearCmd ;

      chatter_pub.publish(message);
      chatter_pub2.publish(message2);
      ros::spinOnce();

      loop_rate.sleep();
    }

    return 0;
}
