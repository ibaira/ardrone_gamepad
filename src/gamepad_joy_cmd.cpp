#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
//!#include <ardrone_api.h>

#define DEFAULT_NUM_OF_BUTTONS		10
#define DEFAULT_AXIS_LINEAR_X	    1
#define DEFAULT_AXIS_LINEAR_Y	    0
#define DEFAULT_AXIS_LINEAR_Z	    3
#define DEFAULT_AXIS_ANGULAR_X		3 /////////////5
#define DEFAULT_AXIS_ANGULAR_Y		6
#define DEFAULT_AXIS_ANGULAR_Z		4
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0
#define NUM_BUTTONS                 10


class ARDroneJoy
{

public:

  ARDroneJoy();

private:
  ros::Publisher vel_pub_; //! It will publish into command velocity (for the robot) and the ptz_state (for the pantilt)
  ros::Publisher takeoff_pub ;
  ros::Publisher land_pub;
  ros::Publisher reset_pub;
  ros::Subscriber joy_sub_;//! It will be suscribed to the joystick
  ros::ServiceClient client_1;//!togglecam
  ros::ServiceClient client_2;//!flattrim

  std::string cmd_topic_vel_;  //! Name of the topic where it will be publishing the velocity
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  double current_vel;
  int speed_up_button_, speed_down_button_, takeoff_button_, land_button_, reset_button_, togglecam_button_, flattrim_button_; //! Number of the button for increase or decrease the speed max of the joystick
  int num_of_buttons_; //! Number of buttons of the joystick
  bool bRegisteredButtonEvent[NUM_BUTTONS]; //! Pointer to a vector for controlling the event when pushing the buttons
  int linear_x, linear_y, linear_z, angular_x, angular_y, angular_z;
  double l_scale_, a_scale_;
};

ARDroneJoy::ARDroneJoy()
{
    ros::NodeHandle nh_;
    current_vel = 0.4;
	
    nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	// MOTION CONF
    nh_.param("axis_linear_x", linear_x, DEFAULT_AXIS_LINEAR_X);
    nh_.param("axis_linear_y", linear_y, DEFAULT_AXIS_LINEAR_Y);
    nh_.param("axis_linear_z", linear_z, DEFAULT_AXIS_LINEAR_Z);
	
    nh_.param("axis_angular_x", angular_x, DEFAULT_AXIS_ANGULAR_X);
    nh_.param("axis_angular_y", angular_y, DEFAULT_AXIS_ANGULAR_Y);
    nh_.param("axis_angular_z", angular_z, DEFAULT_AXIS_ANGULAR_Z);
	
    nh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
    nh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
    nh_.param("cmd_topic_vel", cmd_topic_vel_, std::string("cmd_vel"));
    nh_.param("button_speed_up", speed_up_button_, 8);  //4 Thrustmaster
    nh_.param("button_speed_down", speed_down_button_, 10); //5 Thrustmaster
	//
    nh_.param("button_takeoff", takeoff_button_, 1);
    nh_.param("button_land", land_button_, 2);
    nh_.param("button_reset", reset_button_, 3);
	
    nh_.param("gamepad_joy_cmd/button_togglecam", togglecam_button_, 5);
    nh_.param("button_flattrim", flattrim_button_, 4);
	
	for(int i = 0; i < NUM_BUTTONS; i++){
		bRegisteredButtonEvent[i] = false;
	}

	ROS_INFO("Service I/O = [%s]", cmd_topic_vel_.c_str());
	ROS_INFO("Axis linear_x = %d", linear_x);
	ROS_INFO("Axis linear_Y = %d", linear_y);
	ROS_INFO("Axis linear_z = %d", linear_z);
	ROS_INFO("Axis angular_x = %d", angular_x);
	ROS_INFO("Axis angular_y = %d", angular_y);
	ROS_INFO("Axis angular_z = %d", angular_z);
    ROS_INFO("Scale linear = %f", l_scale_);
    ROS_INFO("Scale angular = %f", a_scale_);

  	// Publish through the node handle Twist type messages to the ARDrone_ctrl/command topic
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
	//Publish to topics
    takeoff_pub = nh_.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
    land_pub = nh_.advertise<std_msgs::Empty>("ardrone/land", 1);
    reset_pub = nh_.advertise<std_msgs::Empty>("ardrone/reset", 1);

 	 // Listen through the node handle sensor_msgs::Joy messages from joystick (these are the orders that we will send to ARDrone_ctrl/command)
    joy_sub_ = nh_.subscribe/*<sensor_msgs::Joy>*/("joy", 1, &ARDroneJoy::joyCallback, this);
	//Ask for services
    client_1 = nh_.serviceClient<std_srvs::Empty>("/ardrone/togglecam");
    client_2 = nh_.serviceClient<std_srvs::Empty>("ardrone/flattrim");
    //ROS_INFO("Subscribes made");
	
}


void ARDroneJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist vel;
	std_msgs::Empty emptyMsg;
	std_srvs::Empty emptySrv;
    //ROS_INFO("Callback");

		if (joy->buttons[takeoff_button_]==1)
	    {
	    	if(!bRegisteredButtonEvent[takeoff_button_])
	    	{
	    		bRegisteredButtonEvent[takeoff_button_] = true;
	    		takeoff_pub.publish(emptyMsg);
	    	}
		}
		else{
			bRegisteredButtonEvent[takeoff_button_] = false;
		}
		if (joy->buttons[land_button_]==1)
		{
			if(!bRegisteredButtonEvent[land_button_])
			{
				bRegisteredButtonEvent[land_button_] = true;
				land_pub.publish(emptyMsg);
			}
		}
		else{
					bRegisteredButtonEvent[land_button_] = false;
		}
		if (joy->buttons[reset_button_]==1)
		{
			if(!bRegisteredButtonEvent[reset_button_])
			{
				bRegisteredButtonEvent[reset_button_] = true;
				reset_pub.publish(emptyMsg);
			}
		}
		else{
					bRegisteredButtonEvent[reset_button_] = false;
		}

        if (joy->buttons[togglecam_button_]==1)
		{
			if(!bRegisteredButtonEvent[togglecam_button_])
			{
				bRegisteredButtonEvent[togglecam_button_] = true;
				client_1.call(emptySrv);
			}
		}
		else{
			bRegisteredButtonEvent[togglecam_button_] = false;
		}
		if (joy->buttons[flattrim_button_]==1)
		{
			if(!bRegisteredButtonEvent[flattrim_button_])
			{
				bRegisteredButtonEvent[flattrim_button_] = true;
				client_2.call(emptySrv);
			}
		}
		else{
			bRegisteredButtonEvent[flattrim_button_] = false;
		}

	
	    if ( joy->buttons[speed_down_button_] == 1 )
	    {
	    	if(!bRegisteredButtonEvent[speed_down_button_])
	    	{
	    		if(current_vel > 0.1)
	    		{
	    			current_vel = current_vel - 0.1;
	    			bRegisteredButtonEvent[speed_down_button_] = true;
					ROS_INFO("Velocity: %f%%", current_vel*100.0);	
				}
	    	}
		}
	    else{
	    	bRegisteredButtonEvent[speed_down_button_] = false;
	    }

  		if (joy->buttons[speed_up_button_] == 1)
  		{
			if(!bRegisteredButtonEvent[speed_up_button_])
			{
				if(current_vel < 0.9)
				{
					current_vel = current_vel + 0.1;
					bRegisteredButtonEvent[speed_up_button_] = true;
			 	 	ROS_INFO("Velocity: %f%%", current_vel*100.0);
				}
			}
		}
		else{
			bRegisteredButtonEvent[speed_up_button_] = false;
		}
		 
		//vel.angular.x = current_vel*(a_scale_*joy->axes[angular_x]);
		//vel.angular.y = current_vel*(a_scale_*joy->axes[angular_y]);
		vel.angular.z = current_vel*(a_scale_*joy->axes[angular_z]);
		vel.linear.x = current_vel*l_scale_*joy->axes[linear_x];
		vel.linear.y = current_vel*l_scale_*joy->axes[linear_y];
		vel.linear.z = current_vel*l_scale_*joy->axes[linear_z];

		vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ArDrone_Gamepad");
    ARDroneJoy ardrone_joy;
    ROS_INFO("Clase creada");

	ros::spin();
}


