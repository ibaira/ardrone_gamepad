#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>


class ARDroneJoy
{
public:
  ARDroneJoy();
  geometry_msgs::Twist vel;
  std_msgs::Empty emptyMsg;
  std_srvs::Empty emptySrv;
  double current_vel;
  ros::Publisher vel_pub_; //! It will publish into command velocity (for the robot) and the ptz_state (for the pantilt)
    ros::Publisher takeoff_pub ;
    ros::Publisher land_pub;
    ros::Publisher reset_pub;
    ros::Subscriber joy_sub_;//! It will be suscribed to the joystick
    ros::ServiceClient client_1;//!togglecam
    ros::ServiceClient client_2;//!flattrim
    std::string cmd_topic_vel_;  //! Name of the topic where it will be publishing the velocity
    void  flattrim(float duration);
    void  land(float duration);
    void  takeoff(float duration);
    void  giro(float duration, char  sentido);
    void  desplazamiento_x(float duration, char sentido);
    void  desplazamiento_y(float duration, char sentido);

private:
  ros::NodeHandle nh_;

  //int linear_x, linear_y, linear_z, angular_x, angular_y, angular_z;
  //double l_scale_, a_scale_;

 };


ARDroneJoy::ARDroneJoy()
{

	current_vel = 1.0;

	nh_.param("cmd_topic_vel", cmd_topic_vel_, std::string("cmd_vel"));


	ROS_INFO("Service I/O = [%s]", cmd_topic_vel_.c_str());

  	// Publish through the node handle Twist type messages to the ARDrone_ctrl/command topic
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
	//Publish to topics
	takeoff_pub = nh_.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	land_pub = nh_.advertise<std_msgs::Empty>("ardrone/land", 1);
	reset_pub = nh_.advertise<std_msgs::Empty>("ardrone/reset", 1);


	//Ask for services
	client_1 = nh_.serviceClient<std_srvs::Empty>("ardrone/togglecam");
	client_2 = nh_.serviceClient<std_srvs::Empty>("ardrone/flattrim");

}

void  ARDroneJoy::flattrim(float duration){
  ROS_INFO("Flattrim triggered");
  client_2.call(emptySrv);
  ros::Duration(duration).sleep();
}

void  ARDroneJoy::land(float duration){
  ROS_INFO("Land triggered");
  land_pub.publish(emptyMsg);
  ros::Duration(duration).sleep();
}

void  ARDroneJoy::takeoff(float duration){
  ROS_INFO("Takeoff triggered");
  takeoff_pub.publish(emptyMsg);
  ros::Duration(duration).sleep();
}

void  ARDroneJoy::giro(float duration, char  sentido){
    if(sentido== 'r' || sentido== 'd'){
    	ROS_INFO("Girando a derechas...");
    	vel.angular.z = current_vel;
    }
    if(sentido== 'l' || sentido== 'i'){
        ROS_INFO("Girando a izquierdas...");
        vel.angular.z = current_vel*(-1.0);
    }
    vel_pub_.publish(vel);
    ros::Duration(duration).sleep();
    vel.angular.z = 0;
    vel_pub_.publish(vel);
}
void  ARDroneJoy::desplazamiento_x(float duration, char sentido)
{
	if(sentido== '+'){
		ROS_INFO("Moving Forward...");
		vel.linear.x = current_vel;
	}
	else {
		ROS_INFO("Moving Backward...");
		vel.linear.x = current_vel*(-1);
	}
	vel_pub_.publish(vel);
	ros::Duration(duration).sleep();
	vel.linear.x = 0;
	vel_pub_.publish(vel);
}

void  ARDroneJoy::desplazamiento_y(float duration, char sentido)
{
	if(sentido== 'r'|| sentido == 'd'){
		ROS_INFO("Moving to the right...");
		vel.linear.y = current_vel*(-1);
	}
	if(sentido=='l'|| sentido=='i') {
		ROS_INFO("Moving to the left...");
		vel.linear.y = current_vel;
	}
	vel_pub_.publish(vel);
	ros::Duration(duration).sleep();
	vel.linear.y = 0;
	vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ArDrone_Gamepad");
	ARDroneJoy ardrone_joy;

	//usleep(10000);
	ros::Duration(2).sleep();
	//scanf("%f", &ardrone_joy.current_vel);
	//! Formato: funcion_ accion (duracion, sentido si existe)

	ROS_INFO("About to start sequence");

	ardrone_joy.flattrim(4);
	ardrone_joy.takeoff(7);
	ardrone_joy.giro(5.0, 'd');
	ardrone_joy.giro(5.0, 'i');
	ardrone_joy.land(0.5);

	//ros::spin();
}

