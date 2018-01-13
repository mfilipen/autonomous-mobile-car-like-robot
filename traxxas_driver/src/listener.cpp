#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <math.h> //need PI

#include "udp_connection/RobotCmd.h"
#include "udp_connection/RobotData.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>
#include "JHPWMPCA9685.h"
#include "nav_msgs/Odometry.h"
#include "traxxas_driver/SetYaw.h"
PCA9685 *pca9685;

// The Steering Servo is plugged into the follow PWM channel
#define STEERING_CHANNEL 0
// The ESC is plugged into the following PWM channel
#define ESC_CHANNEL 1


#define PWM_STEERING_FULL_REVERSE 265
#define PWM_STEERING_NEUTRAL 341
#define PWM_STEERING_FULL_FORWARD 415

#define PWM_THROTTLE_FULL_REVERSE 224
#define PWM_THROTTLE_NEUTRAL 325
#define PWM_THROTTLE_FULL_FORWARD 426

#define STEERING_LOCK 17


// FOR OLD ENGINE
//#define PWM_THROTTLE_FULL_REVERSE 279
//#define PWM_THROTTLE_NEUTRAL 288
//#define PWM_THROTTLE_FULL_FORWARD 297


// hedge_position
float steering = 0;
float gas = 0;
bool emergency_stop = false;

double pos_time;
double last_pos_time;

double x = 0;
double y = 0;
double x_last = 0;
double y_last = 0;

double psi = 0;
double dpsi = 0;

double dx = 0;
double dy = 0;

bool pos_received = false;
bool imu_received = false;
bool default_values = false;

double yaw_acc_offset = 0;
double last_cmd_time;

double last_imu_time=0;
/**
* Turning wheels by given command  in range [-1; 1]
*
*/
void turn(float steering_command) {
    //ROS_INFO("Turn command: %f", steering_command);
    pca9685->setPWM(STEERING_CHANNEL,0,PWM_STEERING_NEUTRAL + ceil(75 * steering_command));
}

/**
* Drive given amount to the given direction
*/
void throttle(float amount) {
    pca9685->setPWM(ESC_CHANNEL,0,PWM_THROTTLE_NEUTRAL + 101 * amount);
}

void callback_cmd(const udp_connection::RobotCmd::ConstPtr &msg)
{
  steering = msg->steering_cmd;
  gas =  (emergency_stop) ? gas : msg->gas_cmd;
  ROS_INFO("Turn command: %f", steering);

  last_cmd_time = ros::Time::now().toSec();

  default_values = false;
}

void callback_pos(const nav_msgs::Odometry::ConstPtr &msg)
{
	x_last = x;
	y_last = y;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;

	dx = msg->twist.twist.linear.x;
	dy = msg->twist.twist.linear.x;

	last_pos_time = pos_time;
	pos_time = ros::Time::now().toSec();

	if (!pos_received) {
		pos_received = true;
		ROS_INFO("POS data received");
	}


    double orientation_x, orientation_y, orientation_z, orientation_w;
    orientation_x = msg->pose.pose.orientation.x;
    orientation_y = msg->pose.pose.orientation.y;
    orientation_z = msg->pose.pose.orientation.z;
    orientation_w = msg->pose.pose.orientation.w;

    tf::Quaternion quaternion(orientation_x, orientation_y, orientation_z, orientation_w);

    tf::Matrix3x3 m(quaternion);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

//    psi = yaw;
}


void callback_imu(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!imu_received) {
    	imu_received = true;
        yaw_acc_offset =  msg->angular_velocity.z;
	    //ROS_INFO("IMU data received");
    }

    //calculate time difference imu_dt
    double imu_dt=ros::Time::now().toSec()-last_imu_time;

    dpsi = msg->angular_velocity.z-yaw_acc_offset;
    if (last_imu_time>0){
        psi += dpsi*imu_dt;
        psi = fmod(psi,2*M_PI); //now psi in interval -2pi:2pi

        //converting psi to interval -pi:+pi
        if (psi>=0){
            psi=(psi > M_PI) ? -2*M_PI+psi : psi;
        }
        else {
            psi=(psi < -M_PI) ? 2*M_PI+psi : psi;
        }
    }

    last_imu_time =ros::Time::now().toSec();
}


//This is the method which is aimed to stop the car immidiatelly
void callback_emerg_stop(const std_msgs::Bool::ConstPtr &msg)
{
    emergency_stop = msg->data;
    if (!msg->data){
      ROS_WARN("EMERGENCY STOP DISABLED!");
      return;
    }

    ROS_WARN("STOPPING THE CAR!");
    //add stop logic here
    //publish +0 to be sure we don't go back
    throttle(0.01);
    gas=-1;
    //set gas to <0 so that we brake, not coast
    throttle(gas);
}
void reset_cmd()
{
	steering = 0;
	gas = 0;
}

/**
* Checking the system by turning the wheels right by 70%,
* to the neutral state and left by 70%.
*/
void checkSystems()
{

  float currentSteeringOutput;
  float targetSteeringOutput;
  // reset all registers
  pca9685->setAllPWM(0,0) ;
  pca9685->reset() ;
  pca9685->setPWMFrequency(50) ;

  // init PWM_THROTTLE
  pca9685->setPWM(ESC_CHANNEL,0,PWM_THROTTLE_NEUTRAL);

  sleep(1.0) ;
  // set wheels to the neutral state
  pca9685->setPWM(STEERING_CHANNEL,0,PWM_STEERING_NEUTRAL);

  sleep(1.0);

  targetSteeringOutput = PWM_STEERING_NEUTRAL + 70;
  currentSteeringOutput = PWM_STEERING_NEUTRAL;
  // turning right by 0.2f until target
  while (abs(currentSteeringOutput -targetSteeringOutput) > 0.5f) {
     currentSteeringOutput += 0.2f;
     pca9685->setPWM(STEERING_CHANNEL,0,currentSteeringOutput);
  }
  sleep(1.0);
  // back to the neutral state
  pca9685->setPWM(STEERING_CHANNEL,0,PWM_STEERING_NEUTRAL);
  sleep(1.0);

  targetSteeringOutput = PWM_STEERING_NEUTRAL - 70;
  currentSteeringOutput = PWM_STEERING_NEUTRAL;

  // turning left by 0.2f until target
  while (abs(currentSteeringOutput -targetSteeringOutput) > 0.5f) {
     currentSteeringOutput -= 0.2f;
     pca9685->setPWM(STEERING_CHANNEL,0,currentSteeringOutput);
  }
  sleep(1.0);
  // back to the neutral state
  pca9685->setPWM(STEERING_CHANNEL,0,PWM_STEERING_NEUTRAL);
}

/* Set yaw value via service call */
bool set_yaw(traxxas_driver::SetYaw::Request  &req,
             traxxas_driver::SetYaw::Response &res){
    psi=req.yaw;
    ROS_INFO("PSI set to %f via service call", psi);
    return true;
}

/**
Entry point. MAIN.
*/
int main(int argc, char **argv)
{
  // initialization
  pca9685 = new PCA9685() ;
  int err = pca9685->openPCA9685();
  if (err < 0){
      printf("Error: %d", pca9685->error);
  }
  printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;

  ROS_INFO("Please do not move a car - calibrating gyro!");
  checkSystems();
  printf("System is OK\n");

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub_cmd = n.subscribe("udp_robot_cmd", 0, callback_cmd);
  ros::Subscriber sub_pos = n.subscribe("hedge_odom", 0, callback_pos);
  ros::Subscriber sub_imu = n.subscribe("imu", 0, callback_imu);
  ros::Subscriber sub_emerg_stop = n.subscribe("/emergency_stop", 0, callback_emerg_stop);

  ros::Publisher udp_pub = n.advertise<udp_connection::RobotData>("udp_robot_data", 0);

  //add service
  ros::ServiceServer set_yaw_service = n.advertiseService("set_yaw", set_yaw);

  ros::Rate loop_rate(10);

  while(ros::ok()) {
	  turn(steering);
	  throttle(gas);

	  // SEND RobotData HERE
	  // ROS_INFO("PSI %f", psi);

	  if (imu_received && pos_received) {
		  udp_connection::RobotData data;

		  data.pos[0] = x;
		  data.pos[1] = y;

		  data.pos_a[2] = psi;
		  data.vel_a[2] = dpsi;

		  data.vel[0] = dx;
		  data.vel[1] = dy;

		  udp_pub.publish(data);
	  }

	  loop_rate.sleep();
	  ros::spinOnce();

	  if (ros::Time::now().toSec() - last_cmd_time > 1 && !default_values) {
		reset_cmd();
		default_values = true;
		ROS_INFO("Resetting back to default values");
	  }

  }

  return 0;
}
