#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDrive.h"
#include <string>
#include <stdlib.h>     /* abs */
#include <math.h>
#include "JHPWMPCA9685.h"

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


// FOR OLD ENGINE
//#define PWM_THROTTLE_FULL_REVERSE 279
//#define PWM_THROTTLE_NEUTRAL 288
//#define PWM_THROTTLE_FULL_FORWARD 297

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

void callback(const ackermann_msgs::AckermannDrive::ConstPtr& msg)
{
    ROS_INFO("Speed: %f", msg->speed);

    throttle(msg->speed);


    if (msg->steering_angle != 0) {  
       turn(msg->steering_angle);
    } 
    else {
       turn(0.0);
	}
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

int main(int argc, char **argv)
{
  // initialization
  pca9685 = new PCA9685() ;
  int err = pca9685->openPCA9685();
  if (err < 0){
      printf("Error: %d", pca9685->error);
  }
  printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
  
  checkSystems();
  printf("System is OK\n");
  
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("drivecmd", 1000, callback);
  ros::spin();

  return 0;
}
