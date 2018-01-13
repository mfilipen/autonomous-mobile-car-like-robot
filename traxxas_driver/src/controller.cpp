#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include "ros/ros.h"

#include "udp_connection/RobotData.h"
#include "udp_connection/RobotCmd.h"
#include "udp_connection/RobotMPC.h"

struct udpCmdType{
  float steering_cmd;
  float gas_cmd;
  float brake_cmd;
  int32_t gear_cmd;
  uint8_t ctrl_mode;
};


struct robotMPC{
  float step_size;
  float steps;
  float throttle[100];
  float brakes[100];
  float steering_rate[100];
  float velocity_x[100];
};

ros::Subscriber udp_sub_cmd;
ros::Subscriber udp_sub_mpc;
ros::Subscriber udp_sub_data;

udpCmdType cmd;
robotMPC mpc;

float current_steering;
float current_velocity;

double last_msg_time;
float distance_traveled;
float dist_from_start_line;

bool default_values = false;

bool using_mpc = false;

int current_mpc_index;

void callback_cmd(const udp_connection::RobotCmd::ConstPtr &msg)
{
  cmd.steering_cmd = msg->steering_cmd;
  cmd.gas_cmd = msg->gas_cmd;
  cmd.brake_cmd = msg->brake_cmd;
  cmd.gear_cmd = msg->gear_cmd;
  cmd.ctrl_mode = msg->ctrl_mode;

  ROS_INFO("CMD GAS: %d", cmd.gas_cmd);

  last_msg_time = ros::Time::now().toSec();
  default_values = false;
}

void callback_mpc(const udp_connection::RobotMPC::ConstPtr &msg)
{
  mpc.step_size = msg->step_size;
  mpc.steps = msg->steps;

  for (int i=0; i<mpc.steps; i++) {
    mpc.throttle[i]      = msg->throttle[i];
    mpc.brakes[i]        = msg->brakes[i];
    mpc.steering_rate[i] = msg->steering_rate[i];
    mpc.velocity_x[i]    = msg->velocity_x[i];
  }

  last_msg_time = ros::Time::now().toSec();
  default_values = false;

  current_mpc_index = 0;
  distance_traveled = 0;
  using_mpc = true;
}

void reset_cmd() {
  cmd.gear_cmd = 1;
  cmd.ctrl_mode = 11;
  cmd.steering_cmd = 0;
  cmd.brake_cmd = 1;
  cmd.gas_cmd = 0;

  current_steering = 0;
  current_velocity = 0;
  current_mpc_index = 0;
  dist_from_start_line = 0;
  distance_traveled = 0;
}

void update_cmd() {
  if(default_values) {
    return;
  } else {

    if (current_velocity < 90) {
      cmd.steering_cmd = current_steering + mpc.steering_rate[current_mpc_index + 1]*current_velocity*0.03f;  
    } else {
      cmd.steering_cmd = current_steering + mpc.steering_rate[current_mpc_index + 2]*current_velocity*0.03f;  
    }
    
    cmd.gas_cmd = mpc.throttle[current_mpc_index];
    cmd.brake_cmd = mpc.brakes[current_mpc_index + 2];
    cmd.gear_cmd = 1;
    cmd.ctrl_mode = 11;

    if (distance_traveled / mpc.step_size > 1.0) {
      distance_traveled = 0;
      current_mpc_index += 1;  
    }
  }
}

void callback_robot(const udp_connection::RobotData::ConstPtr &msg) 
{
  current_steering = msg->steering_angle;
  current_velocity = sqrt(msg->vel[0]*msg->vel[0] + msg->vel[1]*msg->vel[1]);
  dist_from_start_line = msg->dist_from_start_line;
}



int main(int argc, char **argv)
{
  boost::asio::io_service io_service;
  UDPClient client(io_service, ROBOT_IP, ROBOT_PORT);

  ros::init(argc, argv, "udp_client_node");

  ros::NodeHandle n;

  udp_sub_cmd = n.subscribe("udp_robot_cmd", 0, callback_cmd);
  udp_sub_mpc = n.subscribe("mpc_robot_cmd", 0, callback_mpc);
  udp_sub_data = n.subscribe("udp_robot_data", 0, callback_robot);

  uint8_t counter = 0;

  //Default starting values
  reset_cmd();
  default_values = true;

  ros::Rate loop_rate(10);
  ros::Time start_time;

  ROS_INFO("UDP client online");

  while (ros::ok()) {
      float gas, brakes, steering; 

      start_time = ros::Time::now();
      
      if (using_mpc) {
        ROS_INFO("Current index:%d", current_mpc_index);
        update_cmd();  
      }
      
      steering = cmd.steering_cmd;
      gas = cmd.gas_cmd; // tempF = 0.0;
      brakes = cmd.brake_cmd;

      // Do some final preprocessing
      if (brakes < 0.1f) {
        brakes = 0.0f;
        // if (gas > 0.2f) {
        //   gas = gas - 0.15f;
        // }
      } else {
        gas = 0.0f;
      }


      std::string send_buf;
      
      int tempInt;
      const unsigned int size_string = 18;
      send_buf.reserve( size_string );
      // std::cout << "Prepearing packet..." << std::endl;
      send_buf.append( (const char*) &steering,  sizeof(float));//4, steering
      send_buf.append( (const char*) &gas,  sizeof(float));//8, gas
      send_buf.append( (const char*) &brakes,  sizeof(float));//12, brakes
      tempInt = cmd.gear_cmd;
      send_buf.append( (const char*) &tempInt,  sizeof(int32_t));//16, gear
      tempInt = cmd.ctrl_mode;
      send_buf.append( (const char*) &tempInt,  sizeof(uint8_t));//17, control mode
      send_buf.append( (const char*) &counter,  sizeof(uint8_t));//18, control mode

      // ROS_INFO("Sending steering:%f", cmd.steering_cmd);

      client.send(send_buf);
      counter++;
      if (counter > 2) counter = 0;

      loop_rate.sleep();

      ros::spinOnce();

      distance_traveled = distance_traveled + current_velocity * (ros::Time::now() - start_time).toSec();

      // ROS_INFO("Distance traveled / step size:%f", distance_traveled / mpc.step_size);
      // ROS_INFO("Distance start line:%f", dist_from_start_line);

      if (using_mpc) {
        if (!default_values && current_mpc_index > mpc.steps - 3) {
              // if (ros::Time::now().toSec() - last_msg_time > 2 && !default_values && current_mpc_index > mpc.steps - 3) {
          reset_cmd();
          default_values = true;
          ROS_INFO("Resetting back to default values.");
        }   
      } else {
        if (ros::Time::now().toSec() - last_msg_time > 2 && !default_values) {
          reset_cmd();
          default_values = true;
          ROS_INFO("Resetting back to default values.");
        }
      }

      
  }
}
