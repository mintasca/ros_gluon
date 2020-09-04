/*
*执行器位置控制
*/

#include <iostream>
#include "actuatorcontroller.h"
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <queue>
#include <vector>

#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#define DEG_TO_RAD(x) ((x)*M_PI / 180.0)
#define RAD_TO_DEG(x) ((x)*180.0 / M_PI)

#define STEERING_GEAR_RATIO 36

#define RAD_TO_POS(x) ((x / (2 * M_PI)) * STEERING_GEAR_RATIO)
#define POS_TO_RAD(x) ((x * (2 * M_PI)) / STEERING_GEAR_RATIO)

using namespace std;

static queue<sensor_msgs::JointState> s_jointStatesList;

void jointStatesCallback(const sensor_msgs::JointStatePtr msg)
{
  if (msg == NULL) return;
  s_jointStatesList.push(*msg);
  if (s_jointStatesList.size() >= 10) {
    s_jointStatesList.pop();
    ROS_WARN("jointStatesCallback, pop");
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gluonControl");
  ros::NodeHandle nh_;

  ros::Subscriber sub_jointstates =
    nh_.subscribe("/joint_states", 5, jointStatesCallback);
    //Initialize the controller
    ActuatorController * pController = ActuatorController::initController();
    //ec Define an error type, ec==0x00 means no error, ec will be passed to pcontroller-> lookupActuators(ec) by reference,
    //when the error occurs, ec value will be modified by SDK to the corresponding error code
    Actuator::ErrorsDefine ec;
    //Find the connected actuators and return the UnifiedID of all actuators found.
    std::vector<ActuatorController::UnifiedID> uIDArray = pController->lookupActuators(ec);
    //If the size of the uIDArray is greater than zero, the connected actuators have been found
    if (uIDArray.size() > 0)
    {
        for(int k = 0; k < uIDArray.size(); k++) {
          ActuatorController::UnifiedID actuator = uIDArray.at(k);
          //Enable actuator
          ROS_INFO("actuator ID %d, ipAddr %s", actuator.actuatorID,actuator.ipAddress.c_str());
          pController->enableActuator(actuator.actuatorID,actuator.ipAddress);
          //activate profile position mode
          pController->activateActuatorMode(actuator.actuatorID,Actuator::Mode_Profile_Pos);

          cout << "set position to 10 revolutions " << endl;
          pController->setPosition(actuator.actuatorID,0);
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        //Disable all connected actuators
        //pController->disableAllActuators();
        //insure that all actuators have been closed
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    else
    {
        cout << "Connected error code:" << hex << ec << endl;
        return -1;
    }

    ros::Rate loop_rate(50);
    while(ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
      if (!s_jointStatesList.empty()) {
        sensor_msgs::JointState js_data = s_jointStatesList.front();
        s_jointStatesList.pop();
        for(int i = 0; i < 6; i++) {
          ActuatorController::UnifiedID actuator = uIDArray.at(i);
          ROS_INFO("jointstate_%d, %s, %f", i+1, js_data.name[i].c_str(), js_data.position[i]);
          pController->setPosition(actuator.actuatorID, RAD_TO_POS(js_data.position[i]));
        }
      }
    }

    return 0;
}
