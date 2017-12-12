/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


// CobGazeboRosControlPlugin
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <cob_gazebo_ros_control/hwi_switch_robot_hw_sim.h>


namespace cob_gazebo_ros_control
{

class HWISwitchGazeboRosControlPlugin : public gazebo_ros_control::GazeboRosControlPlugin
{
public:

  // Overloaded Gazebo entry point
  virtual void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

  // Called by the world update start event
  void Update();

protected:
  void eStopCB(const std_msgs::BoolConstPtr& e_stop_active);
  void stateValidCB(const std_msgs::BoolConstPtr& state_valid);

  bool enable_joint_filtering_;
  std::string filterJointsParam_;

  boost::shared_ptr<cob_gazebo_ros_control::HWISwitchRobotHWSim> hwi_switch_robot_hw_sim_;

  bool state_valid_;
  ros::Subscriber state_valid_sub_;
};


}
