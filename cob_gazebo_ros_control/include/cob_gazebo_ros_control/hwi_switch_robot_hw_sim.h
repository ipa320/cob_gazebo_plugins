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
 

#ifndef _COB_GAZEBO_ROS_CONTROL___HWI_SWITCH_ROBOT_HW_SIM_H_
#define _COB_GAZEBO_ROS_CONTROL___HWI_SWITCH_ROBOT_HW_SIM_H_


// cob_gazebo_ros_control
#include <gazebo_ros_control/default_robot_hw_sim.h>


namespace cob_gazebo_ros_control
{

class HWISwitchRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:

  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual bool enableJointFiltering(ros::NodeHandle nh, std::string filter_joints_param);

  virtual bool canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const;
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);

  virtual void stateValid(const bool active);

protected:

  bool enable_joint_filtering_;
  std::set< std::string > enabled_joints_;

  std::map< std::string, std::set<std::string> > map_hwinterface_to_joints_;
  std::map< std::string, ControlMethod > map_hwinterface_to_controlmethod_;

  bool state_valid_;
};

typedef boost::shared_ptr<HWISwitchRobotHWSim> HWISwitchRobotHWSimPtr;

}

#endif // #ifndef _COB_GAZEBO_ROS_CONTROL___HWI_SWITCH_ROBOT_HW_SIM_H_
