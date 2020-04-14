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


// cob_gazebo_ros_control
#include <cob_gazebo_ros_control/hwi_switch_robot_hw_sim.h>
#include <boost/algorithm/string/replace.hpp>

namespace cob_gazebo_ros_control
{

bool HWISwitchRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  const ros::NodeHandle joint_limit_nh(model_nh, robot_namespace);

  // Resize vectors to our DOF
  if(enable_joint_filtering_)
  {
    n_dof_ = enabled_joints_.size();
    ROS_INFO_STREAM("JointFiltering is enabled! DoF: "<<n_dof_);
  }
  else
  {
    n_dof_ = transmissions.size();
    ROS_INFO_STREAM("JointFiltering is disabled! DoF: "<<n_dof_);
  }

  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  map_hwinterface_to_joints_.clear();
  map_hwinterface_to_controlmethod_.clear();
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);

  // Initialize values
  unsigned int index = 0;
  for(unsigned int j=0; j < transmissions.size(); j++)
  {
    // Check that this transmission has one joint
    if(transmissions[j].joints_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("hwi_switch_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has no associated joints.");
      continue;
    }
    else if(transmissions[j].joints_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("hwi_switch_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has more than one joint. Currently the default robot hardware simulation "
        << " interface only supports one.");
      continue;
    }

    std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
    if (joint_interfaces.empty() &&
        !(transmissions[j].actuators_.empty()) &&
        !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
    {
      // TODO: Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
      ROS_WARN_STREAM_NAMED("hwi_switch_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
        transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
        "The transmission will be properly loaded, but please update " <<
        "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty())
    {
      ROS_WARN_STREAM_NAMED("hwi_switch_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
        "Not adding it to the robot hardware simulation.");
      continue;
    }
    else if (joint_interfaces.size() > 1)
    {
      ROS_DEBUG_STREAM_NAMED("hwi_switch_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
        "This feature is now available.");
    }

    if(enable_joint_filtering_)
    {
      if(enabled_joints_.find(transmissions[j].joints_[0].name_)!=enabled_joints_.end())
      {
        ROS_DEBUG_STREAM_NAMED("hwi_switch_robot_hw_sim", "Found enabled joint '"<<transmissions[j].joints_[0].name_<<"'; j "<<j<<"; index: "<<index);
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED("hwi_switch_robot_hw_sim", "Joint '"<<transmissions[j].joints_[0].name_<<"' is not enabled; j "<<j<<"; index: "<<index);
        continue;
      }
    }
    else
    {
      index = j;
      ROS_DEBUG_STREAM_NAMED("hwi_switch_robot_hw_sim", "JointFiltering is disabled. Use joint '"<<transmissions[j].joints_[0].name_<<"'; j "<<j<<"; index: "<<index);
    }

    // Add data from transmission
    joint_names_[index] = transmissions[j].joints_[0].name_;
    joint_position_[index] = 1.0;
    joint_velocity_[index] = 0.0;
    joint_effort_[index] = 1.0;  // N/m for continuous joints
    joint_effort_command_[index] = 0.0;
    joint_position_command_[index] = 0.0;
    joint_velocity_command_[index] = 0.0;


    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[index], &joint_position_[index], &joint_velocity_[index], &joint_effort_[index]));

    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointHandle joint_handle;

    // Parse all HW-Interfaces available for each joint and store information
    for(unsigned int i=0; i<joint_interfaces.size(); i++)
    {
      // Debug
      ROS_DEBUG_STREAM_NAMED("hwi_switch_robot_hw_sim","Loading joint '" << joint_names_[index]
        << "' of type '" << joint_interfaces[i] << "'");

      // Deprecated Syntax handling
      std::string& hardware_interface = joint_interfaces[i];
      if(hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface") {
        ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface << "' within the <hardwareInterface> tag in joint '" << joint_names_[index] << "'.");
        hardware_interface = "hardware_interface/"+joint_interfaces[i];
      }

      // Add hardware interface and joint to map of map_hwinterface_to_joints_
      std::string hw_interface_type = boost::algorithm::replace_all_copy(hardware_interface, "/", "::");
      if(map_hwinterface_to_joints_.find(hw_interface_type)!=map_hwinterface_to_joints_.end())
      {
        ROS_DEBUG_STREAM_NAMED("hwi_switch_robot_hw_sim", "HW-Interface " << hw_interface_type << " already registered. Adding joint " << joint_names_[index] << " to list.");
        std::map< std::string, std::set<std::string> >::iterator it;
        it=map_hwinterface_to_joints_.find(hw_interface_type);
        it->second.insert(joint_names_[index]);
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED("hwi_switch_robot_hw_sim", "New HW-Interface registered " << hw_interface_type << ". Adding joint " << joint_names_[index] << " to list.");
        std::set<std::string> supporting_joints;
        supporting_joints.insert(joint_names_[index]);
        map_hwinterface_to_joints_.insert( std::pair< std::string, std::set<std::string> >(hw_interface_type, supporting_joints) );
      }

      if(hw_interface_type == "hardware_interface::EffortJointInterface")
      {
        // Create effort joint interface
        ControlMethod control_method = EFFORT;
        if(i==0){ joint_control_methods_[index] = control_method; } //use first entry for startup
        map_hwinterface_to_controlmethod_.insert( std::pair<std::string, ControlMethod>(hw_interface_type, control_method) );

        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[index]),
                                                      &joint_effort_command_[index]);
        ej_interface_.registerHandle(joint_handle);

        registerJointLimits(joint_names_[index], joint_handle, control_method,
                        joint_limit_nh, urdf_model,
                        &joint_types_[index], &joint_lower_limits_[index], &joint_upper_limits_[index],
                        &joint_effort_limits_[index]);
      }
      else if(hw_interface_type == "hardware_interface::PositionJointInterface")
      {
        // Create position joint interface
        ControlMethod control_method = POSITION;
        if(i==0){ joint_control_methods_[index] = control_method; } //use first entry for startup
        map_hwinterface_to_controlmethod_.insert( std::pair<std::string, ControlMethod>(hw_interface_type, control_method) );

        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[index]),
                                                      &joint_position_command_[index]);
        pj_interface_.registerHandle(joint_handle);

        registerJointLimits(joint_names_[index], joint_handle, control_method,
                        joint_limit_nh, urdf_model,
                        &joint_types_[index], &joint_lower_limits_[index], &joint_upper_limits_[index],
                        &joint_effort_limits_[index]);
      }
      else if(hw_interface_type == "hardware_interface::VelocityJointInterface")
      {
        // Create velocity joint interface
        ControlMethod control_method = VELOCITY;
        if(i==0){ joint_control_methods_[index] = control_method; } //use first entry for startup
        map_hwinterface_to_controlmethod_.insert( std::pair<std::string, ControlMethod>(hw_interface_type, control_method) );

        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[index]),
                                                      &joint_velocity_command_[index]);
        vj_interface_.registerHandle(joint_handle);

        registerJointLimits(joint_names_[index], joint_handle, control_method,
                        joint_limit_nh, urdf_model,
                        &joint_types_[index], &joint_lower_limits_[index], &joint_upper_limits_[index],
                        &joint_effort_limits_[index]);
      }
      else
      {
        ROS_FATAL_STREAM_NAMED("hwi_switch_robot_hw_sim","No matching hardware interface found for '"
          << hardware_interface );
        return false;
      }
    }

    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[index]);
    if (!joint)
    {
      ROS_ERROR_STREAM_NAMED("hwi_switch_robot_hw_sim", "This robot has a joint named \"" << joint_names_[index]
        << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_.push_back(joint);


    // get physics engine type
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif
    physics_type_ = physics->GetType();
    if (physics_type_.empty())
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "No physics type found.");
    }


    // ToDo: Can a joint (gazebo::physics::JointPtr) be used for EFFORT if joint->SetMaxForce has been called before?
    if (joint_control_methods_[index] == VELOCITY || joint_control_methods_[index] == POSITION)
    {
      // joint->SetMaxForce() must be called if joint->SetAngle() or joint->SetVelocity() are
      // going to be called. joint->SetMaxForce() must *not* be called if joint->SetForce() is
      // going to be called.
      #if GAZEBO_MAJOR_VERSION > 2
        joint->SetParam("fmax", 0, joint_effort_limits_[index]);
      #else
        joint->SetMaxForce(0, joint_effort_limits_[index]);
      #endif
    }

    index++;
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  // Initialize the emergency stop code.
  state_valid_ = true;
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  return true;
}

bool HWISwitchRobotHWSim::enableJointFiltering(ros::NodeHandle nh, std::string filter_joints_param)
{
  enabled_joints_.clear();
  enable_joint_filtering_ = false;

  std::vector<std::string> joints;
  if(!nh.getParam(filter_joints_param, joints))
  {
    ROS_ERROR_STREAM_NAMED("hwi_switch_robot_hw_sim", "Parameter '"<<filter_joints_param<<"' not set");
    return false;
  }

  for(std::vector<std::string>::iterator it = joints.begin() ; it != joints.end(); ++it)
  {
    enabled_joints_.insert(*it);
  }

  enable_joint_filtering_ = true;
  return true;
}

bool HWISwitchRobotHWSim::canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const
{
  //for all controllers to be started check whether all resources provide the required hardware_interface
  //conflicts, i.e. different hardware_interfaces for the same resource is checked in checkForConflict()
  //stopped controllers stay in there current mode as there is no no_operation_mode in gazebo
  for (std::list<hardware_interface::ControllerInfo>::const_iterator list_it = start_list.begin(); list_it != start_list.end(); ++list_it)
  {
    for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = list_it->claimed_resources.begin(); res_it != list_it->claimed_resources.end(); ++res_it)
    {
      for(std::set<std::string>::iterator set_it = res_it->resources.begin(); set_it != res_it->resources.end(); ++set_it)
      {
        if(map_hwinterface_to_joints_.at(res_it->hardware_interface).find(*set_it) == map_hwinterface_to_joints_.at(res_it->hardware_interface).end())
        {
          ROS_ERROR_STREAM_NAMED("hwi_switch_robot_hw_sim", "Cannot switch because resource \'" << *set_it << "\' does not provide HW-Interface \'" << res_it->hardware_interface << "\'");
          return false;
        }
      }
    }
  }
  return true;
}

void HWISwitchRobotHWSim::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
{
  //for all controllers to be started
  for (std::list<hardware_interface::ControllerInfo>::const_iterator list_it = start_list.begin(); list_it != start_list.end(); ++list_it)
  {
    for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = list_it->claimed_resources.begin(); res_it != list_it->claimed_resources.end(); ++res_it)
    {
      //for all joints corresponding to this RobotHW
      for(unsigned int i=0; i<joint_names_.size(); i++)
      {
        //if joint is in resource list of controller to be started
        if(res_it->resources.find(joint_names_[i]) != res_it->resources.end())
        {
          if(map_hwinterface_to_controlmethod_.find(res_it->hardware_interface) != map_hwinterface_to_controlmethod_.end())
          {
            ControlMethod current_control_method = map_hwinterface_to_controlmethod_.find(res_it->hardware_interface)->second;

            ///semantic Zero
            joint_position_command_[i] = joint_position_[i];
            joint_velocity_command_[i] = 0.0;
            joint_effort_command_[i] = 0.0;

            ///call setCommand once so that the JointLimitsInterface receive the correct value on their getCommand()!
            try{  pj_interface_.getHandle(joint_names_[i]).setCommand(joint_position_command_[i]);  }
            catch(const hardware_interface::HardwareInterfaceException&){}
            try{  vj_interface_.getHandle(joint_names_[i]).setCommand(joint_velocity_command_[i]);  }
            catch(const hardware_interface::HardwareInterfaceException&){}
            try{  ej_interface_.getHandle(joint_names_[i]).setCommand(joint_effort_command_[i]);  }
            catch(const hardware_interface::HardwareInterfaceException&){}

            ///reset joint_limit_interfaces
            pj_sat_interface_.reset();
            pj_limits_interface_.reset();

            joint_control_methods_[i] = current_control_method;

            ROS_DEBUG_STREAM_NAMED("hwi_switch_robot_hw_sim", "Resource \'" << joint_names_[i] << "\' switched to HW-Interface \'" << res_it->hardware_interface << "\'");
          }
        }
      }
    }
  }
}

void HWISwitchRobotHWSim::stateValid(const bool valid)
{
  state_valid_ = valid;
}

}

PLUGINLIB_EXPORT_CLASS(cob_gazebo_ros_control::HWISwitchRobotHWSim, gazebo_ros_control::RobotHWSim)
