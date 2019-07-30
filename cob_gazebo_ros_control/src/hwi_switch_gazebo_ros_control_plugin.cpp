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


#include <cob_gazebo_ros_control/hwi_switch_gazebo_ros_control_plugin.h>
#include <urdf/model.h>

namespace cob_gazebo_ros_control
{

// Overloaded Gazebo entry point
void HWISwitchGazeboRosControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  ROS_INFO_STREAM_NAMED("cob_gazebo_ros_control","Loading cob_gazebo_ros_control plugin");
  enable_joint_filtering_ = false;

  // Save pointers to the model
  parent_model_ = parent;
  sdf_ = sdf;

  // Error message if the model couldn't be found
  if (!parent_model_)
  {
    ROS_ERROR_STREAM_NAMED("cob_gazebo_ros_control","parent model is NULL");
    return;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("cob_gazebo_ros_control","A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Get namespace for nodehandle
  if(sdf_->HasElement("robotNamespace"))
  {
    robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
  }
  else
  {
    robot_namespace_ = parent_model_->GetName(); // default
  }

  // Get robot_description ROS param name
  if (sdf_->HasElement("robotParam"))
  {
    robot_description_ = sdf_->GetElement("robotParam")->Get<std::string>();
  }
  else
  {
    robot_description_ = "robot_description"; // default
  }

  // Get the robot simulation interface type
  if(sdf_->HasElement("robotSimType"))
  {
    //robot_hw_sim_type_str_ = sdf_->Get<std::string>("robotSimType");
    robot_hw_sim_type_str_ = "cob_gazebo_ros_control/HWISwitchRobotHWSim";
    ROS_WARN_STREAM_NAMED("cob_gazebo_ros_control","Tag 'robotSimType' is currently ignored. Using default plugin for RobotHWSim \""<<robot_hw_sim_type_str_<<"\"");
  }
  else
  {
    robot_hw_sim_type_str_ = "cob_gazebo_ros_control/HWISwitchRobotHWSim";
    ROS_DEBUG_STREAM_NAMED("cob_gazebo_ros_control","Using default plugin for RobotHWSim (none specified in URDF/SDF)\""<<robot_hw_sim_type_str_<<"\"");
  }

  // Get the Gazebo simulation period
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif
  ros::Duration gazebo_period(physics->GetMaxStepSize());

  // Decide the plugin control period
  if(sdf_->HasElement("controlPeriod"))
  {
    control_period_ = ros::Duration(sdf_->Get<double>("controlPeriod"));

    // Check the period against the simulation period
    if( control_period_ < gazebo_period )
    {
      ROS_ERROR_STREAM_NAMED("cob_gazebo_ros_control","Desired controller update period ("<<control_period_
        <<" s) is faster than the gazebo simulation period ("<<gazebo_period<<" s).");
    }
    else if( control_period_ > gazebo_period )
    {
      ROS_WARN_STREAM_NAMED("cob_gazebo_ros_control","Desired controller update period ("<<control_period_
        <<" s) is slower than the gazebo simulation period ("<<gazebo_period<<" s).");
    }
  }
  else
  {
    control_period_ = gazebo_period;
    ROS_DEBUG_STREAM_NAMED("cob_gazebo_ros_control","Control period not found in URDF/SDF, defaulting to Gazebo period of "
      << control_period_);
  }

  // Get parameters/settings for controllers from ROS param server
  model_nh_ = ros::NodeHandle(robot_namespace_);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;
  if (sdf_->HasElement("eStopTopic"))
  {
    const std::string e_stop_topic = sdf_->GetElement("eStopTopic")->Get<std::string>();
    e_stop_sub_ = model_nh_.subscribe(e_stop_topic, 1, &HWISwitchGazeboRosControlPlugin::eStopCB, this);
  }

  // Initialize the code to handle state_valid.
  state_valid_ = true;
  if (sdf_->HasElement("stateValidTopic"))
  {
    const std::string state_valid_topic = sdf_->GetElement("stateValidTopic")->Get<std::string>();
    state_valid_sub_ = model_nh_.subscribe(state_valid_topic, 1, &HWISwitchGazeboRosControlPlugin::stateValidCB, this);
  }

  // Determine whether to filter joints
  if(sdf_->HasElement("filterJointsParam"))
  {
    enable_joint_filtering_ = true;
    filterJointsParam_ = sdf_->Get<std::string>("filterJointsParam");
    ROS_INFO_STREAM_NAMED("cob_gazebo_ros_control","Enable joint-filtering. Using joint_names from parameter: \""<<filterJointsParam_<<"\"");
  }
  else
  {
    enable_joint_filtering_ = false;
    filterJointsParam_ = "";
    ROS_INFO_STREAM_NAMED("cob_gazebo_ros_control","Joint-filtering is disabled. The plugin will set up JointHandles for all joints!");
  }

  ROS_INFO_NAMED("cob_gazebo_ros_control", "Starting cob_gazebo_ros_control plugin in namespace: %s", robot_namespace_.c_str());

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  const std::string urdf_string = getURDF(robot_description_);
  if (!parseTransmissionsFromURDF(urdf_string))
  {
    ROS_ERROR_NAMED("cob_gazebo_ros_control", "Error parsing URDF in cob_gazebo_ros_control plugin, plugin not active.\n");
    return;
  }

  // Load the RobotHWSim abstraction to interface the controllers with the gazebo model
  try
  {
    //robot_hw_sim_loader_.reset
      //(new pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim>
        //("gazebo_ros_control",
          //"gazebo_ros_control::RobotHWSim"));

    //robot_hw_sim_ = robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str_);

    hwi_switch_robot_hw_sim_.reset(new cob_gazebo_ros_control::HWISwitchRobotHWSim());


    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    if(enable_joint_filtering_)
    {
      if(!hwi_switch_robot_hw_sim_->enableJointFiltering(model_nh_, filterJointsParam_))
      {
        ROS_FATAL_STREAM_NAMED("cob_gazebo_ros_control","Joint-filtering was enabled but param '"<<filterJointsParam_<<"' was not found");
        return;
      }
    }

    if(!hwi_switch_robot_hw_sim_->initSim(robot_namespace_, model_nh_, parent_model_, urdf_model_ptr, transmissions_))
    {
      ROS_FATAL_NAMED("cob_gazebo_ros_control","Could not initialize robot simulation interface");
      return;
    }

    // Create the controller manager
    ROS_DEBUG_STREAM_NAMED("cob_gazebo_ros_control","Loading controller_manager");
    controller_manager_.reset
      (new controller_manager::ControllerManager(hwi_switch_robot_hw_sim_.get(), model_nh_));

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin
      (boost::bind(&HWISwitchGazeboRosControlPlugin::Update, this));

  }
  catch(pluginlib::LibraryLoadException &ex)
  {
    ROS_FATAL_STREAM_NAMED("cob_gazebo_ros_control","Failed to create robot simulation interface loader: "<<ex.what());
  }

  ROS_INFO_NAMED("cob_gazebo_ros_control", "Loaded cob_gazebo_ros_control.");
}

// Called by the world update start event
void HWISwitchGazeboRosControlPlugin::Update()
{
  // Get the simulation time and period
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time gz_time_now = gazebo::physics::get_world()->SimTime();
#else
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
#endif
  ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  hwi_switch_robot_hw_sim_->eStopActive(e_stop_active_);
  hwi_switch_robot_hw_sim_->stateValid(state_valid_);
  e_stop_active_ = e_stop_active_ || not state_valid_;

  // Check if we should update the controllers
  if(sim_period >= control_period_) {
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    // Update the robot simulation with the state of the gazebo model
    hwi_switch_robot_hw_sim_->readSim(sim_time_ros, sim_period);

    // Compute the controller commands
    bool reset_ctrlrs;
    if (e_stop_active_)
    {
      reset_ctrlrs = false;
      last_e_stop_active_ = true;
    }
    else
    {
      if (last_e_stop_active_)
      {
        reset_ctrlrs = true;
        last_e_stop_active_ = false;
      }
      else
      {
        reset_ctrlrs = false;
      }
    }
    controller_manager_->update(sim_time_ros, sim_period, reset_ctrlrs);
  }

  // Update the gazebo model with the result of the controller
  // computation
  hwi_switch_robot_hw_sim_->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  last_write_sim_time_ros_ = sim_time_ros;
}

// Emergency stop callback
void HWISwitchGazeboRosControlPlugin::eStopCB(const std_msgs::BoolConstPtr& e_stop_active)
{
  e_stop_active_ = e_stop_active->data;
}

// State valid callback
void HWISwitchGazeboRosControlPlugin::stateValidCB(const std_msgs::BoolConstPtr& state_valid)
{
  state_valid_ = state_valid->data;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HWISwitchGazeboRosControlPlugin);
} // namespace
