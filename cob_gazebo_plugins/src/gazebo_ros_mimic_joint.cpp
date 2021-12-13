#include <cob_gazebo_plugins/gazebo_ros_mimic_joint.h>

namespace cob_gazebo_ros_control
{

void MimicJoint::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  ROS_INFO_NAMED("mimic_joint", "Starting Mimic Joint Plugin");

  this->model_ = _parent;

  if (!_sdf->HasElement("jointName"))
  {
    ROS_FATAL("mimic_joint missing <jointName>, cannot proceed");
    return;
  }
  else
    this->joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  ROS_INFO_NAMED("mimic_joint", "joint_name_: %s", this->joint_name_.c_str() );

  if (!_sdf->HasElement("mimicJoint"))
  {
    ROS_FATAL("mimic_joint missing <mimicJoint>, cannot proceed");
    return;
  }
  else
    this->mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();
  ROS_INFO_NAMED("mimic_joint", "mimic_joint_name_: %s", this->mimic_joint_name_.c_str() );

  if (!_sdf->HasElement("offset"))
  {
    ROS_INFO("mimic_joint missing <offset>, set default to 0.0");
    this->offset_ = 0.0;
  }
  else
    this->offset_ = _sdf->GetElement("offset")->Get<double>();

  if (!_sdf->HasElement("multiplier"))
  {
    ROS_INFO("mimic_joint missing <multiplier>, set default to 1.0");
    this->multiplier_ = 1.0;
  }
  else
    this->multiplier_ = _sdf->GetElement("multiplier")->Get<double>();


  // Get the name of the parent model
  this->joint_ = model_->GetJoint(joint_name_);
  this->mimic_joint_ = model_->GetJoint(mimic_joint_name_);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&MimicJoint::OnUpdate, this));
}

void MimicJoint::OnUpdate()
{
#if GAZEBO_MAJOR_VERSION >= 8
  const double desired_angle = this->mimic_joint_->Position(0)*this->multiplier_ + this->offset_;
#else
  const double desired_angle = this->mimic_joint_->GetAngle(0).Radian()*this->multiplier_ + this->offset_;
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  this->joint_->SetPosition(0, desired_angle, true);
#else
  this->joint_->SetPosition(0, desired_angle);
#endif
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MimicJoint)
} // namespace
