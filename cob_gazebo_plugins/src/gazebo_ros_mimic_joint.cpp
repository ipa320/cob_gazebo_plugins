#include <cob_gazebo_plugins/gazebo_ros_mimic_joint.h>

namespace gazebo
{

MimicJoint::MimicJoint()
{
  this->joint_.reset();
  this->mimic_joint_.reset();
}

MimicJoint::~MimicJoint()
{
  //event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
}

void MimicJoint::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->model_ = _parent;
  this->world_ = this->model_->GetWorld();


  if (!_sdf->HasElement("jointName"))
  {
    ROS_FATAL("mimic_joint missing <jointName>, cannot proceed");
    return;
  }
  else
    this->joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();

  if (!_sdf->HasElement("mimicJoint"))
  {
    ROS_FATAL("mimic_joint missing <mimicJoint>, cannot proceed");
    return;
  }
  else
    this->mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();

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
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");
  this->joint_ = model_->GetJoint(joint_name_);
  this->mimic_joint_ = model_->GetJoint(mimic_joint_name_);
  std::cout << "Plugin model name: " << modelName << ", joint_name: " << joint_name_ << ", mimic_joint_name: " << mimic_joint_name_ << "\n";

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MimicJoint::UpdateChild, this));
}

void MimicJoint::UpdateChild()
{
#if GAZEBO_MAJOR_VERSION >= 8
  const double desired_angle = this->mimic_joint_->Position(0)*this->multiplier_ + this->offset_;
#else
  const double desired_angle = this->mimic_joint_->GetAngle(0).Radian()*this->multiplier_ + this->offset_;
#endif
  std::cout << "desired_angle: " << desired_angle << "\n";
#if GAZEBO_MAJOR_VERSION >= 9
  this->joint_->SetPosition(0, desired_angle, true);
#else
  this->joint_->SetPosition(0, desired_angle);
#endif
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MimicJoint);
} // namespace