#ifndef GAZEBO_ROS_MIMIC_JOINT_HH
#define GAZEBO_ROS_MIMIC_JOINT_HH

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>

namespace cob_gazebo_ros_control
{
/** \defgroup MimicJoint XML Reference and Example

\brief Ros Gazebo Ros MimicJoint Plugin.
This model allow to mimic a joint without the use of the dynamics

Example Usage:

\verbatim
<gazebo>
  <plugin name="mimic_joint" filename="libgazebo_ros_mimic_joint.so">
    <jointName>joint_name</jointName>
    <mimicJoint>mimic_joint_name</mimicJoint>
    <multiplier>1.0</multiplier>
    <offset>0.0</offset>
    <pgain>1.0</pgain>
  </plugin>
</gazebo>
\endverbatim
\{
*/


/// \brief MimicJoint plugin
/// This plugin allows to simulate mimic joints in gazebo
class MimicJoint : public gazebo::ModelPlugin
{
  /// \brief Load the controller
  public: void Load( gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  /// \brief Update the controller
  public: void OnUpdate();

  /// \brief A pointer to the Gazebo model
  private: gazebo::physics::ModelPtr model_;

  // Pointer to the update event connection
  private: gazebo::event::ConnectionPtr update_connection_;

  /// \brief store bodyname
  private: std::string joint_name_;

  /// \brief store name of mimic joint
  private: std::string mimic_joint_name_;

  /// \brief A pointer to the Gazebo joint
  private: gazebo::physics::JointPtr joint_;
  
  /// \brief A pointer to the Gazebo joint
  private: gazebo::physics::JointPtr mimic_joint_;

  /// \brief multiplier
  private: double multiplier_;

  /// \brief offset
  private: double offset_;
};


} //namespace

#endif
