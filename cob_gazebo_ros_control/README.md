# Gazebo ros_control Interfaces

This is a ROS package for integrating the `ros_control` controller architecture
with the [Gazebo](http://gazebosim.org/) simulator. 

This package provides a Gazebo plugin which instantiates a ros_control
controller manager and connects it to a Gazebo model.

It extends the default plugin available in [gazebo_ros_control](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/indigo-devel/gazebo_ros_control).  
This plugin has originally been discussed and proposed [here](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/256).

Besides the features provided by the default ```gazebo_ros_control``` plugin, this plugin here adds the following additional features:
 - Support for multiple HardwareInterfaces
 
 
## Usage

[Documentation](http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros) related to the default plugin is provided on Gazebo's website.

To use the plugin, add the following to your robot's URDF:

```
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libmulti_hw_interface_gazebo_ros_control.so">
      <robotNamespace>NAMESPACE</robotNamespace>
    </plugin>
  </gazebo>
```

The ```robotNamespace``` is used as a prefix for the ```controller_manager``` instantiated by the plugin.  
The tag is optional and will default to global namespace '/' if not set.  
In the example above the services will be advertised under ```/NAMESPACE/controller_manager```.  
__NOTE__: Do not use a trailing '/' before NAMESPACE!

You can also use the tags ```robotParam``` and ```controlPeriod``` as for the default plugin.

The tag ```robotSimType``` is ignored and defaults to ```cob_gazebo_ros_control/MultiHWInterfaceRobotHWSim``` which is a specialized HardwareInterface which is also provided in this package. ```MultiHWInterfaceRobotHWSim``` derives from ```DefaultRobotHWSim``` of the default plugin.  

With this plugin, you can now specify multiple HardwareInterfaces for the transmissions of your joints, like this:  
```
    <transmission name="arm_1_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="arm_1_joint">
        <hardwareInterface>PositionJointInterface</hardwareInterface>
        <hardwareInterface>VelocityJointInterface</hardwareInterface>
      </joint>
      <actuator name="arm_1_motor">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
```
You can specify any HardwareInterface out of [```PositionJointInterface```, ```VelocityJointInterface```, ```EffortJointInterface```]. The order of does not matter.  

---

This plugin allows to use ros\_controllers requiring different HardwareInterfaces within the same gazebo session.  
(No need to change the URDF for a different control_mode).

For example:
```
#position controller
arm_1_joint_position_controller:
  type: position_controllers/JointPositionController
  joint: arm_1_joint

#velocity controller
arm_1_joint_velocity_controller:
  type: velocity_controllers/JointVelocityController
  joint: arm_1_joint
```

can be loaded to the parameter server (under the NAMESPACE of the controller\_manager) and then those two controllers can be switched using the ControllerManagers ```switch_controller```-Service "on-the-fly".

A side-benefit of this is that no "PID-Parameter-Tuning" would be required for simulation anymore, as gazebo supports all the (Standard-)HardwareInterfaces. 
