# Gazebo ros_control Interfaces

This is a ROS package for integrating the `ros_control` controller architecture
with the [Gazebo](http://gazebosim.org/) simulator. 

This package provides a Gazebo plugin which instantiates a ros_control
controller manager and connects it to a Gazebo model.

It extends the default plugin available in [gazebo_ros_control](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/indigo-devel/gazebo_ros_control).  
This plugin has originally been discussed and proposed [here](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/256).

Besides the features provided by the default ```gazebo_ros_control``` plugin, this plugin here adds the following additional features:
 - Support for HardwareInterface-Switching
 - Enable joint filtering

__NOTE__: The control\_methods ```POSITION_PID``` and ```VELOCITY_PID``` are not supported anymore, i.e. PID parameters loaded to the parameter server under namespace ```/gazebo_ros_control/pid_gains/``` are ignored. In case you want to command e.g. positions but want to write efforts to Gazebo, use the ```effort_controllers/JointPositionController``` (or similar) and set your PID values thers. 

--- 
 
## Usage

[Documentation](http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros) related to the default plugin is provided on Gazebo's website.

To use the plugin, add the following to your robot's URDF:

```
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libhwi_switch_gazebo_ros_control.so">
      <robotNamespace>NAMESPACE</robotNamespace>
    </plugin>
  </gazebo>
```

The ```robotNamespace``` is used as a prefix for the ```controller_manager``` instantiated by the plugin.  
The tag is optional and will default to global namespace '/' if not set.  
In the example above the services will be advertised under ```/NAMESPACE/controller_manager```.  
__NOTE__: Do not use a trailing '/' before NAMESPACE!

You can also use the tags ```robotParam``` and ```controlPeriod``` as for the default plugin.

The tag ```robotSimType``` is ignored and defaults to ```cob_gazebo_ros_control/HWISwitchRobotHWSim``` which is a specialized HardwareInterface which is also provided in this package. ```HWISwitchRobotHWSim``` derives from ```DefaultRobotHWSim``` of the default plugin.  

#### Support for HardwareInterface-Switching

With this plugin, you can now specify multiple HardwareInterfaces for the transmissions of your joints, like this:  
```
    <transmission name="arm_1_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="arm_1_joint">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      </joint>
      <actuator name="arm_1_motor">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
```
You can specify any HardwareInterface out of [```PositionJointInterface```, ```VelocityJointInterface```, ```EffortJointInterface```]. The order of does not matter.  

#### Enable joint filtering

The default ```gazebo_ros_control``` plugin creates JointHandles for all the joints present in your URDF. In order to only assign a specific set of joints to one plugin - and then use several plugins under different ```robotNamespaces``` - you can use the new tag ```filterJointsParam```.

```
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libhwi_switch_gazebo_ros_control.so">
      <robotNamespace>NAMESPACE</robotNamespace>
      <filterJointsParam>joint_names</filterJointsParam>
    </plugin>
  </gazebo>
```

The plugin will only create JointHandles given in the list loaded to the parameter server under ```/NAMESPACE/joint_names```. In case the parameter cannot be found, the plugin fails to load.

The ```joint_names``` parameter might look like this:  
```joint_names: [arm_1_joint, arm_2_joint, arm_3_joint, arm_4_joint, arm_5_joint, arm_6_joint, arm_7_joint]```



--- 
 
## Benefit

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
