# computed_torque_controller

## computed_torque_controller/ComputedTorqueController
### <u>Formulation</u>
The controller expects a general rigid body dynamics system, whose equations of motion are given in the form below.

<img src="https://latex.codecogs.com/gif.latex?M(q)\ddot{q}+C(q,\dot{q})+g(q)=\tau" />

where <img src="https://latex.codecogs.com/gif.latex?q" />, <img src="https://latex.codecogs.com/gif.latex?M(\cdot)" />, <img src="https://latex.codecogs.com/gif.latex?C(\cdot)" />, and <img src="https://latex.codecogs.com/gif.latex?g(\cdot)" /> are the joint position vector, inertia matrix, Colioris vector, and gravity vector.

The controller computes the joint effort commands <img src="https://latex.codecogs.com/gif.latex?\tau_d" /> to track the desired joint positions <img src="https://latex.codecogs.com/gif.latex?q_d" /> with complensation for the inertia, Colioris, and gravity forces.

<img src="https://latex.codecogs.com/gif.latex?\tau_d=M(q)(\ddot{q}+\textup{PID}(q_d-q))+C(q,\dot{q})+g(q)" />

where <img src="https://latex.codecogs.com/gif.latex?\textup{PID}(\cdot)" /> is the control input to the system based on a PID controller for the position tracking errors.

### <u>Prerequisites</u>
* robot description in URDF to build a dynamics model (using dart::utils::DartLoader)
* hardware_interface/JointStateHandle for all joints appeared in the model
* hardware_interface/EffortJointHandle for controlled joints specified by the controller param

### <u>Subscribed topics</u>
___<controller_namespace>/<joint_name>/command___ (std_msgs/Float64)
* Position command for each controlled joint

### <u>Parameters</u>
___<controller_namespace>/robot_description___ or ___robot_description___ (string, required)
* robot description in URDF

___<controller_namespace>/joints/<joint_name>___ (struct, required)
* control_toolbox::Pid for each controlled joint will be initialized using these parameters

```
# Parameter example
computed_torque_conotroller:
    type: computed_torque_controller/ComputedTorqueController
    joints:
        # Controlled joint 1
        joint_a:
            p: 1.
            i: 1.
            d: 1.
            publish_state: true
        # Controlled joint 2
        joint_b:
            ...
```