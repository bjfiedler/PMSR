rostopic pub -1 /arm_1/gripper_controller/position_command brics_actuator/JointPositions '{positions: [
{joint_uri: gripper_finger_joint_r, unit: m, value: 0.01},
{joint_uri: gripper_finger_joint_l, unit: m, value: 0.01}
]}'
