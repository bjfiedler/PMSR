rostopic pub -1 /arm_controller/position_command  brics_actuator/JointPositions '{positions: [
{joint_uri: arm_joint_1, unit: rad, value: 3.0},
{joint_uri: arm_joint_2, unit: rad, value: 0.1},
{joint_uri: arm_joint_3, unit: rad, value: -0.8},
{joint_uri: arm_joint_4, unit: rad, value: 3.3},
{joint_uri: arm_joint_5, unit: rad, value: 3.0}
]}' 


