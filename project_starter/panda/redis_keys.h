/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai::sim::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai::sim::panda::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai::sim::panda::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai::sim::panda::controller";

const std::string GRIPPER_JOINT_ANGLES_KEY = "sai::sim::panda_gripper::sensors::q";
const std::string GRIPPER_JOINT_VELOCITIES_KEY = "sai::sim::panda_gripper::sensors::dq";
