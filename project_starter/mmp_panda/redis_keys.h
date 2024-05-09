/**
 * @file redis_keys.h
 * @author William Chong (williamchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai2::sim::mmp_panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::sim::mmp_panda::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::mmp_panda::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::mmp_panda::controller";
