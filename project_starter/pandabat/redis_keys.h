/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai2::sim::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::sim::panda::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::panda::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::panda::controller";

const std::string BAT_JOINT_ANGLES_KEY = "sai2::sim::panda_bat::sensors::q";
const std::string BAT_JOINT_VELOCITIES_KEY = "sai2::sim::panda_bat::sensors::dq";

const std::string RIGID_BODY_POS = "sai2::optitrack::rigid_body_pos::1";
const std::string RIGID_BODY_ORI = "sai2::optitrack::rigid_body_ori::1";

const std::string BALL_TRAJECTORY = "sai2::sim::panda_bat::dynamic_object::ball_trajectory";
const std::string NEW_BALL_SIGNAL = "sai2::sim::panda_bat::dynamic_object::new_ball_signal";

const std::string BALL_POS = "sai2::sim::panda_bat::dynamic_object::ball_pos";
const std::string BALL_VEL = "sai2::sim::panda_bat::dynamic_object::ball_vel";

const std::string BUTTON_LAST_STATE = "sai2::sim::panda_bat::dynamic_object::ball_button";
const std::string CONTROLLER_STATE = "sai2::sim::panda_bat::dynamic_object::controller_state";
