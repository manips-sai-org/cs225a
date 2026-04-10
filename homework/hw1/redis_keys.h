#pragma once

// - read:
const std::string JOINT_ANGLES_KEY = "sai::sensors::PANDA::joint_positions";
const std::string JOINT_VELOCITIES_KEY = "sai::sensors::PANDA::joint_velocities";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai::commands::PANDA::control_torques";
const std::string GRAVITY_COMP_ENABLED_KEY = "sai::simviz::gravity_comp_enabled";
const std::string SIMVIZ_RUNNING_KEY = "sai::simviz::running";