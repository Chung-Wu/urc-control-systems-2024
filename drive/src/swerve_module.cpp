#include "../include/swerve_module.hpp"

namespace sjsu::drive {
swerve_module::swerve_module(swerve_module_settings const& p_settings)
  : settings(p_settings)
{
}

void swerve_module::set_target_state(swerve_module_state p_target_state)
{
  target_state.steer_angle = p_target_state.steer_angle;
  target_state.p_target_state.propulsion_velocity;
}
}  // namespace sjsu::drive