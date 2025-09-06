#pragma once

#include "swerve_structs.hpp"
#include "vector2d.hpp"
#include <libhal/servo.hpp>
#include <libhal/units.hpp>

namespace sjsu::drive {

struct swerve_module_settings
{
  vector2d position;
  meters_per_sec max_speed;
  meters_per_sec_per_sec acceleration;
  deg_per_sec turn_speed;
  hal::degrees min_angle;
  hal::degrees max_angle;
  hal::degrees position_tolerance;
  meters_per_sec velocity_tolerance;
  sec tolerance_timeout;
};

class swerve_module
{
public:
  swerve_module_settings settings;

  // vars:
  // steer postion read & control (maybe also contains restrictions) (has homing
  // function) prop vel read & control (maybe also contrans restrictions)
  // settings (positions & maybe constraints of motors if not included)
  // most recent measurements (so it don't send a read command every time)

  // methods:
  // calc target state for vector (closest angle for valid state)
  // set target state
  void set_target_state(swerve_module_state p_target_state);
  bool can_reach_state(swerve_module_state p_state);

  // reads state cache
  swerve_module_state get_actual_state_cache();
  // refresh state cache
  swerve_module_state refresh_actual_state_cache();

  //get target state
  swerve_module_state get_target_state();
  //update tolorance debouncer
  void update_tolerance_debouncer();
  //tolorance timeout
  bool tolerance_timed_out();
};
}  // namespace sjsu::drive