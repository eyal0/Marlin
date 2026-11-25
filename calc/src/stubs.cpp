// Definitions for symbols the linked Marlin code references but marlin-calc
// doesn't use.

#include "src/MarlinCore.h"
#include "src/HAL/shared/Delay.h"
#include "src/gcode/gcode.h"
#include "src/lcd/marlinui.h"
#include "src/module/endstops.h"
#include "src/module/stepper.h"
#include "src/module/temperature.h"

MarlinState Marlin::state = MF_RUNNING;
bool Marlin::wait_for_heatup;
PGMSTR(M112_KILL_STR, "M112 Shutdown");

void Marlin::kill(FSTR_P const lcd_error, FSTR_P const, const bool) {
  fprintf(stderr, "Marlin killed: %s\n", lcd_error ? FTOP(lcd_error) : "");
  exit(1);
}

// Handlers for commands that don't affect print time. main.cpp handles G4
// and G28 itself.
void GcodeSuite::G4() {}
void GcodeSuite::G28() {}
void GcodeSuite::M17() {}
void GcodeSuite::M18_M84() {}
void GcodeSuite::M31() {}
void GcodeSuite::M75() {}
void GcodeSuite::M76() {}
void GcodeSuite::M77() {}
void GcodeSuite::M81() {}
void GcodeSuite::M85() {}
void GcodeSuite::M104_M109(const bool) {}
void GcodeSuite::M105() {}
void GcodeSuite::M106() {}
void GcodeSuite::M107() {}
void GcodeSuite::M108() {}
void GcodeSuite::M110() {}
void GcodeSuite::M111() {}
void GcodeSuite::M112() {}
void GcodeSuite::M113() {}
void GcodeSuite::M114() {}
void GcodeSuite::M115() {}
void GcodeSuite::M118() {}
void GcodeSuite::M119() {}
void GcodeSuite::M120() {}
void GcodeSuite::M121() {}
void GcodeSuite::M140_M190(const bool) {}
void GcodeSuite::M155() {}
void GcodeSuite::M206() {}
void GcodeSuite::M217() {}
void GcodeSuite::T(const int8_t) {}
void GcodeSuite::M301() {}
void GcodeSuite::M303() {}
void GcodeSuite::M410() {}
void GcodeSuite::M428() {}
void GcodeSuite::M500() {}
void GcodeSuite::M501() {}
void GcodeSuite::M502() {}
void GcodeSuite::M503() {}
void GcodeSuite::M876() {}
void GcodeSuite::M999() {}

// Marlin::idle() consumes planner blocks in place of the stepper module.
bool Stepper::abort_current_block;
bool Stepper::is_block_busy(const block_t * const) { return false; }
int32_t Stepper::position(const AxisEnum) { return 0; }
void Stepper::set_position(const xyze_long_t &) {}
void Stepper::set_e_position(const int32_t &) {}
void Stepper::report_a_position(const xyz_long_t &) {}
void Stepper::report_positions() {}
void Stepper::endstop_triggered(const AxisEnum) {}
int32_t Stepper::triggered_position(const AxisEnum) { return 0; }
void Stepper::enable_axis(const AxisEnum) {}
void Stepper::enable_extruder(E_TERN_(const uint8_t)) {}
bool Stepper::disable_extruder(E_TERN_(const uint8_t)) { return true; }
void Stepper::disable_all_steppers() {}

void Endstops::validate_homing_move() {}
void Temperature::task() {}
void MarlinUI::status_printf_P(int8_t, PGM_P const, ...) {}
void MarlinUI::_set_status(const char * const, const bool, const bool) {}

#if defined(__arm__) || defined(__thumb__)
  // Delay.h takes 32-bit ARM builds for microcontrollers.
  DelayImpl DelayCycleFnc = [](uint32_t) {};
#endif

void HAL_timer_enable_interrupt(const uint8_t) {}
void HAL_timer_disable_interrupt(const uint8_t) {}
bool HAL_timer_interrupt_enabled(const uint8_t) { return false; }
