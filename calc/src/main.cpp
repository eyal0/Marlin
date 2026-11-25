// marlin-calc: estimates how long Marlin takes to run a G-code file.
//
// Commands go through Marlin's own parser, G-code handlers and planner. In
// place of the stepper, Marlin::idle(), which the planner calls while it waits
// for room, takes the next planned block and adds up the time its trapezoid
// takes.

#include "src/MarlinCore.h"
#include "src/feature/fwretract.h"
#include "src/gcode/gcode.h"
#include "src/gcode/parser.h"
#include "src/module/motion.h"
#include "src/module/planner.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

Marlin marlin;

namespace {

struct BlockSource {
  double filepos;   // Fraction of the file read when the block was queued.
  double filament;  // Filament extruded before the block, in mm.
};

double current_filepos;
double filament_queued;
BlockSource block_source[BLOCK_BUFFER_SIZE];
bool block_recorded[BLOCK_BUFFER_SIZE];

double total_time;
double filament_used[EXTRUDERS];
double position[NUM_AXES];  // Where executed blocks have moved the head, in mm.
double area_min[NUM_AXES], area_max[NUM_AXES];

double extruded_mm(const block_t &block) {
  const double mm = block.steps.e / planner.settings.axis_steps_per_mm[E_AXIS_N(block.extruder)];
  return block.direction_bits.e ? mm : -mm;
}

void record_new_blocks() {
  for (uint8_t i = planner.block_buffer_tail; i != planner.block_buffer_head; i = (i + 1) % BLOCK_BUFFER_SIZE) {
    if (block_recorded[i]) continue;
    block_source[i] = { current_filepos, filament_queued };
    filament_queued += extruded_mm(planner.block_buffer[i]);
    block_recorded[i] = true;
  }
}

double block_time(const block_t &block) {
  const double accelerating = block.accelerate_before,
               cruising = double(block.decelerate_start) - block.accelerate_before,
               decelerating = double(block.step_event_count) - block.decelerate_start;
  return accelerating * 2 / (block.initial_rate + block.cruise_rate)
       + cruising / block.cruise_rate
       + decelerating * 2 / (block.cruise_rate + block.final_rate);
}

void track_motion(const block_t &block) {
  bool moved = false;
  LOOP_NUM_AXES(i) {
    const double mm = block.steps[i] / planner.settings.axis_steps_per_mm[i];
    position[i] += block.direction_bits[i] ? mm : -mm;
    moved |= block.steps[i] != 0;
  }
  const double extruded = extruded_mm(block);
  filament_used[block.extruder] += extruded;
  if (extruded && moved) {
    LOOP_NUM_AXES(i) {
      area_min[i] = _MIN(area_min[i], position[i]);
      area_max[i] = _MAX(area_max[i], position[i]);
    }
  }
}

// HalSerial::write() waits for room in its buffer, so something must read it.
std::atomic<bool> serial_open{true};
void forward_serial_to_stderr() {
  while (serial_open || usb_serial.transmit_buffer.available()) {
    while (usb_serial.transmit_buffer.available()) fputc(usb_serial.transmit_buffer.read(), stderr);
    std::this_thread::yield();
  }
}

// What MarlinSettings::reset() and postprocess() do for the compiled features.
void reset_settings() {
  constexpr float steps[] = DEFAULT_AXIS_STEPS_PER_UNIT, feedrates[] = DEFAULT_MAX_FEEDRATE;
  constexpr uint32_t accelerations[] = DEFAULT_MAX_ACCELERATION;
  for (uint8_t i = 0; i < DISTINCT_AXES; ++i) {
    planner.settings.axis_steps_per_mm[i] = steps[_MIN(i, COUNT(steps) - 1)];
    planner.settings.max_feedrate_mm_s[i] = feedrates[_MIN(i, COUNT(feedrates) - 1)];
    planner.settings.max_acceleration_mm_per_s2[i] = accelerations[_MIN(i, COUNT(accelerations) - 1)];
  }
  planner.settings.min_segment_time_us = DEFAULT_MINSEGMENTTIME;
  planner.settings.acceleration = DEFAULT_ACCELERATION;
  planner.settings.retract_acceleration = DEFAULT_RETRACT_ACCELERATION;
  planner.settings.travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  planner.settings.min_feedrate_mm_s = DEFAULT_MINIMUMFEEDRATE;
  planner.settings.min_travel_feedrate_mm_s = DEFAULT_MINTRAVELFEEDRATE;
  planner.max_jerk.set(DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK);
  planner.max_jerk.e = DEFAULT_EJERK;
  planner.junction_deviation_mm = JUNCTION_DEVIATION_MM;
  fwretract.reset();
  parser.volumetric_enabled = ENABLED(VOLUMETRIC_DEFAULT_ON);
  for (auto &diameter : planner.filament_size) diameter = DEFAULT_NOMINAL_FILAMENT_DIA;
  planner.set_advance_k(ADVANCE_K);

  planner.refresh_acceleration_rates();
  planner.calculate_volumetric_multipliers();
  TERN_(HAS_LINEAR_E_JERK, planner.recalculate_max_e_jerk());
  planner.refresh_positioning();
}

// G4, without waiting in real time.
void dwell() {
  millis_t ms = 0;
  if (parser.seenval('P')) ms = parser.value_millis();
  if (parser.seenval('S')) ms = parser.value_millis_from_seconds();
  planner.synchronize();
  total_time += ms / 1000.0;
}

// Marlin's command queue strips comments and checksums before parsing.
bool parse(std::string &command) {
  command.erase(std::min(command.find_first_of(";*"), command.size()));
  if (command.find_first_not_of(" \t\r") == std::string::npos) return false;
  parser.parse(command.data());
  return true;
}

// The printer's M503 report shows which algorithm its firmware uses: M205
// reports J with Junction Deviation and X/Y/Z/E with Classic Jerk.
void select_jerk_mode(const std::string &settings) {
  std::istringstream lines(settings);
  for (std::string line; std::getline(lines, line);) {
    if (!parse(line) || !parser.is_command('M', 205)) continue;
    if (parser.seen('J')) planner.classic_jerk = false;
    else if (parser.seen("XYZE")) planner.classic_jerk = true;
  }
}

void process(std::string command) {
  if (!parse(command)) return;
  switch (parser.command_letter) {
    case 'G':
      if (parser.codenum == 4) return dwell();
      if (parser.codenum == 28) {  // Homing isn't simulated.
        total_time += 5;
        return;
      }
      break;
    case 'T':  // Tool changes switch extruders without moving.
      if (WITHIN(parser.codenum, 0, EXTRUDERS - 1)) motion.extruder = parser.codenum;
      return;
  }
  gcode.process_parsed_command(true);
}

void print_analysis() {
  printf("Analysis: {\"estimatedPrintTime\": %.17f, ", total_time);
  printf("\"printingArea\": {\"maxX\": %.17f, \"maxY\": %.17f, \"maxZ\": %.17f, "
         "\"minX\": %.17f, \"minY\": %.17f, \"minZ\": %.17f}, ",
         area_max[X_AXIS], area_max[Y_AXIS], area_max[Z_AXIS],
         area_min[X_AXIS], area_min[Y_AXIS], area_min[Z_AXIS]);
  printf("\"dimensions\": {\"width\": %.17f, \"depth\": %.17f, \"height\": %.17f}, ",
         area_max[X_AXIS] - area_min[X_AXIS], area_max[Y_AXIS] - area_min[Y_AXIS],
         area_max[Z_AXIS] - area_min[Z_AXIS]);
  printf("\"filament\": {");
  const char *separator = "";
  for (uint8_t e = 0; e < EXTRUDERS; ++e) {
    if (!filament_used[e]) continue;
    const double radius = planner.filament_size[e] / 2;
    printf("%s\"tool%d\": {\"length\": %.17f, \"volume\": %.17f}", separator, e,
           filament_used[e], filament_used[e] * M_PI * radius * radius / 1000);
    separator = ", ";
  }
  printf("}}\n");
}

}  // namespace

void Marlin::idle(const bool) {
  record_new_blocks();
  const block_t * const block = planner.get_current_block();
  if (!block) return;
  const uint8_t index = planner.block_buffer_tail;
  if (block->step_event_count) {
    total_time += block_time(*block);
    printf("Progress: %.17f, %.17f, %.17f\n",
           block_source[index].filepos, block_source[index].filament, total_time);
    track_motion(*block);
  }
  block_recorded[index] = false;
  planner.release_current_block();
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s <G-code file> [settings, one command per line]\n", argv[0]);
    return 2;
  }
  std::ifstream in(argv[1], std::ifstream::binary);
  if (!in) {
    fprintf(stderr, "Can't read %s\n", argv[1]);
    return 1;
  }
  in.seekg(0, in.end);
  const double size = in.tellg();
  in.seekg(0, in.beg);

  std::thread serial_reader(forward_serial_to_stderr);
  LOOP_NUM_AXES(i) {
    area_min[i] = INFINITY;
    area_max[i] = -INFINITY;
  }
  reset_settings();
  motion.sync_plan_position();

  if (argc > 2) {
    select_jerk_mode(argv[2]);
    std::istringstream settings(argv[2]);
    for (std::string line; std::getline(settings, line);) process(line);
  }
  for (std::string line; std::getline(in, line);) {
    const auto pos = in.tellg();
    current_filepos = (pos < 0 ? size : double(pos)) / size;
    process(line);
    record_new_blocks();
  }
  while (planner.has_blocks_queued()) marlin.idle();

  serial_open = false;
  serial_reader.join();
  print_analysis();
  return 0;
}
