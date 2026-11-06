/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2026 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * QIDI X-Pro / Chitu V4.6 (STM32F407ZE, LQFP144)
 *
 * Pin map compiled from the Chitu V4.6 reverse-engineering notes at
 * https://narfation.org/2022/08/24/klipper-on-qidi-x-max and the
 * QIDI X-Plus / X-Max Klipper configurations.
 *
 * This is a CNC/embroidery build: no hotends, no bed, no stock touchscreen.
 * The Janome handwheel is driven by the board's E1-MOTOR connector, which is
 * mapped to Marlin's logical E0 axis.
 *
 * IMPORTANT: the stock X/Y/Z endstops are at the MAX end of each axis.
 * Homing therefore runs toward X+, Y+ and Z+.
 */

#include "env_validate.h"

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "QIDI_XPRO_V46 supports one logical E stepper (the E1-MOTOR connector)."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "QIDI X-Pro Chitu V4.6"
#endif

#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "QIDI X-Pro CNC"
#endif

// Persist M500 settings in the STM32F407 backup SRAM.
#define SRAM_EEPROM_EMULATION

//
// Limit Switches
//
// Stock QIDI wiring homes to MAX on all three axes.
#define X_MAX_PIN                           PC15  // X- endstop header, wired as X+
#define Y_MAX_PIN                           PC14  // Y- endstop header, wired as Y+
#define Z_MAX_PIN                           PC13  // Z- endstop header, wired as Z+
#define X_MIN_PIN                           -1
#define Y_MIN_PIN                           PE6   // "E-" input (M119: y_min, also aliased as TDC)
#define Z_MIN_PIN                           PE3   // spare "EXT" input (M119: z_min)
#define Z_MIN_PROBE_PIN                     -1

//
// Steppers
//
#define X_STEP_PIN                          PE5
#define X_DIR_PIN                           PF1
#define X_ENABLE_PIN                        PF0

#define Y_STEP_PIN                          PF9
#define Y_DIR_PIN                           PF3
#define Y_ENABLE_PIN                        PF5

#define Z_STEP_PIN                          PA6
#define Z_DIR_PIN                           PF15
#define Z_ENABLE_PIN                        PA5

// Logical E0 is the board's E1-MOTOR connector (first extruder).
#define E0_STEP_PIN                         PB1
#define E0_DIR_PIN                          PF13
#define E0_ENABLE_PIN                       PF14

//
// TDC / needle phase input
//
// The TDC switch goes to the board's E- header (PE6). The stock endstop inputs
// are active-low with a pull-up, so a switch to GND reads TRIGGERED.
// The EXT header (PE3) is left as a spare input and appears as z_min in M119.
#define TDC_PIN                             Y_MIN_PIN
#define TDC_ENDSTOP_INVERTING               true
#define TDC_SEEK_STEP                       1.0f   // degrees/E-units per seek step
#define TDC_SEEK_FEED                       1200   // E-units per minute (20 units/s)

//
// Temperature sensors / heaters (none in this build)
//
#define TEMP_0_PIN                          -1
#define TEMP_BED_PIN                        -1
#define TEMP_CHAMBER_PIN                    -1
#define HEATER_0_PIN                        -1
#define HEATER_BED_PIN                      -1
#define HEATER_CHAMBER_PIN                  -1

//
// Fans (left mapped for future use, not enabled by the CNC config)
//
#define FAN0_PIN                            PD7   // part fan
#define FAN1_PIN                            PG7   // chamber fans
#define FAN2_PIN                            PB6   // extruder heatsink fan

//
// Power / indicators
//
// PE0 must be driven HIGH as early as possible or the QIDI power module
// switches the machine off again. PG10 is the on-board LED (active low).
#define PS_ON_PIN                           PE0
#define LED_PIN                             PG10
#define LED_INVERTING                       true

#define BOARD_INIT() do{ OUT_WRITE(PS_ON_PIN, HIGH); OUT_WRITE(LED_PIN, LOW); }while(0)

//
// Misc
//
#define BEEPER_PIN                          PA8
#define SD_DETECT_PIN                       -1
#define KILL_PIN                            -1
#define FIL_RUNOUT_PIN                      -1
