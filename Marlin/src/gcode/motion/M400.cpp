/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/motion.h"
#include "../../module/stepper.h"
#include "../../MarlinCore.h"
#include "../../core/serial.h"

/**
 * M400: Finish all moves
 */
void GcodeSuite::M400() {

  planner.synchronize();

}

/**
 * M4000-M4002: needle top-dead-centre (TDC) helpers for the QIDI X-Pro CNC
 *
 *   M4000 [S<ms>] [K<0|1>]  Wait for the TDC switch. With K1 (default) a
 *                           timeout halts the machine instead of letting the
 *                           next XY move run with the needle down.
 *   M4001                   Report TDC state: "TDC:<0|1> PIN:<0|1>".
 *   M4002 [E<max>] [F<feed>] [P<step>] [D<dir>] [S<ms>] [K<0|1>]
 *                           Advance E until TDC triggers, at most E units.
 *                           E defaults to 400, F to TDC_SEEK_FEED (mm/min),
 *                           P to TDC_SEEK_STEP, D to +1, S to 60000 ms.
 *                           A timeout reports TDC:0 by default (K0) so the
 *                           host can pause; K1 halts the firmware.
 *
 * Typical stitch sequence:
 *   M4002 E400 F1200      ; rotate the handwheel until the needle is at TDC
 *   M4000 S10000          ; safety wait (normally returns immediately)
 *   G1 X... Y... F...     ; move the hoop only while the needle is up
 *   G1 E360 F600          ; one stitch
 */

#if PIN_EXISTS(TDC)
  volatile bool tdc_seek_active = false;
  volatile bool tdc_seek_hit = false;
  // Latched when a seek fails. While set, all G0/G1 motion and further seeks
  // are ignored so a jam or slipping clutch cannot grind through the rest of
  // the queued stitches. Clear with M4003.
  bool tdc_fault_latched = false;

  static inline bool tdc_triggered() {
    return READ(TDC_PIN) == (TDC_ENDSTOP_INVERTING ? LOW : HIGH);
  }
#endif

/**
 * M4000: Wait for the TDC switch
 */
void GcodeSuite::M4000() {
  #if PIN_EXISTS(TDC)
    const millis_t timeout = parser.ulongval('S', 10000);
    const bool kill_on_timeout = parser.boolval('K', true);
    const millis_t start = millis();

    while (!tdc_triggered()) {
      if (ELAPSED(millis(), start, timeout)) {
        SERIAL_ECHOLNPGM("TDC:0");
        if (kill_on_timeout) marlin.kill(F("TDC wait timeout"));
        return;
      }
      marlin.idle();
    }

    SERIAL_ECHOLNPGM("TDC:1");
  #else
    SERIAL_ERROR_MSG("TDC_PIN not configured");
  #endif
}

/**
 * M4001: Report the TDC switch state
 */
void GcodeSuite::M4001() {
  #if PIN_EXISTS(TDC)
    SERIAL_ECHOPGM("TDC:");
    SERIAL_ECHO(tdc_triggered() ? 1 : 0);
    SERIAL_ECHOPGM(" PIN:");
    SERIAL_ECHOLN(READ(TDC_PIN));
  #else
    SERIAL_ERROR_MSG("TDC_PIN not configured");
  #endif
}

/**
 * M4002: Seek TDC with one continuous E move, stopped by the stepper ISR
 */
void GcodeSuite::M4002() {
  #if PIN_EXISTS(TDC)
    // A latched fault blocks all seek motion until M4003 clears it. Report the
    // latch (not TDC:0) so queued blocked seeks don't look like new faults.
    if (tdc_fault_latched) {
      SERIAL_ECHOLNPGM("TDC_FAULT:1");
      return;
    }

    const float max_e = _MAX(0.1f, parser.floatval('E', 400.0f));
    const float feed_mm_min = parser.floatval('F', TDC_SEEK_FEED);
    const float dir = (parser.seenval('D') && parser.value_float() < 0) ? -1.0f : 1.0f;
    const millis_t timeout = parser.ulongval('S', 60000);
    // Do not halt the machine by default: report TDC:0 so the host can pause
    // the job, clear the clutch/thread, and resume. Use K1 for a hard halt.
    const bool kill_on_timeout = parser.boolval('K', false);

    if (tdc_triggered()) {
      SERIAL_ECHOLNPGM("TDC:1");
      return;
    }

    const float start_e = motion.position.e;

    // G91 sets the per-axis E mode, which overrides motion.relative_mode.
    // Force E absolute for the seek move, then restore the host mode.
    const bool saved_e_relative = axis_is_relative(E_AXIS);
    set_e_absolute();

    const long target_milli = lroundf((start_e + dir * max_e) * 1000.0f);
    const long abs_milli = labs(target_milli);
    MString<48> cmd;
    cmd.setf_P(PSTR("G1 E%s%ld.%03ld F%u"),
      target_milli < 0 ? "-" : "", abs_milli / 1000, abs_milli % 1000,
      (unsigned int)feed_mm_min
    );

    tdc_seek_hit = false;
    tdc_seek_active = true;
    const millis_t start = millis();
    process_subcommands_now(cmd);

    // The stepper ISR samples the switch on every step pulse and aborts the
    // block as soon as TDC triggers. This is one continuous move, not a
    // move-wait-move loop, so it runs at the requested feed rate.
    while (planner.busy()) {
      if (tdc_seek_hit || tdc_triggered()) break;
      if (ELAPSED(millis(), start, timeout)) {
        // Drop the whole planner queue: a failed seek means the stitch cannot
        // complete, so nothing queued behind it should run.
        planner.quick_stop();
        break;
      }
      marlin.idle();
    }

    if (!tdc_seek_hit && tdc_triggered()) stepper.quick_stop();
    while (planner.busy()) marlin.idle();
    tdc_seek_active = false;

    const bool found = tdc_seek_hit || tdc_triggered();
    tdc_seek_hit = false;

    // Aborting a block leaves the logical E position at the block target.
    // Resync it to the actual stepper position so a short seek does not create
    // a growing logical offset.
    const float actual_e = stepper.position(E_AXIS) * planner.mm_per_step[E_AXIS];
    motion.set_and_sync_e(actual_e);

    if (saved_e_relative) set_e_relative(); else set_e_absolute();

    if (found) {
      SERIAL_ECHOLNPGM("TDC:1");
      return;
    }

    SERIAL_ECHOLNPGM("TDC:0");
    if (kill_on_timeout) {
      marlin.kill(F("TDC seek timeout"));
    }
    else {
      // Stop here and block the queued stitches. The host pauses the job and
      // can clear this with M4003 after the jam/clutch is fixed.
      tdc_fault_latched = true;
      SERIAL_ECHOLNPGM("TDC_FAULT:1");
    }
  #else
    SERIAL_ERROR_MSG("TDC_PIN not configured");
  #endif
}

/**
 * M4003: Clear (or report) the TDC fault latch
 *
 *   M4003      Clear the latch and report TDC_FAULT:0.
 *   M4003 S0   Report the latch state without clearing it.
 */
void GcodeSuite::M4003() {
  #if PIN_EXISTS(TDC)
    if (!parser.seen('S') || parser.value_bool()) tdc_fault_latched = false;
    SERIAL_ECHOPGM("TDC_FAULT:");
    SERIAL_ECHOLN(tdc_fault_latched ? 1 : 0);
  #else
    SERIAL_ERROR_MSG("TDC_PIN not configured");
  #endif
}
