#ifndef CLOCK_H
#define CLOCK_H

#include <functional>

/// Clock types
enum class ClockType {
  SYS, CPU,
  PER, PER2, PER4,
  ASY,
};

/** @brief Callback for clock events
 */
using ClockCallback = std::function<void()>;

/** Clock event object
 *
 * Events are executed by the device after the CPU instruction.
 * They are allocated to by the creator of the event then scheduled on the
 * device.
 */
struct ClockEvent {
  ClockEvent(ClockType clock, ClockCallback callback):
      clock(clock), callback(callback) {}
  ClockType clock;  ///< Clock the event is scheduled for
  ClockCallback callback;  ///< Event callback
  unsigned int period = 0;  ///< Event period in ticks SYS ticks (must be a multiple scale)
  unsigned int tick = 0;  ///< Due SYS tick
  unsigned int scale = 0;  ///< Event's clock scale (used to reschedule on clock's config change)
};

#endif
