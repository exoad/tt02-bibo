/*
 * ---------------------------------------------------------------------------
 * A fake hal, for the chassis suite.
 *
 * chassis.hxx holds the car's safety rules and includes hal.hxx, which is the
 * Pico SDK. It calls only servo::open/release/writeUs and the slew deadline, so
 * those are faked here, and every servo event is recorded in order: a test
 * asserts what reached the PINS, not what the module says it did.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "../../lib/shared.hxx"

namespace bibo
{

  /* The pin type the real hal exposes. */
  typedef Int32 Pin;

  /* What the fake saw. Public on purpose: the test reads it. */
  namespace fake
  {

    constexpr Size MAX_EVENTS = 256;

    enum EventKind
    {
      EVENT_NONE = 0,
      EVENT_SERVO_OPEN,
      EVENT_SERVO_RELEASE,
      EVENT_SERVO_US
    };

    struct Event
    {
      EventKind kind;
      Int32     pin;
      Int32     value;   /* microseconds for a pulse, 0 otherwise */
    };

    static Event  events[MAX_EVENTS];
    static Size   count = 0;
    static UInt64 nowUs = 0;      /* tests drive time by hand */

    /**
     * @brief Clears every recorded event and rewinds the fake clock to zero.
     */
    static Void reset(Void)
    {
      count = 0;
      nowUs = 0;
      for(Size i = 0; i < MAX_EVENTS; ++i)
      {
        events[i].kind = EVENT_NONE;
        events[i].pin = -1;
        events[i].value = 0;
      }
    }

    /**
     * @brief Appends one event to the recording.
     *
     * @param k the kind of event that happened
     * @param pin the pin the event happened on
     * @param value microseconds for a servo write, 0 otherwise
     *
     * @note Silently dropped once MAX_EVENTS events have been recorded.
     */
    static Void record(EventKind k, Int32 pin, Int32 value)
    {
      if(count < MAX_EVENTS)
      {
        events[count].kind = k;
        events[count].pin = pin;
        events[count].value = value;
        ++count;
      }
    }

    /**
     * @brief The last microsecond value written to a pin.
     *
     * @param pin the pin to look up
     * @return the last value written via servo::writeUs() to `pin`, or -1
     *         if it never was
     */
    static Int32 lastUs(Int32 pin)
    {
      Int32 out = -1;
      for(Size i = 0; i < count; ++i)
      {
        if(events[i].kind == EVENT_SERVO_US && events[i].pin == pin)
        {
          out = events[i].value;
        }
      }
      return out;
    }

  }

  namespace servo
  {

    /**
     * @brief Records that a pin was opened for servo output.
     *
     * @param pin the pin claimed for the servo
     */
    static Void open(Pin pin)
    {
      fake::record(fake::EVENT_SERVO_OPEN, pin, 0);
    }

    /**
     * @brief Records that a servo pin was released.
     *
     * @param pin the pin no longer driven as a servo
     */
    static Void release(Pin pin)
    {
      fake::record(fake::EVENT_SERVO_RELEASE, pin, 0);
    }

    /**
     * @brief Records a servo pulse width written to a pin.
     *
     * @param pin the pin the pulse was written to
     * @param us the pulse width, in microseconds
     */
    static Void writeUs(Pin pin, UInt32 us)
    {
      fake::record(fake::EVENT_SERVO_US, pin, static_cast<Int32>(us));
    }

  }

  namespace timing
  {

    /**
     * @brief Advances the fake clock. Time passes only when a test says so.
     *
     * @param n how much time to add, in milliseconds
     */
    static Void ms(UInt32 n)
    {
      fake::nowUs += static_cast<UInt64>(n) * 1000u;
    }

    /* Microseconds since the fake epoch. */
    typedef UInt64 Deadline;

    /**
     * @brief Computes a deadline a given span of time from now.
     *
     * @param ms how far in the future the deadline should sit, in
     *           milliseconds
     * @return the fake clock's value at which the deadline is reached
     */
    static Deadline armMs(UInt32 ms)
    {
      return fake::nowUs + (static_cast<UInt64>(ms) * 1000u);
    }

    /**
     * @brief Whether a deadline has passed.
     *
     * @param d a deadline previously produced by armMs()
     * @return true once the fake clock has reached or passed `d`
     */
    static Bool reached(Deadline d)
    {
      return fake::nowUs >= d;
    }

  }

}
