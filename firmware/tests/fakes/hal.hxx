/*
 * A fake hal for the chassis suite. chassis.hxx calls only servo::open, release
 * and writeUs and the slew deadline, so those are faked here, and every servo
 * event is recorded in order: a test asserts what reached the PINS.
 */
#pragma once

#include "../../lib/shared.hxx"

namespace bibo
{
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

    /** Silently dropped once MAX_EVENTS events are recorded. */
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

    /** The last servo::writeUs() value on pin, or -1 if it never had one. */
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
    static Void open(Pin pin)
    {
      fake::record(fake::EVENT_SERVO_OPEN, pin, 0);
    }

    static Void release(Pin pin)
    {
      fake::record(fake::EVENT_SERVO_RELEASE, pin, 0);
    }

    static Void writeUs(Pin pin, UInt32 us)
    {
      fake::record(fake::EVENT_SERVO_US, pin, static_cast<Int32>(us));
    }
  }

  namespace timing
  {
    /** Advances the fake clock: time passes only when a test says so. */
    static Void ms(UInt32 n)
    {
      fake::nowUs += static_cast<UInt64>(n) * 1000u;
    }

    /* Microseconds since the fake epoch. */
    typedef UInt64 Deadline;

    static Deadline armMs(UInt32 ms)
    {
      return fake::nowUs + (static_cast<UInt64>(ms) * 1000u);
    }

    static Bool reached(Deadline d)
    {
      return fake::nowUs >= d;
    }
  }
}
