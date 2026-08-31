/* ---------------------------------------------------------------------------
 * A fake hal, for host tests.
 *
 * WHAT THIS IS FOR
 *
 * chassis.hxx holds the safety property this whole project rests on - "the ESC
 * is disarmed until asked, and every throttle command is refused until
 * drive::arm(true)" - and it had no test, because it includes hal.hxx, which
 * includes twelve Pico SDK headers and uses sixty-one symbols from them. That
 * is not a thing you compile on a laptop.
 *
 * It does not need to be. The modules under test barely touch the hardware:
 *
 *     chassis   servo::open, servo::release, servo::writeUs
 *     lights    gpio::open, gpio::write
 *     cue       serial::printf, timing::nowUs
 *
 * Seven functions. Faking those is a page; faking the SDK is a project.
 *
 * WHAT IT RECORDS
 *
 * Every write, in order, so a test can assert what reached the pins rather
 * than only what the module says about itself. That distinction is the whole
 * point: lights::enable(false) used to park the lamps and then not stay off,
 * and lights::enabled() cheerfully reported true throughout. A test that
 * believed the module would have passed. One that reads the pins does not.
 * ------------------------------------------------------------------------- */
#pragma once

#include "../../lib/types.hxx"
#include "../../lib/pins.hxx"

#include <stdarg.h>
#include <stdio.h>

namespace bibo
{

  /* The pin type the real hal exposes. */
  typedef Int32 Pin;

  /* ---- what the fake saw --------------------------------------------------
   *
   * File-scope and deliberately public: a test wants to read it, and there is
   * no second consumer to protect it from.
   */
  namespace fake
  {

    constexpr Size MAX_EVENTS = 256;

    enum EventKind
    {
      EVENT_NONE = 0,
      EVENT_SERVO_OPEN,
      EVENT_SERVO_RELEASE,
      EVENT_SERVO_US,
      EVENT_GPIO_OPEN,
      EVENT_GPIO_WRITE
    };

    struct Event
    {
      EventKind kind;
      Int32     pin;
      Int32     value;   /* microseconds, or 0/1 for a gpio level */
    };

    static Event  events[MAX_EVENTS];
    static Size   count   = 0;
    static UInt64 nowUs   = 0;      /* tests drive time by hand */

    static Void reset(Void)
    {
      count = 0;
      nowUs = 0;
      for(Size i = 0; i < MAX_EVENTS; ++i)
      {
        events[i].kind  = EVENT_NONE;
        events[i].pin   = -1;
        events[i].value = 0;
      }
    }

    static Void record(EventKind k, Int32 pin, Int32 value)
    {
      if(count < MAX_EVENTS)
      {
        events[count].kind  = k;
        events[count].pin   = pin;
        events[count].value = value;
        ++count;
      }
    }

    /* The last microsecond value written to a pin, or -1 if it never was. */
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

    /* The last level driven onto a pin, or -1 if it never was. */
    static Int32 lastLevel(Int32 pin)
    {
      Int32 out = -1;
      for(Size i = 0; i < count; ++i)
      {
        if(events[i].kind == EVENT_GPIO_WRITE && events[i].pin == pin)
        {
          out = events[i].value;
        }
      }
      return out;
    }

    /* How many times anything was written to a pin. */
    static Size writes(Int32 pin)
    {
      Size n = 0;
      for(Size i = 0; i < count; ++i)
      {
        const Bool isWrite = (events[i].kind == EVENT_SERVO_US)
                          || (events[i].kind == EVENT_GPIO_WRITE);
        if(isWrite && events[i].pin == pin)
        {
          ++n;
        }
      }
      return n;
    }

  } // namespace fake

  /* ---- the seven functions the modules under test actually call ----------- */

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

  } // namespace servo

  namespace gpio
  {

    /* The real one takes a direction enum; tests do not care which, so this
     * takes an Int32 and records only that the pin was claimed. */
    static Void open(Pin pin, Int32 dir)
    {
      static_cast<Void>(dir);
      fake::record(fake::EVENT_GPIO_OPEN, pin, 0);
    }

    static Void write(Pin pin, Bool level)
    {
      fake::record(fake::EVENT_GPIO_WRITE, pin, level ? 1 : 0);
    }

  } // namespace gpio

  namespace timing
  {

    /* Time does not pass on its own here. A test that wants a blink to advance
     * sets fake::nowUs, which is the only way to make a timing test that does
     * not take as long as the thing it is timing. */
    static UInt64 nowUs(Void)
    {
      return fake::nowUs;
    }

    static UInt32 nowMs(Void)
    {
      return static_cast<UInt32>(fake::nowUs / 1000u);
    }

    static Void ms(UInt32 n)
    {
      fake::nowUs += static_cast<UInt64>(n) * 1000u;
    }

    static Void us(UInt32 n)
    {
      fake::nowUs += n;
    }

    /* ---- deadlines ------------------------------------------------------
     *
     * Microseconds since the fake epoch, so a test can step time with
     * timing::ms() and watch a slew limiter advance without waiting for it.
     */
    typedef UInt64 Deadline;

    static Deadline armMs(UInt32 ms)
    {
      return fake::nowUs + (static_cast<UInt64>(ms) * 1000u);
    }

    static Bool reached(Deadline d)
    {
      return fake::nowUs >= d;
    }

  } // namespace timing

  namespace serial
  {

    /* Swallowed. A test asserting on console text would be asserting on
     * wording, which is the thing most likely to change for good reasons. */
    static Void printf(const Utf8* fmt, ...)
    {
      static_cast<Void>(fmt);
    }

  } // namespace serial

} // namespace bibo
