/*
 * ---------------------------------------------------------------------------
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
 * -------------------------------------------------------------------------
 */
#pragma once

#include "../../lib/shared.hxx"
#include "../../lib/pins.hxx"

#include <stdarg.h>
#include <stdio.h>

namespace bibo
{

  /* The pin type the real hal exposes. */
  typedef Int32 Pin;

  /*
   * ---- what the fake saw --------------------------------------------------
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

    /**
     * @brief Clears every recorded event and rewinds the fake clock to zero.
     *
     * @note Meant to be called between tests, since the fake otherwise keeps
     *       accumulating events and time across them.
     */
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

    /**
     * @brief Appends one event to the recording.
     *
     * @param k the kind of event that happened
     * @param pin the pin the event happened on
     * @param value the event's payload - microseconds for a servo write, or
     *              0/1 for a gpio level
     *
     * @note Silently dropped once MAX_EVENTS events have already been
     *       recorded; there is no overflow flag.
     */
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

    /**
     * @brief The last level driven onto a pin.
     *
     * @param pin the pin to look up
     * @return the last level written via gpio::write() to `pin`, or -1 if
     *         it never was
     */
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

    /**
     * @brief How many times anything was written to a pin.
     *
     * @param pin the pin to look up
     * @return the count of servo::writeUs() and gpio::write() calls
     *         recorded against `pin`
     */
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

  }

  /* ---- the seven functions the modules under test actually call ----------- */

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

  namespace gpio
  {

    /**
     * @brief Records that a pin was opened for GPIO use.
     *
     * The real one takes a direction enum; tests do not care which, so this
     * takes an Int32 and records only that the pin was claimed.
     *
     * @param pin the pin being opened
     * @param dir the direction the real hal would take; ignored here
     */
    static Void open(Pin pin, Int32 dir)
    {
      static_cast<Void>(dir);
      fake::record(fake::EVENT_GPIO_OPEN, pin, 0);
    }

    /**
     * @brief Records a level driven onto a GPIO pin.
     *
     * @param pin the pin written to
     * @param level the level written; recorded as 1 for true, 0 for false
     */
    static Void write(Pin pin, Bool level)
    {
      fake::record(fake::EVENT_GPIO_WRITE, pin, level ? 1 : 0);
    }

  }

  namespace timing
  {

    /**
     * @brief The fake clock's current time.
     *
     * Time does not pass on its own here. A test that wants a blink to
     * advance sets fake::nowUs, which is the only way to make a timing test
     * that does not take as long as the thing it is timing.
     *
     * @return the fake clock's value, in microseconds since the fake epoch
     */
    static UInt64 nowUs(Void)
    {
      return fake::nowUs;
    }

    /**
     * @brief The fake clock's current time, in milliseconds.
     *
     * @return fake::nowUs truncated down to whole milliseconds
     */
    static UInt32 nowMs(Void)
    {
      return static_cast<UInt32>(fake::nowUs / 1000u);
    }

    /**
     * @brief Advances the fake clock.
     *
     * @param n how much time to add, in milliseconds
     */
    static Void ms(UInt32 n)
    {
      fake::nowUs += static_cast<UInt64>(n) * 1000u;
    }

    /**
     * @brief Advances the fake clock.
     *
     * @param n how much time to add, in microseconds
     */
    static Void us(UInt32 n)
    {
      fake::nowUs += n;
    }

    /*
     * ---- deadlines ------------------------------------------------------
     *
     * Microseconds since the fake epoch, so a test can step time with
     * timing::ms() and watch a slew limiter advance without waiting for it.
     */
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

  namespace serial
  {

    /**
     * @brief Discards a formatted console message instead of sending it anywhere.
     *
     * Swallowed. A test asserting on console text would be asserting on
     * wording, which is the thing most likely to change for good reasons.
     *
     * @param fmt a printf-style format string; never actually formatted
     */
    static Void printf(const Utf8* fmt, ...)
    {
      static_cast<Void>(fmt);
    }

  }

}
