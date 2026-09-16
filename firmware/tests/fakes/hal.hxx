/*
 * A fake hal for the chassis and encoder suites. chassis.hxx calls only
 * servo::open, release and writeUs and the slew deadline; encoder.hxx reads
 * the gpio port, watches edges and disables interrupts. Those are faked
 * here, every servo and gpio event is recorded in order, and a test sets the
 * lines and fires the interrupt by hand: it asserts what reached the PINS
 * and what the loop would read.
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
      EVENT_SERVO_US,
      EVENT_GPIO_INPUT,
      EVENT_GPIO_WATCH
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
    static UInt32 lines = 0;      /* the gpio port, one bit a pin, set by hand */
    static UInt32 pending = 0;    /* pins with an edge raised and not yet served */
    static Void (*edgeHandler)(Void) = nullptr;

    static Void reset(Void)
    {
      count = 0;
      nowUs = 0;
      lines = 0;
      pending = 0;
      edgeHandler = nullptr;
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

    static Void setLine(Int32 pin, Bool on)
    {
      const UInt32 bit = 1u << static_cast<UInt32>(pin);
      lines = on ? (lines | bit) : (lines & ~bit);
    }

    /** The edge interrupt runs, as it would after a line moved. */
    static Void edge(Void)
    {
      if(edgeHandler != nullptr)
      {
        edgeHandler();
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

  namespace gpio
  {
    typedef Void (*EdgeHandler)(Void);

    static Void openInputDown(Pin pin)
    {
      fake::record(fake::EVENT_GPIO_INPUT, pin, 0);
    }

    static UInt32 readAll(Void)
    {
      return fake::lines;
    }

    static Void watchEdges(Pin pin, EdgeHandler handler)
    {
      fake::edgeHandler = handler;
      fake::record(fake::EVENT_GPIO_WATCH, pin, 0);
    }

    static Bool edgePending(Pin pin)
    {
      return ((fake::pending >> static_cast<UInt32>(pin)) & 1u) != 0u;
    }
  }

  /* No interrupts on a laptop: the window is a number handed back. */
  namespace sync
  {
    [[nodiscard]] static UInt32 disable(Void)
    {
      return 0;
    }

    static Void restore(UInt32 saved)
    {
      static_cast<Void>(saved);
    }
  }

  namespace timing
  {
    /** Advances the fake clock: time passes only when a test says so. */
    static Void ms(UInt32 n)
    {
      fake::nowUs += static_cast<UInt64>(n) * 1000u;
    }

    static UInt64 nowUs(Void)
    {
      return fake::nowUs;
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
