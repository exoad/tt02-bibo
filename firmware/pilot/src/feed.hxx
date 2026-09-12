// The scan feed's network half: a TCP server that hands complete lines to
// every client and takes MOTOR and QUIT back, on a thread of its own.
//
// ---------------------------------------------------------------------------
// WHAT THIS IS
//
// The half of tools/scanfeed.cxx that has nothing to do with a lidar, lifted
// out so two programs can serve one wire. scanfeed owns the C1 for whoever
// connects; the pilot owns it outright while it drives, and the hub, or `nc`
// from a phone, wants to watch the car see. Both formats lines with
// scanwire.hxx and hands them here; this file moves them and never looks at
// what they say.
//
// ---------------------------------------------------------------------------
// THE FEED THREAD NEVER TOUCHES THE LIDAR
//
// lidar::* shares file-scope state with no lock (one device, one caller -
// lidar.hxx), and grab() blocks for up to two seconds, which a socket loop
// cannot afford. So the thread this module runs calls nothing of the owner's
// but the two callbacks in Policy, and both are wishes rather than acts: a
// client asked for the motor, the client count changed. The OWNER decides what
// to do about either, on its own thread, and publishes the outcome when it is
// real. That is what lets scanfeed spin the device up for the first client and
// the pilot refuse to stop it for anyone, with the same sockets underneath.
//
// ---------------------------------------------------------------------------
// A SLOW CLIENT IS DROPPED, NEVER WAITED FOR
//
// Every socket is non-blocking. A line is appended to each client's pending
// buffer and pushed with send(); whatever the socket will not take stays
// pending and goes out on POLLOUT. A client whose oldest pending byte is more
// than BEHIND_MS old is closed. A phone on the far side of a hotspot can stall
// for seconds, and a blocking write to it would stall the revolution for every
// other viewer - and, in the pilot, the tick that drives the car.
//
// Lines are not queued per client beyond that half second: the picture is
// live or it is nothing, which is the same rule the hub's own lidar worker
// applies to a stale revolution.
//
// ---------------------------------------------------------------------------
// publish() NEVER BLOCKS THE CALLER
//
// It takes a mutex for a push, writes one byte to a self-pipe to wake poll(),
// and returns. The sends happen on the feed thread, so the pilot's tick pays
// microseconds for a viewer whether the viewer is fast, slow or absent. With
// no client connected it pays even less: the line is dropped at the door.
//
// ---------------------------------------------------------------------------
// A CLIENT MAY BE RELAYED TO ANOTHER FEED
//
// In the field scanfeed is already listening on scanwire::PORT, idle, when
// the operator starts the pilot; the pilot takes the lidar and falls back to
// scanwire::PILOT_PORT (Policy::fallbackPort), and nobody has root on the
// board to stop the service. So the address a viewer dials stays scanfeed's,
// and scanfeed, finding the device held, asks this module to relay(): every
// client not yet served gets its own upstream connection, everything the
// upstream sends goes to that client - the pilot greets on connect, so the
// client is greeted naturally - and the client's own lines go upstream, where
// the pilot answers MOTOR itself. When the upstream closes, the client is
// closed too, and its reconnect a second later finds the device free. One
// upstream per client rather than one shared: the pilot's feed already fans
// out and drops slow viewers, and a shared upstream would have to do both a
// second time here. The connect is non-blocking and happens on this thread,
// so the rule above holds: the owner asks, the feed acts, the lidar is never
// touched from here.
//
// Linux only, like link and lidar. Elsewhere start() refuses and the rest do
// nothing, so a program built on a laptop says "no feed" rather than listens.
#pragma once

#include "shared.hxx"

namespace feed
{

  // What a client's MOTOR request does. Either way the request is one line on
  // the wire and never a call into the lidar from this thread.
  enum class Motor
  {
      // Policy::onMotor is called with the wish, on the feed thread. Nothing
      // is answered here: the owner publishes the MOTOR line when the device
      // has actually changed state, and setGreeting() so late joiners hear it.
      MOTOR_OBEY,

      // onMotor never runs. The asking client alone is answered with
      // Policy::motorOn - the true state, which it may not change - and the
      // log says so once per client, not once per request.
      MOTOR_REFUSE,
  };

  struct Policy
  {
      // Sent to every new client before anything else: INFO and HEALTH, then
      // the MOTOR line, in that order. May be empty (scanfeed, before the
      // device is open); replaced later with setGreeting().
      Str greeting;

      Motor motor = Motor::MOTOR_REFUSE;

      // MOTOR_REFUSE: the state every MOTOR request is answered with.
      Bool motorOn = false;

      // MOTOR_REFUSE: said in the log, after the client's name, the first
      // time each client asks. Empty prints a plain "refused".
      Str refusal;

      // MOTOR_OBEY: the wish, on the feed thread. Must return quickly - set a
      // flag, notify a condition variable - and must not call lidar::*.
      Fn<Void(Bool on)> onMotor;

      // Called on the feed thread whenever the number of clients changes, with
      // the new count. scanfeed opens the device on 0 -> 1 and parks it on
      // -> 0; the pilot does not care. Same rules as onMotor.
      Fn<Void(Size clients)> onClients;

      // Tried, once, when start()'s port is already taken (EADDRINUSE and
      // nothing else); 0 means refuse instead. The pilot sets it to
      // scanwire::PILOT_PORT. scanfeed leaves it 0 on purpose: its port is
      // the one address viewers dial, and a scanfeed that quietly moved next
      // door would relay to itself - systemd retrying it is the right answer.
      UInt16 fallbackPort = 0;

      // Called on the feed thread once per client a relay() was tried for:
      // `up` true when the upstream accepted, false when it did not, with
      // `detail` naming the upstream on success and the error otherwise. The
      // client has by then been sent ERR and closed on failure; this is the
      // owner's chance to count or say so. Same rules as onMotor.
      Fn<Void(Bool up, const Str& detail)> onRelay;
  };

  // Listens on 0.0.0.0:port and starts the thread. 0 asks the system for a
  // port; port() says which - and says so when it is Policy::fallbackPort
  // rather than the one asked for. false, with the reason printed, when the
  // socket could not be made, bound or listened on - and false, saying so,
  // when called twice without a stop() between.
  [[nodiscard]] Bool start(UInt16 port, const Policy& p);

  // The port actually bound; 0 when not started.
  [[nodiscard]] UInt16 port();

  // Every client connected now and not yet relayed gets its own connection
  // to host:port and hears that feed instead of this one; clients that
  // arrive later are not included, so ask again for them. A client whose
  // upstream refuses is sent `ERR <orElse>` and closed - scanfeed passes the
  // lidar's reason, so the answer is what it would have been without the
  // relay. One log line per relay started and one per relay stopped. Does
  // nothing when not started.
  Void relay(const Str& host, UInt16 port, const Str& orElse);

  // Replaces Policy::greeting for every client that connects from now on.
  // Clients already connected are not told; publish() the change to them.
  Void setGreeting(const Str& lines);

  // One complete line (with its '\n') for every client. By value so a caller
  // that is done with the line can move it in; a caller that keeps it pays
  // one copy. Dropped without a trace when nobody is connected - the picture
  // is live or nothing, and there is nobody to be live for.
  Void publish(Str line);

  // ERR <why> to every client, then closes them all. The next client to
  // connect starts from nothing but the greeting. scanfeed's answer to a
  // device that would not open; the pilot has no use for it.
  Void fail(const Str& why);

  // How many clients are connected right now.
  [[nodiscard]] Size clients();

  // Stops the thread and closes every client and the listening socket. Safe
  // when not started.
  Void stop();

}
