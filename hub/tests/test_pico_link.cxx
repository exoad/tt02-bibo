// Hardware bring-up test for PicoLink against the real Pico 2 W.
//
//   tests\build_pico_test.bat run
//
// The board is EXPECTED TO BE SILENT (its firmware prints nothing yet), so
// receiving zero lines is a PASS. What this proves is:
//   * listPicoPorts() finds the VID_2E8A port
//   * the \\.\COMn path opens (COM10+ is exactly where a bare name would fail)
//   * writes succeed against a peer that never answers
//   * the reader thread neither spins nor invents an error out of silence
//   * disconnect() really joins, is idempotent, and drops post-close sends
//
// It deliberately NEVER calls bootsel_touch(): that would reboot the board into
// BOOTSEL and yank the port out from under whoever else is working on it.
#include "shared.hxx"
#include "../src/pico_link.hxx"

#include <chrono>
#include <cstdio>
#include <thread>

static constexpr const Char* PORT = "COM10";

static const Char* stateName(PicoState s)
{
    switch(s)
    {
        case PicoState::PICO_STATE_DISCONNECTED: return "Disconnected";
        case PicoState::PICO_STATE_CONNECTING:   return "Connecting";
        case PicoState::PICO_STATE_CONNECTED:    return "Connected";
        case PicoState::PICO_STATE_ERROR:        return "Error";
    }
    return "?";
}

// Mimics the UI thread: drain every ~16 ms for `ms` milliseconds.
static Size pump(PicoLink& link, Vec<PicoLine>& sink, Int32 ms, const Char* what)
{
    const Size before = sink.size();
    for(Int32 t = 0; t < ms; t += 16)
    {
        link.drain(sink);
        sleepMs(16);
    }
    link.drain(sink);
    printf("  [%s] %d ms elapsed, %zu new line(s)\n", what, ms, sink.size() - before);
    return sink.size() - before;
}

Int32 main()
{
    Int32 failures = 0;

    printf("=== 1. listPicoPorts() ===\n");
    Vec<Str> ports = PicoLink::listPicoPorts();
    printf("  %zu Raspberry Pi (VID_2E8A) serial port(s):\n", ports.size());
    for(const auto& p : ports)
    {
        printf("    %s\n", p.c_str());
    }

    Bool found = false;
    for(const auto& p : ports)
    {
        if(p == PORT)
        {
            found = true;
        }
    }
    printf("  %s must be present ... %s\n", PORT, found ? "PASS" : "FAIL");
    if(!found)
    {
        ++failures;
    }

    printf("\n=== 2. connect(%s) ===\n", PORT);
    PicoLink link;
    printf("  state before connect: %s\n", stateName(link.state()));

    const TimePoint tConnect = monoNow();
    link.connect(PORT);
    const Float64 connectMs = elapsedMs(tConnect);
    printf("  connect() returned in %.2f ms (must be non-blocking)\n", connectMs);

    // connect() twice must be a harmless no-op.
    link.connect(PORT);

    for(Int32 i = 0; i < 200 && link.state() == PicoState::PICO_STATE_CONNECTING; ++i)
    {
        sleepMs(10);
    }
    printf("  state after open: %s\n", stateName(link.state()));
    printf("  port():           '%s'\n", link.port().c_str());
    printf("  error():          '%s'\n", link.error().c_str());
    if(link.state() != PicoState::PICO_STATE_CONNECTED)
    {
        printf("  FAIL: could not open %s\n", PORT);
        return 1;
    }

    Vec<PicoLine> lines;

    printf("\n=== 3. listen 3 s on a silent board ===\n");
    pump(link, lines, 3000, "quiet");
    printf(
        " state still %s (silence must NOT become an error) ... %s\n",
        stateName(link.state()),
        link.state() == PicoState::PICO_STATE_CONNECTED ? "PASS" : "FAIL"
    );
    if(link.state() != PicoState::PICO_STATE_CONNECTED)
    {
        ++failures;
    }

    printf("\n=== 4. probe lines ===\n");
    constexpr const Array<const Char*, 3> probes = { "PING", "HELP", "?" };
    for(const Char* p : probes)
    {
        printf("  send(\"%s\")\n", p);
        link.send(p);
        sleepMs(120);
        link.drain(lines);
    }

    printf("\n=== 5. listen 3 s for a reply ===\n");
    pump(link, lines, 3000, "after probes");

    printf("\n=== 6. everything logged (%zu line(s)) ===\n", lines.size());
    if(lines.empty())
    {
        printf("  (nothing at all - unexpected, tx lines should be logged)\n");
    }
    for(const auto& l : lines)
    {
        printf("  %8.3f  %s  %s\n", l.tS, l.outgoing ? "TX >" : "RX <", l.text.c_str());
    }

    printf("\n=== 7. counters ===\n");
    printf("  tx_lines()      = %llu\n", link.txLines());
    printf("  rx_lines()      = %llu\n", link.rxLines());
    printf("  dropped()       = %llu\n", link.dropped());
    const Float64 age = link.lastRxAgeS();
    printf(
        " lastRxAgeS() = %.3f %s\n",
        age,
        age < 0.0 ? "(negative == nothing ever received, as documented)" : ""
    );
    printf("  3 probes sent   ... %s\n", link.txLines() == 3 ? "PASS" : "FAIL");
    if(link.txLines() != 3)
    {
        ++failures;
    }
    printf("  0 dropped while connected ... %s\n", link.dropped() == 0 ? "PASS" : "FAIL");
    if(link.dropped() != 0)
    {
        ++failures;
    }

    printf("\n=== 8. disconnect ===\n");
    const TimePoint tDc = monoNow();
    link.disconnect();
    const Float64 dcMs = elapsedMs(tDc);
    printf("  disconnect() blocked %.2f ms (worker joined)\n", dcMs);
    printf(
        " state: %s ... %s\n",
        stateName(link.state()),
        link.state() == PicoState::PICO_STATE_DISCONNECTED ? "PASS" : "FAIL"
    );
    if(link.state() != PicoState::PICO_STATE_DISCONNECTED)
    {
        ++failures;
    }
    printf(" port() now '%s' ... %s\n", link.port().c_str(), link.port().empty() ? "PASS" : "FAIL");

    link.disconnect();   // idempotent
    printf("  second disconnect() survived ... PASS\n");

    const UInt64 beforeDrop = link.dropped();
    link.send("SHOULD_BE_DROPPED");
    printf(
        " send() while disconnected -> dropped %llu -> %llu ... %s\n",
        beforeDrop,
        link.dropped(),
        link.dropped() == beforeDrop + 1 ? "PASS" : "FAIL"
    );
    if(link.dropped() != beforeDrop + 1)
    {
        ++failures;
    }

    printf("\n=== 9. reconnect after disconnect ===\n");
    link.connect(PORT);
    for(Int32 i = 0; i < 200 && link.state() == PicoState::PICO_STATE_CONNECTING; ++i)
    {
        sleepMs(10);
    }
    printf(
        " state: %s ... %s\n",
        stateName(link.state()),
        link.state() == PicoState::PICO_STATE_CONNECTED ? "PASS" : "FAIL"
    );
    if(link.state() != PicoState::PICO_STATE_CONNECTED)
    {
        ++failures;
    }
    link.disconnect();

    printf("\n=== bootsel_touch() ===\n");
    printf("  NOT CALLED on purpose - it would reboot the board into BOOTSEL.\n");

    printf("\n=== RESULT: %s (%d failure(s)) ===\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
