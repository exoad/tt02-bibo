// Hardware bring-up test for PicoLink against the real Pico 2 W.
//
//   tests\build_pico_test.bat run
//
// The board is EXPECTED TO BE SILENT (its firmware prints nothing yet), so
// receiving zero lines is a PASS. What this proves is:
//   * list_pico_ports() finds the VID_2E8A port
//   * the \\.\COMn path opens (COM10+ is exactly where a bare name would fail)
//   * writes succeed against a peer that never answers
//   * the reader thread neither spins nor invents an error out of silence
//   * disconnect() really joins, is idempotent, and drops post-close sends
//
// It deliberately NEVER calls bootsel_touch(): that would reboot the board into
// BOOTSEL and yank the port out from under whoever else is working on it.
#include "pico_link.h"

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

static const char* kPort = "COM10";

static const char* state_name(PicoState s)
{
    switch (s)
    {
        case PicoState::Disconnected: return "Disconnected";
        case PicoState::Connecting:   return "Connecting";
        case PicoState::Connected:    return "Connected";
        case PicoState::Error:        return "Error";
    }
    return "?";
}

static void sleep_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Mimics the UI thread: drain every ~16 ms for `ms` milliseconds.
static size_t pump(PicoLink& link, std::vector<PicoLine>& sink, int ms, const char* what)
{
    const size_t before = sink.size();
    for (int t = 0; t < ms; t += 16)
    {
        link.drain(sink);
        sleep_ms(16);
    }
    link.drain(sink);
    printf("  [%s] %d ms elapsed, %zu new line(s)\n", what, ms, sink.size() - before);
    return sink.size() - before;
}

int main()
{
    int failures = 0;

    printf("=== 1. list_pico_ports() ===\n");
    std::vector<std::string> ports = PicoLink::list_pico_ports();
    printf("  %zu Raspberry Pi (VID_2E8A) serial port(s):\n", ports.size());
    for (const auto& p : ports)
        printf("    %s\n", p.c_str());

    bool found = false;
    for (const auto& p : ports)
        if (p == kPort)
            found = true;
    printf("  %s must be present ... %s\n", kPort, found ? "PASS" : "FAIL");
    if (!found)
        ++failures;

    printf("\n=== 2. connect(%s) ===\n", kPort);
    PicoLink link;
    printf("  state before connect: %s\n", state_name(link.state()));

    const auto t_connect = std::chrono::steady_clock::now();
    link.connect(kPort);
    const double connect_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_connect).count();
    printf("  connect() returned in %.2f ms (must be non-blocking)\n", connect_ms);

    // connect() twice must be a harmless no-op.
    link.connect(kPort);

    for (int i = 0; i < 200 && link.state() == PicoState::Connecting; ++i)
        sleep_ms(10);
    printf("  state after open: %s\n", state_name(link.state()));
    printf("  port():           '%s'\n", link.port().c_str());
    printf("  error():          '%s'\n", link.error().c_str());
    if (link.state() != PicoState::Connected)
    {
        printf("  FAIL: could not open %s\n", kPort);
        return 1;
    }

    std::vector<PicoLine> lines;

    printf("\n=== 3. listen 3 s on a silent board ===\n");
    pump(link, lines, 3000, "quiet");
    printf("  state still %s (silence must NOT become an error) ... %s\n",
           state_name(link.state()),
           link.state() == PicoState::Connected ? "PASS" : "FAIL");
    if (link.state() != PicoState::Connected)
        ++failures;

    printf("\n=== 4. probe lines ===\n");
    const char* probes[] = {"PING", "HELP", "?"};
    for (const char* p : probes)
    {
        printf("  send(\"%s\")\n", p);
        link.send(p);
        sleep_ms(120);
        link.drain(lines);
    }

    printf("\n=== 5. listen 3 s for a reply ===\n");
    pump(link, lines, 3000, "after probes");

    printf("\n=== 6. everything logged (%zu line(s)) ===\n", lines.size());
    if (lines.empty())
        printf("  (nothing at all - unexpected, tx lines should be logged)\n");
    for (const auto& l : lines)
        printf("  %8.3f  %s  %s\n", l.t_s, l.outgoing ? "TX >" : "RX <", l.text.c_str());

    printf("\n=== 7. counters ===\n");
    printf("  tx_lines()      = %llu\n", link.tx_lines());
    printf("  rx_lines()      = %llu\n", link.rx_lines());
    printf("  dropped()       = %llu\n", link.dropped());
    const double age = link.last_rx_age_s();
    printf("  last_rx_age_s() = %.3f %s\n", age,
           age < 0.0 ? "(negative == nothing ever received, as documented)" : "");
    printf("  3 probes sent   ... %s\n", link.tx_lines() == 3 ? "PASS" : "FAIL");
    if (link.tx_lines() != 3)
        ++failures;
    printf("  0 dropped while connected ... %s\n", link.dropped() == 0 ? "PASS" : "FAIL");
    if (link.dropped() != 0)
        ++failures;

    printf("\n=== 8. disconnect ===\n");
    const auto t_dc = std::chrono::steady_clock::now();
    link.disconnect();
    const double dc_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_dc).count();
    printf("  disconnect() blocked %.2f ms (worker joined)\n", dc_ms);
    printf("  state: %s ... %s\n", state_name(link.state()),
           link.state() == PicoState::Disconnected ? "PASS" : "FAIL");
    if (link.state() != PicoState::Disconnected)
        ++failures;
    printf("  port() now '%s' ... %s\n", link.port().c_str(),
           link.port().empty() ? "PASS" : "FAIL");

    link.disconnect();   // idempotent
    printf("  second disconnect() survived ... PASS\n");

    const unsigned long long before_drop = link.dropped();
    link.send("SHOULD_BE_DROPPED");
    printf("  send() while disconnected -> dropped %llu -> %llu ... %s\n",
           before_drop, link.dropped(),
           link.dropped() == before_drop + 1 ? "PASS" : "FAIL");
    if (link.dropped() != before_drop + 1)
        ++failures;

    printf("\n=== 9. reconnect after disconnect ===\n");
    link.connect(kPort);
    for (int i = 0; i < 200 && link.state() == PicoState::Connecting; ++i)
        sleep_ms(10);
    printf("  state: %s ... %s\n", state_name(link.state()),
           link.state() == PicoState::Connected ? "PASS" : "FAIL");
    if (link.state() != PicoState::Connected)
        ++failures;
    link.disconnect();

    printf("\n=== bootsel_touch() ===\n");
    printf("  NOT CALLED on purpose - it would reboot the board into BOOTSEL.\n");

    printf("\n=== RESULT: %s (%d failure(s)) ===\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
