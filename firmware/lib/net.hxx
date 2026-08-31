/* ---------------------------------------------------------------------------
 * net - the same command link, over Wi-Fi.
 *
 * One UDP datagram is one command line, in exactly the text the USB console
 * already speaks. "ESC 1547\n" over the cable and "ESC 1547" in a datagram are
 * the same line reaching the same handler, so the command table, the console,
 * the hub and every note written about them keep working without knowing which
 * way a line arrived. A second, binary, wireless-only protocol would have been
 * a second thing to keep in step with the first.
 *
 * ---- why it JOINS a network instead of making one -------------------------
 *
 * Access-point mode is the obvious shape for a car - it takes the house router
 * out of the path and works in a car park. It also needs a DHCP server, and
 * lwIP ships a client only: the server every Pico access-point example uses
 * lives in pico-examples, which is not vendored here. So joining an existing
 * network is what can be built out of what is actually on this machine, and AP
 * mode is a later job with a DHCP server in it.
 *
 * ---- the credentials are not in this repository ---------------------------
 *
 * No SSID, no password, no default. They arrive at run time over the USB
 * console - WIFI JOIN <ssid> <password> - and live in RAM until the board is
 * reset. A network password baked into a source file is a network password in
 * the commit history forever, and this repository is pushed.
 *
 * ---- what this link does NOT do -------------------------------------------
 *
 * It does not authenticate. Anything on the same network that can reach this
 * port can steer the car, and that is worth knowing rather than discovering.
 * What limits the damage is the deadman in main.c: the board stops itself when
 * the commands stop, so the worst a silent attacker achieves is a stop.
 *
 * On a board with no radio every function here is a stub that answers "no
 * radio", so main.c reads the same on both images and needs no #ifdef around
 * any call site.
 * ------------------------------------------------------------------------- */
#pragma once

#include "types.hxx"
#include "hal.hxx"

/* The wireless stack, on a board that has one. UP HERE, not down inside the
 * implementation, because an #include inside a namespace drags every
 * declaration in that header into it - which for lwIP is several hundred names
 * that would end up as net::something. */
#ifdef BIBO_WIRELESS
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include <string.h>
#endif

namespace bibo
{

  namespace net
  {

    /* The port. Arbitrary, above the registered range, and the same number the hub
     * uses - hub/src/pico_link.hxx. Change one and change the other. */
#define NET_PORT 4242

    /*
     * One line's worth. The same 128 as main.c's LINE_CAP, and for the same
     * reason: no command this protocol has is anywhere near it, and a bigger
     * buffer here would only let an over-long line travel further before being
     * refused.
     */
#define NET_LINE_CAP 128

    /*
     * How many complete lines can be held between one net::poll() and the next.
     *
     * The loop drains this every millisecond or so, so eight is already generous;
     * it exists for the burst that arrives while the loop is off doing something
     * slow, not for steady traffic.
     */
#define NET_QUEUE_LINES 8

    enum State
    {
        STATE_ABSENT = 0,  /* no radio on this board */
        STATE_OFF,         /* radio up, not joined to anything */
        STATE_JOINING,
        STATE_UP,
        STATE_FAILED
    };

    typedef Void (*LineHandler)(Utf8* line);

#ifdef BIBO_WIRELESS

    /*
     * OFF, not ABSENT: this half of the file only compiles on a board that HAS the
     * chip, so "no radio" is never the answer here. ABSENT belongs to the stub half
     * at the bottom, and keeping the two meanings apart is what lets a caller print
     * net::stateWord() without having to know which build it is in.
     */
    inline State       stateNow  = STATE_OFF;
    inline Bool           started   = false;
    inline LineHandler handler   = nullptr;
    inline struct udp_pcb* pcb      = nullptr;

    /* The last host that said anything. Replies go here - including replies to
     * commands that arrived over USB, which is deliberate: a console watching
     * wirelessly should see the whole conversation, not only its half of it. */
    inline ip_addr_t peerAddr;
    inline UInt16    peerPort = 0;
    inline Bool      peerKnownNow = false;

    /* The join in progress. cyw43's async connect keeps no copy of these, so they
     * have to outlive the call that starts it. */
    inline Utf8 joinSsid[40];
    inline Utf8 joinPass[68];

    /* The line queue, filled from the lwIP callback and drained by net::poll(). */
    inline Utf8  queue[NET_QUEUE_LINES][NET_LINE_CAP];
    inline Size  queueHead  = 0;   /* next to drain */
    inline Size  queueCount = 0;
    inline UInt32 dropped   = 0;

    /* Assembly buffer: a datagram is USUALLY one line, and is not guaranteed to be
     * - a sender is free to put two in one packet, or split one across two. */
    inline Utf8 partial[NET_LINE_CAP];
    inline Size partialLen = 0;

    [[nodiscard]] static Bool present(Void)
    {
        return true;
    }

    inline State status(Void)
    {
        return stateNow;
    }

    [[nodiscard]] static Bool peerKnown(Void)
    {
        return peerKnownNow;
    }

    inline UInt32 droppedCount(Void)
    {
        return dropped;
    }

    inline Void setLineHandler(LineHandler fn)
    {
        handler = fn;
    }

    /*
     * The car's address on the network, or "-" before it has one.
     *
     * Points into lwIP's own static formatting buffer, so it is valid until the
     * next call. Print it, do not keep it.
     */
    inline CharSeq address(Void)
    {
        if(stateNow != STATE_UP || netif_default == nullptr)
        {
            return "-";
        }
        return ip4addr_ntoa(netif_ip4_addr(netif_default));
    }

    /* One finished line into the queue. */
    inline Void queuePush(const Utf8* text, Size len)
    {
        if(len == 0)
        {
            return;   /* a bare newline is not a command */
        }
        if(len >= NET_LINE_CAP)
        {
            len = NET_LINE_CAP - 1;
        }

        /*
         * Full: drop the OLDEST, not this one.
         *
         * On a control link the freshest command is the one that matters - it is
         * the one describing where the car should be NOW - and a queue that
         * discarded arrivals would preferentially discard the newest, which is
         * exactly backwards. Losing a stale keepalive costs nothing.
         */
        if(queueCount == NET_QUEUE_LINES)
        {
            queueHead = (queueHead + 1) % NET_QUEUE_LINES;
            queueCount--;
            dropped++;
        }

        const Size slot = (queueHead + queueCount) % NET_QUEUE_LINES;
        memcpy(queue[slot], text, len);
        queue[slot][len] = '\0';
        queueCount++;
    }

    /* Bytes off the wire, split into lines. */
    inline Void feed(const Utf8* data, Size len)
    {
        for(Size i = 0; i < len; ++i)
        {
            const Utf8 c = data[i];

            if(c == '\n' || c == '\r')
            {
                queuePush(partial, partialLen);
                partialLen = 0;
            }
            else if(partialLen + 1 < NET_LINE_CAP)
            {
                partial[partialLen++] = c;
            }
            else
            {
                /* Over-long. Drop the whole line rather than letting its tail
                 * become a command nobody sent - the same rule, and the same
                 * reasoning, as the USB reader in main.c. */
                partialLen = 0;
            }
        }

        /*
         * A datagram that ends without a newline is a COMPLETE line.
         *
         * UDP preserves message boundaries - unlike a stream, where the end of a
         * read means nothing - so requiring a trailing newline would silently
         * ignore every sender that does not bother with one. Which is most of
         * them, including a one-line test from a shell.
         */
        if(partialLen > 0)
        {
            queuePush(partial, partialLen);
            partialLen = 0;
        }
    }

    /* The signature is lwIP's, not ours - it is a callback that stack calls, so
     * every parameter is spelled the way udp_recv_fn declares it. */
    inline Void onPacket(Void* arg, struct udp_pcb* pcb, struct pbuf* p, const ip_addr_t* addr, u16_t port)
    {
        static_cast<Void>(arg);
        static_cast<Void>(pcb);

        if(p == nullptr)
        {
            return;
        }

        peerAddr     = *addr;
        peerPort     = port;
        peerKnownNow = true;

        /* pbufs can be chained; walking the chain is not optional even for small
         * packets, because "small" is the sender's decision and not ours. */
        for(struct pbuf* q = p; q != nullptr; q = q->next)
        {
            feed(static_cast<const Utf8*>(q->payload), static_cast<Size>(q->len));
        }

        pbuf_free(p);
    }

    /*
     * Brings the radio up in station mode. Does NOT join anything.
     *
     * Power management is turned OFF here, and that is not a detail. The default
     * parks the radio between beacons and adds latency spikes well past a hundred
     * milliseconds - which on a link that carries hold-to-drive commands means the
     * deadman fires mid-drive and it looks like a fault in the car.
     */
    [[nodiscard]] static Bool start(Void)
    {
        if(started)
        {
            return true;
        }

        /*
         * radio::open(), not cyw43_arch_init().
         *
         * The LED on this board hangs off the same chip and main() brings it up at
         * boot, so by the time anybody asks for wireless the radio is ALREADY
         * initialised. A second cyw43_arch_init() does not return an error - it
         * quietly leaves the chip half re-initialised, and the next thing to touch
         * it behaves strangely for reasons that are nowhere near the code that
         * caused them. hal.h owns that call and answers it once.
         */
        if(!radio::open())
        {
            stateNow = STATE_FAILED;
            return false;
        }

        cyw43_arch_enable_sta_mode();
        cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);

        pcb = udp_new();
        if(pcb == nullptr)
        {
            stateNow = STATE_FAILED;
            return false;
        }

        if(udp_bind(pcb, IP_ANY_TYPE, NET_PORT) != ERR_OK)
        {
            udp_remove(pcb);
            pcb      = nullptr;
            stateNow = STATE_FAILED;
            return false;
        }

        udp_recv(pcb, onPacket, nullptr);

        started  = true;
        stateNow = STATE_OFF;
        return true;
    }

    /*
     * Starts joining. Returns immediately.
     *
     * Asynchronous on purpose: the blocking form of this call sits inside the SDK
     * for up to thirty seconds, and every one of those seconds is a second the main
     * loop is not running drive::pump(), not slewing an output, and not honouring the
     * deadman. A join that freezes the car is a join nobody can make while the car
     * is switched on.
     */
    [[nodiscard]] static Bool join(CharSeq ssid, CharSeq pass)
    {
        if(!started && !start())
        {
            return false;
        }

        /* Copies, because the async connect keeps pointers to these. */
        snprintf(joinSsid, sizeof(joinSsid), "%s", ssid);
        snprintf(joinPass, sizeof(joinPass), "%s", (pass != nullptr) ? pass : "");

        const UInt32 auth = (joinPass[0] == '\0')
                                ? CYW43_AUTH_OPEN
                                : CYW43_AUTH_WPA2_AES_PSK;

        /* nullptr rather than an empty string for an open network: the SDK tests the
         * pointer, not what it points at. */
        CharSeq key = (joinPass[0] == '\0') ? nullptr : joinPass;

        if(cyw43_arch_wifi_connect_async(joinSsid, key, auth) != 0)
        {
            stateNow = STATE_FAILED;
            return false;
        }

        stateNow = STATE_JOINING;
        return true;
    }

    /* One line out to whoever last spoke. Silently does nothing if nobody has. */
    inline Void sendLine(CharSeq text)
    {
        if(stateNow != STATE_UP || !peerKnownNow || pcb == nullptr)
        {
            return;
        }

        const Size len = strlen(text);
        if(len == 0)
        {
            return;
        }

        struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, static_cast<u16_t>(len), PBUF_RAM);
        if(p == nullptr)
        {
            return;   /* out of buffers: dropping a reply beats blocking the loop */
        }

        memcpy(p->payload, text, len);
        udp_sendto(pcb, p, &peerAddr, peerPort);
        pbuf_free(p);
    }

    /*
     * Pumps the stack and hands over anything that arrived.
     *
     * Call it every time round the main loop. In NO_SYS mode nothing in lwIP
     * happens on its own: no packet is received, no join completes and no timer
     * fires except inside this call.
     */
    inline Void poll(Void)
    {
        if(!started)
        {
            return;
        }

        cyw43_arch_poll();

        if(stateNow == STATE_JOINING)
        {
            const Int32 link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

            if(link == CYW43_LINK_UP)
            {
                stateNow = STATE_UP;
            }
            else if(link < 0)
            {
                stateNow = STATE_FAILED;
            }
        }
        else if(stateNow == STATE_UP)
        {
            if(cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP)
            {
                stateNow = STATE_FAILED;
            }
        }

        while(queueCount > 0)
        {
            Utf8* line = queue[queueHead];

            queueHead = (queueHead + 1) % NET_QUEUE_LINES;
            queueCount--;

            if(handler != nullptr)
            {
                handler(line);
            }
        }
    }

#else /* no radio on this board */

    [[nodiscard]] static Bool present(Void)
    {
        return false;
    }

    inline State status(Void)
    {
        return STATE_ABSENT;
    }

    [[nodiscard]] static Bool peerKnown(Void)
    {
        return false;
    }

    inline UInt32 droppedCount(Void)
    {
        return 0;
    }

    inline Void setLineHandler(LineHandler fn)
    {
        static_cast<Void>(fn);
    }

    inline CharSeq address(Void)
    {
        return "-";
    }

    [[nodiscard]] static Bool start(Void)
    {
        return false;
    }

    [[nodiscard]] static Bool join(CharSeq ssid, CharSeq pass)
    {
        static_cast<Void>(ssid);
        static_cast<Void>(pass);
        return false;
    }

    inline Void sendLine(CharSeq text)
    {
        static_cast<Void>(text);
    }

    inline Void poll(Void)
    {
    }

#endif /* BIBO_WIRELESS */

    /* The word for a state, for anything that prints one. */
    inline CharSeq stateWord(State s)
    {
        switch(s)
        {
            case STATE_ABSENT:  return "no-radio";
            case STATE_OFF:     return "off";
            case STATE_JOINING: return "joining";
            case STATE_UP:      return "up";
            case STATE_FAILED:  return "failed";
            default:                return "?";
        }
    }


  } // namespace net

} // namespace bibo