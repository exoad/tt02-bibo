/*
 * ---------------------------------------------------------------------------
 * net - the same command link, over Wi-Fi.
 *
 * One UDP datagram is one command line, in exactly the text the USB console
 * already speaks, so the command table, the console and the hub keep working
 * without knowing which way a line arrived.
 *
 * It JOINS a network rather than making one: access-point mode needs a DHCP
 * server and lwIP ships a client only. Credentials never live in this
 * repository - WIFI JOIN <ssid> <password> over the console puts them in RAM
 * until reset. The link does not authenticate: anything that can reach this
 * port can steer the car, and what limits the damage is the deadman in main.c,
 * which stops the board when the commands stop. On a board with no radio every
 * function here is a stub answering "no radio", so main.c needs no #ifdef.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "types.hxx"
#include "hal.hxx"

/*
 * The wireless stack, on a board that has one. UP HERE, not inside the
 * namespace: an #include there drags every lwIP declaration into it.
 */
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

    /* Arbitrary, above the registered range, and the same number hub/src/pico_link.hxx uses. */
#define NET_PORT 4242

    /* One line's worth. The same 128 as main.c's LINE_CAP. */
#define NET_LINE_CAP 128

    /*
     * Lines held between one net::poll() and the next. The loop drains it every
     * millisecond or so, so eight is for a burst, not for steady traffic.
     */
#define NET_QUEUE_LINES 8

    /**
     * @brief The link's lifecycle, from no hardware through a live connection.
     */
    enum State
    {
        STATE_ABSENT = 0,  /* no radio on this board */
        STATE_OFF,         /* radio up, not joined to anything */
        STATE_JOINING,
        STATE_UP,
        STATE_FAILED
    };

    /**
     * @brief A callback invoked once per complete line received off the wire.
     *
     * @param line the received line, nul-terminated; valid only for the call
     */
    typedef Void (*LineHandler)(Utf8* line);

#ifdef BIBO_WIRELESS

    /*
     * OFF, not ABSENT: this half only compiles on a board that HAS the chip, so
     * "no radio" is never the answer here. ABSENT belongs to the stub half.
     */
    inline State       stateNow  = STATE_OFF;
    inline Bool           started   = false;
    inline LineHandler handler   = nullptr;
    inline udp_pcb* pcb      = nullptr;

    /* The last host that said anything. Replies go here, USB commands included, so a wireless console sees both halves. */
    inline ip_addr_t peerAddr;
    inline UInt16    peerPort = 0;
    inline Bool      peerKnownNow = false;

    /* The join in progress. cyw43's async connect keeps no copy of these. */
    inline Utf8 joinSsid[40];
    inline Utf8 joinPass[68];

    /* The line queue, filled from the lwIP callback and drained by net::poll(). */
    inline Utf8  queue[NET_QUEUE_LINES][NET_LINE_CAP];
    inline Size  queueHead  = 0;   /* next to drain */
    inline Size  queueCount = 0;
    inline UInt32 dropped   = 0;

    /* Assembly buffer: a datagram is USUALLY one line, and is not guaranteed to be. */
    inline Utf8 partial[NET_LINE_CAP];
    inline Size partialLen = 0;

    /**
     * @brief Whether this board has wireless hardware at all.
     *
     * @return true: this build was compiled with BIBO_WIRELESS
     */
    [[nodiscard]] static Bool present(Void)
    {
        return true;
    }

    /**
     * @brief The link's current state.
     *
     * @return one of the net::State values
     */
    inline State status(Void)
    {
        return stateNow;
    }

    /**
     * @brief Whether a peer has sent anything yet.
     *
     * @return true once at least one datagram has set the last-known sender
     */
    [[nodiscard]] static Bool peerKnown(Void)
    {
        return peerKnownNow;
    }

    /**
     * @brief How many queued lines have been discarded to make room for
     *        newer ones.
     *
     * @return the running count of lines dropped by queuePush()
     */
    inline UInt32 droppedCount(Void)
    {
        return dropped;
    }

    /**
     * @brief Registers the function called with each complete line received.
     *
     * @param fn the handler to call from poll(), or nullptr to stop
     *           delivering lines
     */
    inline Void setLineHandler(const LineHandler fn)
    {
        handler = fn;
    }

    /**
     * @brief The car's address on the network, or "-" before it has one.
     *
     * Points into lwIP's own static formatting buffer, so it is valid until the
     * next call. Print it, do not keep it.
     *
     * @return the dotted-quad address, or "-" before the link is up
     */
    inline CharSeq address(Void)
    {
        if(stateNow != STATE_UP || netif_default == nullptr)
        {
            return "-";
        }
        return ip4addr_ntoa(netif_ip4_addr(netif_default));
    }

    /**
     * @brief Appends one finished line to the queue, dropping the oldest one
     *        if it is full.
     *
     * @param text the line's bytes, not necessarily nul-terminated
     * @param len number of bytes in text; zero is silently ignored
     */
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
         * Full: drop the OLDEST, not this one. On a control link the freshest
         * command is the one that matters, so discarding arrivals instead would
         * be exactly backwards. Losing a stale keepalive costs nothing.
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

    /**
     * @brief Splits a chunk of received bytes into lines and queues each one.
     *
     * @param data the bytes to consume
     * @param len number of bytes in data
     */
    inline Void feed(const Utf8* data, const Size len)
    {
        for(Size i = 0; i < len; ++i)
        {
            if(const Utf8 c = data[i]; c == '\n' || c == '\r')
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
                /*
                 * Over-long. Drop the whole line rather than let its tail become
                 * a command nobody sent, as the USB reader in main.c does.
                 */
                partialLen = 0;
            }
        }

        /*
         * A datagram that ends without a newline is a COMPLETE line: UDP
         * preserves message boundaries, and most senders - a one-line test from
         * a shell among them - do not bother with a trailing newline.
         */
        if(partialLen > 0)
        {
            queuePush(partial, partialLen);
            partialLen = 0;
        }
    }

    /**
     * @brief lwIP's UDP receive callback: feeds a datagram to feed() and
     *        records its sender.
     *
     * The signature is lwIP's, not ours - it is a callback the stack calls,
     * so every parameter is spelled the way udp_recv_fn declares it.
     *
     * @param arg the argument passed to udp_recv(); unused here
     * @param pcb the control block the datagram arrived on; unused here
     * @param p the received data; freed before this function returns
     * @param addr the sender's address, recorded so sendLine() can reply
     * @param port the sender's port, recorded so sendLine() can reply
     *
     * @note `arg` and `pcb` are NOT const, and cannot be made so. This has to
     *       match lwIP's udp_recv_fn exactly, and a const on a POINTER
     *       parameter is part of the function's type - unlike a const on a
     *       by-value one, which the signature ignores. `const Void*` and
     *       `Void*` are two different functions as far as udp_recv() is
     *       concerned, so const-qualifying them stops this compiling against
     *       the callback it exists to be:
     *
     *           net.hxx:317: invalid conversion from 'Void (*)(const Void*, ...)'
     *                        to 'udp_recv_fn'
     *
     *       `addr` keeps its const because lwIP declares it that way, and
     *       `port` keeps its because a top-level const on a value parameter
     *       is not part of the type.
     */
    inline Void onPacket(Void* arg, udp_pcb* pcb, pbuf* p, const ip_addr_t* addr, const u16_t port)
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

        /* pbufs can be chained; walking the chain is not optional, "small" being the sender's decision. */
        for(const pbuf* q = p; q != nullptr; q = q->next)
        {
            feed(static_cast<const Utf8*>(q->payload), q->len);
        }

        pbuf_free(p);
    }

    /**
     * @brief Brings the radio up in station mode. Does NOT join anything.
     *
     * Power management is turned OFF here, and that is not a detail. The
     * default parks the radio between beacons and adds latency spikes well
     * past a hundred milliseconds - which on a link that carries
     * hold-to-drive commands means the deadman fires mid-drive and it looks
     * like a fault in the car.
     *
     * @return true once the radio is up and the socket is bound; false on
     *         any failure, which also leaves status() at STATE_FAILED
     */
    [[nodiscard]] static Bool start(Void)
    {
        if(started)
        {
            return true;
        }

        /*
         * radio::open(), not cyw43_arch_init(). The LED hangs off the same chip
         * and main() brings it up at boot, so the radio is ALREADY initialized
         * by the time anybody asks for wireless. A second cyw43_arch_init()
         * returns no error and quietly leaves the chip half re-initialized.
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

    /**
     * @brief Starts joining a network. Returns immediately.
     *
     * Asynchronous on purpose: the blocking form of this call sits inside the
     * SDK for up to thirty seconds, and every one of those seconds is a
     * second the main loop is not running drive::pump(), not slewing an
     * output, and not honoring the deadman. A join that freezes the car is
     * a join nobody can make while the car is switched on.
     *
     * @param ssid the network name to join
     * @param pass the network's password, or nullptr/empty for an open
     *             network
     * @return true once the join has started; check status() for the
     *         outcome, since the connect itself finishes later in poll()
     */
    [[nodiscard]] static Bool join(const CharSeq ssid, const CharSeq pass)
    {
        if(!started && !start())
        {
            return false;
        }

        /* Copies, because the async connect keeps pointers to these. */
        snprintf(joinSsid, sizeof(joinSsid), "%s", ssid);
        snprintf(joinPass, sizeof(joinPass), "%s", pass != nullptr ? pass : "");

        const UInt32 auth = joinPass[0] == '\0'
                                ? CYW43_AUTH_OPEN
                                : CYW43_AUTH_WPA2_AES_PSK;

        /* nullptr rather than an empty string: the SDK tests the pointer, not what it points at. */

        if(const CharSeq key = joinPass[0] == '\0' ? nullptr : joinPass; cyw43_arch_wifi_connect_async(joinSsid, key, auth) != 0)
        {
            stateNow = STATE_FAILED;
            return false;
        }

        stateNow = STATE_JOINING;
        return true;
    }

    /**
     * @brief Sends one line out to whoever last spoke.
     *
     * Silently does nothing if nobody has, or if the link is not up.
     *
     * @param text the line to send
     */
    inline Void sendLine(const CharSeq text)
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

        pbuf* p = pbuf_alloc(PBUF_TRANSPORT, static_cast<u16_t>(len), PBUF_RAM);
        if(p == nullptr)
        {
            return;   /* out of buffers: dropping a reply beats blocking the loop */
        }

        memcpy(p->payload, text, len);
        udp_sendto(pcb, p, &peerAddr, peerPort);
        pbuf_free(p);
    }

    /**
     * @brief Pumps the stack and hands over anything that arrived.
     *
     * Call it every time round the main loop. In NO_SYS mode nothing in lwIP
     * happens on its own: no packet is received, no join completes and no
     * timer fires except inside this call.
     *
     * @note Must be called regularly for the link to make any progress -
     *       joining, receiving and sending all happen inside this call.
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
            if(const Int32 link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA); link == CYW43_LINK_UP)
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

    /**
     * @brief Whether this board has wireless hardware at all.
     *
     * @return false, always: this build has no radio
     */
    [[nodiscard]] static Bool present(Void)
    {
        return false;
    }

    /**
     * @brief The link's current state.
     *
     * @return net::STATE_ABSENT, always
     */
    inline State status(Void)
    {
        return STATE_ABSENT;
    }

    /**
     * @brief Whether a peer has sent anything yet.
     *
     * @return false, always: there is no radio to hear one
     */
    [[nodiscard]] static Bool peerKnown(Void)
    {
        return false;
    }

    /**
     * @brief How many queued lines have been discarded to make room for
     *        newer ones.
     *
     * @return 0, always: there is no queue on this board
     */
    inline UInt32 droppedCount(Void)
    {
        return 0;
    }

    /**
     * @brief Registers the function called with each complete line received.
     *
     * @param fn ignored: with no radio, no line ever arrives to hand it
     */
    inline Void setLineHandler(LineHandler fn)
    {
        static_cast<Void>(fn);
    }

    /**
     * @brief The car's address on the network, or "-" before it has one.
     *
     * @return "-", always: there is no radio to have an address
     */
    inline CharSeq address(Void)
    {
        return "-";
    }

    /**
     * @brief Brings the radio up in station mode. Does NOT join anything.
     *
     * @return false, always: there is no radio to bring up
     */
    [[nodiscard]] static Bool start(Void)
    {
        return false;
    }

    /**
     * @brief Starts joining a network. Returns immediately.
     *
     * @param ssid ignored: there is no radio to join with
     * @param pass ignored: there is no radio to join with
     * @return false, always
     */
    [[nodiscard]] static Bool join(CharSeq ssid, CharSeq pass)
    {
        static_cast<Void>(ssid);
        static_cast<Void>(pass);
        return false;
    }

    /**
     * @brief Sends one line out to whoever last spoke.
     *
     * @param text ignored: there is no radio to send it over
     */
    inline Void sendLine(CharSeq text)
    {
        static_cast<Void>(text);
    }

    /**
     * @brief Pumps the stack and hands over anything that arrived.
     *
     * Does nothing: there is no stack to pump on this board.
     */
    inline Void poll(Void)
    {
    }

#endif /* BIBO_WIRELESS */

    /**
     * @brief The human-readable word for a link state.
     *
     * @param s the state to name
     * @return a short lowercase word suitable for a status line
     */
    inline CharSeq stateWord(const State s)
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


  }

}