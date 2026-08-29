/*
 * lwIP configuration.
 *
 * lwIP has no build system of its own worth the name: it is configured by a
 * header called exactly this, found on the include path, and every option it
 * does not find here falls back to a default in lwip/opt.h. That is why this
 * file sits at the firmware root rather than in lib/ - it is not part of the
 * tt02 library, it is a contract with somebody else's code, and a reader
 * looking for "what does this project provide" should not trip over it.
 *
 * NO_SYS mode: no threads, no operating system, everything driven from the one
 * main loop by cyw43_arch_poll(). That matches how this firmware already works
 * - one loop, one place things happen - and it is why net.h can be plain
 * inline C with no locking anywhere in it.
 *
 * Trimmed to what a UDP command link actually needs. TCP stays in because the
 * cyw43 glue and lwIP's own defaults assume it and turning it off buys a few
 * kilobytes in exchange for a build that fails in somebody else's file.
 */
#ifndef TT02_LWIPOPTS_H
#define TT02_LWIPOPTS_H

/* ---- no operating system ------------------------------------------------ */

#define NO_SYS                      1
#define LWIP_SOCKET                 0   /* no BSD sockets - raw API only */
#define LWIP_NETCONN                0   /* nor the sequential API */

/* ---- memory --------------------------------------------------------------
 *
 * The RP2350 has 520 KB and this program uses very little of it, so these are
 * the ordinary pico-examples numbers rather than anything squeezed. If packets
 * ever start being dropped under load, PBUF_POOL_SIZE is the first dial.
 */
#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4000
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24

/* ---- protocols ---------------------------------------------------------- */

#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1   /* so the laptop can ping the car */
#define LWIP_RAW                    1
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_UDP                    1
#define LWIP_TCP                    1
#define LWIP_DNS                    0   /* nothing here resolves a name */

/*
 * The DHCP CLIENT, which is how the car gets an address from the house router.
 *
 * There is no DHCP server here and there cannot easily be one: lwIP ships a
 * client only, and the server every Pico access-point example uses lives in
 * pico-examples, not in the SDK. That is the whole reason this link joins an
 * existing network instead of making its own - see net.h.
 */
#define LWIP_DHCP                   1
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0

/* ---- TCP sizing (defaults, kept explicit so they are visible) ------------ */

#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))

/* ---- callbacks the link status depends on ------------------------------- */

#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1

/* ---- statistics ---------------------------------------------------------
 *
 * Off. They cost RAM and a scattering of counters, and nothing in this program
 * reads them. Turn LWIP_STATS on if a packet ever goes missing and the answer
 * is not obvious.
 */
#define LWIP_STATS                  0
#define LWIP_STATS_DISPLAY          0

/* ---- checksums ---------------------------------------------------------- */

#define LWIP_CHKSUM_ALGORITHM       3

#endif /* TT02_LWIPOPTS_H */
