/*
 * M68K SDK - Network API
 * ======================
 * BSD-style socket API using the bus controller network device.
 * The host ESP32-P4 translates these into lwIP socket operations.
 */

#ifndef M68K_NET_H
#define M68K_NET_H

#include "m68k_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IP address construction helper: net_ip(192,168,0,1) */
#define NET_IP(a, b, c, d) \
    (((uint32_t)(a) << 24) | ((uint32_t)(b) << 16) | \
     ((uint32_t)(c) << 8)  | (uint32_t)(d))

/* Create a socket. Returns socket ID (>= 0) or -1 on error.
 * type: NET_SOCK_STREAM (TCP), NET_SOCK_DGRAM (UDP) */
int32_t net_socket(uint32_t type);

/* Connect a TCP socket to a remote host.
 * ip: target IP (use NET_IP macro), port: target port.
 * Returns 0 on success, -1 on error. */
int32_t net_connect(int32_t sock_id, uint32_t ip, uint16_t port);

/* Bind a socket to a local port.
 * ip: local IP (0 for any), port: local port.
 * Returns 0 on success, -1 on error. */
int32_t net_bind(int32_t sock_id, uint32_t ip, uint16_t port);

/* Start listening for incoming connections (TCP server).
 * backlog: max pending connections.
 * Returns 0 on success, -1 on error. */
int32_t net_listen(int32_t sock_id, int backlog);

/* Accept an incoming connection. Returns new socket ID or -1.
 * After accept, remote IP and port can be read from the device registers. */
int32_t net_accept(int32_t sock_id);

/* Send data on a connected socket.
 * Returns bytes sent or -1 on error. */
int32_t net_send(int32_t sock_id, const void *data, uint32_t len);

/* Receive data from a connected socket.
 * Returns bytes received, 0 if connection closed, or -1 on error. */
int32_t net_recv(int32_t sock_id, void *buf, uint32_t len);

/* Send UDP datagram to specified address. */
int32_t net_sendto(int32_t sock_id, const void *data, uint32_t len,
                   uint32_t ip, uint16_t port);

/* Receive UDP datagram. Sender address stored in device registers. */
int32_t net_recvfrom(int32_t sock_id, void *buf, uint32_t len);

/* Close a socket. */
int32_t net_close(int32_t sock_id);

/* Ping a host. Returns average RTT in ms, or -1 if unreachable.
 * count: number of pings (default 4 if 0). */
int32_t net_ping(uint32_t ip, uint32_t count);

/* Get the remote IP address from last accept/recvfrom. */
uint32_t net_get_remote_ip(void);

/* Get the remote port from last accept/recvfrom. */
uint16_t net_get_remote_port(void);

#ifdef __cplusplus
}
#endif

#endif /* M68K_NET_H */
