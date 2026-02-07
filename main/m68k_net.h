/*
 * M68K Network Library
 * Provides BSD socket-like API for M68K programs
 * Link this with your M68K applications
 */

#ifndef M68K_NET_H
#define M68K_NET_H

// Network device base address
#define NET_DEV_BASE 0x00F00000

// Socket API types
typedef int SOCKET;

#define INVALID_SOCKET -1
#define SOCKET_ERROR   -1

// Socket types
#define SOCK_STREAM 0x01
#define SOCK_DGRAM  0x02

// Address families
#define AF_INET     0x02

// Network address structure
typedef struct {
    unsigned short sin_family;
    unsigned short sin_port;
    unsigned long  sin_addr;
    char           sin_zero[8];
} sockaddr_in;

// Function prototypes for M68K programs

/**
 * Create a socket
 * @param af Address family (AF_INET)
 * @param type Socket type (SOCK_STREAM or SOCK_DGRAM)
 * @param protocol Protocol (usually 0)
 * @return Socket ID or INVALID_SOCKET on error
 */
SOCKET socket_create(int af, int type, int protocol);

/**
 * Bind socket to local address
 * @param sockfd Socket ID
 * @param addr Local address to bind to
 * @param addrlen Size of address structure
 * @return 0 on success, -1 on error
 */
int socket_bind(SOCKET sockfd, const sockaddr_in *addr, int addrlen);

/**
 * Listen for incoming connections (TCP)
 * @param sockfd Socket ID
 * @param backlog Maximum number of pending connections
 * @return 0 on success, -1 on error
 */
int socket_listen(SOCKET sockfd, int backlog);

/**
 * Accept incoming connection (TCP)
 * @param sockfd Listening socket ID
 * @param addr Pointer to store client address
 * @param addrlen Pointer to address length
 * @return New socket ID for connection, or INVALID_SOCKET on error
 */
SOCKET socket_accept(SOCKET sockfd, sockaddr_in *addr, int *addrlen);

/**
 * Connect to remote server (TCP)
 * @param sockfd Socket ID
 * @param addr Remote address to connect to
 * @param addrlen Size of address structure
 * @return 0 on success, -1 on error
 */
int socket_connect(SOCKET sockfd, const sockaddr_in *addr, int addrlen);

/**
 * Send data over TCP connection
 * @param sockfd Socket ID
 * @param buf Data buffer
 * @param len Length of data
 * @param flags Send flags (usually 0)
 * @return Number of bytes sent, or -1 on error
 */
int socket_send(SOCKET sockfd, const void *buf, int len, int flags);

/**
 * Receive data from TCP connection
 * @param sockfd Socket ID
 * @param buf Buffer to store received data
 * @param len Maximum length to receive
 * @param flags Receive flags (usually 0)
 * @return Number of bytes received, 0 if connection closed, -1 on error
 */
int socket_recv(SOCKET sockfd, void *buf, int len, int flags);

/**
 * Send UDP datagram
 * @param sockfd Socket ID
 * @param buf Data buffer
 * @param len Length of data
 * @param flags Send flags (usually 0)
 * @param to Destination address
 * @param tolen Size of address structure
 * @return Number of bytes sent, or -1 on error
 */
int socket_sendto(SOCKET sockfd, const void *buf, int len, int flags,
                  const sockaddr_in *to, int tolen);

/**
 * Receive UDP datagram
 * @param sockfd Socket ID
 * @param buf Buffer to store received data
 * @param len Maximum length to receive
 * @param flags Receive flags (usually 0)
 * @param from Pointer to store sender address
 * @param fromlen Pointer to address length
 * @return Number of bytes received, or -1 on error
 */
int socket_recvfrom(SOCKET sockfd, void *buf, int len, int flags,
                    sockaddr_in *from, int *fromlen);

/**
 * Close socket
 * @param sockfd Socket ID
 * @return 0 on success, -1 on error
 */
int socket_close(SOCKET sockfd);

/**
 * Get local IP address
 * @return IP address as 32-bit value
 */
unsigned long net_get_ip(void);

/**
 * Convert IP address from "a.b.c.d" string format
 * @param cp IP address string
 * @return IP address as 32-bit value
 */
unsigned long inet_addr(const char *cp);

/**
 * Convert port to network byte order
 */
unsigned short htons(unsigned short hostshort);

/**
 * Convert port from network byte order
 */
unsigned short ntohs(unsigned short netshort);

/**
 * Convert IP to network byte order
 */
unsigned long htonl(unsigned long hostlong);

/**
 * Convert IP from network byte order
 */
unsigned long ntohl(unsigned long netlong);

#endif // M68K_NET_H
