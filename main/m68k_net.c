/*
 * M68K Network Library Implementation
 * Compile with M68K cross-compiler and link with M68K applications
 * Communicates with bus controller via memory-mapped I/O
 */

#include "m68k_net.h"

// Network device register offsets
#define NET_REG_COMMAND     (NET_DEV_BASE + 0x00)
#define NET_REG_STATUS      (NET_DEV_BASE + 0x04)
#define NET_REG_SOCKET_ID   (NET_DEV_BASE + 0x08)
#define NET_REG_ADDR_TYPE   (NET_DEV_BASE + 0x0C)
#define NET_REG_ADDR_IP     (NET_DEV_BASE + 0x10)
#define NET_REG_ADDR_PORT   (NET_DEV_BASE + 0x14)
#define NET_REG_DATA_LEN    (NET_DEV_BASE + 0x18)
#define NET_REG_DATA_PTR    (NET_DEV_BASE + 0x1C)
#define NET_REG_FLAGS       (NET_DEV_BASE + 0x20)
#define NET_REG_RESULT      (NET_DEV_BASE + 0x24)

// Network commands
#define NET_CMD_SOCKET      0x01
#define NET_CMD_BIND        0x02
#define NET_CMD_LISTEN      0x03
#define NET_CMD_ACCEPT      0x04
#define NET_CMD_CONNECT     0x05
#define NET_CMD_SEND        0x06
#define NET_CMD_RECV        0x07
#define NET_CMD_SENDTO      0x08
#define NET_CMD_RECVFROM    0x09
#define NET_CMD_CLOSE       0x0A
#define NET_CMD_GETINFO     0x0D

// Helper macros for register access
#define NET_WRITE(reg, val) (*((volatile unsigned long*)(reg)) = (val))
#define NET_READ(reg)       (*((volatile unsigned long*)(reg)))

// Wait for operation to complete
static void net_wait_ready(void) {
    while (NET_READ(NET_REG_STATUS) & 0x02) {
        // Wait while busy
    }
}

SOCKET socket_create(int af, int type, int protocol) {
    net_wait_ready();
    
    // Set socket type
    NET_WRITE(NET_REG_ADDR_TYPE, (unsigned long)type);
    
    // Issue socket command
    NET_WRITE(NET_REG_COMMAND, NET_CMD_SOCKET);
    
    // Wait for completion
    net_wait_ready();
    
    // Read result (socket ID)
    return (SOCKET)NET_READ(NET_REG_RESULT);
}

int socket_bind(SOCKET sockfd, const sockaddr_in *addr, int addrlen) {
    if (!addr) return -1;
    
    net_wait_ready();
    
    NET_WRITE(NET_REG_SOCKET_ID, (unsigned long)sockfd);
    NET_WRITE(NET_REG_ADDR_IP, addr->sin_addr);
    NET_WRITE(NET_REG_ADDR_PORT, addr->sin_port);
    
    NET_WRITE(NET_REG_COMMAND, NET_CMD_BIND);
    
    net_wait_ready();
    
    return (int)NET_READ(NET_REG_RESULT);
}

int socket_listen(SOCKET sockfd, int backlog) {
    net_wait_ready();
    
    NET_WRITE(NET_REG_SOCKET_ID, (unsigned long)sockfd);
    NET_WRITE(NET_REG_FLAGS, (unsigned long)backlog);
    
    NET_WRITE(NET_REG_COMMAND, NET_CMD_LISTEN);
    
    net_wait_ready();
    
    return (int)NET_READ(NET_REG_RESULT);
}

SOCKET socket_accept(SOCKET sockfd, sockaddr_in *addr, int *addrlen) {
    net_wait_ready();
    
    NET_WRITE(NET_REG_SOCKET_ID, (unsigned long)sockfd);
    
    NET_WRITE(NET_REG_COMMAND, NET_CMD_ACCEPT);
    
    net_wait_ready();
    
    SOCKET new_sock = (SOCKET)NET_READ(NET_REG_RESULT);
    
    // Read client address if requested
    if (new_sock >= 0 && addr) {
        addr->sin_family = AF_INET;
        addr->sin_addr = NET_READ(NET_REG_ADDR_IP);
        addr->sin_port = (unsigned short)NET_READ(NET_REG_ADDR_PORT);
        if (addrlen) {
            *addrlen = sizeof(sockaddr_in);
        }
    }
    
    return new_sock;
}

int socket_connect(SOCKET sockfd, const sockaddr_in *addr, int addrlen) {
    if (!addr) return -1;
    
    net_wait_ready();
    
    NET_WRITE(NET_REG_SOCKET_ID, (unsigned long)sockfd);
    NET_WRITE(NET_REG_ADDR_IP, addr->sin_addr);
    NET_WRITE(NET_REG_ADDR_PORT, addr->sin_port);
    
    NET_WRITE(NET_REG_COMMAND, NET_CMD_CONNECT);
    
    net_wait_ready();
    
    return (int)NET_READ(NET_REG_RESULT);
}

int socket_send(SOCKET sockfd, const void *buf, int len, int flags) {
    if (!buf || len <= 0) return -1;
    
    net_wait_ready();
    
    NET_WRITE(NET_REG_SOCKET_ID, (unsigned long)sockfd);
    NET_WRITE(NET_REG_DATA_PTR, (unsigned long)buf);
    NET_WRITE(NET_REG_DATA_LEN, (unsigned long)len);
    NET_WRITE(NET_REG_FLAGS, (unsigned long)flags);
    
    NET_WRITE(NET_REG_COMMAND, NET_CMD_SEND);
    
    net_wait_ready();
    
    return (int)NET_READ(NET_REG_RESULT);
}

int socket_recv(SOCKET sockfd, void *buf, int len, int flags) {
    if (!buf || len <= 0) return -1;
    
    net_wait_ready();
    
    NET_WRITE(NET_REG_SOCKET_ID, (unsigned long)sockfd);
    NET_WRITE(NET_REG_DATA_PTR, (unsigned long)buf);
    NET_WRITE(NET_REG_DATA_LEN, (unsigned long)len);
    NET_WRITE(NET_REG_FLAGS, (unsigned long)flags);
    
    NET_WRITE(NET_REG_COMMAND, NET_CMD_RECV);
    
    net_wait_ready();
    
    return (int)NET_READ(NET_REG_RESULT);
}

int socket_sendto(SOCKET sockfd, const void *buf, int len, int flags,
                  const sockaddr_in *to, int tolen) {
    if (!buf || len <= 0 || !to) return -1;
    
    net_wait_ready();
    
    NET_WRITE(NET_REG_SOCKET_ID, (unsigned long)sockfd);
    NET_WRITE(NET_REG_DATA_PTR, (unsigned long)buf);
    NET_WRITE(NET_REG_DATA_LEN, (unsigned long)len);
    NET_WRITE(NET_REG_FLAGS, (unsigned long)flags);
    NET_WRITE(NET_REG_ADDR_IP, to->sin_addr);
    NET_WRITE(NET_REG_ADDR_PORT, to->sin_port);
    
    NET_WRITE(NET_REG_COMMAND, NET_CMD_SENDTO);
    
    net_wait_ready();
    
    return (int)NET_READ(NET_REG_RESULT);
}

int socket_recvfrom(SOCKET sockfd, void *buf, int len, int flags,
                    sockaddr_in *from, int *fromlen) {
    if (!buf || len <= 0) return -1;
    
    net_wait_ready();
    
    NET_WRITE(NET_REG_SOCKET_ID, (unsigned long)sockfd);
    NET_WRITE(NET_REG_DATA_PTR, (unsigned long)buf);
    NET_WRITE(NET_REG_DATA_LEN, (unsigned long)len);
    NET_WRITE(NET_REG_FLAGS, (unsigned long)flags);
    
    NET_WRITE(NET_REG_COMMAND, NET_CMD_RECVFROM);
    
    net_wait_ready();
    
    int result = (int)NET_READ(NET_REG_RESULT);
    
    // Read sender address if requested
    if (result > 0 && from) {
        from->sin_family = AF_INET;
        from->sin_addr = NET_READ(NET_REG_ADDR_IP);
        from->sin_port = (unsigned short)NET_READ(NET_REG_ADDR_PORT);
        if (fromlen) {
            *fromlen = sizeof(sockaddr_in);
        }
    }
    
    return result;
}

int socket_close(SOCKET sockfd) {
    net_wait_ready();
    
    NET_WRITE(NET_REG_SOCKET_ID, (unsigned long)sockfd);
    
    NET_WRITE(NET_REG_COMMAND, NET_CMD_CLOSE);
    
    net_wait_ready();
    
    return (int)NET_READ(NET_REG_RESULT);
}

unsigned long net_get_ip(void) {
    net_wait_ready();
    
    NET_WRITE(NET_REG_COMMAND, NET_CMD_GETINFO);
    
    net_wait_ready();
    
    return NET_READ(NET_REG_ADDR_IP);
}

unsigned long inet_addr(const char *cp) {
    unsigned long addr = 0;
    unsigned char parts[4];
    int part = 0;
    unsigned long val = 0;
    
    while (*cp && part < 4) {
        if (*cp >= '0' && *cp <= '9') {
            val = val * 10 + (*cp - '0');
        } else if (*cp == '.') {
            parts[part++] = (unsigned char)val;
            val = 0;
        } else {
            return 0xFFFFFFFF;  // Invalid
        }
        cp++;
    }
    
    if (part == 3) {
        parts[3] = (unsigned char)val;
        addr = (parts[0]) | (parts[1] << 8) | (parts[2] << 16) | (parts[3] << 24);
        return addr;
    }
    
    return 0xFFFFFFFF;
}

unsigned short htons(unsigned short hostshort) {
    return ((hostshort & 0xFF) << 8) | ((hostshort >> 8) & 0xFF);
}

unsigned short ntohs(unsigned short netshort) {
    return htons(netshort);  // Same operation
}

unsigned long htonl(unsigned long hostlong) {
    return ((hostlong & 0xFF) << 24) |
           ((hostlong & 0xFF00) << 8) |
           ((hostlong & 0xFF0000) >> 8) |
           ((hostlong >> 24) & 0xFF);
}

unsigned long ntohl(unsigned long netlong) {
    return htonl(netlong);  // Same operation
}
