/*
 * M68K SDK - Network Implementation
 */

#include "m68k_net.h"
#include "m68k_io.h"

int32_t net_socket(uint32_t type) {
    IO_WRITE32(NET_ADDR_TYPE, type);
    IO_WRITE32(NET_COMMAND, NET_CMD_SOCKET);
    return (int32_t)IO_READ32(NET_RESULT);
}

int32_t net_connect(int32_t sock_id, uint32_t ip, uint16_t port) {
    IO_WRITE32(NET_SOCKET_ID, (uint32_t)sock_id);
    IO_WRITE32(NET_ADDR_IP, ip);
    IO_WRITE32(NET_ADDR_PORT, (uint32_t)port);
    IO_WRITE32(NET_COMMAND, NET_CMD_CONNECT);
    return (int32_t)IO_READ32(NET_RESULT);
}

int32_t net_bind(int32_t sock_id, uint32_t ip, uint16_t port) {
    IO_WRITE32(NET_SOCKET_ID, (uint32_t)sock_id);
    IO_WRITE32(NET_ADDR_IP, ip);
    IO_WRITE32(NET_ADDR_PORT, (uint32_t)port);
    IO_WRITE32(NET_COMMAND, NET_CMD_BIND);
    return (int32_t)IO_READ32(NET_RESULT);
}

int32_t net_listen(int32_t sock_id, int backlog) {
    IO_WRITE32(NET_SOCKET_ID, (uint32_t)sock_id);
    IO_WRITE32(NET_FLAGS, (uint32_t)backlog);
    IO_WRITE32(NET_COMMAND, NET_CMD_LISTEN);
    return (int32_t)IO_READ32(NET_RESULT);
}

int32_t net_accept(int32_t sock_id) {
    IO_WRITE32(NET_SOCKET_ID, (uint32_t)sock_id);
    IO_WRITE32(NET_COMMAND, NET_CMD_ACCEPT);
    return (int32_t)IO_READ32(NET_RESULT);
}

int32_t net_send(int32_t sock_id, const void *data, uint32_t len) {
    IO_WRITE32(NET_SOCKET_ID, (uint32_t)sock_id);
    IO_WRITE32(NET_DATA_PTR, (uint32_t)data);
    IO_WRITE32(NET_DATA_LEN, len);
    IO_WRITE32(NET_COMMAND, NET_CMD_SEND);
    return (int32_t)IO_READ32(NET_RESULT);
}

int32_t net_recv(int32_t sock_id, void *buf, uint32_t len) {
    IO_WRITE32(NET_SOCKET_ID, (uint32_t)sock_id);
    IO_WRITE32(NET_DATA_PTR, (uint32_t)buf);
    IO_WRITE32(NET_DATA_LEN, len);
    IO_WRITE32(NET_COMMAND, NET_CMD_RECV);
    return (int32_t)IO_READ32(NET_RESULT);
}

int32_t net_sendto(int32_t sock_id, const void *data, uint32_t len,
                   uint32_t ip, uint16_t port) {
    IO_WRITE32(NET_SOCKET_ID, (uint32_t)sock_id);
    IO_WRITE32(NET_DATA_PTR, (uint32_t)data);
    IO_WRITE32(NET_DATA_LEN, len);
    IO_WRITE32(NET_ADDR_IP, ip);
    IO_WRITE32(NET_ADDR_PORT, (uint32_t)port);
    IO_WRITE32(NET_COMMAND, NET_CMD_SENDTO);
    return (int32_t)IO_READ32(NET_RESULT);
}

int32_t net_recvfrom(int32_t sock_id, void *buf, uint32_t len) {
    IO_WRITE32(NET_SOCKET_ID, (uint32_t)sock_id);
    IO_WRITE32(NET_DATA_PTR, (uint32_t)buf);
    IO_WRITE32(NET_DATA_LEN, len);
    IO_WRITE32(NET_COMMAND, NET_CMD_RECVFROM);
    return (int32_t)IO_READ32(NET_RESULT);
}

int32_t net_close(int32_t sock_id) {
    IO_WRITE32(NET_SOCKET_ID, (uint32_t)sock_id);
    IO_WRITE32(NET_COMMAND, NET_CMD_CLOSE);
    return (int32_t)IO_READ32(NET_RESULT);
}

int32_t net_ping(uint32_t ip, uint32_t count) {
    IO_WRITE32(NET_ADDR_IP, ip);
    IO_WRITE32(NET_DATA_LEN, count ? count : 4);
    IO_WRITE32(NET_COMMAND, NET_CMD_PING);
    return (int32_t)IO_READ32(NET_RESULT);
}

uint32_t net_get_remote_ip(void) {
    return IO_READ32(NET_ADDR_IP);
}

uint16_t net_get_remote_port(void) {
    return (uint16_t)IO_READ32(NET_ADDR_PORT);
}
