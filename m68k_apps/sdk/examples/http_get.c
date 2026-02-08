/*
 * M68K SDK - Example: HTTP GET Request
 *
 * Demonstrates networking by making a simple HTTP GET request
 * to a specified IP address and port.
 *
 * Build:  ./build.sh examples/http_get.c
 * Run:    loadrun /sdcard/http_get.bin
 */

#include "m68k_sdk.h"

/* Configuration - modify for your server */
#define SERVER_IP     NET_IP(192,168,0,100)
#define SERVER_PORT   80
#define HTTP_HOST     "192.168.0.100"
#define HTTP_PATH     "/"

static char recv_buf[1024];

int main(void) {
    con_println("=== M68K HTTP GET Client ===");
    con_println("");

    /* Create TCP socket */
    con_print("Creating socket... ");
    int sock = net_socket(NET_AF_INET, NET_SOCK_STREAM, 0);
    if (sock < 0) {
        con_println("FAILED");
        return 1;
    }
    con_printf("OK (fd=%d)\n", sock);

    /* Connect */
    con_printf("Connecting to %s:%d... ", HTTP_HOST, SERVER_PORT);
    int ret = net_connect(sock, SERVER_IP, SERVER_PORT);
    if (ret < 0) {
        con_println("FAILED");
        net_close(sock);
        return 1;
    }
    con_println("OK");

    /* Build HTTP request */
    char request[256];
    m68k_snprintf(request, sizeof(request),
        "GET %s HTTP/1.0\r\n"
        "Host: %s\r\n"
        "Connection: close\r\n"
        "\r\n",
        HTTP_PATH, HTTP_HOST);

    /* Send request */
    con_print("Sending request... ");
    int sent = net_send(sock, request, m68k_strlen(request));
    if (sent < 0) {
        con_println("FAILED");
        net_close(sock);
        return 1;
    }
    con_printf("OK (%d bytes)\n", sent);
    con_println("");

    /* Receive response */
    con_println("--- Response ---");
    int total = 0;
    while (1) {
        int n = net_recv(sock, recv_buf, sizeof(recv_buf) - 1);
        if (n <= 0) break;
        recv_buf[n] = '\0';
        con_print(recv_buf);
        total += n;
    }
    con_println("");
    con_println("--- End ---");
    con_printf("Total received: %d bytes\n", total);

    net_close(sock);
    return 0;
}
