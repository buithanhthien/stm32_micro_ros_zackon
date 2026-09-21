#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <errno.h>

// --- LWIP ---
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include <lwip/sockets.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UDP_PORT        8888
static int sock_fd = -1;

//default -- blocking method
bool cubemx_transport_open(struct uxrCustomTransport *transport)
{
    (void)transport;

    // ========================================================
    // Nếu socket cũ vẫn còn thì đóng trước.
    // Không cho phép leak socket qua các lần reconnect.
    // ========================================================

    if (sock_fd >= 0)
    {
        closesocket(sock_fd);
        sock_fd = -1;
    }

    // ========================================================
    // Tạo UDP socket mới
    // ========================================================

    sock_fd = socket(
        AF_INET,
        SOCK_DGRAM,
        0
    );

    if (sock_fd < 0)
    {
        sock_fd = -1;

        return false;
    }

    // ========================================================
    // Cho phép tái sử dụng socket
    // ========================================================

    int reuse = 1;

    (void)setsockopt(
        sock_fd,
        SOL_SOCKET,
        SO_REUSEADDR,
        &reuse,
        sizeof(reuse)
    );

    // ========================================================
    // Bind LOCAL port tự động.
    //
    // Agent vẫn là port 8888.
    // STM32 không cần dùng local port 8888.
    // ========================================================

    struct sockaddr_in local_addr;

    memset(
        &local_addr,
        0,
        sizeof(local_addr)
    );

    local_addr.sin_family = AF_INET;

    // Port 0:
    // LwIP tự cấp một ephemeral port.
    local_addr.sin_port = htons(0);

    local_addr.sin_addr.s_addr =
        htonl(INADDR_ANY);

    if (
        bind(
            sock_fd,
            (struct sockaddr *)&local_addr,
            sizeof(local_addr)
        ) < 0
    )
    {
        // Cực kỳ quan trọng:
        // bind fail phải giải phóng socket.
        closesocket(sock_fd);

        sock_fd = -1;

        return false;
    }

    return true;
}

//non-blocking
//bool cubemx_transport_open(struct uxrCustomTransport * transport)
//{
//    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
//    if (sock_fd < 0) return false;
//
//    struct sockaddr_in addr;
//    memset(&addr, 0, sizeof(addr));
//    addr.sin_family = AF_INET;
//    addr.sin_port = htons(UDP_PORT);
//    addr.sin_addr.s_addr = htonl(INADDR_ANY);
//
//    int one = 1;
//    setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
//
//    if (bind(sock_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
//        closesocket(sock_fd);
//        sock_fd = -1;
//        return false;
//    }
//
//    // Non-blocking
//    int flags = fcntl(sock_fd, F_GETFL, 0);
//    if (flags >= 0) {
//        (void)fcntl(sock_fd, F_SETFL, flags | O_NONBLOCK);
//    }
//
//    int rxbuf = 16 * 1024;
//    setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF, &rxbuf, sizeof(rxbuf));
//
//    return true;
//}

bool cubemx_transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;

    if (sock_fd >= 0)
    {
        closesocket(sock_fd);
        sock_fd = -1;
    }

    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                              uint8_t * buf, size_t len, uint8_t * err)
{
    if (sock_fd < 0)
    {
        *err = 1;
        return 0;
    }

    const char * ip_addr = (const char*) transport->args;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    addr.sin_addr.s_addr = inet_addr(ip_addr);

    if (addr.sin_addr.s_addr == INADDR_NONE)
    {
        *err = 1;
        return 0;
    }

    int ret = sendto(sock_fd, buf, (int)len, 0,
                     (struct sockaddr *)&addr, sizeof(addr));

    if (ret > 0)
    {
        *err = 0;
        return (size_t)ret;
    }

    // --- ERROR PATH ---
    *err = 1;
    return 0;
}

//Ham write ngay 19/04/2026////////////////////////////////////////////////////
//size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
//    if (sock_fd == -1)
//    {
//        return 0;
//    }
//    const char * ip_addr = (const char*) transport->args;
//    struct sockaddr_in addr;
//    addr.sin_family = AF_INET;
//    addr.sin_port = htons(UDP_PORT);
//    addr.sin_addr.s_addr = inet_addr(ip_addr);
//    int ret = 0;
//    ret = sendto(sock_fd, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
//    size_t writed = ret>0? ret:0;
//
//    return writed;
//}

//size_t cubemx_transport_write(struct uxrCustomTransport* transport,
//                              uint8_t* buf, size_t len, uint8_t* err)
//{
//    if (sock_fd < 0) { *err = 1; return 0; }
//
//    const char * ip_addr = (const char*) transport->args;
//    struct sockaddr_in addr;
//    memset(&addr, 0, sizeof(addr));
//    addr.sin_family = AF_INET;
//    addr.sin_port = htons(UDP_PORT);
//    addr.sin_addr.s_addr = inet_addr(ip_addr);
//
//    int ret = sendto(sock_fd, buf, (int)len, 0,
//                     (struct sockaddr *)&addr, sizeof(addr));
//
//    if (ret > 0) { *err = 0; return (size_t)ret; }
//    *err = 1;
//    return 0;
//}

//size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
//
//    int ret = 0;
//    //set timeout
//    struct timeval tv_out;
//    tv_out.tv_sec = timeout / 1000;
//    tv_out.tv_usec = (timeout % 1000) * 1000;
//    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO,&tv_out, sizeof(tv_out));
//    ret = recv(sock_fd, buf, len, MSG_WAITALL);
//    size_t readed = ret > 0 ? ret : 0;
//    return readed;
//}

size_t cubemx_transport_read(struct uxrCustomTransport* transport,
                             uint8_t* buf, size_t len, int timeout, uint8_t* err)
{
    (void)transport;

    if (sock_fd < 0) { *err = 1; return 0; }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock_fd, &rfds);

    struct timeval tv;
    tv.tv_sec  = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

    int sel = select(sock_fd + 1, &rfds, NULL, NULL, (timeout >= 0) ? &tv : NULL);
    if (sel == 0) {          // timeout, no data
        *err = 0;
        return 0;
    }
    if (sel < 0) {           // select error
        *err = 1;
        return 0;
    }

    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);

    int ret = recvfrom(sock_fd, buf, (int)len, 0,
                       (struct sockaddr*)&from, &fromlen);

    if (ret > 0) { *err = 0; return (size_t)ret; }

    if (ret < 0 && (errno == EWOULDBLOCK || errno == EAGAIN)) {
        *err = 0;
        return 0;
    }

    *err = (ret < 0) ? 1 : 0;
    return 0;
}

#endif
