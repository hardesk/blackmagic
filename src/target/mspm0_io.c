#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>

#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>

#ifndef DEBUG_ERROR
void debug_error_mspm0(char const* msg, ...) {
    va_list vl;
    va_start(vl, msg);
    vprintf(msg, vl);
    va_end(vl);
}
#define DEBUG_ERROR debug_error_mspm0
#endif

enum mspm0_conn_state {
    k_conn_closed = 0,
    // k_ss_idle,
    k_conn_connecting0,
    k_conn_connecting1,
    k_conn_connected
};
enum mspm0_start_io_result {
    k_start_ok = 0,
    k_start_wait,
    k_start_retry
};

struct mspm0_io_device
{
    ssize_t (*send)(struct mspm0_io_device* self, char const* data, size_t size);
    enum mspm0_start_io_result (*connect)(struct mspm0_io_device* self);
    enum mspm0_conn_state state;
};

struct mspm0_io_file
{
    struct mspm0_io_device io;
    int fd;
};

static enum mspm0_start_io_result mspm0_io_file_connect(struct mspm0_io_device* self)
{
    struct mspm0_io_file* io = (struct mspm0_io_file*)self;
    io->io.state = k_conn_connected;
    return k_start_ok;
}

static ssize_t mspm0_io_file_write(struct mspm0_io_device* self, char const* data, size_t size)
{
    struct mspm0_io_file* io = (struct mspm0_io_file*)self;
    // return fwrite(data, 1, size, stdout);
    return write(io->fd, data, size);
}

struct mspm0_io_socket
{
    struct mspm0_io_device io;
    union {
        struct sockaddr_storage sa;
        struct sockaddr_in sa4;
        struct sockaddr_in6 sa6;
    } ss;
    int sock;
    errno_t err;
    enum mspm0_conn_state state;
    char const* host; /* ip only for now */
    int port;
};

static bool init_socket(struct mspm0_io_socket* io, char const* host, int port)
{
    if ((io->sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        io->err = errno;
        DEBUG_ERROR("failed to create a socket (%d)\n", io->err);
        goto fail;
    }

    // Set socket to non-blocking
    int flags = fcntl(io->sock, F_GETFL, 0);
    if (flags < 0) {
        io->err = errno;
        DEBUG_ERROR("failed to get socket flags (%d)\n", io->err);
        close(io->sock);
        goto fail;
    }
    if (fcntl(io->sock, F_SETFL, flags | O_NONBLOCK) == -1) {
        io->err = errno;
        DEBUG_ERROR("failed to set socket flags (%d)\n", io->err);
        close(io->sock);
        goto fail;
    }

    memset(&io->ss, 0, sizeof(io->ss));
    io->ss.sa4.sin_family = AF_INET;
    io->ss.sa4.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &io->ss.sa4.sin_addr) <= 0) {
        io->err = errno;
        DEBUG_ERROR("'%s' is an invalid address or address not supported (%d)", host, io->err);
        close(io->sock);
        goto fail;
    }

    return true;

fail:
    return false;
}

static enum mspm0_start_io_result mspm0_io_socket_connect(struct mspm0_io_device* self)
{
    struct fd_set writefds;
    struct mspm0_io_socket* io = (struct mspm0_io_socket*)self;
    do { // for a possible loop

#define SWITCH_STATE(new_state) do { \
    io->io.state = new_state; \
    goto again; \
    } while (false)

#define RETURN_STATE(new_state, result) do { \
    io->io.state = new_state; \
    return result; \
    } while (false)

again:
    io->err = 0;

    switch(io->io.state) {
    case k_conn_closed:
        if (init_socket(io, io->host, io->port)) {
            SWITCH_STATE(k_conn_connecting0);
        } else {
            io->err = errno;
            return k_start_wait;
        }
        break;

    case k_conn_connecting0:
        if (connect(io->sock, (const struct sockaddr*)&io->ss.sa4, sizeof(io->ss.sa4)) < 0) {
            if ((io->err = errno) == EINPROGRESS) {
                SWITCH_STATE(k_conn_connecting1);
                // return k_start_wait;
            } else {
                close(io->sock);
                RETURN_STATE(k_conn_closed, k_start_retry);
            }
            return k_start_wait;
        } else
            SWITCH_STATE(k_conn_connected);

    case k_conn_connecting1:
        FD_ZERO(&writefds);
        FD_SET(io->sock, &writefds);
        struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };
        int selres = select(io->sock + 1, NULL, &writefds, NULL, &timeout);
        if (selres > 0 && FD_ISSET(io->sock, &writefds)) {

            int so_error;
            socklen_t len = sizeof(so_error);
            if (getsockopt(io->sock, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0) {
                DEBUG_ERROR("getsockopt failed");
                close(io->sock);
                RETURN_STATE(k_conn_closed, k_start_retry);
            }

            if (so_error == 0) {
                // Retrieve and print the IP and port to which the socket is connected
                char ip_str[INET_ADDRSTRLEN] = {0};
                struct sockaddr_in peer_addr;
                socklen_t peer_addr_len = sizeof(peer_addr);
                if (getpeername(io->sock, (struct sockaddr *)&peer_addr, &peer_addr_len) != -1)
                    inet_ntop(AF_INET, &peer_addr.sin_addr, ip_str, sizeof(ip_str));
                printf("Connected successfully %s:%d\n", ip_str, ntohs(peer_addr.sin_port));
                SWITCH_STATE(k_conn_connected);
            }

            io->err = errno;
            // char str_errno[256], str_so_error[256];
            // strerror_r(io->err, str_errno, sizeof str_errno);
            // strerror_r(io->err, str_so_error, sizeof str_so_error);
            // printf("Failed to connect. so_error %d %s, closing socket (errno %d %s)\n", so_error, str_errno, io->err, str_so_error);

            close(io->sock);
            RETURN_STATE(k_conn_closed, k_start_wait);

        } else if (selres == 0) { // timeout
            return k_start_wait;
        } else {
            io->err = errno;
            close(io->sock);
            RETURN_STATE(k_conn_closed, k_start_retry);
        }
        break;

    case k_conn_connected: {
        return k_start_ok;
    } // case
    } // switch

    } while(false);

    return k_start_ok;
}

ssize_t mspm0_io_socket_send(struct mspm0_io_device* self, char const* data, size_t size)
{
    struct mspm0_io_socket* io = (struct mspm0_io_socket*)self;
    int sent;
    if ( (sent = send(io->sock, data, size, MSG_NOSIGNAL) ) < 0) {
        io->err = errno;
        printf("Closed connection (%d)\n", io->err);
        close(io->sock);
        RETURN_STATE(k_conn_closed, 0);
    }

    return sent;
}

