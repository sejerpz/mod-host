/*
 * This file is part of mod-system-control.
 *
 * /sys_msgs over TCP, where there is no POSIX shared memory (Windows) or where the two sides do not share it
 * (mod-host in mod-desktop, the HMI in mod-hw-emulator).
 *
 * Each side keeps the shared memory layout in its own memory, so the code around sys_serial_open() does not
 * change: it writes and reads the two ring buffers and waits on their semaphores as before. Two threads bridge
 * them to the socket: one sends what this side writes to the other side's channel, the other receives what the
 * other side writes and puts it in this side's channel, posting its semaphore at the end of each message. On the
 * wire a message is the bytes it is in a ring buffer: the event type, the page and the subpage from the client,
 * then the text up to its null byte.
 *
 * MOD_SYS_SERIAL_PORT=tcp://HOST:PORT selects it: the server (mod-system-control) listens there, the client
 * (mod-host) connects there. On Windows it is the only transport, on tcp://127.0.0.1:9901 by default.
 *
 * The server must be up first, as with shared memory; then the client reconnects when the server restarts and
 * the server takes a new client when the old one goes away.
 */

#pragma once

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sys_serial_tcp_socket;
#define SYS_SERIAL_TCP_INVALID INVALID_SOCKET
#define SYS_SERIAL_TCP_SEND_FLAGS 0
#define sys_serial_tcp_closesocket closesocket
#else
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int sys_serial_tcp_socket;
#define SYS_SERIAL_TCP_INVALID (-1)
#ifdef MSG_NOSIGNAL
#define SYS_SERIAL_TCP_SEND_FLAGS MSG_NOSIGNAL
#else
#define SYS_SERIAL_TCP_SEND_FLAGS 0
#endif
#define sys_serial_tcp_closesocket close
#endif

#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

// what sys_serial_open() hands to sys_serial_close() for this transport, in place of a shared memory fd
#define SYS_SERIAL_TCP_FD (-2)

typedef struct {
    // first, the pointer sys_serial_open() hands out
    sys_serial_shm_data data;
    char host[256], port[16];
    volatile bool running;
    sys_serial_tcp_socket listener, sock;
    pthread_mutex_t sock_mutex;
    pthread_t sender, receiver;
} sys_serial_tcp;

// the address to use, or false for the shared memory
static inline
bool sys_serial_tcp_address(char host[256], char port[16])
{
    const char* addr = getenv("MOD_SYS_SERIAL_PORT");
#ifdef _WIN32
    if (addr == NULL || *addr == '\0')
        addr = "tcp://127.0.0.1:9901";
#else
    if (addr == NULL || *addr == '\0')
        return false;
#endif
    if (strncmp(addr, "tcp://", 6) == 0)
        addr += 6;

    const char* const colon = strrchr(addr, ':');
    if (colon == NULL)
    {
        snprintf(host, 256, "127.0.0.1");
        snprintf(port, 16, "%s", addr);
    }
    else
    {
        snprintf(host, 256, "%.*s", (int)(colon - addr), addr);
        snprintf(port, 16, "%s", colon + 1);
        if (host[0] == '\0')
            snprintf(host, 256, "127.0.0.1");
    }
    return true;
}

static inline
void sys_serial_tcp_sleep_ms(const int ms)
{
#ifdef _WIN32
    Sleep(ms);
#else
    const struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
#endif
}

// waits up to 200 ms for SOCK to be readable, so the threads see `running` go down
static inline
int sys_serial_tcp_wait_readable(const sys_serial_tcp_socket sock)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    struct timeval tv = { 0, 200 * 1000 };
    return select((int)sock + 1, &fds, NULL, NULL, &tv);
}

static inline
sys_serial_tcp_socket sys_serial_tcp_connect_or_listen(const char* const host, const char* const port, const bool listening)
{
    struct addrinfo hints, *res = NULL, *ai;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = listening ? AI_PASSIVE : 0;

    if (getaddrinfo(host, port, &hints, &res) != 0)
        return SYS_SERIAL_TCP_INVALID;

    sys_serial_tcp_socket sock = SYS_SERIAL_TCP_INVALID;

    for (ai = res; ai != NULL; ai = ai->ai_next)
    {
        sock = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (sock == SYS_SERIAL_TCP_INVALID)
            continue;

        if (listening)
        {
#ifndef _WIN32
            const int yes = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));
#endif
            if (bind(sock, ai->ai_addr, (int)ai->ai_addrlen) == 0 && listen(sock, 1) == 0)
                break;
        }
        else
        {
            if (connect(sock, ai->ai_addr, (int)ai->ai_addrlen) == 0)
            {
                const int yes = 1;
                setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (const char*)&yes, sizeof(yes));
#if defined(__APPLE__) && defined(SO_NOSIGPIPE)
                setsockopt(sock, SOL_SOCKET, SO_NOSIGPIPE, (const char*)&yes, sizeof(yes));
#endif
                break;
            }
        }

        sys_serial_tcp_closesocket(sock);
        sock = SYS_SERIAL_TCP_INVALID;
    }

    freeaddrinfo(res);
    return sock;
}

static inline
void sys_serial_tcp_disconnect(sys_serial_tcp* const tcp)
{
    pthread_mutex_lock(&tcp->sock_mutex);
    if (tcp->sock != SYS_SERIAL_TCP_INVALID)
    {
        sys_serial_tcp_closesocket(tcp->sock);
        tcp->sock = SYS_SERIAL_TCP_INVALID;
    }
    pthread_mutex_unlock(&tcp->sock_mutex);
}

// the channel this side writes, and the one it reads
static inline
sys_serial_shm_data_channel* sys_serial_tcp_out(sys_serial_tcp* const tcp)
{
#ifdef SERVER_MODE
    return &tcp->data.client;
#else
    return &tcp->data.server;
#endif
}

static inline
sys_serial_shm_data_channel* sys_serial_tcp_in(sys_serial_tcp* const tcp)
{
#ifdef SERVER_MODE
    return &tcp->data.server;
#else
    return &tcp->data.client;
#endif
}

// sends what this side writes; drops it while there is no connection, as nobody reads the shared memory then
static inline
void* sys_serial_tcp_sender_run(void* const arg)
{
    sys_serial_tcp* const tcp = (sys_serial_tcp*)arg;
    sys_serial_shm_data_channel* const out = sys_serial_tcp_out(tcp);

    while (tcp->running)
    {
        if (out->head == out->tail && sem_timedwait_secs(&out->sem, 1) != 0)
            continue;

        const uint32_t head = out->head;
        uint32_t tail = out->tail;

        pthread_mutex_lock(&tcp->sock_mutex);
        while (tail != head && tcp->sock != SYS_SERIAL_TCP_INVALID)
        {
            const uint32_t end = head > tail ? head : SYS_SERIAL_SHM_DATA_SIZE;
            const int sent = send(tcp->sock, (const char*)out->buffer + tail, (int)(end - tail), SYS_SERIAL_TCP_SEND_FLAGS);
            if (sent <= 0)
                break;
            tail += (uint32_t)sent;
            if (tail == SYS_SERIAL_SHM_DATA_SIZE)
                tail = 0;
        }
        pthread_mutex_unlock(&tcp->sock_mutex);

        // sent, or nobody to send it to
        out->tail = head;
    }

    return NULL;
}

// receives what the other side writes; the reader sees a message once it is whole
static inline
void* sys_serial_tcp_receiver_run(void* const arg)
{
    sys_serial_tcp* const tcp = (sys_serial_tcp*)arg;
    sys_serial_shm_data_channel* const in = sys_serial_tcp_in(tcp);
#ifdef SERVER_MODE
    // event type, page and subpage come before the text
    const uint32_t header = 3;
#else
    const uint32_t header = 1;
#endif
    uint8_t buf[1024];
    uint32_t head = in->head, pos = 0;

    while (tcp->running)
    {
        if (tcp->sock == SYS_SERIAL_TCP_INVALID)
        {
#ifdef SERVER_MODE
            if (sys_serial_tcp_wait_readable(tcp->listener) <= 0)
                continue;
            const sys_serial_tcp_socket sock = accept(tcp->listener, NULL, NULL);
            if (sock == SYS_SERIAL_TCP_INVALID)
                continue;
            const int yes = 1;
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (const char*)&yes, sizeof(yes));
#if defined(__APPLE__) && defined(SO_NOSIGPIPE)
            setsockopt(sock, SOL_SOCKET, SO_NOSIGPIPE, (const char*)&yes, sizeof(yes));
#endif
#else
            sys_serial_tcp_sleep_ms(500);
            const sys_serial_tcp_socket sock = sys_serial_tcp_connect_or_listen(tcp->host, tcp->port, false);
            if (sock == SYS_SERIAL_TCP_INVALID)
                continue;
#endif
            pthread_mutex_lock(&tcp->sock_mutex);
            tcp->sock = sock;
            pthread_mutex_unlock(&tcp->sock_mutex);

            // a message cut by the old connection is dropped
            head = in->head;
            pos = 0;
        }

        const int ready = sys_serial_tcp_wait_readable(tcp->sock);
        if (ready == 0)
            continue;

        const int got = ready > 0 ? recv(tcp->sock, (char*)buf, sizeof(buf), 0) : -1;
        if (got <= 0)
        {
            // the other side went away: the server waits for a new client, the client connects again
            sys_serial_tcp_disconnect(tcp);
            continue;
        }

        for (int i = 0; i < got && tcp->running; ++i)
        {
            uint32_t next = head + 1;
            if (next == SYS_SERIAL_SHM_DATA_SIZE)
                next = 0;

            // full: wait for the reader
            while (next == in->tail && tcp->running)
                sys_serial_tcp_sleep_ms(1);

            in->buffer[head] = buf[i];
            head = next;

            if (++pos > header && buf[i] == '\0')
            {
                __sync_synchronize();
                in->head = head;
                pos = 0;
                sem_post(&in->sem);
            }
        }
    }

    return NULL;
}

static inline
bool sys_serial_tcp_open(int* const shmfd, sys_serial_shm_data** const data, const char* const host, const char* const port)
{
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
        return false;
#endif

    sys_serial_tcp* const tcp = (sys_serial_tcp*)calloc(1, sizeof(sys_serial_tcp));
    if (tcp == NULL)
        goto fail;

    snprintf(tcp->host, sizeof(tcp->host), "%s", host);
    snprintf(tcp->port, sizeof(tcp->port), "%s", port);
    tcp->listener = SYS_SERIAL_TCP_INVALID;
    tcp->sock = SYS_SERIAL_TCP_INVALID;

#ifdef SERVER_MODE
    tcp->listener = sys_serial_tcp_connect_or_listen(host, port, true);
    if (tcp->listener == SYS_SERIAL_TCP_INVALID)
    {
        fprintf(stderr, "sys_serial: cannot listen on %s:%s\n", host, port);
        goto fail_free;
    }
#else
    tcp->sock = sys_serial_tcp_connect_or_listen(host, port, false);
    if (tcp->sock == SYS_SERIAL_TCP_INVALID)
    {
        fprintf(stderr, "sys_serial: cannot connect to %s:%s\n", host, port);
        goto fail_free;
    }
#endif

    // both sides in this process only
    sem_init(&tcp->data.server.sem, 0, 0);
    sem_init(&tcp->data.client.sem, 0, 0);
    pthread_mutex_init(&tcp->sock_mutex, NULL);

    tcp->running = true;
    pthread_create(&tcp->sender, NULL, sys_serial_tcp_sender_run, tcp);
    pthread_create(&tcp->receiver, NULL, sys_serial_tcp_receiver_run, tcp);

    *shmfd = SYS_SERIAL_TCP_FD;
    *data = &tcp->data;
    return true;

fail_free:
    if (tcp->listener != SYS_SERIAL_TCP_INVALID)
        sys_serial_tcp_closesocket(tcp->listener);
    if (tcp->sock != SYS_SERIAL_TCP_INVALID)
        sys_serial_tcp_closesocket(tcp->sock);
    free(tcp);
fail:
#ifdef _WIN32
    WSACleanup();
#endif
    *shmfd = 0;
    *data = NULL;
    return false;
}

static inline
void sys_serial_tcp_close(sys_serial_shm_data* const data)
{
    sys_serial_tcp* const tcp = (sys_serial_tcp*)data;

    tcp->running = false;
    sem_post(&sys_serial_tcp_out(tcp)->sem);
    pthread_join(tcp->sender, NULL);
    pthread_join(tcp->receiver, NULL);

    sys_serial_tcp_disconnect(tcp);
    if (tcp->listener != SYS_SERIAL_TCP_INVALID)
        sys_serial_tcp_closesocket(tcp->listener);

    sem_destroy(&tcp->data.server.sem);
    sem_destroy(&tcp->data.client.sem);
    pthread_mutex_destroy(&tcp->sock_mutex);
    free(tcp);

#ifdef _WIN32
    WSACleanup();
#endif
}
