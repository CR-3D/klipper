// Test shim for macOS: poll() does not support tty devices - use select()
#include_next <poll.h>
#include <stddef.h>
#include <sys/select.h>
static inline int shim_poll(struct pollfd *fds, nfds_t n, int timeout)
{
    fd_set r, w;
    FD_ZERO(&r);
    FD_ZERO(&w);
    int maxfd = -1;
    for (nfds_t i = 0; i < n; i++) {
        fds[i].revents = 0;
        if (fds[i].fd < 0)
            continue;
        if (fds[i].events & POLLIN)
            FD_SET(fds[i].fd, &r);
        if (fds[i].events & POLLOUT)
            FD_SET(fds[i].fd, &w);
        if (fds[i].fd > maxfd)
            maxfd = fds[i].fd;
    }
    struct timeval tv, *ptv = NULL;
    if (timeout >= 0) {
        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        ptv = &tv;
    }
    int ret = select(maxfd + 1, &r, &w, NULL, ptv);
    if (ret <= 0)
        return ret;
    int cnt = 0;
    for (nfds_t i = 0; i < n; i++) {
        if (fds[i].fd < 0)
            continue;
        if (FD_ISSET(fds[i].fd, &r))
            fds[i].revents |= POLLIN;
        if (FD_ISSET(fds[i].fd, &w))
            fds[i].revents |= POLLOUT;
        if (fds[i].revents)
            cnt++;
    }
    return cnt;
}
#define poll shim_poll
