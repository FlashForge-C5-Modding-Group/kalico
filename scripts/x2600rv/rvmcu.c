// Host tool for the X2600 RISC-V core (Klipper MCU over soc_mcu)
//
//   rvmcu load FILE        load an image through /dev/mcu and start the core
//   rvmcu stop             hold the core in reset
//   rvmcu bridge [PATH]    expose the core as a serial port for Klipper
//                          (default /tmp/klipper_host_soc)
//
// Build: mips-linux-gnu-gcc -O2 -static -pthread -o rvmcu rvmcu.c
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <pty.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

// soc_mcu ioctls (MIPS encoding)
#define MCU_STOP   0x20004D70
#define MCU_START  0x20004D72
#define MCU_LOAD   0x80044D73
#define MCU_WRITE  0x80044D76
#define MCU_READ   0x80044D77

#define IMAGE_MAGIC 0x4C455054 // "TPEL"

static int mcu_fd;

static int
open_mcu(void)
{
    mcu_fd = open("/dev/mcu", O_RDWR);
    if (mcu_fd < 0) {
        perror("/dev/mcu");
        return -1;
    }
    return 0;
}

static int
cmd_load(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        perror(path);
        return 1;
    }
    static uint8_t img[0x52000];
    size_t len = fread(img, 1, sizeof(img), f);
    fclose(f);
    uint32_t magic;
    memcpy(&magic, img + 4, 4);
    if (len < 40 || magic != IMAGE_MAGIC) {
        fprintf(stderr, "%s: not an X2600 RISC-V image\n", path);
        return 1;
    }
    if (open_mcu())
        return 1;
    // The first chunk carries the header: the driver takes the load
    // address from it and stops the core before copying
    for (size_t off = 0; off < len; off += 4096) {
        uint32_t arg[3] = { off, (uint32_t)(uintptr_t)(img + off)
                            , len - off < 4096 ? len - off : 4096 };
        if (ioctl(mcu_fd, MCU_LOAD, arg) < 0) {
            perror("load");
            return 1;
        }
    }
    if (ioctl(mcu_fd, MCU_START, 0) < 0) {
        perror("start");
        return 1;
    }
    printf("loaded %zu bytes, core started\n", len);
    return 0;
}

static int
cmd_stop(void)
{
    if (open_mcu())
        return 1;
    if (ioctl(mcu_fd, MCU_STOP, 0) < 0) {
        perror("stop");
        return 1;
    }
    return 0;
}

static int pty_fd;

// Core -> Klipper
static void *
reader(void *unused)
{
    static uint8_t buf[512];
    for (;;) {
        // The driver only returns once the whole request is filled (or
        // times out), so block for one byte, then take the rest without
        // waiting (a zero timeout reads what is there)
        uint32_t arg[3] = { (uint32_t)(uintptr_t)buf, 1, 500000 };
        int n = ioctl(mcu_fd, MCU_READ, arg);
        if (n < 0) {
            if (errno == ETIMEDOUT || errno == EINTR)
                continue;
            perror("read");
            exit(1);
        }
        uint32_t rest[3] = { (uint32_t)(uintptr_t)(buf + n), sizeof(buf) - n, 0 };
        int m = ioctl(mcu_fd, MCU_READ, rest);
        if (m > 0)
            n += m;
        for (int done = 0; done < n; ) {
            int w = write(pty_fd, buf + done, n - done);
            if (w < 0) {
                if (errno == EAGAIN) {
                    usleep(1000);
                    continue;
                }
                perror("pty write");
                exit(1);
            }
            done += w;
        }
    }
    return NULL;
}

static int
cmd_bridge(const char *link)
{
    if (open_mcu())
        return 1;
    struct termios ti;
    memset(&ti, 0, sizeof(ti));
    cfmakeraw(&ti);
    int sfd;
    if (openpty(&pty_fd, &sfd, NULL, &ti, NULL)) {
        perror("openpty");
        return 1;
    }
    unlink(link);
    if (symlink(ttyname(sfd), link)) {
        perror("symlink");
        return 1;
    }
    chmod(ttyname(sfd), 0660);
    pthread_t th;
    pthread_create(&th, NULL, reader, NULL);
    // Klipper -> core
    static uint8_t buf[512];
    struct pollfd pfd = { .fd = pty_fd, .events = POLLIN };
    for (;;) {
        if (poll(&pfd, 1, -1) < 0) {
            if (errno == EINTR)
                continue;
            perror("poll");
            return 1;
        }
        int n = read(pty_fd, buf, sizeof(buf));
        if (n <= 0) {
            // No reader on the other side yet
            usleep(10000);
            continue;
        }
        uint32_t arg[3] = { (uint32_t)(uintptr_t)buf, n, 100000 };
        if (ioctl(mcu_fd, MCU_WRITE, arg) < 0 && errno != EPERM)
            perror("write");
    }
}

int
main(int argc, char **argv)
{
    if (argc >= 3 && !strcmp(argv[1], "load"))
        return cmd_load(argv[2]);
    if (argc >= 2 && !strcmp(argv[1], "stop"))
        return cmd_stop();
    if (argc >= 2 && !strcmp(argv[1], "bridge"))
        return cmd_bridge(argc >= 3 ? argv[2] : "/tmp/klipper_host_soc");
    fprintf(stderr, "usage: %s load FILE | stop | bridge [PATH]\n", argv[0]);
    return 2;
}
