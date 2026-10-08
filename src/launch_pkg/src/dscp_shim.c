/* libautosdv_dscp_shim.so (HPC copy; the ZCU builds the same file from
 * S32G3_ZCU/src/dscp_shim/dscp_shim.c): sets IP_TOS on every IPv4 UDP socket a process
 * creates, so one process carries one traffic class on the wire.
 *
 *   LD_PRELOAD=.../libautosdv_dscp_shim.so AUTOSDV_IP_TOS=0xc0 zcu_control
 *
 * AUTOSDV_IP_TOS is the whole TOS byte (CS5 = 0xa0, CS6 = 0xc0). Unset leaves
 * sockets untouched. This works for every backend (Fast DDS, Cyclone DDS,
 * zenoh-c, zenoh-pico) because all of them create their sockets with socket(),
 * and it marks discovery and session upkeep together with the data, which is
 * the per-process policy the SJA1110 queues are configured for.
 *
 * Everything is resolved in the constructor: calling dlsym() lazily from
 * socket() deadlocks when another thread holds the dynamic loader lock (seen
 * with lttng-ust threads creating sockets while the main thread runs dlopen).
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

static int (*real_socket)(int, int, int);
static int tos = -1;

__attribute__((constructor)) static void dscp_shim_init(void)
{
    /* POSIX-sanctioned way to store dlsym() in a function pointer. */
    *(void **)&real_socket = dlsym(RTLD_NEXT, "socket");
    const char *value = getenv("AUTOSDV_IP_TOS");
    if (value && *value)
        tos = (int)strtol(value, NULL, 0) & 0xff;
}

int socket(int domain, int type, int protocol)
{
    int fd = real_socket ? real_socket(domain, type, protocol)
                         : (int)syscall(SYS_socket, domain, type, protocol);
    if (fd >= 0 && tos >= 0 && domain == AF_INET && (type & 0xf) == SOCK_DGRAM)
        setsockopt(fd, IPPROTO_IP, IP_TOS, &tos, sizeof tos);
    return fd;
}
