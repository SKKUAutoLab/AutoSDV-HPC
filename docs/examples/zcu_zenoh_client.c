/* Minimal ZCU-side reference: a pure zenoh client talking to the HPC's
 * zenoh-bridge-ros2dds. No DDS, no bridge and no liveliness token on this side.
 *
 * This is the whole contract the ZCU has to meet:
 *   keyexpr  = the ROS topic name without its leading '/'
 *   payload  = the DDS-serialized sample, i.e. encapsulated CDR, which the ZCU
 *              already produces today
 *
 * Verified against zenoh-bridge-ros2dds 1.10.0 and zenoh-c 1.6.2 (the same
 * zenoh-c the boards run): this program exchanges std_msgs/String with ordinary
 * rmw_cyclonedds nodes in both directions through one bridge on the HPC.
 *
 * build:
 *   Z=/opt/ros/jazzy/opt/zenoh_cpp_vendor
 *   gcc -O1 -o zcu_zenoh_client docs/examples/zcu_zenoh_client.c \
 *       -I$Z/include -L$Z/lib -lzenohc -Wl,-rpath,$Z/lib
 * run:
 *   ./zcu_zenoh_client 'udp/10.0.0.10:7448#dscp=0xa0'
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "zenoh.h"

/* CDR encapsulation (little endian) + uint32 length + bytes + NUL. */
static size_t encode_string(const char *text, unsigned char *out)
{
    unsigned int n = (unsigned int)strlen(text) + 1;
    out[0] = 0x00; out[1] = 0x01; out[2] = 0x00; out[3] = 0x00;
    memcpy(out + 4, &n, 4);
    memcpy(out + 8, text, n);
    return 8 + n;
}

static void on_control(z_loaned_sample_t *sample, void *context)
{
    (void)context;
    z_owned_slice_t slice;
    if (z_bytes_to_slice(z_sample_payload(sample), &slice) != 0) return;
    const unsigned char *p = z_slice_data(z_loan(slice));
    size_t len = z_slice_len(z_loan(slice));
    if (len > 8) printf("ZCU received control: %s\n", (const char *)(p + 8));
    fflush(stdout);
    z_drop(z_move(slice));
}

int main(int argc, char **argv)
{
    const char *endpoint = argc > 1 ? argv[1] : "udp/127.0.0.1:7448";
    char config[512];
    snprintf(config, sizeof(config),
             "{mode:'client',connect:{endpoints:['%s']},"
             "scouting:{multicast:{enabled:false},gossip:{enabled:false}}}",
             endpoint);

    z_owned_config_t cfg;
    if (zc_config_from_str(&cfg, config) != 0) { puts("bad config"); return 1; }
    z_owned_session_t session;
    if (z_open(&session, z_move(cfg), NULL) != 0) { puts("open failed"); return 1; }

    z_view_keyexpr_t ke_sub;
    z_view_keyexpr_from_str(&ke_sub, "topic_control_signal");
    z_owned_closure_sample_t closure;
    z_closure(&closure, on_control, NULL, NULL);
    z_owned_subscriber_t sub;
    if (z_declare_subscriber(z_loan(session), &sub, z_loan(ke_sub),
                             z_move(closure), NULL) != 0) {
        puts("subscribe failed"); return 1;
    }

    z_view_keyexpr_t ke_pub;
    z_view_keyexpr_from_str(&ke_pub, "topic_status_signal");
    z_owned_publisher_t pub;
    if (z_declare_publisher(z_loan(session), &pub, z_loan(ke_pub), NULL) != 0) {
        puts("declare publisher failed"); return 1;
    }

    unsigned char buf[128];
    size_t len = encode_string("hello_from_zcu", buf);
    for (int i = 0; i < 40; i++) {
        z_owned_bytes_t payload;
        z_bytes_copy_from_buf(&payload, buf, len);
        z_publisher_put(z_loan(pub), z_move(payload), NULL);
        usleep(250000);
    }
    z_drop(z_move(pub));
    z_drop(z_move(sub));
    z_drop(z_move(session));
    return 0;
}
