#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../../common/bitinfo.h"

#define BITINFO_PATH "/dev/ompss_fpga/bitinfo"

static uint32_t *bitinfo;

int read_bitinfo(const uint32_t *bitinfo);
int read_bitstream_userid(const uint32_t *bitinfo);

int init() {
    bitinfo = malloc(BITINFO_MAX_SIZE);
    if (bitinfo == NULL) {
        perror("Unable to allocate memory for the bitinfo");
        goto err_bitinfo_malloc;
    }
    int bitinfo_fd = open(BITINFO_PATH, O_RDONLY);
    if (bitinfo_fd < 0) {
        perror("Unable to open bitinfo file " BITINFO_PATH);
        goto err_bitinfo_open;
    }
    int nread = read(bitinfo_fd, bitinfo, BITINFO_MAX_SIZE);
    close(bitinfo_fd);
    if (nread != BITINFO_MAX_SIZE) {
        perror("Unable to read bitinfo");
        goto err_bitinfo_read;
    }

    return 0;

err_bitinfo_malloc:
err_bitinfo_open:
err_bitinfo_read:
    return 1;
}

int main(int argc, char **argv) {
    int opt;
    int ret = 0;
    int read_userid = 0;

    if (init()) {
        return 1;
    }

    while ((opt = getopt(argc, argv, "u")) != -1) {
        switch (opt) {
            case 'u':
                read_userid = 1;
                break;
            default:
                abort();
        }
    }

    if (read_userid) {
        ret = read_bitstream_userid(bitinfo);
    } else {
        ret = read_bitinfo(bitinfo);
    }

    free(bitinfo);

    return ret;
}
