#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "pci_dev.h"

#define MAX_DEVICES 16
#define BITINFO_MAX_SIZE 4096
#define BITINFO_ADDRESS 0x0

static int _ndevs = 0;
static uint32_t *bitinfo[MAX_DEVICES];
char **_pciDevNames;

int read_bitinfo(const uint32_t *bitinfo);

// Open devices and copy bitinfo data
int init()
{
    xtasks_stat stat;

    char *pciDevListStr;
    stat = getPciDevListStr(&pciDevListStr);
    if (stat != XTASKS_SUCCESS) return 1;
    stat = getPciDevList(pciDevListStr, &_ndevs, &_pciDevNames);
    if (stat != XTASKS_SUCCESS) {
        return 1;
    }

    for (int i = 0; i < _ndevs; i++) {
        int fd;
        uint32_t *pciBar;
        stat = mapPciDevice(_pciDevNames[i], &pciBar);
        if (stat != XTASKS_SUCCESS) {
            return 1;
        }

        bitinfo[i] = malloc(BITINFO_MAX_SIZE);
        memcpy(bitinfo[i], (void *)(pciBar + BITINFO_ADDRESS / sizeof(*pciBar)), BITINFO_MAX_SIZE);
        unmapPciDev(pciBar);
    }

    return 0;
}

int main()
{
    if (init()) {
        return 1;
    }

    for (unsigned ndev = 0; ndev < _ndevs; ndev++) {
        printf("Bitinfo of FPGA %s:\n", _pciDevNames[ndev]);
        read_bitinfo(bitinfo[ndev]);
        free(bitinfo[ndev]);
    }

    return 0;
}
