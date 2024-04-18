#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define BITINFO_ADDRESS 0x0
#define BITINFO_MAX_SIZE 4096
#define PCI_DEV_BASE_PATH "/sys/bus/pci/devices"
#define PCI_BAR_FILE "resource2"
#define XTASKS_PCIDEV_ENV "XTASKS_PCI_DEV"
#define PCI_MAX_PATH_LEN 64
#define PCI_BAR_SIZE 0x200000  // map 2MB

#define MAX_DEVICES 16

static int _ndevs = 0;
static char *_pciDevNames[MAX_DEVICES];
static uint32_t *bitinfo[MAX_DEVICES];

int read_bitinfo(const uint32_t *bitinfo);

static const char *getPciDevList()
{
    const char *devList = getenv(XTASKS_PCIDEV_ENV);
    if (devList == NULL) {
        fprintf(stderr,
            "Environment variable XTASKS_PCIDEV_ENV"
            " not found, set it to the pci device ID list,"
            " in the form of xxxx:xx:xx.x see `lspci -Dd 10ee:`\n");
    }
    return devList;
}

int init()
{
    const char *pciDevListEnv = getPciDevList();
    if (pciDevListEnv == NULL) {
        goto init_devlist_err;
    }

    // Map PCI BAR into user space
    int ndevs = 0;
    _ndevs = 0;
    char *pciDevName;
    char *pciDevList = malloc(strlen(pciDevListEnv) + 1);
    strcpy(pciDevList, pciDevListEnv);
    pciDevName = strtok(pciDevList, " ");
    while (pciDevName != NULL) {
        if (ndevs == MAX_DEVICES) {
            fprintf(stderr, "Found too many devices\n");
            goto init_maxdev_err;
        }
        uint32_t *pciBar;
        char pciBarPath[PCI_MAX_PATH_LEN];
        snprintf(pciBarPath, PCI_MAX_PATH_LEN, "%s/%s/%s", PCI_DEV_BASE_PATH, pciDevName, PCI_BAR_FILE);
        int pciBarFd = open(pciBarPath, O_RDWR);
        if (pciBarFd < 0) {
            perror("Could not open PCIe register window");
            if (errno == ENOENT) {
                fprintf(stderr, "Cound not open %s\n", pciBarPath);
            }
            goto init_open_bar_err;
        }
        pciBar = mmap(NULL, PCI_BAR_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, pciBarFd, 0);
        close(pciBarFd);
        if (pciBar == MAP_FAILED) {
            fprintf(stderr, "Could not map BAR into process memory space\n");
            goto init_map_bar_err;
        }

        fprintf(stderr, "Found device %s\n", pciDevName);

        bitinfo[ndevs] = malloc(BITINFO_MAX_SIZE);
        memcpy(bitinfo[ndevs], (void *)(pciBar + BITINFO_ADDRESS / sizeof(*pciBar)), BITINFO_MAX_SIZE);
        munmap((void *)pciBar, PCI_BAR_SIZE);

        _pciDevNames[ndevs] = strdup(pciDevName);

        ++ndevs;
        pciDevName = strtok(NULL, " ");
    }
    _ndevs = ndevs;

    return 0;

init_devlist_err:
init_maxdev_err:
init_open_bar_err:
init_map_bar_err:
    return 1;
}

int main()
{
    if (init()) {
        return 1;
    }

    unsigned ndev;
    for (ndev = 0; ndev < _ndevs; ndev++) {
        printf("Reading bitinfo of FPGA %s\n", _pciDevNames[ndev]);
        read_bitinfo(bitinfo[ndev]);
        free(bitinfo[ndev]);
        free(_pciDevNames[ndev]);
    }

    return 0;
}
