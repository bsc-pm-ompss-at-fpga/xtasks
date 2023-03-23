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
#define PCI_DEV "0000:02:00.0"
#define PCI_BAR_FILE "resource2"
#define XTASKS_PCIDEV_ENV "XTASKS_PCI_DEV"
#define PCI_MAX_PATH_LEN 64
#define PCI_BAR_SIZE 0x40000  // map only 256KB even if the BAR is larger

static uint32_t *bitinfo;

int read_bitinfo(const uint32_t *bitinfo);

const char *getPciDevName()
{
    const char *dev = getenv(XTASKS_PCIDEV_ENV);
    return dev ? dev : PCI_DEV;
}

int init()
{
    // Map PCI BAR into user space
    char pciBarPath[PCI_MAX_PATH_LEN];
    const char *pciDevname = getPciDevName();
    volatile uint32_t *_pciBar;
    int _pciBarFd;

    snprintf(pciBarPath, PCI_MAX_PATH_LEN, "%s/%s/%s", PCI_DEV_BASE_PATH, pciDevname, PCI_BAR_FILE);
    _pciBarFd = open(pciBarPath, O_RDWR);
    if (_pciBarFd < 0) {
        perror("Could not open PCIe register window");
        if (errno == ENOENT) {
            fprintf(stderr, "Note: Set " XTASKS_PCIDEV_ENV
                            " to the pci device ID, in the form of\
                    xxxx:xx:xx.x see `lspci -Dd 10ee:`");
        }
        goto init_pcibar_open_err;
    }
    _pciBar = mmap(NULL, PCI_BAR_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, _pciBarFd, 0);
    close(_pciBarFd);
    if (_pciBar == MAP_FAILED) {
        perror("Could not map BAR into process memory space");
        goto init_pcibar_mmap_err;
    }

    bitinfo = malloc(BITINFO_MAX_SIZE);
    memcpy(bitinfo, (void *)(_pciBar + BITINFO_ADDRESS / sizeof(*_pciBar)), BITINFO_MAX_SIZE);
    munmap((void *)_pciBar, PCI_BAR_SIZE);

    return 0;

init_pcibar_open_err:
init_pcibar_mmap_err:
    return 1;
}

int main()
{
    if (init()) {
        return 1;
    }

    int ret = read_bitinfo(bitinfo);

    free(bitinfo);

    return ret;
}
