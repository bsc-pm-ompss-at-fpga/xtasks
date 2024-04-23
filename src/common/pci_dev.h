#ifndef __LIBXDMA_PCI_DEV_H__
#define __LIBXDMA_PCI_DEV_H__

#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>


#include "libxtasks.h"

#define MAX_PCI_DEVS    16
#define PCI_DEV_BASE_PATH "/sys/bus/pci/devices"
#define PCI_BAR_FILE "resource2"
#define XTASKS_PCIDEV_ENV "XTASKS_PCI_DEV"
#define PCI_MAX_PATH_LEN 64
#define PCI_BAR_SIZE 0x200000  // map 2MB

#define CMS_REG_MAP         0x028000
#define MB_RESETN_REG       0x020000
#define HOST_STATUS2_REG    0x030C

/*!
 * Gets the list of pci device names from the XTASKS_PCI_DEV
 * environment variable
 *
 * \param[out]  nDevices    Number of devices in the device list
 * \param[out]  devNames    Reference to an array of device names, allocated by this function
 * \returns     XTASKS_SUCCESS on success, error code otherwise
 *
 */
static xtasks_stat getPciDevList(int *nDevices, char ***devNames) {
    const char *pciDevListEnv = getenv(XTASKS_PCIDEV_ENV);
    if (pciDevListEnv == NULL) {
        fprintf(stderr,
            "Environment variable XTASKS_PCIDEV_ENV"
            " not found, set it to the pci device ID list,"
            " in the form of xxxx:xx:xx.x see `lspci -Dd 10ee:`\n");
        return XTASKS_ENODEV;
    }

    int ndevs = 0;
    char * pciDevName;
    char **names;
    names = malloc(MAX_PCI_DEVS * sizeof(*devNames));
    //*devNames = malloc(MAX_PCI_DEVS * sizeof(*devNames));

    //Need a string copy as strtok will destroy the original string
    char *pciDevList = malloc(strlen(pciDevListEnv) + 1);
    strcpy(pciDevList, pciDevListEnv);
    pciDevName = strtok(pciDevList, " ");
    while (pciDevName != NULL && ndevs != MAX_PCI_DEVS) {
        names[ndevs] = strdup(pciDevName);
        ++ndevs;
        pciDevName = strtok(NULL, " ");
    }

    if (ndevs == 0) {
        fprintf(stderr, "No PCIe devices found\n");
        return XTASKS_ENODEV;
    }

    if (ndevs == MAX_PCI_DEVS && pciDevName != NULL) {
        fprintf(stderr, "Warning: Too many devices found, using first %d devices\n",
                MAX_PCI_DEVS);
    }
    *nDevices = ndevs;
    *devNames = names;
    return XTASKS_SUCCESS;
}

/*!
 * Map PCIe device BAR into process' address space
 * \param   pciDevName[in]  PCIe BDF including domain, in the form of
 *                          dddd:BB:DD:FF
 * \param   pcibar[out]     Pointer pointing to PCIe BAR
 * \returns                 XTASKS_SUCCESS on success, error code otherwise
 */
static xtasks_stat mapPciDevice(char *pciDevName, uint32_t **pciBar) {
    char pciBarPath[PCI_MAX_PATH_LEN];
    snprintf(pciBarPath, PCI_MAX_PATH_LEN, "%s/%s/%s", PCI_DEV_BASE_PATH, pciDevName, PCI_BAR_FILE);
    int pciBarFd = open(pciBarPath, O_RDWR);
    xtasks_stat ret;
    if (pciBarFd < 0) {
        perror("Could not open PCIe register window");
        if (errno == ENOENT) {
            fprintf(stderr, "Cound not open %s\n", pciBarPath);
        }
        ret = XTASKS_EFILE;
        goto init_open_bar_err;
    }
    *pciBar = mmap(NULL, PCI_BAR_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, pciBarFd, 0);
    close(pciBarFd);
    if (pciBar == MAP_FAILED) {
        fprintf(stderr, "Could not map BAR into process memory space\n");
        ret = XTASKS_EFILE;
        goto init_map_bar_err;
    }
    return XTASKS_SUCCESS;
init_map_bar_err:
init_open_bar_err:
    return ret;
}

/*!
 * Unmap PCIe BAR from user space
 * \param   pciBar[in]  Pointer to PCIe bar
 * \returns XTASKS_SUCCESS
 */
static xtasks_stat unmapPciDev(uint32_t *pciBar) {
        munmap((void *)pciBar, PCI_BAR_SIZE);
        return XTASKS_SUCCESS;
}


uint32_t pci_read(uint32_t *bar, off_t offset) {
    uint32_t val = bar[offset/sizeof(*bar)];
    return val;
}

void pci_write(uint32_t *bar, off_t offset, uint32_t val) {
    bar[offset/sizeof(*bar)] = val;
}

static int cms_enable_power_monitor(uint32_t *cms) {
    // Get the cms out of reset status
    pci_write(cms, MB_RESETN_REG, 0x01);

    // Wait until REG_MAP (bit 0) is set in the HOST_STATUS2_REG
    uint32_t host_status_reg = pci_read(cms, CMS_REG_MAP + HOST_STATUS2_REG);
    int attempts=0;
    while( (!(host_status_reg & 0x0001)) && (attempts<10) ) {
        usleep(100000);
        host_status_reg = pci_read(cms, CMS_REG_MAP + HOST_STATUS2_REG);
        attempts++;
    }
    if (attempts==10) {
        fprintf(stderr,"CMS did not respond after 10 retries\n");
        return 1;
    }
    return 0;
}

static void cms_disable_power_monitor(uint32_t *cms) {
    pci_write(cms, MB_RESETN_REG, 0x00);
}

static int cms_reset_power_monitor(uint32_t *cms) {
    cms_disable_power_monitor(cms);
    return cms_enable_power_monitor(cms);
}

static int cms_start_power_monitor(uint32_t *cms) {

    // Get card model (actually, software profile)
    uint32_t software_profile = pci_read(cms, CMS_REG_MAP + 0x0014);
    return cms_enable_power_monitor(cms);
}

static void cms_stop_power_monitor(uint32_t *cms) {
    cms_disable_power_monitor(cms);
}



#endif //DMA_PCI_DEV_H__
