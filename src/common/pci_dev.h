#ifndef __LIBXDMA_PCI_DEV_H__
#define __LIBXDMA_PCI_DEV_H__

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "libxtasks.h"

#define MAX_PCI_DEVS 16
#define PCI_DEV_BASE_PATH "/sys/bus/pci/devices"
#define PCI_BAR_FILE "resource2"
#define XTASKS_PCIDEV_ENV "XTASKS_PCI_DEV"
#define PCI_MAX_PATH_LEN 64
#define PCI_BAR_SIZE 0x200000  // map 2MB

#define CMS_REG_MAP 0x028000
#define MB_RESETN_REG 0x020000
#define HOST_STATUS2_REG 0x030C

#define CMS_FPGA_T_REG 0x100
#define CMS_FAN_T_REG 0x10C
#define CMS_FAN_S_REG 0x16C

/*!
 * Gets the list of pci device names string from the XTASKS_PCI_DEV
 * environment variable
 *
 * \param[out]  pciDevListStr    String allocated by this funcion containing a copy of the XTASKS_PCI_DEV environment
 *                               variable
 * \returns     XTASKS_SUCCESS on success, XTASKS_ENODEV if XTASKS_PCI_DEV is not defined
 */
static xtasks_stat getPciDevListStr(char **pciDevListStr) {
    char *pciDevListEnv = getenv(XTASKS_PCIDEV_ENV);
    if (pciDevListEnv == NULL) {
        fprintf(stderr,
                "Environment variable XTASKS_PCIDEV_ENV"
                " not found, set it to the pci device ID list,"
                " in the form of xxxx:xx:xx.x see `lspci -Dd 10ee:`\n");
        return XTASKS_ENODEV;
    }
    size_t envlen = strlen(pciDevListEnv);
    *pciDevListStr = (char *)malloc(envlen + 1);  // We have to add the terminating NULL character
    strcpy(*pciDevListStr, pciDevListEnv);
    return XTASKS_SUCCESS;
}

/*!
 * Gets the list of pci device names from the pciDevListStr string
 *
 * \param[in]   pciDevListStr    String with the list of PCI names separated by spaces
 * \param[out]  nDevices         Number of devices in the device list
 * \param[out]  devNames         Reference to an array of device names, allocated by this function
 * \returns     XTASKS_SUCCESS on success, error code otherwise
 */
static xtasks_stat getPciDevList(char *pciDevListStr, int *nDevices, char ***devNames) {
    int ndevs = 0;
    char *pciDevName;
    char **names;
    names = malloc(MAX_PCI_DEVS * sizeof(*devNames));
    *devNames = names;

    pciDevName = strsep(&pciDevListStr, " ");
    while (pciDevName != NULL && ndevs != MAX_PCI_DEVS) {
        names[ndevs] = pciDevName;
        ++ndevs;
        pciDevName = strsep(&pciDevListStr, " ");
    }

    if (ndevs == 0) {
        fprintf(stderr, "No PCIe devices found\n");
        return XTASKS_ENODEV;
    }

    if (ndevs == MAX_PCI_DEVS && pciDevName != NULL) {
        fprintf(stderr, "Found too many devices, limit is %d\n", MAX_PCI_DEVS);
        return XTASKS_ERROR;
    }
    *nDevices = ndevs;
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
    uint32_t val = bar[offset / sizeof(*bar)];
    return val;
}

void pci_write(uint32_t *bar, off_t offset, uint32_t val) { bar[offset / sizeof(*bar)] = val; }

static int cms_enable_power_monitor(uint32_t *cms) {
    // TODO: Check for a proper software profile (board code + sw version)
    // uint32_t software_profile = pci_read(cms, CMS_REG_MAP + 0x0014);
    // Get the cms out of reset status
    pci_write(cms, MB_RESETN_REG, 0x01);

    // Wait until CMS_REG_MAP (bit 0) is set in the HOST_STATUS2_REG
    uint32_t host_status_reg = pci_read(cms, CMS_REG_MAP + HOST_STATUS2_REG);
    int attempts = 0;
    while ((!(host_status_reg & 0x0001)) && (attempts < 10)) {
        usleep(100000);
        host_status_reg = pci_read(cms, CMS_REG_MAP + HOST_STATUS2_REG);
        attempts++;
    }
    if (attempts == 10) {
        fprintf(stderr, "CMS did not respond after 10 retries\n");
        return 1;
    }
    return 0;
}

static void cms_disable_power_monitor(uint32_t *cms) { pci_write(cms, MB_RESETN_REG, 0x00); }

static int cms_reset_power_monitor(uint32_t *cms) {
    cms_disable_power_monitor(cms);
    return cms_enable_power_monitor(cms);
}

static void cms_stop_power_monitor(uint32_t *cms) { cms_disable_power_monitor(cms); }

static int cms_power_monitor_read_values(uint32_t *cmsAddr, xtasks_monitor_info *info) {
    // XXX TODO u200/u280/... logic
    // XXX TODO move hex values to constats

    bzero(info, sizeof(*info));

    info->average_voltage_12V_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x0024);
    info->instant_voltage_12V_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x0028);
    info->average_current_12V_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x00CC);
    info->instant_current_12V_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x00D0);
    info->computed_average_power_12V_PEX = (info->average_voltage_12V_PEX * info->average_current_12V_PEX) / 1000000.0;
    info->computed_instant_power_12V_PEX = (info->instant_voltage_12V_PEX * info->instant_current_12V_PEX) / 1000000.0;

    info->average_voltage_3V3_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x0030);
    info->instant_voltage_3V3_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x0034);
    info->average_current_3V3_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x027C);
    info->instant_current_3V3_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x0280);
    info->computed_average_power_3V3_PEX = (info->average_voltage_3V3_PEX * info->average_current_3V3_PEX) / 1000000.0;
    info->computed_instant_power_3V3_PEX = (info->instant_voltage_3V3_PEX * info->instant_current_3V3_PEX) / 1000000.0;

    info->average_voltage_3V3_AUX = pci_read(cmsAddr, CMS_REG_MAP + 0x003C);
    info->instant_voltage_3V3_AUX = pci_read(cmsAddr, CMS_REG_MAP + 0x0040);
    info->average_current_3V3_AUX = pci_read(cmsAddr, CMS_REG_MAP + 0x02F4);
    info->instant_current_3V3_AUX = pci_read(cmsAddr, CMS_REG_MAP + 0x02F8);
    info->computed_average_power_3V3_AUX = (info->average_voltage_3V3_AUX * info->average_current_3V3_AUX) / 1000000.0;
    info->computed_instant_power_3V3_AUX = (info->instant_voltage_3V3_AUX * info->instant_current_3V3_AUX) / 1000000.0;

    info->average_voltage_12V_AUX = pci_read(cmsAddr, CMS_REG_MAP + 0x0048);
    info->instant_voltage_12V_AUX = pci_read(cmsAddr, CMS_REG_MAP + 0x004C);
    info->average_current_12V_AUX = pci_read(cmsAddr, CMS_REG_MAP + 0x00D8);
    info->instant_current_12V_AUX = pci_read(cmsAddr, CMS_REG_MAP + 0x00DC);
    info->computed_average_power_12V_AUX = (info->average_voltage_12V_AUX * info->average_current_12V_AUX) / 1000000.0;
    info->computed_instant_power_12V_AUX = (info->instant_voltage_12V_AUX * info->instant_current_12V_AUX) / 1000000.0;

    info->average_power_12V_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x02DC);
    info->instant_power_12V_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x02E0);
    info->average_power_3V3_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x02E8);
    info->instant_power_3V3_PEX = pci_read(cmsAddr, CMS_REG_MAP + 0x02EC);
    info->average_power_VCCINT = pci_read(cmsAddr, CMS_REG_MAP + 0x0378);
    info->instant_power_VCCINT = pci_read(cmsAddr, CMS_REG_MAP + 0x037C);

    info->computed_average_power_total = info->computed_average_power_12V_PEX + info->computed_average_power_3V3_PEX +
                                         info->computed_average_power_3V3_AUX + info->computed_average_power_12V_AUX;

    info->computed_instant_power_total = info->computed_instant_power_12V_PEX + info->computed_instant_power_3V3_PEX +
                                         info->computed_instant_power_3V3_AUX + info->computed_instant_power_12V_AUX;

    info->instant_fpga_temp = pci_read(cmsAddr, CMS_REG_MAP + CMS_FPGA_T_REG);
    info->instant_fan_temp = pci_read(cmsAddr, CMS_REG_MAP + CMS_FAN_T_REG);
    info->instant_fan_speed = pci_read(cmsAddr, CMS_REG_MAP + CMS_FAN_S_REG);
    return 0;
}

#endif  // DMA_PCI_DEV_H__
