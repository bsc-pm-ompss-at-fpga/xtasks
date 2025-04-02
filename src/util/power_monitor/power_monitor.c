#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "common/bitinfo.h"
#include "common/pci_dev.h"

#define SYSMON_T_REG 0x400
#define CMS_T_REG 0x100
#define CMS_FAN_REG 0x016C

#define PEX_12V_INS_REG 0x0028
#define PEX_12V_I_IN_INS_REG 0x00D0
#define PEX_3V3_INS_REG 0x0034
#define PEX_3V3_I_IN_INS_REG 0x0280
#define AUX_3V3_INS_REG 0x0040
#define AUX_3V3_I_INS_REG 0x02F8
#define AUX_12V_INS_REG 0x004C
#define AUX_12V_I_IN_INS_REG 0x00DC

uintptr_t _baseAddr;
volatile int _done;

void handle_quit(int signal) { _done = 1; }

float cms_read_power(uint32_t *cms_addr) {
    // instant_voltage_12V_PEX = pci_read(REG_MAP + 0x0028);
    // instant_current_12V_PEX = pci_read(REG_MAP + 0x00D0);
    // instant_voltage_3V3_PEX = pci_read(REG_MAP + 0x0034);
    // instant_current_3V3_PEX = pci_read(REG_MAP + 0x0280);
    // instant_voltage_3V3_AUX = pci_read(REG_MAP + 0x0040);
    // instant_current_3V3_AUX = pci_read(REG_MAP + 0x02F8);
    // instant_voltage_12V_AUX = pci_read(REG_MAP + 0x004C);
    // instant_current_12V_AUX = pci_read(REG_MAP + 0x00DC);

    float power_12v_pex =
        (pci_read(cms_addr, CMS_REG_MAP + PEX_12V_INS_REG) * pci_read(cms_addr, CMS_REG_MAP + PEX_12V_I_IN_INS_REG)) /
        1000000.0;
    float power_3v3_pex =
        (pci_read(cms_addr, CMS_REG_MAP + PEX_3V3_INS_REG) * pci_read(cms_addr, CMS_REG_MAP + PEX_3V3_I_IN_INS_REG)) /
        1000000.0;
    float power_3v3_aux =
        (pci_read(cms_addr, CMS_REG_MAP + AUX_3V3_INS_REG) * pci_read(cms_addr, CMS_REG_MAP + AUX_3V3_I_INS_REG)) /
        1000000.0;
    float power_12v_aux =
        (pci_read(cms_addr, CMS_REG_MAP + AUX_12V_INS_REG) * pci_read(cms_addr, CMS_REG_MAP + AUX_12V_I_IN_INS_REG)) /
        1000000.0;

    return power_12v_pex + power_12v_aux + power_3v3_pex + power_3v3_aux;
}

static inline float get_sysmon_temp(int adc) {
    // magic numbers are defined in xilinx's UG580
    // They are related to some physical constants and physical sensor properties
    return ((adc * 501.3743) / 65536) - 273.6777;
}

int main(int argc, char *argv[]) {
    uint32_t **pciBar, **cmsAddr, **sysmonAddr;
    int nDevs;
    char **devList;
    char *devListStr;
    xtasks_stat stat;

    stat = getPciDevListStr(&devListStr);
    if (stat != XTASKS_SUCCESS) return 1;
    stat = getPciDevList(devListStr, &nDevs, &devList);
    if (stat != XTASKS_SUCCESS) {
        return 1;
    }

    sysmonAddr = malloc(nDevs * sizeof(*sysmonAddr));
    cmsAddr = malloc(nDevs * sizeof(*cmsAddr));
    pciBar = malloc(nDevs * sizeof(*pciBar));

    if (signal(SIGINT, handle_quit) == SIG_ERR) {
        perror("Cannot install signal handler");
        return 1;
    };

    // Map device BARs
    for (int i = 0; i < nDevs; i++) {
        uintptr_t addr;
        fprintf(stderr, "Mapping device idx %d (%s):\n", i, devList[i]);
        mapPciDevice(devList[i], &pciBar[i]);
        if (bitinfo_get_feature(pciBar[i], BIT_FEATURE_CMS_EN)) {
            addr = bitinfo_get_cms_addr(pciBar[i]);
            fprintf(stderr, "  Using CMS in 0x%lx\n", addr);
            cmsAddr[i] = pciBar[i] + (addr / sizeof(*pciBar[i]));
            // We can start monitor here, we're not interested in max/avg values
            // TODO: Error handling if CMS does not wake up from reset
            cms_enable_power_monitor(cmsAddr[i]);
        } else {
            cmsAddr[i] = NULL;
            fprintf(stderr, "  Sysmon not enabled\n");
        }
        if (bitinfo_get_feature(pciBar[i], BIT_FEATURE_SYSMON_EN)) {
            addr = bitinfo_get_sysmon_addr(pciBar[i]);
            fprintf(stderr, "  Using sysmon in 0x%lx\n", addr);
            sysmonAddr[i] = pciBar[i] + (addr / sizeof(*pciBar[i]));
        } else {
            sysmonAddr[i] = NULL;
            fprintf(stderr, "  CMS not enabled\n");
        }
    }

    // Print headers
    printf("time");
    for (int i = 0; i < nDevs; i++) {
        if (sysmonAddr) {
            printf(",sysmon_temp_%d", i);
        }
        if (cmsAddr[i]) {
            printf(",cms_temp_%d,cms_power_%d", i, i);
        }
    }
    printf("\n");

    float sysmon_temp = 0, cms_temp = 0, cms_power = 0;
    struct timeval tv;

    for (_done = 0; !_done;) {
        gettimeofday(&tv, NULL);
        printf("%ld.%06ld", tv.tv_sec, tv.tv_usec);
        for (int i = 0; i < nDevs; i++) {
            if (sysmonAddr[i]) {
                sysmon_temp = get_sysmon_temp(pci_read(sysmonAddr[i], SYSMON_T_REG));
                printf(",%f", sysmon_temp);
            }
            if (cmsAddr[i]) {
                cms_temp = (float)pci_read(cmsAddr[i], CMS_REG_MAP + CMS_T_REG);
                // Not reading fan speed
                float cms_fan = (float)pci_read(cmsAddr[i], CMS_REG_MAP + CMS_FAN_REG);
                cms_power = cms_read_power(cmsAddr[i]);
                printf(",%f,%f", cms_temp, cms_power);
            }
        }
        printf("\n");

        fflush(stdout);
        // For increased precision read&print time should be subtracted
        //  from sleep time
        usleep(100000);
    }
    fflush(stdout);

    // cleanup
    for (int i = 0; i < nDevs; i++) {
        if (cmsAddr[i]) {
            cms_disable_power_monitor(cmsAddr[i]);
        }
        unmapPciDev(pciBar[i]);
    }
    free(sysmonAddr);
    free(cmsAddr);
    free(pciBar);
}
