#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include "common/pci_dev.h"
#include "common/bitinfo.h"

#define CMS_REG_MAP         0x028000
#define MB_RESETN_REG       0x020000
#define HOST_STATUS2_REG    0x030C
#define SYSMON_T_REG        0x400
#define CMS_T_REG           0x100
#define CMS_FAN_REG         0x016C

#define PEX_12V_INS_REG         0x0028
#define PEX_12V_I_IN_INS_REG    0x00D0
#define PEX_3V3_INS_REG         0x0034
#define PEX_3V3_I_IN_INS_REG    0x0280
#define AUX_3V3_INS_REG         0x0040
#define AUX_3V3_I_INS_REG       0x02F8
#define AUX_12V_INS_REG         0x004C
#define AUX_12V_I_IN_INS_REG    0x00DC

uintptr_t _baseAddr;
volatile int _done;

uint32_t pcie_read(uint32_t *bar, off_t offset) {
    uint32_t val = bar[offset/sizeof(*bar)];
    return val;
}

void pcie_write(uint32_t *bar, off_t offset, uint32_t val) {
    bar[offset/sizeof(*bar)] = val;
}

void handle_quit(int signal) {
    _done = 1;
}


int cms_enable_power_monitor(uint32_t *cms) {
    // Get the cms out of reset status
    pcie_write(cms, MB_RESETN_REG, 0x01);

    // Wait until REG_MAP (bit 0) is set in the HOST_STATUS2_REG
    uint32_t host_status_reg = pcie_read(cms, CMS_REG_MAP + HOST_STATUS2_REG);
    int attempts=0;
    while( (!(host_status_reg & 0x0001)) && (attempts<10) ) {
        usleep(100000);
        host_status_reg = pcie_read(cms, CMS_REG_MAP + HOST_STATUS2_REG);
        attempts++;
    }
    if (attempts==10) {
        fprintf(stderr,"CMS did not respond after 10 retries\n");
        return 1;
    }
    return 0;
}

void cms_disable_power_monitor(uint32_t *cms) {
    pcie_write(cms, MB_RESETN_REG, 0x00);
}

int cms_reset_power_monitor(uint32_t *cms) {
    cms_disable_power_monitor(cms);
    return cms_enable_power_monitor(cms);
}

int cms_start_power_monitor(uint32_t *cms) {

    // Get card model (actually, software profile)
    uint32_t software_profile = pcie_read(cms, CMS_REG_MAP + 0x0014);
    return cms_enable_power_monitor(cms);
}

void cms_stop_power_monitor(uint32_t *cms) {
    cms_disable_power_monitor(cms);
}



float cms_read_power(uint32_t *cms_addr) {
    //instant_voltage_12V_PEX = pcie_read(REG_MAP + 0x0028);
    //instant_current_12V_PEX = pcie_read(REG_MAP + 0x00D0);
    //instant_voltage_3V3_PEX = pcie_read(REG_MAP + 0x0034);
    //instant_current_3V3_PEX = pcie_read(REG_MAP + 0x0280);
    //instant_voltage_3V3_AUX = pcie_read(REG_MAP + 0x0040);
    //instant_current_3V3_AUX = pcie_read(REG_MAP + 0x02F8);
    //instant_voltage_12V_AUX = pcie_read(REG_MAP + 0x004C);
    //instant_current_12V_AUX = pcie_read(REG_MAP + 0x00DC);

    float power_12v_pex = (pcie_read(cms_addr, CMS_REG_MAP + PEX_12V_INS_REG) *
            pcie_read(cms_addr, CMS_REG_MAP + PEX_12V_I_IN_INS_REG))/1000000.0;
    float power_3v3_pex = (pcie_read(cms_addr, CMS_REG_MAP + PEX_3V3_INS_REG) *
         pcie_read(cms_addr, CMS_REG_MAP + PEX_3V3_I_IN_INS_REG))/1000000.0;
    float power_3v3_aux = (pcie_read(cms_addr, CMS_REG_MAP + AUX_3V3_INS_REG) *
            pcie_read(cms_addr, CMS_REG_MAP + AUX_3V3_I_INS_REG))/1000000.0;
    float power_12v_aux = (pcie_read(cms_addr, CMS_REG_MAP + AUX_12V_INS_REG) *
            pcie_read(cms_addr, CMS_REG_MAP + AUX_12V_I_IN_INS_REG))/1000000.0;

    return power_12v_pex + power_12v_aux + power_3v3_pex + power_3v3_aux;
}

static inline float get_sysmon_temp(int adc) {
    //magic numbers are defined in xilinx's UG580
    //They are related to some physical constants and physical sensor properties
    return ((adc * 501.3743)/65536)-273.6777;
}


int main(int argc, char *argv[]) {

    uint32_t **pciBar, **cmsAddr, **sysmonAddr;
    int nDevs;
    char **devList;
    xtasks_stat stat;

    stat = getPciDevList(&nDevs, &devList);
    if (stat != XTASKS_SUCCESS) {
        fprintf(stderr, "No devices found\n");
        return 1;
    }

    sysmonAddr = malloc(nDevs * sizeof(*sysmonAddr));
    cmsAddr = malloc(nDevs * sizeof(*cmsAddr));
    pciBar = malloc(nDevs * sizeof(*pciBar));

    if(signal(SIGINT, handle_quit)==SIG_ERR) {
        perror("Cannot install signal handler");
        return 1;
    };

    //Map device BARs
    for (int i=0; i<nDevs; i++) {
        uintptr_t addr;
        fprintf(stderr, "Mapping device idx %d (%s):\n",
                i, devList[i]);
        mapPciDevice(devList[i], &pciBar[i]);
        if (bitinfo_get_feature(pciBar[i], BIT_FEATURE_CMS_EN)) {
            addr = bitinfo_get_cms_addr(pciBar[i]);
            fprintf(stderr, "  Using CMS in 0x%lx\n", addr);
            cmsAddr[i] = pciBar[i] + (addr/sizeof(*pciBar[i]));
            //We can start monitor here, we're not interested in max/avg values
            //TODO: Error handling if CMS does not wake up from reset
            cms_start_power_monitor(cmsAddr[i]);
        } else {
            cmsAddr[i] = NULL;
            fprintf(stderr, "  Sysmon not enabled\n");
        }
        if (bitinfo_get_feature(pciBar[i], BIT_FEATURE_SYSMON_EN)) {
            addr = bitinfo_get_sysmon_addr(pciBar[i]);
            fprintf(stderr, "  Using sysmon in 0x%lx\n", addr);
            sysmonAddr[i] = pciBar[i] + (addr/sizeof(*pciBar[i]));
        } else {
            sysmonAddr[i] = NULL;
            fprintf(stderr, "  CMS not enabled\n");
        }
    }

    //Print headers
    printf("time");
	for (int i=0; i<nDevs; i++) {
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

    for (_done=0; !_done; ) {
        gettimeofday(&tv, NULL);
        printf("%ld.%06ld", tv.tv_sec, tv.tv_usec);
        for (int i=0; i<nDevs; i++) {
            if (sysmonAddr[i]) {
                sysmon_temp = get_sysmon_temp(pcie_read(sysmonAddr[i], SYSMON_T_REG));
                printf(",%f", sysmon_temp);
            }
            if (cmsAddr[i]) {
                cms_temp = (float)pcie_read(cmsAddr[i], CMS_REG_MAP + CMS_T_REG);
                //Not reading fan speed
                float cms_fan = (float)pcie_read(cmsAddr[i], CMS_REG_MAP + CMS_FAN_REG);
                cms_power = cms_read_power(cmsAddr[i]);
                printf(",%f,%f", cms_temp, cms_power);
            }
        }
        printf("\n");

        fflush(stdout);
        //For increased precision read&print time should be subtracted
        //  from sleep time
        usleep(100000);
    }
    fflush(stdout);

    //cleanup
    for (int i=0; i<nDevs; i++) {
        if (cmsAddr[i]) {
            cms_disable_power_monitor(cmsAddr[i]);
        }
        unmapPciDev(pciBar[i]);
    }
    free(sysmonAddr);
    free(cmsAddr);
    free(pciBar);
}
