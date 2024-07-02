#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../common/bitinfo.h"

int read_bitinfo(const uint32_t* bitinfo)
{
    // Check if bitstream is compatible
    uint32_t bitinfoRev = bitinfo_get_version(bitinfo);
    // The bitinfo version is old
    if (bitinfoRev < BITINFO_MIN_REV) {
        fprintf(stderr, "Found bitinfo version %u, which is older than minimum supported version %u\n", bitinfoRev,
            BITINFO_MIN_REV);
        return 1;
    }
    printf("Bitinfo version:\t%d\n", bitinfoRev);
    printf("Bitstream user-id:\t0x%X\n", bitinfo_get_bitstream_userid(bitinfo));
    int major, minor, patch;
    bitinfo_get_AIT_version(bitinfo, &major, &minor, &patch);
    printf("AIT version:\t%u.%u.%u\n", major, minor, patch);
    printf("Wrapper version\t%u\n", bitinfo_get_wrapper_version(bitinfo));
    printf("Number of acc:\t%d\n", bitinfo_get_acc_count(bitinfo));
    printf("Board base frequency (MHz)\t%f\n", (float)bitinfo_get_board_base_freq(bitinfo) / 1000000);
    printf("Interleaving ");
    bitinfo_get_interleaving_stride(bitinfo) ? printf("stride\t%u\n", bitinfo_get_interleaving_stride(bitinfo))
                                             : printf("not enabled\n");
    printf("\nFeatures:\n");
    printf("[%c] Instrumentation\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_INST) ? 'x' : ' ');
    printf("[%c] Hardware counter\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_HWCOUNTER) ? 'x' : ' ');
    printf("[%c] Performance interconnect\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_INTERCONNECT_OPT) ? 'x' : ' ');
    printf(
        "[%c] Simplified interconnection\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_INTERCONNECT_SIMPL) ? 'x' : ' ');
    printf("[%c] POM AXI-Lite\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_POM_AXILITE) ? 'x' : ' ');
    printf("[%c] POM task creation\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_POM_TASK_CREATE) ? 'x' : ' ');
    printf("[%c] POM dependencies\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_POM_DEPS) ? 'x' : ' ');
    printf("[%c] POM lock\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_POM_LOCK) ? 'x' : ' ');
    printf("[%c] POM spawn queues\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_SPAWN_Q) ? 'x' : ' ');
    printf("[%c] Power monitor (CMS)\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_CMS_EN) ? 'x' : ' ');
    printf("[%c] Thermal monitor (sysmon)\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_SYSMON_EN) ? 'x' : ' ');
    printf("\n");
    printf("Managed rstn addr 0x%" PRIX64 "\n", bitinfo_get_managed_rstn_addr(bitinfo));
    printf("Cmd In addr 0x%" PRIX64 " len %u\n", bitinfo_get_cmd_in_addr(bitinfo), bitinfo_get_cmd_in_len(bitinfo));
    printf("Cmd Out addr 0x%" PRIX64 " len %u\n", bitinfo_get_cmd_out_addr(bitinfo), bitinfo_get_cmd_out_len(bitinfo));
    printf("Spawn In ");
    bitinfo_get_spawn_in_len(bitinfo)
        ? printf("addr 0x%" PRIX64 " len %u\n", bitinfo_get_spawn_in_addr(bitinfo), bitinfo_get_spawn_in_len(bitinfo))
        : printf("not enabled\n");
    printf("Spawn Out ");
    bitinfo_get_spawn_out_len(bitinfo)
        ? printf("addr 0x%" PRIX64 " len %u\n", bitinfo_get_spawn_out_addr(bitinfo), bitinfo_get_spawn_out_len(bitinfo))
        : printf("not enabled\n");
    printf("Hardware counter ");
    bitinfo_get_hwcounter_addr(bitinfo) ? printf("addr 0x%" PRIX64 "\n", bitinfo_get_hwcounter_addr(bitinfo))
                                        : printf("not enabled\n");
    printf("POM AXI-Lite ");
    bitinfo_get_pom_axilite_addr(bitinfo) ? printf("addr 0x%" PRIX64 "\n", bitinfo_get_pom_axilite_addr(bitinfo))
                                          : printf("not enabled\n");
    printf("Power monitor (CMS) ");
    bitinfo_get_cms_addr(bitinfo) ? printf("addr 0x%" PRIX64 "\n", bitinfo_get_cms_addr(bitinfo))
                                  : printf("not enabled\n");
    printf("Thermal monitor (sysmon) ");
    bitinfo_get_sysmon_addr(bitinfo) ? printf("addr 0x%" PRIX64 "\n", bitinfo_get_sysmon_addr(bitinfo))
                                     : printf("not enabled\n");

    int nacc_types = bitinfo_get_acc_type_count(bitinfo);
    bit_acc_type_t* accs = malloc(nacc_types * sizeof(bit_acc_type_t));
    bitinfo_init_acc_types(bitinfo, accs);

    printf("\nxtasks accelerator config:\n");
    printf("type\t\tcount\tfreq(KHz)\tdescription\n");
    for (int i = 0; i < nacc_types; ++i) {
        printf("%" PRIu64 "\t%u\t%u\t\t%s\n", accs[i].type, accs[i].count, accs[i].freq, accs[i].description);
    }

    printf("\nait command line:\n");
    char* ait_call = malloc(bitinfo_get_ait_call_size(bitinfo) + 1);
    bitinfo_get_ait_call(bitinfo, ait_call);
    printf("%s\n", ait_call);

    printf("\nHardware runtime VLNV:\n");
    char* hwr_vlnv = malloc(bitinfo_get_hwr_vlnv_size(bitinfo) + 1);
    bitinfo_get_hwr_vlnv(bitinfo, hwr_vlnv);
    printf("%s\n", hwr_vlnv);

    if (bitinfo_get_notes_size(bitinfo)) {
        printf("\nbitinfo note:\n");
        char* notes = malloc(bitinfo_get_notes_size(bitinfo) + 1);
        bitinfo_get_notes(bitinfo, notes);
        printf("%s\n", notes);
        free(notes);
    }

    free(hwr_vlnv);
    free(ait_call);
    free(accs);
    return 0;
}

int read_bitstream_userid(const uint32_t* bitinfo)
{
    // Check if bitstream is compatible
    uint32_t bitinfoRev = bitinfo_get_version(bitinfo);
    // The bitinfo version is old
    if (bitinfoRev < BITINFO_MIN_REV) {
        fprintf(stderr, "Found bitinfo version %u, which is older than minimum supported version %u\n", bitinfoRev,
            BITINFO_MIN_REV);
        return 1;
    }

    printf("0x%08X\n", bitinfo_get_bitstream_userid(bitinfo));
    return 0;
}
