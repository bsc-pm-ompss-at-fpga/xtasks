#include <stdio.h>
#include <stdlib.h>
#include "../util/bitinfo.h"

int read_bitinfo(const uint32_t* bitinfo)
{
    // Check if bitstream is compatible
    uint32_t bitinfoRev = bitinfo_get_version(bitinfo);
    // The bitstream info BRAM version is old
    if (bitinfoRev != BITINFO_MIN_REV) {
        fprintf(stderr, "Found bitstream version %u, but only version %u is compatible\n", bitinfoRev, BITINFO_MIN_REV);
        return 1;
    }
    printf("Bitstream info version:\t%d\n", bitinfoRev);
    printf("Number of acc:\t%d\n", bitinfo_get_acc_count(bitinfo));
    int major, minor, patch;
    bitinfo_get_AIT_version(bitinfo, &major, &minor, &patch);
    printf("AIT version:\t%d.%d.%d\n", major, minor, patch);
    printf("Wrapper version\t%u\n", bitinfo_get_wrapper_version(bitinfo));
    printf("Board base frequency (Hz)\t%u\n", bitinfo_get_board_base_freq(bitinfo));
    printf("Interleaving stride\t%u\n", bitinfo_get_interleaving_stride(bitinfo));
    printf("Features:\n");
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
    printf("[%c] CMS enabled\n", bitinfo_get_feature(bitinfo, BIT_FEATURE_CMS_EN) ? 'x' : ' ');

    printf("Cmd In addr 0x%lX len %u\n", bitinfo_get_cmd_in_addr(bitinfo), bitinfo_get_cmd_in_len(bitinfo));
    printf("Cmd Out addr 0x%lX len %u\n", bitinfo_get_cmd_out_addr(bitinfo), bitinfo_get_cmd_out_len(bitinfo));
    printf("Spawn In addr 0x%lX len %u\n", bitinfo_get_spawn_in_addr(bitinfo), bitinfo_get_spawn_in_len(bitinfo));
    printf("Spawn Out addr 0x%lX len %u\n", bitinfo_get_spawn_out_addr(bitinfo), bitinfo_get_spawn_out_len(bitinfo));
    printf("Managed rstn addr 0x%lX\n", bitinfo_get_managed_rstn_addr(bitinfo));
    printf("Hardware counter addr 0x%lX\n", bitinfo_get_hwcounter_addr(bitinfo));
    printf("POM AXI-Lite addr 0x%lX\n", bitinfo_get_pom_axilite_addr(bitinfo));
    printf("CMS addr 0x%lX\n", bitinfo_get_cms_addr(bitinfo));

    int nacc_types = bitinfo_get_acc_type_count(bitinfo);
    bit_acc_type_t* accs = malloc(nacc_types * sizeof(bit_acc_type_t));
    bitinfo_init_acc_types(bitinfo, accs);

    printf("xtasks accelerator config:\n");
    printf("type\t\tcount\tfreq(KHz)\tdescription\n");
    for (int i = 0; i < nacc_types; ++i) {
        printf("%lu\t%u\t%u\t\t%s\n", accs[i].type, accs[i].count, accs[i].freq, accs[i].description);
    }
    printf("\n");

    printf("ait command line:\n");
    char* ait_call = malloc(bitinfo_get_ait_call_size(bitinfo) + 1);
    bitinfo_get_ait_call(bitinfo, ait_call);
    printf("%s\n\n", ait_call);

    printf("Hardware runtime VLNV:\n");
    char* hwr_vlnv = malloc(bitinfo_get_hwr_vlnv_size(bitinfo) + 1);
    bitinfo_get_hwr_vlnv(bitinfo, hwr_vlnv);
    printf("%s\n\n", hwr_vlnv);

    printf("bitinfo note:\n");
    char* notes = malloc(bitinfo_get_notes_size(bitinfo) + 1);
    bitinfo_get_notes(bitinfo, notes);
    printf("%s\n", notes);

    free(notes);
    free(hwr_vlnv);
    free(ait_call);
    free(accs);
    return 0;
}
