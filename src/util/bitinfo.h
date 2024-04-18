#ifndef BITINFO_H
#define BITINFO_H

#include <stdint.h>
#include <string.h>

#define BITINFO_MAX_SIZE 4096
#define BITINFO_MAX_WORDS (BITINFO_MAX_SIZE / sizeof(uint32_t))
#define BITINFO_MIN_REV 11
#define BITINFO_REV_IDX 0
#define BITINFO_NUMACCS_IDX 1
#define BITINFO_FEATURES_IDX 2
#define BITINFO_AIT_VER_IDX 3
#define BITINFO_WRAPPER_IDX 4
#define BITINFO_BASE_FREQ_IDX 5
#define BITINFO_STRIDE_IDX 6
#define CMD_IN_BITINFO_ADDR_OFFSET 7
#define CMD_IN_BITINFO_LEN_OFFSET 9
#define CMD_OUT_BITINFO_ADDR_OFFSET 10
#define CMD_OUT_BITINFO_LEN_OFFSET 12
#define SPWN_IN_BITINFO_ADDR_OFFSET 13
#define SPWN_IN_BITINFO_LEN_OFFSET 15
#define SPWN_OUT_BITINFO_ADDR_OFFSET 16
#define SPWN_OUT_BITINFO_LEN_OFFSET 18
#define RST_BITINFO_ADDR_OFFSET 19
#define HWCOUNTER_BITINFO_ADDR_OFFSET 21
#define POM_AXILITE_BITINFO_ADDR_OFFSET 23
#define CMS_BITINFO_ADDR_OFFSET 25
#define SYSMON_BITINFO_ADDR_OFFSET 27
#define BIT_FEATURES_INST_B 0
#define BIT_FEATURE_HWCOUNTER_B 1
#define BIT_FEATURE_INTERCONNECT_OPT_B 2
#define BIT_FEATURE_INTERCONNECT_SIMPL_B 3
#define BIT_FEATURE_POM_AXILITE_B 4
#define BIT_FEATURE_POM_TASK_CREATE_B 5
#define BIT_FEATURE_POM_DEPS_B 6
#define BIT_FEATURE_POM_LOCK_B 7
#define BIT_FEATURES_SPAWN_Q_B 8
#define BIT_FEATURE_CMS_EN_B 9
#define BIT_XTASKS_CONFIG_IDX 29
#define BIT_AIT_CALL_IDX 30
#define BIT_HWR_VLNV_IDX 31
#define BIT_NOTES_IDX 32

typedef enum {
    BIT_FEATURE_INST,
    BIT_FEATURE_HWCOUNTER,
    BIT_FEATURE_INTERCONNECT_OPT,
    BIT_FEATURE_INTERCONNECT_SIMPL,
    BIT_FEATURE_POM_AXILITE,
    BIT_FEATURE_POM_TASK_CREATE,
    BIT_FEATURE_POM_DEPS,
    BIT_FEATURE_POM_LOCK,
    BIT_FEATURE_SPAWN_Q,
    BIT_FEATURE_CMS_EN,
    BIT_FEATURE_SYSMON_EN
} bit_feature_t;

typedef struct {
    uint32_t count;
    uint64_t type;
    uint32_t freq;
    char description[32];
} bit_acc_type_t;

static inline uint32_t bitinfo_get_version(const uint32_t* bitinfo) { return bitinfo[BITINFO_REV_IDX]; }

static inline uint32_t bitinfo_get_wrapper_version(const uint32_t* bitinfo) { return bitinfo[BITINFO_WRAPPER_IDX]; }

static inline uint32_t bitinfo_get_acc_count(const uint32_t* bitinfo) { return bitinfo[BITINFO_NUMACCS_IDX]; }

static inline uint64_t bitinfo_get_cmd_in_addr(const uint32_t* bitinfo)
{
    return bitinfo[CMD_IN_BITINFO_ADDR_OFFSET] | ((uint64_t)bitinfo[CMD_IN_BITINFO_ADDR_OFFSET + 1] << 32);
}

static inline uint32_t bitinfo_get_cmd_in_len(const uint32_t* bitinfo) { return bitinfo[CMD_IN_BITINFO_LEN_OFFSET]; }

static inline uint64_t bitinfo_get_cmd_out_addr(const uint32_t* bitinfo)
{
    return bitinfo[CMD_OUT_BITINFO_ADDR_OFFSET] | ((uint64_t)bitinfo[CMD_OUT_BITINFO_ADDR_OFFSET + 1] << 32);
}

static inline uint32_t bitinfo_get_cmd_out_len(const uint32_t* bitinfo) { return bitinfo[CMD_OUT_BITINFO_LEN_OFFSET]; }

static inline uint64_t bitinfo_get_spawn_in_addr(const uint32_t* bitinfo)
{
    return bitinfo[SPWN_IN_BITINFO_ADDR_OFFSET] | ((uint64_t)bitinfo[SPWN_IN_BITINFO_ADDR_OFFSET + 1] << 32);
}

static inline uint32_t bitinfo_get_spawn_in_len(const uint32_t* bitinfo) { return bitinfo[SPWN_IN_BITINFO_LEN_OFFSET]; }

static inline uint64_t bitinfo_get_spawn_out_addr(const uint32_t* bitinfo)
{
    return bitinfo[SPWN_OUT_BITINFO_ADDR_OFFSET] | ((uint64_t)bitinfo[SPWN_OUT_BITINFO_ADDR_OFFSET + 1] << 32);
}

static inline uint32_t bitinfo_get_spawn_out_len(const uint32_t* bitinfo)
{
    return bitinfo[SPWN_OUT_BITINFO_LEN_OFFSET];
}

static inline uint64_t bitinfo_get_managed_rstn_addr(const uint32_t* bitinfo)
{
    return bitinfo[RST_BITINFO_ADDR_OFFSET] | ((uint64_t)bitinfo[RST_BITINFO_ADDR_OFFSET + 1] << 32);
}

static inline uint64_t bitinfo_get_hwcounter_addr(const uint32_t* bitinfo)
{
    return bitinfo[HWCOUNTER_BITINFO_ADDR_OFFSET] | ((uint64_t)bitinfo[HWCOUNTER_BITINFO_ADDR_OFFSET + 1] << 32);
}

static inline uint64_t bitinfo_get_pom_axilite_addr(const uint32_t* bitinfo)
{
    return bitinfo[POM_AXILITE_BITINFO_ADDR_OFFSET] | ((uint64_t)bitinfo[POM_AXILITE_BITINFO_ADDR_OFFSET + 1] << 32);
}

static inline uint64_t bitinfo_get_cms_addr(const uint32_t* bitinfo)
{
    return bitinfo[CMS_BITINFO_ADDR_OFFSET] | ((uint64_t)bitinfo[CMS_BITINFO_ADDR_OFFSET + 1] << 32);
}

static inline uint64_t bitinfo_get_sysmon_addr(const uint32_t* bitinfo)
{
    return bitinfo[SYSMON_BITINFO_ADDR_OFFSET] | ((uint64_t)bitinfo[SYSMON_BITINFO_ADDR_OFFSET + 1] << 32);
}

static inline uint32_t bitinfo_get_acc_type_count(const uint32_t* bitinfo)
{
    return (bitinfo[BIT_XTASKS_CONFIG_IDX] & 0xFFFF) / 44;  // 44 bytes per acc type
}

static inline void bitinfo_init_acc_types(const uint32_t* bitinfo, bit_acc_type_t* acc_types)
{
    int count = bitinfo_get_acc_type_count(bitinfo);
    uint32_t offset = bitinfo[BIT_XTASKS_CONFIG_IDX] >> 16;
    const uint32_t* bit_acc_type = bitinfo + offset;
    for (int i = 0; i < count; ++i) {
        acc_types[i].count = bit_acc_type[0] & 0xFFFF;
        acc_types[i].type = (((uint64_t)bit_acc_type[1] & 0xFFFFFF) << 16) | (bit_acc_type[0] >> 16);
        acc_types[i].freq = ((bit_acc_type[2] & 0xFFFF) << 8) | (bit_acc_type[1] >> 24);
        uint32_t curWord = 0;
        int char_offset = 0;
        int word_offset = 0;
        for (int j = 0; j < 31; ++j) {
            if (j % sizeof(uint32_t) == 0) {
                char_offset = 0;
                curWord = bit_acc_type[3 + word_offset++];
            } else {
                char_offset++;
            }
            acc_types[i].description[j] = (curWord >> char_offset * 8) & 0xFF;
        }
        acc_types[i].description[31] = '\0';
        bit_acc_type += 11;
    }
}

static inline int bitinfo_get_feature(const uint32_t* bitinfo, bit_feature_t feature)
{
    uint32_t features = bitinfo[BITINFO_FEATURES_IDX];
    switch (feature) {
        case BIT_FEATURE_INST:
            return (features >> BIT_FEATURES_INST_B) & 0x1;
        case BIT_FEATURE_HWCOUNTER:
            return (features >> BIT_FEATURE_HWCOUNTER_B) & 0x1;
        case BIT_FEATURE_INTERCONNECT_OPT:
            return (features >> BIT_FEATURE_INTERCONNECT_OPT_B) & 0x1;
        case BIT_FEATURE_INTERCONNECT_SIMPL:
            return (features >> BIT_FEATURE_INTERCONNECT_SIMPL_B) & 0x1;
        case BIT_FEATURE_POM_AXILITE:
            return (features >> BIT_FEATURE_POM_AXILITE_B) & 0x1;
        case BIT_FEATURE_POM_TASK_CREATE:
            return (features >> BIT_FEATURE_POM_TASK_CREATE_B) & 0x1;
        case BIT_FEATURE_POM_DEPS:
            return (features >> BIT_FEATURE_POM_DEPS_B) & 0x1;
        case BIT_FEATURE_POM_LOCK:
            return (features >> BIT_FEATURE_POM_LOCK_B) & 0x1;
        case BIT_FEATURE_SPAWN_Q:
            return (features >> BIT_FEATURES_SPAWN_Q_B) & 0x1;
        case BIT_FEATURE_CMS_EN:
            return (features >> BIT_FEATURE_CMS_EN_B) & 0x1;
        case BIT_FEATURE_SYSMON_EN:
            return (features >> BIT_FEATURE_SYSMON_EN) & 0x1;
    }
    return 0;
}

static inline void bitinfo_get_AIT_version(const uint32_t* bitinfo, int* major, int* minor, int* patch)
{
    *patch = (bitinfo[BITINFO_AIT_VER_IDX] >> 0) & 0x7FF;
    *minor = (bitinfo[BITINFO_AIT_VER_IDX] >> 11) & 0x7FF;
    *major = (bitinfo[BITINFO_AIT_VER_IDX] >> 22) & 0x3FF;
}

static inline uint32_t bitinfo_get_board_base_freq(const uint32_t* bitinfo) { return bitinfo[BITINFO_BASE_FREQ_IDX]; }

static inline uint32_t bitinfo_get_ait_call_size(const uint32_t* bitinfo) { return bitinfo[BIT_AIT_CALL_IDX] & 0xFFFF; }

static inline uint32_t bitinfo_get_hwr_vlnv_size(const uint32_t* bitinfo) { return bitinfo[BIT_HWR_VLNV_IDX] & 0xFFFF; }

static inline uint32_t bitinfo_get_notes_size(const uint32_t* bitinfo) { return bitinfo[BIT_NOTES_IDX] & 0xFFFF; }

static inline void bitinfo_get_ait_call(const uint32_t* bitinfo, char* ait_call)
{
    const uint32_t* ait_call_ptr = bitinfo + (bitinfo[BIT_AIT_CALL_IDX] >> 16);
    int size = bitinfo_get_ait_call_size(bitinfo);
    memcpy(ait_call, ait_call_ptr, size);
    ait_call[size] = '\0';
}

static inline void bitinfo_get_hwr_vlnv(const uint32_t* bitinfo, char* hwr_vlnv)
{
    const uint32_t* hwr_vlnv_ptr = bitinfo + (bitinfo[BIT_HWR_VLNV_IDX] >> 16);
    int size = bitinfo_get_hwr_vlnv_size(bitinfo);
    memcpy(hwr_vlnv, hwr_vlnv_ptr, size);
    hwr_vlnv[size] = '\0';
}

static inline void bitinfo_get_notes(const uint32_t* bitinfo, char* notes)
{
    const uint32_t* notes_ptr = bitinfo + (bitinfo[BIT_NOTES_IDX] >> 16);
    int size = bitinfo_get_notes_size(bitinfo);
    memcpy(notes, notes_ptr, size);
    notes[size] = '\0';
}

static inline uint32_t bitinfo_get_interleaving_stride(const uint32_t* bitinfo) { return bitinfo[BITINFO_STRIDE_IDX]; }

#endif  // BITINFO_H
