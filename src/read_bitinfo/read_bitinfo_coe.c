#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int read_bitinfo(const uint32_t* bitinfo);

int main(int argc, char* argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <coe file>\n", argv[0]);
        return 1;
    }

    FILE* fp = fopen(argv[1], "r");
    if (fp == NULL) {
        fprintf(stderr, "Error opening file %s\n", argv[1]);
        perror("errno");
        return 1;
    }

    // Max 4K
    uint32_t bitinfo[1024];
    char header[1024];

    // This implementation assumes the coe has been generated by AIT, with the first two lines being headers
    fscanf(fp, "%s", header);
    fscanf(fp, "%s", header);

    int i = 0;
    uint32_t val;

    while (fscanf(fp, "%X", &val) == 1) {
        if (i == 1024) {
            fprintf(stderr, "More values than expected: %d\n", i);
            fclose(fp);
            return 1;
        }
        bitinfo[i++] = val;
    }

    int ret = read_bitinfo(bitinfo);

    fclose(fp);
    return ret;
}
