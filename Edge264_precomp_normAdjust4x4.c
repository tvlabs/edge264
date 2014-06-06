#include <stdint.h>
#include <stdio.h>

int main() {
    static const uint8_t v4x4[6][3] = {
        {10, 16, 13},
        {11, 18, 14},
        {13, 20, 16},
        {14, 23, 18},
        {16, 25, 20},
        {18, 29, 23},
    };
    static const uint8_t scan4x4[16] = {
        0, 2, 0, 2,
        2, 1, 2, 1,
        0, 2, 0, 2,
        2, 1, 2, 1,
    };
    static const uint8_t v8x8[6][6] = {
        {20, 18, 32, 19, 25, 24},
        {22, 19, 35, 21, 28, 26},
        {26, 23, 42, 24, 33, 31},
        {28, 25, 45, 26, 35, 33},
        {32, 28, 51, 30, 40, 38},
        {36, 32, 58, 34, 46, 43},
    };
    static const uint8_t scan8x8[64] = {
        0, 3, 4, 3, 0, 3, 4, 3,
        3, 1, 5, 1, 3, 1, 5, 1,
        4, 5, 2, 5, 4, 5, 2, 5,
        3, 1, 5, 1, 3, 1, 5, 1,
        0, 3, 4, 3, 0, 3, 4, 3,
        3, 1, 5, 1, 3, 1, 5, 1,
        4, 5, 2, 5, 4, 5, 2, 5,
        3, 1, 5, 1, 3, 1, 5, 1,
    };
    
    printf("    static const uint16_t normAdjust4x4[6][16] __attribute__((aligned(16))) = {\n");
    for (int qP = 0; qP < 6; qP++) {
        printf("        {%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u},\n",
            v4x4[qP][scan4x4[0]], v4x4[qP][scan4x4[1]], v4x4[qP][scan4x4[2]],
            v4x4[qP][scan4x4[3]], v4x4[qP][scan4x4[4]], v4x4[qP][scan4x4[5]],
            v4x4[qP][scan4x4[6]], v4x4[qP][scan4x4[7]], v4x4[qP][scan4x4[8]],
            v4x4[qP][scan4x4[9]], v4x4[qP][scan4x4[10]], v4x4[qP][scan4x4[11]],
            v4x4[qP][scan4x4[12]], v4x4[qP][scan4x4[13]], v4x4[qP][scan4x4[14]],
            v4x4[qP][scan4x4[15]]);
    }
    printf("    };\n    static const uint16_t normAdjust8x8[6][64] __attribute__((aligned(16))) = {\n");
    for (int qP = 0; qP < 6; qP++) {
        for (int i = 0; i < 64; i++)
            printf("%s%u", (i == 0) ? "        {" : (i % 18) ? ", " : ",\n        ", v8x8[qP][scan8x8[i]]);
        printf("},\n");
    }
    printf("    };\n\n");
    return 0;
}
