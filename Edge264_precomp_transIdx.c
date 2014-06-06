#include <stdio.h>

int main()
{
    int transIdxLPS[64], transIdxMPS[64], pStateIdx, i, j;
    
    for (i = 0; i < 64; i+= 16) {
        for (j = i; j < i + 16; j++)
            scanf("%d", &transIdxLPS[j]);
        for (j = i; j < i + 16; j++)
            scanf("%d", &transIdxMPS[j]);
    }
    
    for (i = 0; i < 256; i++) {
        if (i < 128) {
            pStateIdx = 63 - i / 2;
            printf("0x%02x,", transIdxLPS[pStateIdx] << 1 | ((pStateIdx == 0) ? i % 2 : 1 - i % 2));
        } else {
            pStateIdx = i / 2 - 64;
            printf("0x%02x,", transIdxMPS[pStateIdx] << 1 | (i % 2));
        }
        putchar((i == 127) ? '\n' : ' ');
    }
    
    return 0;
}
