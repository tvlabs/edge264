#include <stdio.h>

int main()
{
    int transIdxLPS[64], transIdxMPS[64];
    
    for (unsigned i = 0; i < 64; i+= 16) {
        for (unsigned j = i; j < i + 16; j++)
            scanf("%d", &transIdxLPS[j]);
        for (unsigned j = i; j < i + 16; j++)
            scanf("%d", &transIdxMPS[j]);
    }
    
	// Unpack the input byte (i), perform the transition, then repack it.
	for (unsigned i = 0, pStateIdx, valMPS; i < 256; i++) {
		if (i & 2) {
			pStateIdx = i >> 2 ^ 63;
			valMPS = i & 1 ^ 1;
			if (pStateIdx == 0)
				valMPS ^= 1;
			pStateIdx = transIdxLPS[pStateIdx];
		} else {
			pStateIdx = i >> 2;
			valMPS = i & 1;
			pStateIdx = transIdxMPS[pStateIdx];
		}
		printf("%3d, ", pStateIdx * 4 + valMPS);
	}
    return 0;
}
