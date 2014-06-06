#include <stdio.h>

/* [left][top][top-right] */
static unsigned int Intra4x4[2][2][2][9] =
    {{{{0, 1, 2, 6, 8, 9, 10, 11, 13}, {0, 1, 2, 7, 8, 9, 10, 12, 13}},
    {{5, 1, 3, 5, 1, 5, 1, 5, 13}, {5, 1, 3, 5, 1, 5, 1, 5, 13}}},
    {{{0, 5, 4, 6, 0, 0, 5, 11, 5}, {0, 5, 4, 7, 0, 0, 5, 12, 5}},
    {{5, 5, 5, 5, 5, 5, 5, 5, 5}, {5, 5, 5, 5, 5, 5, 5, 5, 5}}}};

/* [left][top][top_right][top_left] */
static unsigned int Intra8x8[9][2][2][2][2] =
    {0, 1, 2, 3, 16, 16, 16, 16, 0, 1, 2, 3, 16, 16, 16, 16,
    4, 5, 4, 5, 4, 5, 4, 5, 16, 16, 16, 16, 16, 16, 16, 16,
    6, 7, 8, 9, 10, 11, 10, 11, 12, 13, 14, 15, 16, 16, 16, 16,
    17, 18, 19, 20, 16, 16, 16, 16, 17, 18, 19, 20, 16, 16, 16, 16,
    21, 7, 22, 9, 4, 5, 4, 5, 0, 1, 2, 3, 16, 16, 16, 16,
    23, 1, 24, 3, 16, 16, 16, 16, 0, 1, 2, 3, 16, 16, 16, 16,
    25, 5, 25, 5, 4, 5, 4, 5, 16, 16, 16, 16, 16, 16, 16, 16,
    26, 27, 28, 29, 16, 16, 16, 16, 26, 27, 28, 29, 16, 16, 16, 16,
    30, 31, 30, 31, 30, 31, 30, 31, 16, 16, 16, 16, 16, 16, 16, 16};



int main()
{
    unsigned int left_top, left_bottom, top, top_right, top_left, u;
    
    for (left_top = 0; left_top < 2; left_top++) {
        for (left_bottom = 0; left_bottom < 2; left_bottom++) {
            for (top = 0; top < 2; top++) {
                for (top_right = 0; top_right < 2; top_right++) {
                    printf(top_right ? "    }, {\n" :
                        top ? "    }}, {{\n" :
                        left_bottom ? "    }}}, {{{\n" :
                        left_top ? "    }}}}, {{{{\n" :
                        "static const uint8_t Edge264_Intra4x4_mode2idx[2][2][2][2][9][16] = {{{{{\n");
                    for (u = 0; u < 9; u++) {
                        printf("    {%2u, %2u, %2u, %2u, %2u, %2u, %2u, %2u, %2u, %2u, %2u, %2u, %2u, %2u, %2u, %2u},\n",
                            Intra4x4[left_top][top][top][u],
                            Intra4x4[0][top][top][u],
                            Intra4x4[left_top][0][0][u],
                            Intra4x4[0][0][1][u],
                            Intra4x4[0][top][top][u],
                            Intra4x4[0][top][top_right][u],
                            Intra4x4[0][0][0][u],
                            Intra4x4[0][0][1][u],
                            Intra4x4[left_bottom][0][0][u],
                            Intra4x4[0][0][0][u],
                            Intra4x4[left_bottom][0][0][u],
                            Intra4x4[0][0][1][u],
                            Intra4x4[0][0][0][u],
                            Intra4x4[0][0][1][u],
                            Intra4x4[0][0][0][u],
                            Intra4x4[0][0][1][u]);
                    }
                }
            }
        }
    }
    printf("}}}}};\n\n");
    
    for (left_top = 0; left_top < 2; left_top++) {
        for (left_bottom = 0; left_bottom < 2; left_bottom++) {
            for (top = 0; top < 2; top++) {
                for (top_right = 0; top_right < 2; top_right++) {
                    for (top_left = 0; top_left < 2; top_left++) {
                        printf(top_left ? "    }, {" :
                            top_right ? "    }}, {{" :
                            top ? "    }}}, {{{" :
                            left_bottom ? "    }}}}, {{{{" :
                            left_top ? "    }}}}}, {{{{{" :
                            "static const uint8_t Edge264_Intra8x8_mode2idx[2][2][2][2][2][9][4] = {{{{{{");
                        for (u = 0; u < 9; u++) {
                            printf("%s{%2u, %2u, %2u, %2u},",
                                (u % 4) ? " " : "\n    ",
                                Intra8x8[u][left_top][top][top][top_left],
                                Intra8x8[u][0][top][top_right][top],
                                Intra8x8[u][left_bottom][0][0][left_top],
                                Intra8x8[u][0][0][1][0]);
                        }
                        putchar('\n');
                    }
                }
            }
        }
    }
    printf("}}}}}};\n\n");
    return 0;
}
