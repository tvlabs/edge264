#include "edge264_internal.h"

// This file is compiled twice: once for CAVLC and once for CABAC
#undef CAFUNC
#undef CACALL
#undef CAJUMP
#undef CACOND
#ifndef CABAC
	#define CABAC 0
#endif
#if !CABAC
	#define CAFUNC(f, ...) f ## _cavlc(Edge264Context *ctx, ## __VA_ARGS__)
	#define CACALL(f, ...) f ## _cavlc(ctx, ## __VA_ARGS__)
	#define CAJUMP(f, ...) { f ## _cavlc(ctx, ## __VA_ARGS__); return; }
	#define CACOND(cavlc, cabac) cavlc
#else
	#define CAFUNC(f, ...) f ## _cabac(Edge264Context *ctx, ## __VA_ARGS__)
	#define CACALL(f, ...) f ## _cabac(ctx, ## __VA_ARGS__)
	#define CAJUMP(f, ...) { f ## _cabac(ctx, ## __VA_ARGS__); return; }
	#define CACOND(cavlc, cabac) cabac
	
	static const i8x16 sig_inc_4x4 =
		{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
	static const i8x16 sig_inc_8x8[2][4] = {{
		{ 0,  1,  2,  3,  4,  5,  5,  4,  4,  3,  3,  4,  4,  4,  5,  5},
		{ 4,  4,  4,  4,  3,  3,  6,  7,  7,  7,  8,  9, 10,  9,  8,  7},
		{ 7,  6, 11, 12, 13, 11,  6,  7,  8,  9, 14, 10,  9,  8,  6, 11},
		{12, 13, 11,  6,  9, 14, 10,  9, 11, 12, 13, 11, 14, 10, 12,  0},
		}, {
		{ 0,  1,  1,  2,  2,  3,  3,  4,  5,  6,  7,  7,  7,  8,  4,  5},
		{ 6,  9, 10, 10,  8, 11, 12, 11,  9,  9, 10, 10,  8, 11, 12, 11},
		{ 9,  9, 10, 10,  8, 11, 12, 11,  9,  9, 10, 10,  8, 13, 13,  9},
		{ 9, 10, 10,  8, 13, 13,  9,  9, 10, 10, 14, 14, 14, 14, 14,  0},
	}};
	static const i8x16 last_inc_8x8[4] = {
		{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
		{3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4},
		{5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8},
	};
	static const i8x16 sig_inc_chromaDC[2] =
		{{0, 1, 2, 0, 0, 1, 2, 0}, {0, 0, 1, 1, 2, 2, 2, 0, 0, 0, 1, 1, 2, 2, 2, 0}};
	
	static const i16x4 ctxIdxOffsets_16x16DC[3][2] = {
		{{85, 105, 166, 227}, {85, 277, 338, 227}}, // ctxBlockCat==0
		{{460, 484, 572, 952}, {460, 776, 864, 952}}, // ctxBlockCat==6
		{{472, 528, 616, 982}, {472, 820, 908, 982}}, // ctxBlockCat==10
	};
	static const i16x4 ctxIdxOffsets_16x16AC[3][2] = {
		{{89, 119, 180, 237}, {89, 291, 352, 237}}, // ctxBlockCat==1
		{{464, 498, 586, 962}, {464, 790, 878, 962}}, // ctxBlockCat==7
		{{476, 542, 630, 992}, {476, 834, 922, 992}}, // ctxBlockCat==11
	};
	static const i16x4 ctxIdxOffsets_chromaDC[2] =
		{{97, 149, 210, 257}, {97, 321, 382, 257}}; // ctxBlockCat==3
	static const i16x4 ctxIdxOffsets_chromaAC[2] =
		{{101, 151, 212, 266}, {101, 323, 384, 266}}; // ctxBlockCat==4
	static const i16x4 ctxIdxOffsets_4x4[3][2] = {
		{{93, 134, 195, 247}, {93, 306, 367, 247}}, // ctxBlockCat==2
		{{468, 528, 616, 972}, {468, 805, 893, 972}}, // ctxBlockCat==8
		{{480, 557, 645, 1002}, {480, 849, 937, 1002}}, // ctxBlockCat==12
	};
	static const i16x4 ctxIdxOffsets_8x8[3][2] = {
		{{1012, 402, 417, 426}, {1012, 436, 451, 426}}, // ctxBlockCat==5
		{{1016, 660, 690, 708}, {1016, 675, 699, 708}}, // ctxBlockCat==9
		{{1020, 718, 748, 766}, {1020, 733, 757, 766}}, // ctxBlockCat==13
	};
#endif



/**
 * Parse the coeff_token header for a non-ChromaDCLevel residual block encoded
 * with CAVLC (9.2.1)
 *
 * While both CAVLC and residual coefficients are not too critical to optimize,
 * this function is designed to be simple and compact.
 */
#if !CABAC
	static int parse_coeff_token_cavlc(Edge264Context *ctx, int i4x4, int nA, int nB) {
		static const uint8_t nC_offset[8] = {184, 184, 80, 80, 0, 0, 0, 0};
		static const int16_t tokens[38 * 8] = {
			543, 539, 535, 531, 527, 522, 517, 512, // 4 <= nC < 8
			661, 662, 657, 658, 653, 675, 654, 649,
			780, 798, 797, 776, 807, 794, 793, 772,
			924, 920, 934, 916, 939, 930, 929, 912,
			1075, 1070, 1065, 1060, 1071, 1066, 1061, 1056,
			1200, 1206, 1201, 1196, 1207, 1202, 1197, 1192,
			1341, 1336, 1339, 1338, 1337, 1332, 1205, 1205,
			1345, 1345, 1340, 1340, 1343, 1343, 1342, 1342,
			1347, 1347, 1347, 1347, 1346, 1346, 1346, 1346,
			1344, 1344, 1344, 1344, 1344, 1344, 1344, 1344,
			261, 261, 261, 261, 256, 256, 256, 256, // 2 <= cN < 4
			531, 531, 527, 527, 394, 394, 394, 394,
			795, 782, 781, 772, 663, 663, 649, 649,
			799, 799, 786, 786, 785, 785, 776, 776,
			931, 931, 918, 918, 917, 917, 908, 908,
			1044, 1044, 1050, 1050, 1049, 1049, 1040, 1040,
			1191, 1191, 1182, 1182, 1181, 1181, 1176, 1176,
			1455, 1446, 1445, 1440, 1451, 1442, 1441, 1436,
			1580, 1582, 1581, 1576, 1587, 1578, 1577, 1572,
			1723, 1718, 1717, 1716, 1719, 1714, 1713, 1712,
			1853, 1852, 1854, 1849, 1722, 1722, 1720, 1720,
			1859, 1859, 1858, 1858, 1857, 1857, 1856, 1856,
			1727, 1727, 1727, 1727, 1727, 1727, 1727, 1727,
			128, 128, 128, 128, 128, 128, 128, 128, // 0 <= nC < 2
			261, 261, 261, 261, 261, 261, 261, 261,
			394, 394, 394, 394, 394, 394, 394, 394,
			777, 777, 772, 772, 655, 655, 655, 655,
			919, 919, 910, 910, 787, 787, 787, 787,
			1051, 1051, 1042, 1042, 1037, 1037, 1032, 1032,
			1183, 1183, 1174, 1174, 1169, 1169, 1164, 1164,
			1315, 1315, 1306, 1306, 1301, 1301, 1296, 1296,
			1447, 1447, 1438, 1438, 1433, 1433, 1428, 1428,
			1696, 1702, 1697, 1692, 1707, 1698, 1693, 1688,
			1843, 1838, 1833, 1832, 1839, 1834, 1829, 1828,
			1979, 1974, 1969, 1968, 1975, 1970, 1965, 1964,
			2115, 2110, 2109, 2104, 2111, 2106, 2105, 2100,
			2112, 2112, 2114, 2114, 2113, 2113, 2108, 2108,
			1973, 1973, 1973, 1973, 1973, 1973, 1973, 1973,
		};
		
		int sum = nA + nB;
		int nC = !(ctx->unavail4x4[i4x4] & 3) ? (sum + 1) >> 1 : sum;
		int coeff_token, v;
		if (__builtin_expect(nC < 8, 1)) {
			int leadingZeroBits = clz(ctx->t._gb.msb_cache | (size_t)1 << (SIZE_BIT - 15));
			unsigned fourBits = ctx->t._gb.msb_cache >> (SIZE_BIT - 4 - leadingZeroBits);
			int token = tokens[nC_offset[nC] + leadingZeroBits * 8 + fourBits - 8];
			coeff_token = token & 127;
			v = token >> 7;
		} else {
			coeff_token = (ctx->t._gb.msb_cache >> (SIZE_BIT - 6)) + 4;
			if (coeff_token == 7)
				coeff_token = 0;
			v = 6;
		}
		ctx->t._gb.msb_cache = shld(ctx->t._gb.lsb_cache, ctx->t._gb.msb_cache, v);
		if (ctx->t._gb.lsb_cache <<= v)
			return coeff_token;
		return refill(&ctx->t._gb, coeff_token);
	}
	
	// 4:2:0 is best handled separately due to the open-ended 0000000 code and 3 bit suffixes
	static int parse_DC2x2_coeff_token_cavlc(Edge264Context *ctx) {
		static const int16_t tokens[] = {
			133, 133, 133, 133,
			256, 256, 256, 256,
			394, 394, 394, 394,
			776, 783, 777, 772,
			784, 784, 780, 780,
			910, 910, 909, 909,
			1042, 1042, 1041, 1041,
			915, 915, 915, 915,
		};
		int leadingZeroBits = clz(ctx->t._gb.msb_cache | (size_t)1 << (SIZE_BIT - 8));
		unsigned suffix = ctx->t._gb.msb_cache >> (SIZE_BIT - 3 - leadingZeroBits) & 3;
		int token = tokens[leadingZeroBits * 4 + suffix];
		int coeff_token = token & 127;
		int v = token >> 7;
		ctx->t._gb.msb_cache = shld(ctx->t._gb.lsb_cache, ctx->t._gb.msb_cache, v);
		if (ctx->t._gb.lsb_cache <<= v)
			return coeff_token;
		return refill(&ctx->t._gb, coeff_token);
	}
#endif



/**
 * Temporary functions for two alternatives to parse total_zeros.
 */
#if !CABAC
	static inline int parse_total_zeros(Edge264Context *ctx, int endIdx, int TotalCoeff) {
		static const uint8_t codes[27][9 * 4] = { // [tzVlcIndex][leadingZeroBits][suffix]
			// 2x2 blocks
			{16, 16, 16, 16, 33, 33, 33, 33, 50, 50, 50, 50, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51},
			{16, 16, 16, 16, 33, 33, 33, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34},
			{16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17},
			{}, // 2x4 blocks
			{16, 16, 16, 16, 49, 49, 50, 50, 67, 67, 68, 68, 69, 69, 69, 69, 86, 86, 86, 86, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87},
			{51, 52, 53, 54, 33, 33, 33, 33, 50, 50, 50, 50, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48},
			{35, 35, 52, 53, 34, 34, 34, 34, 49, 49, 49, 49, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48},
			{35, 35, 48, 52, 34, 34, 34, 34, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33},
			{34, 34, 35, 35, 33, 33, 33, 33, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32},
			{18, 18, 18, 18, 33, 33, 33, 33, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32},
			{17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16},
			{}, // 4x4 blocks
			{16, 16, 16, 16, 50, 50, 49, 49, 68, 68, 67, 67, 86, 86, 85, 85, 104, 104, 103, 103, 122, 122, 121, 121, 140, 140, 139, 139, 158, 158, 157, 157, 159, 159, 159, 159},
			{51, 50, 49, 48, 70, 69, 52, 52, 72, 72, 71, 71, 90, 90, 89, 89, 108, 108, 107, 107, 109, 109, 109, 109, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110},
			{54, 51, 50, 49, 68, 64, 55, 55, 72, 72, 69, 69, 90, 90, 89, 89, 92, 92, 92, 92, 107, 107, 107, 107, 109, 109, 109, 109, 109, 109, 109, 109, 109, 109, 109, 109},
			{54, 53, 52, 49, 67, 66, 56, 56, 73, 73, 71, 71, 90, 90, 80, 80, 91, 91, 91, 91, 92, 92, 92, 92, 92, 92, 92, 92, 92, 92, 92, 92, 92, 92, 92, 92},
			{54, 53, 52, 51, 65, 64, 55, 55, 72, 72, 66, 66, 74, 74, 74, 74, 89, 89, 89, 89, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91},
			{53, 52, 51, 50, 55, 55, 54, 54, 57, 57, 57, 57, 72, 72, 72, 72, 81, 81, 81, 81, 96, 96, 96, 96, 106, 106, 106, 106, 106, 106, 106, 106, 106, 106, 106, 106},
			{51, 50, 37, 37, 54, 54, 52, 52, 56, 56, 56, 56, 71, 71, 71, 71, 81, 81, 81, 81, 96, 96, 96, 96, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105},
			{37, 37, 36, 36, 54, 54, 51, 51, 55, 55, 55, 55, 65, 65, 65, 65, 82, 82, 82, 82, 96, 96, 96, 96, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104, 104},
			{36, 36, 35, 35, 38, 38, 38, 38, 53, 53, 53, 53, 66, 66, 66, 66, 87, 87, 87, 87, 96, 96, 96, 96, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97, 97},
			{36, 36, 35, 35, 37, 37, 37, 37, 50, 50, 50, 50, 70, 70, 70, 70, 80, 80, 80, 80, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81},
			{20, 20, 20, 20, 51, 51, 53, 53, 50, 50, 50, 50, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64},
			{19, 19, 19, 19, 34, 34, 34, 34, 52, 52, 52, 52, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64},
			{18, 18, 18, 18, 35, 35, 35, 35, 49, 49, 49, 49, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48},
			{18, 18, 18, 18, 33, 33, 33, 33, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32},
			{17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16},
		};
		int leadingZeroBits = clz(ctx->t._gb.msb_cache | (size_t)1 << (SIZE_BIT - 9));
		int suffix = ctx->t._gb.msb_cache >> ((leadingZeroBits + 2) ^ (SIZE_BIT - 1)) & 3;
		int code = codes[endIdx + TotalCoeff - 4][leadingZeroBits * 4 + suffix];
		int v = code >> 4;
		ctx->t._gb.msb_cache = shld(ctx->t._gb.lsb_cache, ctx->t._gb.msb_cache, v);
		ctx->t._gb.lsb_cache <<= v;
		return code & 15;
	}
#endif



/**
 * Parse a CAVLC or CABAC residual block without coeff_token nor
 * coded_block_flag (9.2 and 9.3.2.3).
 * 
 * For CAVLC, while the spec is quite convoluted level_prefix+level_suffix just
 * look like Exp-Golomb codes, so we need only compute a code length (v) and an
 * offset to add to the input bits.
 * For CABAC, all bypass bits can be extracted using a binary division (!!).
 * coeff_abs_level expects at most 2^(7+14)-14, i.e 41 bits as Exp-Golomb, so
 * we can get all of them on 64 bit machines.
 * While residual coefficients are not too critical to optimize, both CAVLC and
 * CABAC versions of this function are designed to be simple and compact.
 */
static void CAFUNC(parse_residual_block, int startIdx, int endIdx, int token_or_cbf) {
	#if !CABAC
		int TrailingOnes = token_or_cbf & 3;
		int TotalCoeff = token_or_cbf >> 2;
		
		// parse all level values from end to start
		int32_t level[16];
		size_t signs = ~ctx->t._gb.msb_cache;
		level[0] = (signs >> (SIZE_BIT - 2) & 2) - 1;
		level[1] = (signs >> (SIZE_BIT - 3) & 2) - 1;
		level[2] = (signs >> (SIZE_BIT - 4) & 2) - 1;
		ctx->t._gb.msb_cache = shld(ctx->t._gb.lsb_cache, ctx->t._gb.msb_cache, TrailingOnes);
		ctx->t._gb.lsb_cache <<= TrailingOnes;
		for (int i = TrailingOnes, suffixLength = 1, v, offset; i < TotalCoeff; i++) {
			int level_prefix = clz(ctx->t._gb.msb_cache | (size_t)1 << (SIZE_BIT - 26)); // limit given in 9.2.2.1
			if (level_prefix >= 15) {
				v = level_prefix * 2 - 2;
				offset = (15 << suffixLength) - 4096;
			} else if (i > TrailingOnes || (TotalCoeff > 10 && TrailingOnes < 3)) {
				v = level_prefix + suffixLength + 1;
				offset = (level_prefix - 1) << suffixLength;
			} else if (level_prefix < 14) {
				v = level_prefix + 1;
				offset = level_prefix - 1;
			} else {
				v = 19;
				offset = -2;
			}
			#if SIZE_BIT == 32
				v -= level_prefix;
				ctx->t._gb.msb_cache = shld(ctx->t._gb.lsb_cache, ctx->t._gb.msb_cache, level_prefix);
				if (!(ctx->t._gb.lsb_cache <<= level_prefix))
					refill(&ctx->t._gb, 0);
			#endif
			int levelCode = get_uv(&ctx->t._gb, v) + offset;
			levelCode += (i == TrailingOnes && TrailingOnes < 3) * 2;
			level[i] = (levelCode % 2) ? (-levelCode - 1) >> 1 : (levelCode + 2) >> 1;
			suffixLength = min(suffixLength + (levelCode >= (3 << suffixLength)), 6);
		}
		
		// store level values at proper positions in memory
		int zerosLeft = 0;
		if (TotalCoeff <= endIdx - startIdx)
			zerosLeft = parse_total_zeros(ctx, endIdx, TotalCoeff);
		int8_t *scan = ctx->scan + startIdx + zerosLeft + TotalCoeff - 1;
		ctx->c[*scan] = level[0];
		for (int i = 1, v, run_before; i < TotalCoeff; i++) {
			scan--;
			if (zerosLeft > 0) {
				int threeBits = ctx->t._gb.msb_cache >> (SIZE_BIT - 3);
				if (zerosLeft <= 6) {
					static int8_t run_before_codes[6][8] = {
						{9, 9, 9, 9, 8, 8, 8, 8},
						{18, 18, 17, 17, 8, 8, 8, 8},
						{19, 19, 18, 18, 17, 17, 16, 16},
						{28, 27, 18, 18, 17, 17, 16, 16},
						{29, 28, 27, 26, 17, 17, 16, 16},
						{25, 26, 28, 27, 30, 29, 16, 16},
					};
					int code = run_before_codes[zerosLeft - 1][threeBits];
					v = code >> 3;
					run_before = code & 7;
				} else if (threeBits > 0) {
					v = 3;
					run_before = threeBits ^ 7; // 7 - threeBits
				} else {
					v = clz(ctx->t._gb.msb_cache) + 1;
					run_before = min(v + 3, zerosLeft);
				}
				scan -= run_before;
				zerosLeft -= run_before;
				ctx->t._gb.msb_cache = shld(ctx->t._gb.lsb_cache, ctx->t._gb.msb_cache, v);
				ctx->t._gb.lsb_cache <<= v;
			}
			ctx->c[*scan] = level[i];
		}
		
		// trailing_ones_sign_flags+total_zeros+run_before consumed at most 31 bits, so we can delay refill here
		if (!ctx->t._gb.lsb_cache)
			refill(&ctx->t._gb, 0);
	#else // CABAC
		// significant_coeff_flags are stored as a bit mask
		uint64_t significant_coeff_flags = 0;
		int i = startIdx;
		do {
			if (get_ae(ctx, ctx->ctxIdxOffsets[1] + ctx->sig_inc[i])) {
				significant_coeff_flags |= (uint64_t)1 << i;
				if (get_ae(ctx, ctx->ctxIdxOffsets[2] + ctx->last_inc[i]))
					break;
			}
		} while (++i < endIdx);
		significant_coeff_flags |= (uint64_t)1 << i;
		
		// Now loop on set bits to parse all non-zero coefficients.
		int ctxIdx0 = ctx->ctxIdxOffsets[3] + 1;
		int ctxIdx1 = ctx->ctxIdxOffsets[3] + 5;
		do {
			int coeff_level = 1;
			if (!get_ae(ctx, ctxIdx0)) {
				static const int8_t trans[5] = {0, 2, 3, 4, 4};
				ctxIdx0 = ctx->ctxIdxOffsets[3] + trans[ctxIdx0 - ctx->ctxIdxOffsets[3]];
				coeff_level = get_bypass(ctx) ? -coeff_level : coeff_level;
			} else {
				coeff_level++;
				while (coeff_level < 15 && get_ae(ctx, ctxIdx1))
					coeff_level++;
				ctxIdx0 = ctx->ctxIdxOffsets[3];
				ctxIdx1 = ctxIdx0 + ctx->coeff_abs_inc[ctxIdx1 - ctxIdx0 - 5];
				#if SIZE_BIT == 32
					if (coeff_level >= 15) {
						// the biggest value to encode is 2^(14+7)-14, for which k=20 (see 9.3.2.3)
						int k = 0;
						while (get_bypass(ctx) && k < 20)
							k++;
						coeff_level = 1;
						while (k--)
							coeff_level += coeff_level + get_bypass(ctx);
						coeff_level += 14;
					}
					coeff_level = get_bypass(ctx) ? -coeff_level : coeff_level;
				#elif SIZE_BIT == 64
					if (coeff_level >= 15) {
						// we need at least 51 bits in codIOffset to get 42 bits with a division by 9 bits
						int zeros = clz(ctx->t._gb.codIRange);
						if (zeros > 64 - 51) {
							ctx->t._gb.codIOffset = shld(get_bytes(&ctx->t._gb, zeros >> 3), ctx->t._gb.codIOffset, zeros & -8);
							ctx->t._gb.codIRange <<= zeros & -8;
							zeros &= 7;
						}
						ctx->t._gb.codIRange >>= 64 - 9 - zeros;
						size_t quo = ctx->t._gb.codIOffset / ctx->t._gb.codIRange; // requested bits are in lsb and zeros+9 empty bits above
						size_t rem = ctx->t._gb.codIOffset % ctx->t._gb.codIRange;
						int k = clz(~quo << (zeros + 9) | (size_t)1 << (SIZE_BIT - 21));
						int unused = 64 - 9 - zeros - k * 2 - 2;
						coeff_level = 14 + (1 << k | (quo >> unused >> 1 & (((size_t)1 << k) - 1)));
						coeff_level = (quo & (size_t)1 << unused) ? -coeff_level : coeff_level;
						ctx->t._gb.codIOffset = (quo & (((size_t)1 << unused) - 1)) * ctx->t._gb.codIRange + rem;
						ctx->t._gb.codIRange <<= unused;
					} else {
						coeff_level = get_bypass(ctx) ? -coeff_level : coeff_level;
					}
				#endif
			}
		
			// scale and store
			int i = 63 - clz64(significant_coeff_flags);
			ctx->c[ctx->scan[i]] = coeff_level; // beware, scan is transposed already
			significant_coeff_flags &= ~((uint64_t)1 << i);
		} while (significant_coeff_flags != 0);
	#endif
	for (int i = startIdx; i <= endIdx; i++)
		print_slice(ctx, (i < endIdx) ? "%d," : "%d]\n", ctx->c[ctx->scan[i]]);
}



/**
 * As its name says, parses mb_qp_delta (9.3.2.7 and 9.3.3.1.1.5).
 */
static void CAFUNC(parse_mb_qp_delta)
{
	#if !CABAC
		int mb_qp_delta = get_se16(&ctx->t._gb, -26, 25); // FIXME QpBdOffset
		if (mb_qp_delta) {
			int sum = ctx->t.QP[0] + mb_qp_delta;
			int QP_Y = (sum < 0) ? sum + 52 : (sum >= 52) ? sum - 52 : sum;
			mb->QP_s = ctx->t.QP_s = (i8x4){QP_Y, ctx->QP_C[0][QP_Y], ctx->QP_C[1][QP_Y]};
		}
	#else
		int mb_qp_delta_nz = get_ae(ctx, 60 + ctx->mb_qp_delta_nz);
		ctx->mb_qp_delta_nz = mb_qp_delta_nz;
		int mb_qp_delta = 0;
		if (mb_qp_delta_nz) {
			unsigned count = 1, ctxIdx = 62;
			while (get_ae(ctx, ctxIdx) && count < 52) // FIXME QpBdOffset
				count++, ctxIdx = 63;
			mb_qp_delta = count & 1 ? count / 2 + 1 : -(count / 2);
			int sum = ctx->t.QP[0] + mb_qp_delta;
			int QP_Y = (sum < 0) ? sum + 52 : (sum >= 52) ? sum - 52 : sum;
			mb->QP_s = ctx->t.QP_s = (i8x4){QP_Y, ctx->QP_C[0][QP_Y], ctx->QP_C[1][QP_Y]};
		}
	#endif
	print_slice(ctx, "  mb_qp_delta: %d\n", mb_qp_delta);
}



/**
 * Parsing for chroma 4:2:0 and 4:2:2 is put in a separate function to be
 * tail-called from parse_NxN_residual and parse_Intra16x16_residual.
 */
static void CAFUNC(parse_chroma_residual)
{
	// As in Intra16x16, DC blocks are parsed to ctx->c[0..15], then transformed to ctx->c[16..31]
	if (mb->f.CodedBlockPatternChromaDC) { // valid also for 4:0:0
		#if CABAC
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_chromaDC[0]; // FIXME 4:2:2
			ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_chromaDC[0];
			ctx->coeff_abs_inc_l = (i8x8){6, 7, 8, 8};
		#endif
		ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (i32x4){};
		int token_or_cbf_Cb = CACOND(
			parse_DC2x2_coeff_token_cavlc(ctx),
			get_ae(ctx, ctx->ctxIdxOffsets[0] + ctx->inc.coded_block_flags_16x16[1]));
		if (token_or_cbf_Cb) {
			#if CABAC
				mb->f.coded_block_flags_16x16[1] = 1;
			#endif
			print_slice(ctx, "  I4x4_DC_coeffLevels[1]: [");
			ctx->scan_s = (i8x4){0, 4, 2, 6};
			CACALL(parse_residual_block, 0, 3, token_or_cbf_Cb);
		}
		int token_or_cbf_Cr = CACOND(
			parse_DC2x2_coeff_token_cavlc(ctx),
			get_ae(ctx, ctx->ctxIdxOffsets[0] + ctx->inc.coded_block_flags_16x16[2]));
		if (token_or_cbf_Cr) {
			#if CABAC
				mb->f.coded_block_flags_16x16[2] = 1;
			#endif
			print_slice(ctx, "  I4x4_DC_coeffLevels[2]: [");
			ctx->scan_s = (i8x4){1, 5, 3, 7};
			CACALL(parse_residual_block, 0, 3, token_or_cbf_Cr);
		}
		transform_dc2x2(ctx);
		
		// Eight or sixteen 4x4 AC blocks for the Cb/Cr components
		if (mb->f.CodedBlockPatternChromaAC) {
			#if CABAC
				ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
				ctx->ctxIdxOffsets_l = ctxIdxOffsets_chromaAC[0];
				ctx->coeff_abs_inc_l = (i8x8){6, 7, 8, 9, 9};
			#endif
			ctx->scan_v[0] = scan_4x4[0];
			for (int i4x4 = 0; i4x4 < 8; i4x4++) {
				int iYCbCr = 1 + (i4x4 >> 2);
				uint8_t *samples = ctx->samples_mb[iYCbCr] + y420[i4x4] * ctx->t.stride[1] + x420[i4x4];
				int nA = *((int8_t *)mb->nC[1] + ctx->ACbCr_int8[i4x4]);
				int nB = *((int8_t *)mb->nC[1] + ctx->BCbCr_int8[i4x4]);
				int token_or_cbf = CACOND(parse_coeff_token_cavlc(ctx, i4x4 << 2 & 15, nA, nB),
					get_ae(ctx, ctx->ctxIdxOffsets[0] + ctx->nC_inc[1][i4x4] + nA + nB * 2));
				if (token_or_cbf) {
					mb->nC[1][i4x4] = CACOND(token_or_cbf >> 2, 1);
					ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (i32x4){};
					print_slice(ctx, "  I4x4_AC_coeffLevels[%d]: [", 16 + i4x4);
					CACALL(parse_residual_block, 1, 15, token_or_cbf);
					add_idct4x4(ctx, iYCbCr, i4x4, samples);
				} else {
					add_dc4x4(ctx, iYCbCr, i4x4, samples);
				}
			}
		}
	}
}



/**
 * Intra16x16 residual blocks have so many differences with Intra4x4 that they
 * deserve their own function.
 */
static void CAFUNC(parse_Intra16x16_residual)
{
	CACALL(parse_mb_qp_delta);
	
	// Both AC and DC coefficients are initially parsed to ctx->c[0..15]
	ctx->scan_v[0] = scan_4x4[0];
	#if CABAC
		ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
	#endif
	for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
		
		// Parse a DC block, then transform it to ctx->c[16..31]
		#if CABAC
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_16x16DC[iYCbCr][0];
			ctx->coeff_abs_inc_l = (i8x8){6, 7, 8, 9, 9};
		#endif
		int token_or_cbf = CACOND(
			parse_coeff_token_cavlc(ctx, 0, mbA->nC[iYCbCr][5], mbB->nC[iYCbCr][10]),
			get_ae(ctx, ctx->ctxIdxOffsets[0] + ctx->inc.coded_block_flags_16x16[iYCbCr]));
		if (token_or_cbf) {
			#if CABAC
				mb->f.coded_block_flags_16x16[iYCbCr] = 1;
			#endif
			ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (i32x4){};
			print_slice(ctx, "  I16x16_DC_coeffLevels[%u]: [", iYCbCr);
			CACALL(parse_residual_block, 0, 15, token_or_cbf);
			transform_dc4x4(ctx, iYCbCr);
		} else {
			if (mb->bits[0] & 1 << 5)
				ctx->c_v[4] = ctx->c_v[5] = ctx->c_v[6] = ctx->c_v[7] = (i32x4){};
		}
		
		// All AC blocks pick a DC coeff, then go to ctx->c[1..15]
		if (mb->bits[0] & 1 << 5) {
			#if CABAC
				ctx->ctxIdxOffsets_l = ctxIdxOffsets_16x16AC[iYCbCr][0];
				ctx->coeff_abs_inc_l = (i8x8){6, 7, 8, 9, 9};
			#endif
			for (int i4x4 = 0; i4x4 < 16; i4x4++) {
				uint8_t *samples = ctx->samples_mb[iYCbCr] + y444[i4x4] * ctx->t.stride[iYCbCr] + x444[i4x4];
				int nA = *((int8_t *)mb->nC[iYCbCr] + ctx->A4x4_int8[i4x4]);
				int nB = *((int8_t *)mb->nC[iYCbCr] + ctx->B4x4_int8[i4x4]);
				int token_or_cbf = CACOND(parse_coeff_token_cavlc(ctx, i4x4, nA, nB),
					get_ae(ctx, ctx->ctxIdxOffsets[0] + ctx->nC_inc[iYCbCr][i4x4] + nA + nB * 2));
				if (token_or_cbf) {
					mb->nC[iYCbCr][i4x4] = CACOND(token_or_cbf >> 2, 1);
					ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (i32x4){};
					print_slice(ctx, "  I16x16_AC_coeffLevels[%u]: [", iYCbCr * 16 + i4x4);
					CACALL(parse_residual_block, 1, 15, token_or_cbf);
					add_idct4x4(ctx, iYCbCr, i4x4, samples);
				} else {
					add_dc4x4(ctx, iYCbCr, i4x4, samples);
				}
			}
		}
		
		// here is how we share the decoding of luma coefficients with 4:4:4 modes
		if (ctx->t.ChromaArrayType <3)
			CAJUMP(parse_chroma_residual);
	}
}



/**
 * This block is dedicated to the parsing of Intra_NxN and Inter_NxN residual
 * blocks, since they share much in common.
 */
static void CAFUNC(parse_NxN_residual)
{
	static const int8_t Intra4x4Modes[9][16] = {
		{I4x4_V_8  , I4x4_V_8   , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_V_8   , I4x4_V_8   , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_V_8   , I4x4_V_8   , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_V_8   , I4x4_V_8   , I4x4_DCAB_8, I4x4_DCAB_8},
		{I4x4_H_8  , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8, I4x4_H_8   , I4x4_DCAB_8},
		{I4x4_DC_8 , I4x4_DCA_8 , I4x4_DCB_8 , I4x4_DCAB_8, I4x4_DC_8  , I4x4_DCA_8 , I4x4_DCB_8 , I4x4_DCAB_8, I4x4_DC_8  , I4x4_DCA_8 , I4x4_DCB_8 , I4x4_DCAB_8, I4x4_DC_8  , I4x4_DCA_8 , I4x4_DCB_8 , I4x4_DCAB_8},
		{I4x4_DDL_8, I4x4_DDL_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DDLC_8, I4x4_DDLC_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DDL_8 , I4x4_DDL_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DDLC_8, I4x4_DDLC_8, I4x4_DCAB_8, I4x4_DCAB_8},
		{I4x4_DDR_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DDR_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8},
		{I4x4_VR_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_VR_8  , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8},
		{I4x4_HD_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_HD_8  , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8, I4x4_DCAB_8},
		{I4x4_VL_8 , I4x4_VL_8  , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_VLC_8 , I4x4_VLC_8 , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_VL_8  , I4x4_VL_8  , I4x4_DCAB_8, I4x4_DCAB_8, I4x4_VLC_8 , I4x4_VLC_8 , I4x4_DCAB_8, I4x4_DCAB_8},
		{I4x4_HU_8 , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8, I4x4_HU_8  , I4x4_DCAB_8},
	};
	static const int8_t Intra8x8Modes[9][16] = {
		{I8x8_V_8  , I8x8_V_8    , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_V_C_8  , I8x8_V_C_8  , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_V_D_8  , I8x8_V_D_8  , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_V_CD_8  , I8x8_V_CD_8  , I8x8_DC_AB_8, I8x8_DC_AB_8},
		{I8x8_H_8  , I8x8_DC_AB_8, I8x8_H_8    , I8x8_DC_AB_8, I8x8_H_8    , I8x8_DC_AB_8, I8x8_H_8    , I8x8_DC_AB_8, I8x8_H_D_8  , I8x8_DC_AB_8, I8x8_H_D_8  , I8x8_DC_AB_8, I8x8_H_D_8   , I8x8_DC_AB_8 , I8x8_H_D_8  , I8x8_DC_AB_8},
		{I8x8_DC_8 , I8x8_DC_A_8 , I8x8_DC_B_8 , I8x8_DC_AB_8, I8x8_DC_C_8 , I8x8_DC_AC_8, I8x8_DC_B_8 , I8x8_DC_AB_8, I8x8_DC_D_8 , I8x8_DC_AD_8, I8x8_DC_BD_8, I8x8_DC_AB_8, I8x8_DC_CD_8 , I8x8_DC_ACD_8, I8x8_DC_BD_8, I8x8_DC_AB_8},
		{I8x8_DDL_8, I8x8_DDL_8  , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DDL_C_8, I8x8_DDL_C_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DDL_D_8, I8x8_DDL_D_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DDL_CD_8, I8x8_DDL_CD_8, I8x8_DC_AB_8, I8x8_DC_AB_8},
		{I8x8_DDR_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DDR_C_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8 , I8x8_DC_AB_8 , I8x8_DC_AB_8, I8x8_DC_AB_8},
		{I8x8_VR_8 , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_VR_C_8 , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8 , I8x8_DC_AB_8 , I8x8_DC_AB_8, I8x8_DC_AB_8},
		{I8x8_HD_8 , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_HD_8   , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_DC_AB_8 , I8x8_DC_AB_8 , I8x8_DC_AB_8, I8x8_DC_AB_8},
		{I8x8_VL_8 , I8x8_VL_8   , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_VL_C_8 , I8x8_VL_C_8 , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_VL_D_8 , I8x8_VL_D_8 , I8x8_DC_AB_8, I8x8_DC_AB_8, I8x8_VL_CD_8 , I8x8_VL_CD_8 , I8x8_DC_AB_8, I8x8_DC_AB_8},
		{I8x8_HU_8 , I8x8_DC_AB_8, I8x8_HU_8   , I8x8_DC_AB_8, I8x8_HU_8   , I8x8_DC_AB_8, I8x8_HU_8   , I8x8_DC_AB_8, I8x8_HU_D_8 , I8x8_DC_AB_8, I8x8_HU_D_8 , I8x8_DC_AB_8, I8x8_HU_D_8  , I8x8_DC_AB_8 , I8x8_HU_D_8 , I8x8_DC_AB_8},
	};
	
	if (mb->f.CodedBlockPatternChromaDC | (mb->bits[0] & 0xac))
		CACALL(parse_mb_qp_delta);
	#if CABAC
		else
			ctx->mb_qp_delta_nz = 0;
	#endif
	
	// next few blocks will share many parameters, so we cache them
	for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
		if (!mb->f.transform_size_8x8_flag) {
			#if CABAC
				ctx->ctxIdxOffsets_l = ctxIdxOffsets_4x4[iYCbCr][0];
				ctx->coeff_abs_inc_l = (i8x8){6, 7, 8, 9, 9};
				ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
			#endif
			ctx->scan_v[0] = scan_4x4[0];
			
			// Decoding directly follows parsing to avoid duplicate loops.
			for (int i4x4 = 0; i4x4 < 16; i4x4++) {
				size_t stride = ctx->t.stride[iYCbCr];
				uint8_t *samples = ctx->samples_mb[iYCbCr] + y444[i4x4] * stride + x444[i4x4];
				if (!mb->mbIsInterFlag)
					decode_intra4x4(Intra4x4Modes[mb->Intra4x4PredMode[i4x4]][ctx->unavail4x4[i4x4]], samples, stride, ctx->t.samples_clip_v[iYCbCr]);
				if (mb->bits[0] & 1 << bit8x8[i4x4 >> 2]) {
					int nA = *((int8_t *)mb->nC[iYCbCr] + ctx->A4x4_int8[i4x4]);
					int nB = *((int8_t *)mb->nC[iYCbCr] + ctx->B4x4_int8[i4x4]);
					int token_or_cbf = CACOND(parse_coeff_token_cavlc(ctx, i4x4, nA, nB),
						get_ae(ctx, ctx->ctxIdxOffsets[0] + ctx->nC_inc[iYCbCr][i4x4] + nA + nB * 2));
					if (token_or_cbf) {
						mb->nC[iYCbCr][i4x4] = CACOND(token_or_cbf >> 2, 1);
						ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (i32x4){};
						print_slice(ctx, "  I4x4_coeffLevels[%u]: [", iYCbCr * 16 + i4x4);
						CACALL(parse_residual_block, 0, 15, token_or_cbf);
						// DC blocks are marginal here (about 16%) so we do not handle them separately
						add_idct4x4(ctx, iYCbCr, -1, samples); // FIXME 4:4:4
					}
				}
			}
		} else {
			#if CABAC
				ctx->ctxIdxOffsets_l = ctxIdxOffsets_8x8[iYCbCr][0];
				ctx->coeff_abs_inc_l = (i8x8){6, 7, 8, 9, 9};
				ctx->sig_inc_v[0] = sig_inc_8x8[0][0];
				ctx->last_inc_v[0] = last_inc_8x8[0];
				ctx->scan_v[0] = scan_8x8_cabac[0][0];
			#endif
			
			for (int i8x8 = 0; i8x8 < 4; i8x8++) {
				size_t stride = ctx->t.stride[iYCbCr];
				uint8_t *samples = ctx->samples_mb[iYCbCr] + y444[i8x8 * 4] * stride + x444[i8x8 * 4];
				if (!mb->mbIsInterFlag)
					decode_intra8x8(Intra8x8Modes[mb->Intra4x4PredMode[i8x8 * 4 + 1]][ctx->unavail4x4[i8x8 * 5]], samples, stride, ctx->t.samples_clip_v[iYCbCr]);
				if (mb->bits[0] & 1 << bit8x8[i8x8]) {
					#if !CABAC
						for (int i = 0; i < 16; i++)
							ctx->c_v[i] = (i32x4){};
						for (int i4x4 = 0; i4x4 < 4; i4x4++) {
							ctx->scan_v[0] = scan_8x8_cavlc[0][i4x4];
							int nA = *((int8_t *)mb->nC[iYCbCr] + ctx->A4x4_int8[i8x8 * 4 + i4x4]);
							int nB = *((int8_t *)mb->nC[iYCbCr] + ctx->B4x4_int8[i8x8 * 4 + i4x4]);
							int token = parse_coeff_token_cavlc(ctx, i8x8 * 4 + i4x4, nA, nB);
							if (token) {
								mb->nC[iYCbCr][i8x8 * 4 + i4x4] = token >> 2;
								print_slice(ctx, "  I8x8_coeffLevels[%u][%u]: [", iYCbCr * 4 + i8x8, i4x4);
								parse_residual_block_cavlc(ctx, 0, 15, token);
							}
						}
						add_idct8x8(ctx, iYCbCr, samples);
					#else
						if (ctx->t.ChromaArrayType < 3 || get_ae(ctx, ctx->ctxIdxOffsets[0] + (mb->bits[1] >> inc8x8[iYCbCr * 4 + i8x8] & 3))) {
							for (int i = 0; i < 16; i++)
								ctx->c_v[i] = (i32x4){};
							mb->bits[1] |= 1 << bit8x8[iYCbCr * 4 + i8x8];
							mb->nC_s[iYCbCr][i8x8] = 0x01010101;
							print_slice(ctx, "  I8x8_coeffLevels[%u]: [", iYCbCr * 4 + i8x8);
							parse_residual_block_cabac(ctx, 0, 63, 1);
							add_idct8x8(ctx, iYCbCr, samples);
						}
					#endif
				}
			}
		}
		
		// nice optimisation for 4:4:4 modes
		if (ctx->t.ChromaArrayType <3)
			CAJUMP(parse_chroma_residual);
	}
}



/**
 * Parses CodedBlockPatternLuma/Chroma (9.3.2.6 and 9.3.3.1.1.4).
 *
 * As with mb_qp_delta, coded_block_pattern is parsed in two distinct code
 * paths, thus put in a distinct function.
 */
static void CAFUNC(parse_coded_block_pattern, const uint8_t *map_me)
{
	// Luma prefix
	#if !CABAC
		int cbp = map_me[get_ue16(&ctx->t._gb, 47)];
		mb->bits[0] |= cbp & 0xac;
	#else
		int bits = mb->bits[0];
		bits |= get_ae(ctx, 76 - (bits      & 3)) << 5;
		bits |= get_ae(ctx, 76 - (bits >> 5 & 3)) << 3;
		bits |= get_ae(ctx, 76 - (bits >> 4 & 3)) << 2;
		bits |= get_ae(ctx, 76 - (bits >> 2 & 3)) << 7;
		mb->bits[0] = bits;
	#endif
	
	// Chroma suffix
	if ((ctx->t.ChromaArrayType == 1 || ctx->t.ChromaArrayType == 2) &&
		CACOND(cbp & 3, get_ae(ctx, 77 + ctx->inc.CodedBlockPatternChromaDC)))
	{
		mb->f.CodedBlockPatternChromaDC = 1;
		mb->f.CodedBlockPatternChromaAC = CACOND(cbp >> 1 & 1, get_ae(ctx, 81 + ctx->inc.CodedBlockPatternChromaAC));
	}
	
	print_slice(ctx, "  coded_block_pattern: %u\n",
		(mb->bits[0] >> 5 & 1) | (mb->bits[0] >> 2 & 2) | (mb->bits[0] & 4) | (mb->bits[0] >> 4 & 8) |
		(mb->f.CodedBlockPatternChromaDC + mb->f.CodedBlockPatternChromaAC) << 4);
}



/**
 * Parses intra_chroma_pred_mode (9.3.2.2 and 9.3.3.1.1.8).
 */
static void CAFUNC(parse_intra_chroma_pred_mode)
{
	static const int8_t IntraChromaModes[4][4] = {
		{IC8x8_DC_8, IC8x8_DCA_8, IC8x8_DCB_8, IC8x8_DCAB_8},
		{IC8x8_H_8 , IC8x8_DCA_8, IC8x8_H_8  , IC8x8_DCAB_8},
		{IC8x8_V_8 , IC8x8_V_8  , IC8x8_DCB_8, IC8x8_DCAB_8},
		{IC8x8_P_8 , IC8x8_DCA_8, IC8x8_DCB_8, IC8x8_DCAB_8},
	};
	
	// Do not optimise too hard to keep the code understandable here.
	int type = ctx->t.ChromaArrayType;
	if (type == 1 || type == 2) {
		#if !CABAC
			int mode = get_ue16(&ctx->t._gb, 3);
		#else
			int ctxIdx = 64 + ctx->inc.intra_chroma_pred_mode_non_zero;
			int mode = 0;
			while (mode <3 && get_ae(ctx, ctxIdx))
				mode++, ctxIdx = 67;
			mb->f.intra_chroma_pred_mode_non_zero = (mode > 0);
		#endif
		print_slice(ctx, "  intra_chroma_pred_mode: %u\n", mode);
		decode_intraChroma(IntraChromaModes[mode][ctx->unavail4x4[0] & 3], ctx->samples_mb[1], ctx->t.stride[1] >> 1, ctx->t.samples_clip_v[1]);
	}
}



/**
 * Parses prev_intraNxN_pred_mode_flag and rem_intraNxN_pred_mode, and returns
 * the given intra_pred_mode (7.3.5.1, 7.4.5.1, 8.3.1.1 and table 9-34).
 */
static int CAFUNC(parse_intraNxN_pred_mode, int luma4x4BlkIdx)
{
	// dcPredModePredictedFlag is enforced by putting -2
	int intraMxMPredModeA = *((int8_t *)mb->Intra4x4PredMode + ctx->A4x4_int8[luma4x4BlkIdx]);
	int intraMxMPredModeB = *((int8_t *)mb->Intra4x4PredMode + ctx->B4x4_int8[luma4x4BlkIdx]);
	int mode = abs(min(intraMxMPredModeA, intraMxMPredModeB));
	int rem_intra_pred_mode = -1;
	if (CACOND(!get_u1(&ctx->t._gb), !get_ae(ctx, 68))) {
		#if !CABAC
			rem_intra_pred_mode = get_uv(&ctx->t._gb, 3);
		#else
			rem_intra_pred_mode = get_ae(ctx, 69);
			rem_intra_pred_mode += get_ae(ctx, 69) * 2;
			rem_intra_pred_mode += get_ae(ctx, 69) * 4;
		#endif
		mode = rem_intra_pred_mode + (rem_intra_pred_mode >= mode);
	}
	print_slice(ctx, "%c%u", luma4x4BlkIdx ? ',' : '[', mode);
	return mode;
}



/**
 * This function parses the syntax elements mb_type, transform_size_8x8_flag,
 * intraNxN_pred_mode (from function), intra_chroma_pred_mode (from function),
 * coded_block_pattern (from function) and PCM stuff for the current Intra
 * macroblock. It proceeds to residual decoding through tail calls.
 *
 * In Intra4x4PredMode the special value -2 is used by unavailable blocks and
 * Inter blocks with constrained_intra_pred_flag, to account for
 * dcPredModePredictedFlag.
 */
static noinline void CAFUNC(parse_I_mb, int mb_type_or_ctxIdx)
{
	static const uint8_t me_intra[48] = {174, 173, 172, 0, 45, 169, 165, 141, 44, 168, 164, 140, 46, 170, 166, 142, 1, 40, 36, 136, 132, 41, 37, 137, 133, 42, 38, 138, 134, 32, 8, 4, 128, 33, 9, 5, 129, 12, 160, 13, 161, 2, 34, 10, 6, 130, 14, 162};
	
	// Intra-specific initialisations
	#if CABAC
		ctx->nC_inc_v[0] = ctx->nC_inc_v[1] = ctx->nC_inc_v[2] = (i8x16){};
		if (ctx->unavail4x4[0] & 1) {
			mb->bits[1] |= 0x111111; // FIXME 4:2:2
			ctx->nC_inc_v[0] += (i8x16){1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0};
			ctx->nC_inc_v[1] += (i8x16){1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			ctx->inc.coded_block_flags_16x16_s |= 0x010101;
		}
		if (ctx->unavail4x4[0] & 2) {
			mb->bits[1] |= 0x424242;
			ctx->nC_inc_v[0] += (i8x16){2, 2, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			ctx->nC_inc_v[1] += (i8x16){2, 2, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			ctx->inc.coded_block_flags_16x16_s |= 0x020202;
		}
	#endif
	mb->mbIsInterFlag = 0;
	mb->f.inter_eqs_s = little_endian32(0x1b5fbbff);
	mb->refIdx_l = -1;
	mb->refPic_l = -1;
	mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (i16x8){};
	
	// I_NxN
	if (CACOND(mb_type_or_ctxIdx == 0, !get_ae(ctx, mb_type_or_ctxIdx))) {
		#if CABAC
			mb->f.mb_type_I_NxN = 1;
			print_slice(ctx, (mb_type_or_ctxIdx == 17) ? "  mb_type: 5\n" : // in P slice
								 (mb_type_or_ctxIdx == 32) ? "  mb_type: 23\n" : // in B slice
																	  "  mb_type: 0\n"); // in I slice
		#endif
		
		// 7.3.5, 7.4.5, 9.3.3.1.1.10 and table 9-34
		int transform_size_8x8_flag = 0;
		if (ctx->t.pps.transform_8x8_mode_flag) {
			transform_size_8x8_flag = CACOND(get_u1(&ctx->t._gb), get_ae(ctx, 399 + ctx->inc.transform_size_8x8_flag));
			print_slice(ctx, "  transform_size_8x8_flag: %u\n", transform_size_8x8_flag);
		}
		
		mb->Intra4x4PredMode_v = (i8x16){-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2}; // default values when A/B is unavailable
		if (transform_size_8x8_flag) {
			mb->f.transform_size_8x8_flag = transform_size_8x8_flag;
			print_slice(ctx, "  Intra8x8PredModes: ");
			for (int i = 0; i < 16; i += 4)
				mb->Intra4x4PredMode[i + 1] = mb->Intra4x4PredMode[i + 2] = mb->Intra4x4PredMode[i + 3] = CACALL(parse_intraNxN_pred_mode, i);
		} else {
			print_slice(ctx, "  Intra4x4PredModes: ");
			for (int i = 0; i < 16; i++)
				mb->Intra4x4PredMode[i] = CACALL(parse_intraNxN_pred_mode, i);
		}
		print_slice(ctx, "]\n");
		
		CACALL(parse_intra_chroma_pred_mode);
		CACALL(parse_coded_block_pattern, me_intra);
		CAJUMP(parse_NxN_residual);
	
	// Intra_16x16
	} else if (__builtin_expect(CACOND(mb_type_or_ctxIdx < 25, !cabac_terminate(ctx)), 1)) {
		#if !CABAC
			int mb_type = mb_type_or_ctxIdx - 1;
			mb->bits[0] = mb_type > 11 ? 0xac : 0; // zeroes ref_idx_nz as byproduct
			mb_type = mb_type > 11 ? mb_type - 12 : mb_type;
			mb->f.CodedBlockPatternChromaDC = mb_type > 3;
			mb->f.CodedBlockPatternChromaAC = mb_type >> 3;
			int mode = mb_type & 3;
		#else
			int ctxIdx = max(mb_type_or_ctxIdx, 5);
			mb->bits[0] = get_ae(ctx, ctxIdx + 1) ? 0xac : 0; // zeroes ref_idx_nz as byproduct
			int CodedBlockPatternChromaDC = get_ae(ctx, ctxIdx + 2);
			ctxIdx = max(ctxIdx, 6);
			if (CodedBlockPatternChromaDC) {
				mb->f.CodedBlockPatternChromaDC = 1;
				mb->f.CodedBlockPatternChromaAC = get_ae(ctx, ctxIdx + 2);
			}
			int mode = get_ae(ctx, ctxIdx + 3) << 1;
			mode += get_ae(ctx, max(ctxIdx + 3, 10));
			print_slice(ctx, "  mb_type: %u\n", (mb->bits[0] & 12) +
				(mb->f.CodedBlockPatternChromaDC + mb->f.CodedBlockPatternChromaAC) * 4 +
				mode + (ctxIdx == 17 ? 6 : ctxIdx == 32 ? 24 : 1));
		#endif
		
		// decode the samples before parsing residuals
		static const int8_t Intra16x16Modes[4][4] = {
			{I16x16_V_8 , I16x16_V_8  , I16x16_DCB_8, I16x16_DCAB_8},
			{I16x16_H_8 , I16x16_DCA_8, I16x16_H_8  , I16x16_DCAB_8},
			{I16x16_DC_8, I16x16_DCA_8, I16x16_DCB_8, I16x16_DCAB_8},
			{I16x16_P_8 , I16x16_DCA_8, I16x16_DCB_8, I16x16_DCAB_8},
		};
		mb->Intra4x4PredMode_v = (i8x16){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
		decode_intra16x16(Intra16x16Modes[mode][ctx->unavail4x4[0] & 3], ctx->samples_mb[0], ctx->t.stride[0], ctx->t.samples_clip_v[0]); // FIXME 4:4:4
		CACALL(parse_intra_chroma_pred_mode);
		CAJUMP(parse_Intra16x16_residual);
		
	// I_PCM
	} else {
		#if !CABAC
			unsigned bits = (SIZE_BIT - 1 - ctz(ctx->t._gb.lsb_cache)) & 7;
			ctx->t._gb.msb_cache = shld(ctx->t._gb.lsb_cache, ctx->t._gb.msb_cache, bits);
			ctx->t._gb.lsb_cache = ctx->t._gb.lsb_cache << bits;
		#else
			print_slice(ctx, (mb_type_or_ctxIdx == 17) ? "  mb_type: 30\n" : (mb_type_or_ctxIdx == 32) ? "  mb_type: 48\n" : "  mb_type: 25\n");
		#endif
		
		ctx->mb_qp_delta_nz = 0;
		mb->f.v |= flags_twice.v; // ChromaDC, ChromaAC and flags_16x16, just what we need :)
		mb->QP_s = (i8x4){0, ctx->QP_C[0][0], ctx->QP_C[1][0]};
		mb->bits_l = (uint64_t)(i32x2){0xac, 0xacacac}; // FIXME 4:2:2
		mb->nC_v[0] = mb->nC_v[1] = mb->nC_v[2] = CACOND(
			((i8x16){16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16}),
			((i8x16){1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}));
		mb->Intra4x4PredMode_v = (i8x16){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
		
		// PCM is so rare that it should be compact rather than fast
		int MbWidth = 16, y = 16;
		for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
			int BitDepth = ctz(ctx->t.samples_clip[iYCbCr][0] + 1);
			for (uint8_t *p = ctx->samples_mb[iYCbCr]; y-- > 0; p += ctx->t.stride[iYCbCr]) {
				if (BitDepth == 8) {
					((uint32_t *)p)[0] = big_endian32(get_uv(&ctx->t._gb, 32));
					((uint32_t *)p)[1] = big_endian32(get_uv(&ctx->t._gb, 32));
					if (MbWidth == 16) {
						((uint32_t *)p)[2] = big_endian32(get_uv(&ctx->t._gb, 32));
						((uint32_t *)p)[3] = big_endian32(get_uv(&ctx->t._gb, 32));
					}
				} else for (int x = 0; x < MbWidth; x++) {
					((uint16_t *)p)[x] = get_uv(&ctx->t._gb, BitDepth);
				}
			}
			MbWidth = (ctx->t.ChromaArrayType < 3) ? 8 : 16;
			y = (int8_t[4]){0, 8, 16, 16}[ctx->t.ChromaArrayType];
		}
		#if CABAC
			cabac_start(ctx);
		#endif
	}
}



/**
 * This function is the entry point to residual parsing in Inter macroblocks.
 * It parses coded_block_pattern and transform_size_8x8_flag, that are parsed
 * in different orders than Intra macroblocks.
 */
static void CAFUNC(parse_inter_residual)
{
	static const uint8_t me_inter[48] = {0, 1, 32, 8, 4, 128, 2, 40, 36, 136, 132, 172, 174, 44, 168, 164, 140, 12, 160, 173, 42, 38, 138, 134, 34, 10, 6, 130, 46, 170, 166, 142, 33, 9, 5, 129, 41, 37, 137, 133, 45, 169, 165, 141, 13, 161, 14, 162};
	CACALL(parse_coded_block_pattern, me_inter);
	
	if ((mb->bits[0] & 0xac) && ctx->transform_8x8_mode_flag) {
		mb->f.transform_size_8x8_flag = CACOND(get_u1(&ctx->t._gb), get_ae(ctx, 399 + ctx->inc.transform_size_8x8_flag));
		print_slice(ctx, "  transform_size_8x8_flag: %u\n", mb->f.transform_size_8x8_flag);
	}
	
	#if CABAC
		ctx->nC_inc_v[0] = ctx->nC_inc_v[1] = ctx->nC_inc_v[2] = (i8x16){};
	#endif
	CAJUMP(parse_NxN_residual);
}



/**
 * Parse both components of a motion vector (7.3.5.1, 7.4.5.1, 9.3.2.3,
 * 9.3.3.1.1.7 and tables 9-34 and 9-39).
 * 
 * As with residual coefficients, bypass bits can be extracted all at once
 * using a binary division. The maximum mvd value is 2^15, i.e 26 bits as
 * Exp-Golomb, so we need a single division on 64-bit machines and two on
 * 32-bit machines.
 */
static i16x8 CAFUNC(parse_mvd_pair, const uint8_t *absMvd_lx, int i4x4) {
	#if !CABAC
		int x = get_se32(&ctx->t._gb, -32768, 32767);
		int y = get_se32(&ctx->t._gb, -32768, 32767);
		print_slice(ctx, "  mvd[%lu]: [%d,%d]\n", absMvd_lx - mb->absMvd + i4x4, x, y);
		return (i16x8){x, y};
	#else
		i16x8 res;
		for (int ctxBase = 40, i = 0;;) {
			int sum = absMvd_lx[ctx->absMvd_A[i4x4] + i] + absMvd_lx[ctx->absMvd_B[i4x4] + i];
			int ctxIdx = ctxBase + (sum >= 3) + (sum > 32);
			int mvd = 0;
			ctxBase += 3;
			while (mvd < 9 && get_ae(ctx, ctxIdx))
				ctxIdx = ctxBase + min(mvd++, 3);
			if (mvd >= 9) {
				// we need at least 35 (or 21) bits in codIOffset to get 26 (or 12) bypass bits
				int zeros = clz(ctx->t._gb.codIRange);
				if (zeros > (SIZE_BIT == 64 ? 64 - 35 : 32 - 21)) {
					ctx->t._gb.codIOffset = shld(get_bytes(&ctx->t._gb, zeros >> 3), ctx->t._gb.codIOffset, zeros & -8);
					ctx->t._gb.codIRange <<= zeros & -8;
					zeros &= 7;
				}
				// for 64-bit we could shift codIOffset down to 37 bits to help iterative hardware dividers, but that would make the code harder to maintain
				ctx->t._gb.codIRange >>= SIZE_BIT - 9 - zeros;
				size_t quo = ctx->t._gb.codIOffset / ctx->t._gb.codIRange; // requested bits are in lsb and zeros+9 empty bits above
				size_t rem = ctx->t._gb.codIOffset % ctx->t._gb.codIRange;
				int k = 3 + clz(~quo << (zeros + 9) | (size_t)1 << (SIZE_BIT - 12));
				int unused = SIZE_BIT - 9 - zeros - k * 2 + 1;
				#if SIZE_BIT == 32
					if (__builtin_expect(unused < 0, 0)) { // FIXME needs testing
						// refill codIOffset with 16 bits then make a new division
						ctx->t._gb.codIOffset = shld(get_bytes(&ctx->t._gb, 2), rem, 16);
						quo = shld((ctx->t._gb.codIOffset / ctx->t._gb.codIRange) << (SIZE_BIT - 16), quo, 16);
						rem = ctx->t._gb.codIOffset % ctx->t._gb.codIRange;
						unused += 16;
					}
				#endif
				mvd = 1 + (1 << k | (quo >> unused >> 1 & (((size_t)1 << k) - 1)));
				mvd = (quo & (size_t)1 << unused) ? -mvd : mvd;
				ctx->t._gb.codIOffset = (quo & (((size_t)1 << unused) - 1)) * ctx->t._gb.codIRange + rem;
				ctx->t._gb.codIRange <<= unused;
			} else if (mvd > 0) {
				mvd = get_bypass(ctx) ? -mvd : mvd;
			}
			
			if (++i == 2) {
				res[1] = mvd;
				print_slice(ctx, "  mvd[%lu]: [%d,%d]\n", absMvd_lx - mb->absMvd + i4x4, res[0], mvd);
				return res;
			}
			ctxBase = 47;
			res = (i16x8){mvd};
		}
	#endif
}



/**
 * Parse all ref_idx_lx in a given macroblock (9.3.3.1.1.6).
 * 
 * f is a bitmask for indices of symbols that should be parsed. These values
 * are then broadcast to other positions according to the inferred block
 * shapes, unless bit 8 is set (to signal Direct_8x8).
 * This function also clips all values to valid ones.
 */
static inline void CAFUNC(parse_ref_idx, unsigned f) {
	u8x16 v = {f, f, f, f, f, f, f, f, f, f, f, f, f, f, f, f};
	u8x16 bits = {1, 2, 4, 8, 16, 32, 64, 128};
	mb->refIdx_l = ((i64x2){mb->refIdx_l} & ~(i64x2)((v & bits) == bits))[0]; // set to 0 if parsed
	for (unsigned u = f & ctx->num_ref_idx_mask; u; u &= u - 1) {
		int i = __builtin_ctz(u);
		int ref_idx = 0;
		#if !CABAC
			if (ctx->clip_ref_idx[i] == 1)
				ref_idx = get_u1(&ctx->t._gb) ^ 1;
			else
				ref_idx = get_ue16(&ctx->t._gb, ctx->clip_ref_idx[i]);
		#else
			if (get_ae(ctx, 54 + (mb->bits[0] >> inc8x8[4 + i] & 3))) {
				ref_idx = 1;
				mb->bits[0] |= 1 << bit8x8[4 + i];
				int ctxIdx = 58;
				while (ref_idx < 32 && get_ae(ctx, ctxIdx))
					ref_idx++, ctxIdx = 59;
			}
		#endif
		mb->refIdx[i] = ref_idx;
	}
	
	// clip and broadcast the values
	i8x16 refIdx_v = (i64x2){mb->refIdx_l};
	#if CABAC
		refIdx_v = min8(refIdx_v, (i64x2){(int64_t)ctx->clip_ref_idx_v});
	#endif
	if (!(f & 0x122)) { // 16xN
		refIdx_v = shuffle(refIdx_v, ((i8x16){0, 0, 2, 2, 4, 4, 6, 6, -1, -1, -1, -1, -1, -1, -1, -1}));
		#if CABAC
			mb->bits[0] |= (mb->bits[0] >> 2 & 0x080800) | (mb->bits[0] << 5 & 0x808000);
		#endif
	}
	if (!(f & 0x144)) { // Nx16
		refIdx_v = shuffle(refIdx_v, ((i8x16){0, 1, 0, 1, 4, 5, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1}));
		#if CABAC
			mb->bits[0] |= (mb->bits[0] >> 3 & 0x040400) | (mb->bits[0] << 4 & 0x808000);
		#endif
	}
	mb->refIdx_l = ((i64x2)refIdx_v)[0];
	print_slice(ctx, "  ref_idx: [%d,%d,%d,%d,%d,%d,%d,%d]\n", mb->refIdx[0], mb->refIdx[1], mb->refIdx[2], mb->refIdx[3], mb->refIdx[4], mb->refIdx[5], mb->refIdx[6], mb->refIdx[7]);
	
	// compute reference picture numbers
	mb->refPic_s[0] = ((i32x4)shufflen(ctx->t.RefPicList_v[0], refIdx_v))[0];
	mb->refPic_s[1] = ((i32x4)shufflen(ctx->t.RefPicList_v[2], refIdx_v))[1];
}



/**
 * Parse sub_mb_type in a B macroblock.
 * 
 * This function is distinct from parse_B_mb to allow different inlinings.
 * For each 8x8 block we test for B_Direct_8x8, otherwise we extract a
 * bitstring and initialize the loop flags with it. Values for CABAC are:
 * _ 0 = B_Bi_8x8
 * _ 1 = B_L0_8x4
 * _ 2 = B_L0_4x8
 * _ 3 = B_L1_8x4
 * _ 4 = B_L0_8x8
 * _ 5 = B_L1_8x8
 * _ 6 = B_L1_4x4
 * _ 7 = B_Bi_4x4
 * _ 8 = B_L1_4x8
 * _ 9 = B_Bi_8x4
 * _ 10 = B_Bi_4x8
 * _ 11 = B_L0_4x4
 */
static void CAFUNC(parse_B_sub_mb) {
	
	// initializations for sub_mb_type
	print_slice(ctx, "  sub_mb_types: [");
	unsigned mvd_flags = 0;
	mb->f.inter_eqs_s = 0;
	for (int i8x8 = 0; i8x8 < 4; i8x8++) {
		int i4x4 = i8x8 * 4;
		#if !CABAC
			int sub_mb_type = get_ue16(&ctx->t._gb, 12);
			print_slice(ctx, (i8x8 < 4) ? "%u," : "%u]\n", sub_mb_type);
		#endif
		if (CACOND(sub_mb_type == 0, !get_ae(ctx, 36))) { // B_Direct_8x8
			mb->f.inter_eqs[i8x8] = 0x1b;
			if (!ctx->t.direct_8x8_inference_flag)
				ctx->transform_8x8_mode_flag = 0;
			#if CABAC
				print_slice(ctx, (i8x8 < 4) ? "0," : "0]\n");
			#endif
		} else {
			#if !CABAC
				static const uint32_t sub_mb_type2flags[13] = {0, 0x00001, 0x10000,
					0x10001, 0x00005, 0x00003, 0x50000, 0x30000, 0x50005, 0x30003,
					0x0000f, 0xf0000, 0xf000f};
				static const uint8_t sub_mb_type2eqs[13] = {0, 0x1b, 0x1b, 0x1b,
					0x11, 0x0a, 0x11, 0x0a, 0x11, 0x0a, 0, 0, 0};
				mvd_flags |= sub_mb_type2flags[sub_mb_type] << i4x4;
				mb->f.inter_eqs[i8x8] = sub_mb_type2eqs[sub_mb_type];
			#else
				int sub = 2;
				if (!get_ae(ctx, 37) || (sub = get_ae(ctx, 38),
					sub += sub + get_ae(ctx, 39),
					sub += sub + get_ae(ctx, 39), sub - 4 < 2u))
				{
					sub += sub + get_ae(ctx, 39);
				}
				static const uint32_t sub2flags[12] = {0x10001, 0x00005, 0x00003, 0x50000,
					0x00001, 0x10000, 0xf0000, 0xf000f, 0x30000, 0x50005, 0x30003, 0x0000f};
				static const uint8_t sub2eqs[12] = {0x1b, 0x11, 0x0a, 0x11, 0x1b,
					0x1b, 0, 0, 0x0a, 0x11, 0x0a, 0};
				mvd_flags |= sub2flags[sub] << i4x4;
				mb->f.inter_eqs[i8x8] = sub2eqs[sub];
				static const uint8_t sub2mb_type[13] = {3, 4, 5, 6, 1, 2, 11, 12, 7, 8, 9, 10, 0};
				print_slice(ctx, (i8x8 < 4) ? "%u," : "%u]\n", sub2mb_type[sub]);
			#endif
			if (CACOND(0x015f & 1 << sub_mb_type, 0x23b & 1 << sub)) { // 8xN
				ctx->unavail4x4[i4x4] = (ctx->unavail4x4[i4x4] & 11) | (ctx->unavail4x4[i4x4 + 1] & 4);
				ctx->unavail4x4[i4x4 + 2] |= 4;
				ctx->refIdx4x4_C[i4x4] = 0x0d63 >> i4x4 & 15;
				ctx->mvs_C[i4x4] = ctx->mvs_C[i4x4 + 1];
				if (CACOND(0x1ff0 & 1 << sub_mb_type, 0xfce & 1 << sub))
					ctx->transform_8x8_mode_flag = 0;
			} else { // 4xN
				ctx->refIdx4x4_C[i4x4] = 0xdc32 >> i4x4 & 15;
				ctx->mvs_C[i4x4] = ctx->mvs_B[i4x4 + 1];
				ctx->transform_8x8_mode_flag = 0;
			}
		}
	}
	
	// initialize direct prediction then parse all ref_idx values
	unsigned direct_flags = (~((mvd_flags & 0xffff) | mvd_flags >> 16) & 0x1111) * 0xf000f;
	if (direct_flags != 0)
		decode_direct_mv_pred(ctx, direct_flags);
	if (mvd_flags == 0) // yes there are streams that use 4 Direct8x8 instead of one Direct16x16
		CAJUMP(parse_inter_residual);
	CACALL(parse_ref_idx, 0x100 | mvd_flags2ref_idx(mvd_flags));
	
	// load neighbouring refIdx values and shuffle them into A/B/C/D
	i8x16 BC = (i64x2){(int64_t)mbB->refIdx_l, (int64_t)mbC->refIdx_l};
	i8x16 Ar = (i64x2){(int64_t)mbA->refIdx_l, (int64_t)mb->refIdx_l};
	i8x16 BCAr0 = unziplo32(BC, Ar);
	i8x16 BCAr1 = unziphi32(BC, Ar);
	i8x16 r0 = shuffle(BCAr0, ((i8x16){12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15}));
	i8x16 r1 = shuffle(BCAr1, ((i8x16){12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15}));
	i8x16 A0 = shuffle(BCAr0, ((i8x16){9, 12, 9, 12, 12, 13, 12, 13, 11, 14, 11, 14, 14, 15, 14, 15}));
	i8x16 A1 = shuffle(BCAr1, ((i8x16){9, 12, 9, 12, 12, 13, 12, 13, 11, 14, 11, 14, 14, 15, 14, 15}));
	i8x16 B0 = shuffle(BCAr0, ((i8x16){2, 2, 12, 12, 3, 3, 13, 13, 12, 12, 14, 14, 13, 13, 15, 15}));
	i8x16 B1 = shuffle(BCAr1, ((i8x16){2, 2, 12, 12, 3, 3, 13, 13, 12, 12, 14, 14, 13, 13, 15, 15}));
	i8x16 C0 = shuffle(BCAr0, ctx->refIdx4x4_C_v);
	i8x16 C1 = shuffle(BCAr1, ctx->refIdx4x4_C_v);
	i8x16 D0 = shuffle(BCAr0, ((i8x16){-1, 2, 9, 12, 2, 3, 12, 13, 9, 12, 11, 14, 12, 13, 14, 15}));
	i8x16 D1 = shuffle(BCAr1, ((i8x16){-1, 2, 9, 12, 2, 3, 12, 13, 9, 12, 11, 14, 12, 13, 14, 15}));
	D0[0] = mbD->refIdx[3];
	D1[0] = mbD->refIdx[7];
	
	// combine them into a vector of 4-bit equality masks
	union { int8_t q[32]; i8x16 v[2]; } refIdx4x4_eq;
	i8x16 u = ctx->unavail4x4_v;
	i8x16 uC = u & 4;
	refIdx4x4_eq.v[0] = (uC - ifelse_mask(uC==4, r0==D0, r0==C0) * 2 - (r0==B0)) * 2 - (r0==A0 | u==14);
	refIdx4x4_eq.v[1] = (uC - ifelse_mask(uC==4, r1==D1, r1==C1) * 2 - (r1==B1)) * 2 - (r1==A1 | u==14);
	
	// loop on mvs
	do {
		int i = __builtin_ctz(mvd_flags);
		int i4x4 = i & 15;
		mb->mvs_s[i] = 0; // value pointed to when A/B/C/D are unavailable
		uint8_t *absMvd_p = mb->absMvd + (i & 16) * 2;
		i16x8 mvd = CACALL(parse_mvd_pair, absMvd_p, i4x4);
		
		// branch on equality mask
		int32_t *mvs_p = mb->mvs_s + (i & 16);
		i16x8 mvp;
		int eq = refIdx4x4_eq.q[i];
		int mvs_DC = eq & 8 ? ctx->mvs_D[i4x4] : ctx->mvs_C[i4x4];
		if (__builtin_expect(0xe9e9 >> eq & 1, 1)) {
			i16x8 mvA = (i32x4){mvs_p[ctx->mvs_A[i4x4]]};
			i16x8 mvB = (i32x4){mvs_p[ctx->mvs_B[i4x4]]};
			i16x8 mvDC = (i32x4){mvs_p[mvs_DC]};
			mvp = median16(mvA, mvB, mvDC);
		} else {
			int mvs_AB = eq & 1 ? ctx->mvs_A[i4x4] : ctx->mvs_B[i4x4];
			mvp = (i32x4){mvs_p[eq & 4 ? mvs_DC : mvs_AB]};
		}
		
		// broadcast absMvd and mvs to memory then call decoding
		static const int8_t masks[16] = {0, 15, 10, 5, 12, 3, 0, 0, 8, 0, 0, 0, 4, 0, 2, 1};
		static const int8_t widths[16] = {0, 8, 4, 4, 8, 8, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		static const int8_t heights[16] = {0, 8, 8, 8, 4, 4, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		int type = mvd_flags >> (i & -4) & 15;
		int m = masks[type];
		int i8x8 = i >> 2;
		i16x8 bits = {1, 2, 4, 8};
		i16x8 absMvd_mask = ((i16x8){m, m, m, m, m, m, m, m} & bits) == bits;
		i16x8 absMvd_old = (i64x2){mb->absMvd_l[i8x8]};
		i16x8 mvs_mask = ziplo16(absMvd_mask, absMvd_mask);
		i32x4 mv = mvp + mvd;
		i16x8 mvs = broadcast32(mv, 0);
		mb->absMvd_l[i8x8] = ((i64x2)ifelse_mask(absMvd_mask, pack_absMvd(mvd), absMvd_old))[0];
		mb->mvs_v[i8x8] = ifelse_mask(mvs_mask, mvs, mb->mvs_v[i8x8]);
		decode_inter(ctx, i, widths[type], heights[type]);
	} while (mvd_flags &= mvd_flags - 1);
	CAJUMP(parse_inter_residual);
}



/**
 * Parse mb_skip_flag and mb_type in a B macroblock.
 * 
 * Quick tests are done for B_Skip and B_Direct_16x16, otherwise we extract a
 * bitstring and use it to branch to further parsing. Values for CABAC are:
 * _ 0 = B_Bi_16x16
 * _ 1 = B_L0_L0_16x8
 * _ 2 = B_L0_L0_8x16
 * _ 3 = B_L1_L1_16x8
 * _ 4 = B_L1_L1_8x16
 * _ 5 = B_L0_L1_16x8
 * _ 6 = B_L0_L1_8x16
 * _ 7 = B_L1_L0_16x8
 * _ 8 = B_L0_16x16
 * _ 9 = B_L1_16x16
 * _ 13 = Intra
 * _ 14 = B_L1_L0_8x16
 * _ 15 = B_8x8
 * _ 16 = B_L0_Bi_16x8
 * _ 17 = B_L0_Bi_8x16
 * _ 18 = B_L1_Bi_16x8
 * _ 19 = B_L1_Bi_8x16
 * _ 20 = B_Bi_L0_16x8
 * _ 21 = B_Bi_L0_8x16
 * _ 22 = B_Bi_L1_16x8
 * _ 23 = B_Bi_L1_8x16
 * _ 24 = B_Bi_Bi_16x8
 * _ 25 = B_Bi_Bi_8x16
 */
static inline void CAFUNC(parse_B_mb)
{
	// Inter initializations
	mb->mbIsInterFlag = 1;
	mb->Intra4x4PredMode_v = (i8x16){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
	#if CABAC
		mb->absMvd_v[0] = mb->absMvd_v[1] = mb->absMvd_v[2] = mb->absMvd_v[3] = (i8x16){};
	#endif
	mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (i16x8){};
	
	// parse mb_skip_run/flag
	#if !CABAC
		if (ctx->mb_skip_run < 0) {
			ctx->mb_skip_run = get_ue32(&ctx->t._gb, 139264);
			print_slice(ctx, "  mb_skip_run: %u\n", ctx->mb_skip_run);
		}
		int mb_skip_flag = ctx->mb_skip_run-- > 0;
	#else
		int mb_skip_flag = get_ae(ctx, 26 - ctx->inc.mb_skip_flag);
		print_slice(ctx, "  mb_skip_flag: %u\n", mb_skip_flag);
	#endif
	
	// earliest handling for B_Skip
	if (mb_skip_flag) {
		#if CABAC
			mb->f.mb_skip_flag = 1;
			mb->f.mb_type_B_Direct = 1;
			ctx->mb_qp_delta_nz = 0;
		#endif
		mb->f.inter_eqs_s = 0;
		decode_direct_mv_pred(ctx, 0xffffffff);
		return;
	}
		
	// B_Direct_16x16
	#if !CABAC
		int mb_type = get_ue16(&ctx->t._gb, 48);
	#endif
	if (CACOND(mb_type == 0, !get_ae(ctx, 29 - ctx->inc.mb_type_B_Direct))) {
		print_slice(ctx, "  mb_type: 0\n");
		#if CABAC
			mb->f.mb_type_B_Direct = 1;
		#endif
		ctx->transform_8x8_mode_flag = ctx->t.pps.transform_8x8_mode_flag & ctx->t.direct_8x8_inference_flag;
		mb->f.inter_eqs_s = 0;
		decode_direct_mv_pred(ctx, 0xffffffff);
		CAJUMP(parse_inter_residual);
	}
	ctx->transform_8x8_mode_flag = ctx->t.pps.transform_8x8_mode_flag;
	
	// initializations and jumps for mb_type
	#if !CABAC
		print_slice(ctx, "  mb_type: %u\n", mb_type);
		if (mb_type > 22)
			CAJUMP(parse_I_mb, mb_type - 23);
		mb->refIdx_l = -1;
		mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (i16x8){};
		if (mb_type == 22)
			CAJUMP(parse_B_sub_mb);
		static const uint8_t mb_type2flags[22] = {0, 0x01, 0x10, 0x11, 0x05, 0x03,
			0x50, 0x30, 0x41, 0x21, 0x14, 0x12, 0x45, 0x23, 0x54, 0x32, 0x15, 0x13,
			0x51, 0x31, 0x55, 0x33};
		int flags8x8 = mb_type2flags[mb_type];
	#else
		int str = 4;
		if (!get_ae(ctx, 30) || (str = get_ae(ctx, 31),
			str += str + get_ae(ctx, 32),
			str += str + get_ae(ctx, 32),
			str += str + get_ae(ctx, 32), str - 8 < 5u))
		{
			str += str + get_ae(ctx, 32);
		}
		if (str == 13)
			CAJUMP(parse_I_mb, 32);
		static const uint8_t str2mb_type[26] = {3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 0, 0,
			0, 0, 11, 22, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
		print_slice(ctx, "  mb_type: %u\n", str2mb_type[str]);
		mb->refIdx_l = -1;
		mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (i16x8){};
		if (str == 15)
			CAJUMP(parse_B_sub_mb);
		static const uint8_t str2flags[26] = {0x11, 0x05, 0x03, 0x50, 0x30, 0x41,
			0x21, 0x14, 0x01, 0x10, 0, 0, 0, 0, 0x12, 0, 0x45, 0x23, 0x54, 0x32,
			0x15, 0x13, 0x51, 0x31, 0x55, 0x33};
		int flags8x8 = str2flags[str];
	#endif
	
	// decoding large blocks
	CACALL(parse_ref_idx, flags8x8);
	if (!(flags8x8 & 0xee)) { // 16x16
		mb->f.inter_eqs_s = little_endian32(0x1b5fbbff);
		if (flags8x8 & 0x01) {
			i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd, 0);
			decode_inter_16x16(ctx, mvd, 0);
		}
		if (flags8x8 & 0x10) {
			i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd + 32, 0);
			decode_inter_16x16(ctx, mvd, 1);
		}
	} else if (!(flags8x8 & 0xcc)) { // 8x16
		mb->f.inter_eqs_s = little_endian32(0x1b1bbbbb);
		if (flags8x8 & 0x01) {
			i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd, 0);
			decode_inter_8x16_left(ctx, mvd, 0);
		}
		if (flags8x8 & 0x02) {
			i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd, 4);
			decode_inter_8x16_right(ctx, mvd, 0);
		}
		if (flags8x8 & 0x10) {
			i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd + 32, 0);
			decode_inter_8x16_left(ctx, mvd, 1);
		}
		if (flags8x8 & 0x20) {
			i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd + 32, 4);
			decode_inter_8x16_right(ctx, mvd, 1);
		}
	} else { // 16x8
		mb->f.inter_eqs_s = little_endian32(0x1b5f1b5f);
		if (flags8x8 & 0x01) {
			i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd, 0);
			decode_inter_16x8_top(ctx, mvd, 0);
		}
		if (flags8x8 & 0x04) {
			i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd, 8);
			decode_inter_16x8_bottom(ctx, mvd, 0);
		}
		if (flags8x8 & 0x10) {
			i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd + 32, 0);
			decode_inter_16x8_top(ctx, mvd, 1);
		}
		if (flags8x8 & 0x40) {
			i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd + 32, 8);
			decode_inter_16x8_bottom(ctx, mvd, 1);
		}
	}
	CAJUMP(parse_inter_residual);
}



/**
 * Parse sub_mb_type in a P macroblock.
 * 
 * This function is distinct from parse_P_mb to allow different inlinings.
 * For each 8x8 block we fill a bitmask for the indices at which mvs will be
 * parsed, then we loop on these bits and broadcast the values accordingly.
 */
static void CAFUNC(parse_P_sub_mb, unsigned ref_idx_flags)
{
	// initializations for sub_mb_type
	print_slice(ctx, "  sub_mb_types:");
	mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = (i16x8){}; // values pointed to when A/B/C/D are unavailable
	mb->f.inter_eqs_s = 0;
	unsigned mvd_flags = 0;
	for (int i8x8 = 0; i8x8 < 4; i8x8++) {
		int i4x4 = i8x8 * 4;
		int flags = 1;
		int eqs = 0x1b;
		#if !CABAC
			int sub_mb_type = get_ue16(&ctx->t._gb, 3);
		#endif
		if (CACOND(sub_mb_type == 0, get_ae(ctx, 21)) || // 8x8
			(ctx->transform_8x8_mode_flag = 0, flags = 5, eqs = 0x11, CACOND(sub_mb_type == 1, !get_ae(ctx, 22)))) { // 8x4
			ctx->unavail4x4[i4x4] = (ctx->unavail4x4[i4x4] & 11) | (ctx->unavail4x4[i4x4 + 1] & 4);
			ctx->unavail4x4[i4x4 + 2] |= 4;
			ctx->refIdx4x4_C[i4x4] = 0x0d63 >> i4x4 & 15;
			ctx->mvs_C[i4x4] = ctx->mvs_C[i4x4 + 1];
		} else { // 4xN
			ctx->refIdx4x4_C[i4x4] = 0xdc32 >> i4x4 & 15;
			ctx->mvs_C[i4x4] = ctx->mvs_B[i4x4 + 1];
			if (CACOND(sub_mb_type == 2, get_ae(ctx, 23)))
				flags = 3, eqs = 0x0a;
			else
				flags = 15, eqs = 0;
		}
		mvd_flags |= flags << i4x4;
		mb->f.inter_eqs[i8x8] = eqs;
		print_slice(ctx, (i8x8 < 4) ? "%u," : "%u]\n", 0xc0000480 >> flags * 2 & 3);
	}
	CACALL(parse_ref_idx, ref_idx_flags);
	
	// load neighbouring refIdx values and shuffle them into A/B/C/D
	i8x16 BC = (i64x2){(int64_t)mbB->refIdx_l, (int64_t)mbC->refIdx_l};
	i8x16 Ar = (i64x2){(int64_t)mbA->refIdx_l, (int64_t)mb->refIdx_l};
	i8x16 BCAr0 = unziplo32(BC, Ar);
	i8x16 r0 = shuffle(BCAr0, ((i8x16){12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15}));
	i8x16 A0 = shuffle(BCAr0, ((i8x16){9, 12, 9, 12, 12, 13, 12, 13, 11, 14, 11, 14, 14, 15, 14, 15}));
	i8x16 B0 = shuffle(BCAr0, ((i8x16){2, 2, 12, 12, 3, 3, 13, 13, 12, 12, 14, 14, 13, 13, 15, 15}));
	i8x16 C0 = shuffle(BCAr0, ctx->refIdx4x4_C_v);
	i8x16 D0 = shuffle(BCAr0, ((i8x16){-1, 2, 9, 12, 2, 3, 12, 13, 9, 12, 11, 14, 12, 13, 14, 15}));
	D0[0] = mbD->refIdx[3];
	
	// combine them into a vector of 4-bit equality masks
	union { int8_t q[16]; i8x16 v; } refIdx4x4_eq;
	i8x16 u = ctx->unavail4x4_v;
	i8x16 uC = u & 4;
	refIdx4x4_eq.v = (uC - ifelse_mask(uC==4, r0==D0, r0==C0) * 2 - (r0==B0)) * 2 - (r0==A0 | u==14);
	
	// loop on mvs
	do {
		int i = __builtin_ctz(mvd_flags);
		i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd, i);
		
		// branch on equality mask
		i16x8 mvp;
		int eq = refIdx4x4_eq.q[i];
		int mvs_DC = eq & 8 ? ctx->mvs_D[i] : ctx->mvs_C[i];
		if (__builtin_expect(0xe9e9 >> eq & 1, 1)) {
			i16x8 mvA = (i32x4){mb->mvs_s[ctx->mvs_A[i]]};
			i16x8 mvB = (i32x4){mb->mvs_s[ctx->mvs_B[i]]};
			i16x8 mvDC = (i32x4){mb->mvs_s[mvs_DC]};
			mvp = median16(mvA, mvB, mvDC);
		} else {
			int mvs_AB = eq & 1 ? ctx->mvs_A[i] : ctx->mvs_B[i];
			mvp = (i32x4){mb->mvs_s[(eq & 4 ? mvs_DC : mvs_AB)]};
		}
		
		// broadcast absMvd and mvs to memory then call decoding
		static const int8_t masks[16] = {0, 15, 10, 5, 12, 3, 0, 0, 8, 0, 0, 0, 4, 0, 2, 1};
		static const int8_t widths[16] = {0, 8, 4, 4, 8, 8, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		static const int8_t heights[16] = {0, 8, 8, 8, 4, 4, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		int type = mvd_flags >> (i & -4) & 15;
		int m = masks[type];
		int i8x8 = i >> 2;
		i16x8 bits = {1, 2, 4, 8};
		i16x8 absMvd_mask = ((i16x8){m, m, m, m, m, m, m, m} & bits) == bits;
		i16x8 absMvd_old = (i64x2){mb->absMvd_l[i8x8]};
		i16x8 mvs_mask = ziplo16(absMvd_mask, absMvd_mask);
		i32x4 mv = mvp + mvd;
		i16x8 mvs = broadcast32(mv, 0);
		mb->absMvd_l[i8x8] = ((i64x2)ifelse_mask(absMvd_mask, pack_absMvd(mvd), absMvd_old))[0];
		mb->mvs_v[i8x8] = ifelse_mask(mvs_mask, mvs, mb->mvs_v[i8x8]);
		decode_inter(ctx, i, widths[type], heights[type]);
	} while (mvd_flags &= mvd_flags - 1);
	CAJUMP(parse_inter_residual);
}



/**
 * Parse mb_skip_flag and mb_type in a P macroblock.
 * 
 * Motion vector prediction is one of the hardest parts to decode (8.4.1.3),
 * here is a summary of the rules:
 * _ A/B/C/D are 4x4 blocks at relative pixel positions (-1,0)/(0,-1)/(W,-1)/(-1,-1)
 * _ if C is unavailable, take its values from D instead
 * _ any further unavailable block counts as refIdx=-1 and mv=(0,0)
 * _ parse refIdx for the current block
 * _ for 8x16 or 16x8, compare it with A(left)/C(right) or B(top)/A(bottom),
 *   if it matches take the mv from the same neighbour
 * _ otherwise if B and C are unavailable, take their values from A instead
 * _ then if only one of A/B/C equals refIdx, take mv from this neighbour
 * _ otherwise predict mv as median(mvA, mvB, mvC)
 * 
 * In general we implement these rules in two steps:
 * _ compare refIdx with A/B/C and produce a 3 bit equality mask (plus a bit
 *   for C->D replacement), which can be computed in parallel for all 4x4
 *   blocks since all refIdx values are parsed before all mvs
 * _ then for each block in sequence, fetch the proper neighbouring mv(s) and
 *   combine them depending on the mask
 */
static inline void CAFUNC(parse_P_mb)
{
	// Inter initializations
	mb->mbIsInterFlag = 1;
	mb->Intra4x4PredMode_v = (i8x16){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
	mb->refIdx_l = (int64_t)(i8x8){0, 0, 0, 0, -1, -1, -1, -1};
	#if CABAC
		mb->absMvd_v[0] = mb->absMvd_v[1] = (i8x16){};
	#endif
	
	// parse mb_skip_run/flag
	#if !CABAC
		if (ctx->mb_skip_run < 0) {
			ctx->mb_skip_run = get_ue32(&ctx->t._gb, 139264);
			print_slice(ctx, "  mb_skip_run: %u\n", ctx->mb_skip_run);
		}
		int mb_skip_flag = ctx->mb_skip_run-- > 0;
	#else
		int mb_skip_flag = get_ae(ctx, 13 - ctx->inc.mb_skip_flag);
		print_slice(ctx, "  mb_skip_flag: %u\n", mb_skip_flag);
	#endif
	
	// earliest handling for P_Skip
	if (mb_skip_flag) {
		#if CABAC
			mb->f.mb_skip_flag = 1;
			ctx->mb_qp_delta_nz = 0;
		#endif
		decode_P_skip(ctx);
		return;
	}
	ctx->transform_8x8_mode_flag = ctx->t.pps.transform_8x8_mode_flag;
	
	// initializations and jumps for mb_type
	#if !CABAC
		int mb_type = get_ue16(&ctx->t._gb, 30);
		print_slice(ctx, "  mb_type: %u\n", mb_type);
		if (mb_type > 4)
			CAJUMP(parse_I_mb, mb_type - 5);
		mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (i16x8){};
		if (mb_type > 2)
			CAJUMP(parse_P_sub_mb, (mb_type + 12) & 15); // 3->15, 4->0
		CACALL(parse_ref_idx, 0x351 >> (mb_type << 2) & 15); // 0->1, 1->5, 2->3
	#else
		if (get_ae(ctx, 14)) { // Intra
			CAJUMP(parse_I_mb, 17);
		}
		int mb_type = get_ae(ctx, 15); // actually 1 and 3 are swapped
		mb_type += mb_type + get_ae(ctx, 16 + mb_type);
		print_slice(ctx, "  mb_type: %u\n", (4 - mb_type) & 3);
		mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (i16x8){};
		if (mb_type == 1)
			CAJUMP(parse_P_sub_mb, 15);
		CACALL(parse_ref_idx, (mb_type + 1) | 1); // 0->1, 2->3, 3->5
	#endif
	
	// decoding large blocks
	if (mb_type == 0) { // 16x16
		mb->f.inter_eqs_s = little_endian32(0x1b5fbbff);
		i16x8 mvd = CACALL(parse_mvd_pair, mb->absMvd, 0);
		decode_inter_16x16(ctx, mvd, 0);
	} else if (mb_type == 2) { // 8x16
		mb->f.inter_eqs_s = little_endian32(0x1b1bbbbb);
		i16x8 mvd0 = CACALL(parse_mvd_pair, mb->absMvd, 0);
		decode_inter_8x16_left(ctx, mvd0, 0);
		i16x8 mvd1 = CACALL(parse_mvd_pair, mb->absMvd, 4);
		decode_inter_8x16_right(ctx, mvd1, 0);
	} else { // 16x8
		mb->f.inter_eqs_s = little_endian32(0x1b5f1b5f);
		i16x8 mvd0 = CACALL(parse_mvd_pair, mb->absMvd, 0);
		decode_inter_16x8_top(ctx, mvd0, 0);
		i16x8 mvd1 = CACALL(parse_mvd_pair, mb->absMvd, 8);
		decode_inter_16x8_bottom(ctx, mvd1, 0);
	}
	CAJUMP(parse_inter_residual);
}



/**
 * This function loops through the macroblocks of a slice, initialising their
 * data and calling parse_{I/P/B}_mb for each one.
 */
static void CAFUNC(parse_slice_data)
{
	static const i8x16 block_unavailability[16] = {
		{ 0,  0,  0,  4,  0,  0,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{ 1,  0,  9,  4,  0,  0,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
		{ 6, 14,  0,  4, 14, 10,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{ 7, 14,  9,  4, 14, 10,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
		{ 0,  0,  0,  4,  0,  4,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{ 1,  0,  9,  4,  0,  4,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
		{ 6, 14,  0,  4, 14, 14,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{ 7, 14,  9,  4, 14, 14,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
		{ 8,  0,  0,  4,  0,  0,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{ 9,  0,  9,  4,  0,  0,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
		{14, 14,  0,  4, 14, 10,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{15, 14,  9,  4, 14, 10,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
		{ 8,  0,  0,  4,  0,  4,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{ 9,  0,  9,  4,  0,  4,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
		{14, 14,  0,  4, 14, 14,  0,  4,  0,  0,  0,  4,  0,  4,  0,  4},
		{15, 14,  9,  4, 14, 14,  0,  4,  9,  0,  9,  4,  0,  4,  0,  4},
	};
	
	print_slice(ctx, "--- # POC %u\n", ctx->PicOrderCnt);
	int end_of_slice_flag = 0;
	do {
		print_slice(ctx, "- mbAddr: %u\n", ctx->CurrMbAddr);
		
		// update flip_bit atomically to signal mb is a priori decoded, otherwise end the slice
		int prev_recovery_bits = __atomic_exchange_n(&mb->recovery_bits, ctx->t.frame_flip_bit, __ATOMIC_ACQ_REL);
		if (prev_recovery_bits == ctx->t.frame_flip_bit)
			return;
		
		// set and reset neighbouring pointers depending on their availability
		int unavail16x16 = (ctx->mbx == 0 ? 9 : 0) | (ctx->mbx == ctx->t.pic_width_in_mbs - 1) << 2 | (ctx->mby == 0 ? 14 : 0);
		int filter_edges = ~unavail16x16;
		mbA = mb - 1;
		mbB = mbA - ctx->t.pic_width_in_mbs;
		mbC = mbB + 1;
		mbD = mbB - 1;
		i8x16 fA = mbA->f.v;
		i8x16 fB = mbB->f.v;
		uint64_t bitsA = mbA->bits_l;
		uint64_t bitsB = mbB->bits_l;
		int decoded = ctx->CurrMbAddr - ctx->t.first_mb_in_slice;
		if (decoded <= ctx->t.pic_width_in_mbs + 1) {
			if (decoded == 1) { // A becomes available
				ctx->A4x4_int8[0] = 5 - (int)sizeof(*mb);
				ctx->A4x4_int8[2] = 7 - (int)sizeof(*mb);
				ctx->A4x4_int8[8] = 13 - (int)sizeof(*mb);
				ctx->A4x4_int8[10] = 15 - (int)sizeof(*mb);
				ctx->absMvd_A[0] = 10 - (int)sizeof(*mb);
				ctx->absMvd_A[2] = 14 - (int)sizeof(*mb);
				ctx->absMvd_A[8] = 26 - (int)sizeof(*mb);
				ctx->absMvd_A[10] = 30 - (int)sizeof(*mb);
				ctx->mvs_A[0] = 5 - (int)(sizeof(*mb) >> 2);
				ctx->mvs_A[2] = 7 - (int)(sizeof(*mb) >> 2);
				ctx->mvs_A[8] = 13 - (int)(sizeof(*mb) >> 2);
				ctx->mvs_A[10] = 15 - (int)(sizeof(*mb) >> 2);
				ctx->mvs_D[2] = 5 - (int)(sizeof(*mb) >> 2);
				ctx->mvs_D[8] = 7 - (int)(sizeof(*mb) >> 2);
				ctx->mvs_D[10] = 13 - (int)(sizeof(*mb) >> 2);
				if (ctx->t.ChromaArrayType == 1) {
					ctx->ACbCr_int8[0] = 1 - (int)sizeof(*mb);
					ctx->ACbCr_int8[2] = 3 - (int)sizeof(*mb);
					ctx->ACbCr_int8[4] = 5 - (int)sizeof(*mb);
					ctx->ACbCr_int8[6] = 7 - (int)sizeof(*mb);
				}
			} else if (decoded == 0) { // A is unavailable
				mbA = &unavail_mb;
				unavail16x16 |= 1;
				fA = unavail_mb.f.v;
				bitsA = unavail_mb.bits_l;
				filter_edges &= ~(ctx->t.disable_deblocking_filter_idc >> 1); // impacts only bit 0
			}
			if (decoded == ctx->t.pic_width_in_mbs + 1) { // D becomes available
				int offD_int32 = ((ctx->t.pic_width_in_mbs + 2) * (int)sizeof(*mb)) >> 2;
				ctx->mvs_D[0] = 15 - offD_int32;
			} else { // D is unavailable
				mbD = &unavail_mb;
				unavail16x16 |= 8;
				if (decoded == ctx->t.pic_width_in_mbs) { // B becomes available
					int offB_int8 = (ctx->t.pic_width_in_mbs + 1) * (int)sizeof(*mb);
					int offB_int32 = offB_int8 >> 2;
					ctx->B4x4_int8[0] = 10 - offB_int8;
					ctx->B4x4_int8[1] = 11 - offB_int8;
					ctx->B4x4_int8[4] = 14 - offB_int8;
					ctx->B4x4_int8[5] = 15 - offB_int8;
					ctx->absMvd_B[0] = 20 - offB_int8;
					ctx->absMvd_B[1] = 22 - offB_int8;
					ctx->absMvd_B[4] = 28 - offB_int8;
					ctx->absMvd_B[5] = 30 - offB_int8;
					ctx->mvs_B[0] = 10 - offB_int32;
					ctx->mvs_B[1] = 11 - offB_int32;
					ctx->mvs_B[4] = 14 - offB_int32;
					ctx->mvs_B[5] = 15 - offB_int32;
					ctx->mvs_C[0] = 11 - offB_int32;
					ctx->mvs_C[1] = 14 - offB_int32;
					ctx->mvs_C[4] = 15 - offB_int32;
					ctx->mvs_D[1] = 10 - offB_int32;
					ctx->mvs_D[4] = 11 - offB_int32;
					ctx->mvs_D[5] = 14 - offB_int32;
					if (ctx->t.ChromaArrayType == 1) {
						ctx->BCbCr_int8[0] = 2 - offB_int8;
						ctx->BCbCr_int8[1] = 3 - offB_int8;
						ctx->BCbCr_int8[4] = 6 - offB_int8;
						ctx->BCbCr_int8[5] = 7 - offB_int8;
					}
				} else { // B is unavailable
					mbB = &unavail_mb;
					unavail16x16 |= 2;
					fB = unavail_mb.f.v;
					bitsB = unavail_mb.bits_l;
					filter_edges &= ~ctx->t.disable_deblocking_filter_idc; // impacts only bit 1
					if (decoded == ctx->t.pic_width_in_mbs - 1) { // C becomes available
						int offC_int32 = (ctx->t.pic_width_in_mbs * (int)sizeof(*mb)) >> 2;
						ctx->mvs_C[5] = 10 - offC_int32;
					} else { // C is unavailable
						mbC = &unavail_mb;
						unavail16x16 |= 4;
					}
				}
			}
		}
		
		// initialize common macroblock values
		ctx->unavail4x4_v = block_unavailability[unavail16x16];
		ctx->inc.v = fA + fB + (fB & flags_twice.v);
		mb->f.v = (i8x16){};
		if (ctx->t.disable_deblocking_filter_idc != 1)
			mb->filter_edges = filter_edges;
		mb->QP_s = ctx->t.QP_s;
		if (ctx->t.ChromaArrayType == 1) { // FIXME 4:2:2
			mb->bits_l = (bitsA >> 3 & 0x11111100111111) | (bitsB >> 1 & 0x42424200424242);
		}
		mb->nC_v[0] = mb->nC_v[1] = mb->nC_v[2] = (i8x16){};
		
		// Would it actually help to push this test outside the loop?
		if (ctx->t.slice_type == 0) {
			CACALL(parse_P_mb);
		} else if (ctx->t.slice_type == 1) {
			CACALL(parse_B_mb);
		} else {
			int mb_type_or_ctxIdx = CACOND(get_ue16(&ctx->t._gb, 25), 5 - ctx->inc.mb_type_I_NxN);
			#if !CABAC
				print_slice(ctx, "  mb_type: %u\n", mb_type_or_ctxIdx);
			#endif
			CACALL(parse_I_mb, mb_type_or_ctxIdx);
		}
		#if CABAC
			end_of_slice_flag = cabac_terminate(ctx);
			print_slice(ctx, "  end_of_slice_flag: %u\n", end_of_slice_flag);
		#endif
		
		// deblock mbB while in cache, then point to the next macroblock
		if (ctx->CurrMbAddr - ctx->t.pic_width_in_mbs == ctx->t.next_deblock_addr) {
			ctx->t.next_deblock_addr += 1;
			mb -= ctx->t.pic_width_in_mbs + 1;
			ctx->samples_mb[0] -= ctx->t.stride[0] * 16;
			ctx->samples_mb[1] -= ctx->t.stride[1] * 8;
			ctx->samples_mb[2] -= ctx->t.stride[1] * 8;
			deblock_mb(ctx);
			mb += ctx->t.pic_width_in_mbs + 2;
			ctx->samples_mb[0] += ctx->t.stride[0] * 16 + 16;
			ctx->samples_mb[1] += ctx->t.stride[1] * 8 + 8;
			ctx->samples_mb[2] += ctx->t.stride[1] * 8 + 8;
		} else {
			mb++;
			ctx->samples_mb[0] += 16; // FIXME 16bit
			ctx->samples_mb[1] += 8; // FIXME 4:2:2, 16bit
			ctx->samples_mb[2] += 8;
		}
		ctx->mbx++;
		ctx->CurrMbAddr++;
		ctx->mbCol++;
		if (ctx->mbx >= ctx->t.pic_width_in_mbs) {
			mb++; // skip the empty macroblock at the edge
			ctx->mbCol++;
			ctx->mby++;
			ctx->mbx = 0;
			ctx->samples_mb[0] += ctx->t.stride[0] * 16 - ctx->t.pic_width_in_mbs * 16;
			ctx->samples_mb[1] += ctx->t.stride[1] * 8 - ctx->t.pic_width_in_mbs * 8; // FIXME 4:2:2
			ctx->samples_mb[2] += ctx->t.stride[1] * 8 - ctx->t.pic_width_in_mbs * 8;
			if (ctx->t.next_deblock_idc >= 0) {
				__atomic_store_n(&ctx->d->next_deblock_addr[ctx->t.next_deblock_idc],
					(ctx->t.disable_deblocking_filter_idc != 1) ? ctx->t.next_deblock_addr : ctx->CurrMbAddr,
					__ATOMIC_RELEASE);
				// not locking mutex here is fine since the last progress broadcast will lock it
				pthread_cond_broadcast(&ctx->d->task_progress);
			}
			if (ctx->mby >= ctx->t.pic_height_in_mbs)
				return;
		}
	} while (CACOND(ctx->mb_skip_run > 0 || ctx->t._gb.msb_cache != (size_t)1 << (SIZE_BIT - 1) || (ctx->t._gb.lsb_cache & (ctx->t._gb.lsb_cache - 1)) || (intptr_t)(ctx->t._gb.end - ctx->t._gb.CPB) > 0, !end_of_slice_flag));
}

#undef CABAC
