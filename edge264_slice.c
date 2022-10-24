#include "edge264_common.h"

#undef CAFUNC
#undef CACALL
#undef CAJUMP
#undef CACOND
#ifndef CABAC
	#define CAFUNC(f, ...) FUNC(f ## _cavlc, ## __VA_ARGS__)
	#define CACALL(f, ...) CALL(f ## _cavlc, ## __VA_ARGS__)
	#define CAJUMP(f, ...) JUMP(f ## _cavlc, ## __VA_ARGS__)
	#define CACOND(cavlc, cabac) cavlc
#else
	#define CAFUNC(f, ...) FUNC(f ## _cabac, ## __VA_ARGS__)
	#define CACALL(f, ...) CALL(f ## _cabac, ## __VA_ARGS__)
	#define CAJUMP(f, ...) JUMP(f ## _cabac, ## __VA_ARGS__)
	#define CACOND(cavlc, cabac) cabac
#endif



/**
 * Parse the coeff_token header for a non-ChromaDCLevel residual block encoded
 * with CAVLC (9.2.1)
 *
 * While both CAVLC and residual coefficients are not too critical to optimize,
 * this function is designed to be simple and compact.
 */
#ifndef CABAC
	static int FUNC(parse_coeff_token_cavlc, int i4x4, int nA, int nB) {
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
		
		// unavailable blocks have the value 32
		int sum = nA + nB;
		int nC = !(ctx->unavail4x4[i4x4] & 3) ? (sum + 1) >> 1 : sum;
		int coeff_token, v;
		if (__builtin_expect(nC < 8, 1)) {
			int leadingZeroBits = clz(msb_cache | (size_t)1 << (SIZE_BIT - 15));
			unsigned fourBits = msb_cache >> (SIZE_BIT - 4 - leadingZeroBits);
			int token = tokens[nC_offset[nC] + leadingZeroBits * 8 + fourBits - 8];
			coeff_token = token & 127;
			v = token >> 7;
		} else {
			coeff_token = (msb_cache >> (SIZE_BIT - 6)) + 4;
			if (coeff_token == 7)
				coeff_token = 0;
			v = 6;
		}
		msb_cache = lsd(msb_cache, lsb_cache, v);
		if (lsb_cache <<= v)
			return coeff_token;
		return CALL(refill, coeff_token);
	}
	
	// 4:2:0 is best handled separately due to the open-ended 0000000 code and 3 bit suffixes
	static int FUNC(parse_DC2x2_coeff_token_cavlc) {
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
		int leadingZeroBits = clz(msb_cache | (size_t)1 << (SIZE_BIT - 8));
		unsigned suffix = msb_cache >> (SIZE_BIT - 3 - leadingZeroBits) & 3;
		int token = tokens[leadingZeroBits * 4 + suffix];
		int coeff_token = token & 127;
		int v = token >> 7;
		msb_cache = lsd(msb_cache, lsb_cache, v);
		if (lsb_cache <<= v)
			return coeff_token;
		return CALL(refill, coeff_token);
	}
#endif



/**
 * Temporary functions for two alternatives to parse total_zeros.
 */
#ifndef CABAC
static inline int FUNC(parse_total_zeros_alt, int endIdx, int TotalCoeff) {
	static const uint8_t codes[] = {
		// 2x2 blocks
		51, 50, 33, 33, 16, 16, 16, 16,
		34, 33, 16, 16,
		17, 16,
		// 2x4 blocks
		87, 86, 69, 69, 67, 67, 68, 68, 49, 49, 49, 49, 50, 50, 50, 50, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
		48, 50, 33, 33, 51, 52, 53, 54,
		48, 49, 34, 34, 35, 35, 52, 53,
		33, 33, 34, 34, 35, 35, 48, 52,
		32, 33, 34, 35,
		32, 33, 18, 18,
		16, 17,
		// 4x4 blocks
		143, 159, 158, 157, 140, 140, 139, 139, 122, 122, 122, 122, 121, 121, 121, 121, 104, 104, 104, 104, 104, 104, 104, 104, 103, 103, 103, 103, 103, 103, 103, 103, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 86, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 68, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
		110, 109, 108, 107, 90, 90, 89, 89, 72, 72, 72, 72, 71, 71, 71, 71, 70, 70, 70, 70, 69, 69, 69, 69, 52, 52, 52, 52, 52, 52, 52, 52, 51, 51, 51, 51, 51, 51, 51, 51, 50, 50, 50, 50, 50, 50, 50, 50, 49, 49, 49, 49, 49, 49, 49, 49, 48, 48, 48, 48, 48, 48, 48, 48,
		109, 107, 92, 92, 90, 90, 89, 89, 72, 72, 72, 72, 69, 69, 69, 69, 68, 68, 68, 68, 64, 64, 64, 64, 55, 55, 55, 55, 55, 55, 55, 55, 54, 54, 54, 54, 54, 54, 54, 54, 51, 51, 51, 51, 51, 51, 51, 51, 50, 50, 50, 50, 50, 50, 50, 50, 49, 49, 49, 49, 49, 49, 49, 49,
		92, 91, 90, 80, 73, 73, 71, 71, 67, 67, 66, 66, 56, 56, 56, 56, 54, 54, 54, 54, 53, 53, 53, 53, 52, 52, 52, 52, 49, 49, 49, 49,
		91, 89, 74, 74, 72, 72, 66, 66, 65, 65, 64, 64, 55, 55, 55, 55, 54, 54, 54, 54, 53, 53, 53, 53, 52, 52, 52, 52, 51, 51, 51, 51,
		106, 96, 81, 81, 72, 72, 72, 72, 57, 57, 57, 57, 57, 57, 57, 57, 55, 55, 55, 55, 55, 55, 55, 55, 54, 54, 54, 54, 54, 54, 54, 54, 53, 53, 53, 53, 53, 53, 53, 53, 52, 52, 52, 52, 52, 52, 52, 52, 51, 51, 51, 51, 51, 51, 51, 51, 50, 50, 50, 50, 50, 50, 50, 50,
		105, 96, 81, 81, 71, 71, 71, 71, 56, 56, 56, 56, 56, 56, 56, 56, 54, 54, 54, 54, 54, 54, 54, 54, 52, 52, 52, 52, 52, 52, 52, 52, 51, 51, 51, 51, 51, 51, 51, 51, 50, 50, 50, 50, 50, 50, 50, 50, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37,
		104, 96, 82, 82, 65, 65, 65, 65, 55, 55, 55, 55, 55, 55, 55, 55, 54, 54, 54, 54, 54, 54, 54, 54, 51, 51, 51, 51, 51, 51, 51, 51, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36,
		97, 96, 87, 87, 66, 66, 66, 66, 53, 53, 53, 53, 53, 53, 53, 53, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35,
		81, 80, 70, 70, 50, 50, 50, 50, 37, 37, 37, 37, 37, 37, 37, 37, 36, 36, 36, 36, 36, 36, 36, 36, 35, 35, 35, 35, 35, 35, 35, 35,
		64, 65, 50, 50, 51, 51, 53, 53, 20, 20, 20, 20, 20, 20, 20, 20,
		64, 65, 52, 52, 34, 34, 34, 34, 19, 19, 19, 19, 19, 19, 19, 19,
		48, 49, 35, 35, 18, 18, 18, 18,
		32, 33, 18, 18,
		16, 17,
	};
	static const int16_t columns[27] = {2, 129, 192, 0, 244, 754, 882, 1010, 1137, 1201, 1264, 0, 1320, 9509, 10533, 11556, 12068, 12581, 13605, 14629, 15653, 16676, 17187, 17443, 17698, 17825, 17888};
	
	int column = columns[endIdx + TotalCoeff - 4];
	int idx = msb_cache >> ((column & 15) ^ (SIZE_BIT - 1));
	int code = codes[(column >> 4) + idx];
	int v = code >> 4;
	msb_cache = lsd(msb_cache, lsb_cache, v);
	lsb_cache <<= v;
	return code & 15;
}

static inline int FUNC(parse_total_zeros, int endIdx, int TotalCoeff) {
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
	int leadingZeroBits = clz(msb_cache | (size_t)1 << (SIZE_BIT - 9));
	int suffix = msb_cache >> ((leadingZeroBits + 2) ^ (SIZE_BIT - 1)) & 3;
	int code = codes[endIdx + TotalCoeff - 4][leadingZeroBits * 4 + suffix];
	int v = code >> 4;
	msb_cache = lsd(msb_cache, lsb_cache, v);
	lsb_cache <<= v;
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
	#ifndef CABAC
		int TrailingOnes = token_or_cbf & 3;
		int TotalCoeff = token_or_cbf >> 2;
		
		// parse all level values from end to start
		int32_t level[16];
		size_t signs = ~msb_cache;
		level[0] = (signs >> (SIZE_BIT - 2) & 2) - 1;
		level[1] = (signs >> (SIZE_BIT - 3) & 2) - 1;
		level[2] = (signs >> (SIZE_BIT - 4) & 2) - 1;
		msb_cache = lsd(msb_cache, lsb_cache, TrailingOnes);
		lsb_cache <<= TrailingOnes;
		for (int i = TrailingOnes, suffixLength = 1, v, offset; i < TotalCoeff; i++) {
			int level_prefix = clz(msb_cache | (size_t)1 << (SIZE_BIT - 26)); // limit given in 9.2.2.1
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
				msb_cache = lsd(msb_cache, lsb_cache, level_prefix);
				if (!(lsb_cache <<= level_prefix))
					CALL(refill, 0);
			#endif
			int levelCode = CALL(get_uv, v) + offset;
			levelCode += (i == TrailingOnes && TrailingOnes < 3) * 2;
			level[i] = (levelCode % 2) ? (-levelCode - 1) >> 1 : (levelCode + 2) >> 1;
			suffixLength = min(suffixLength + (levelCode >= (3 << suffixLength)), 6);
		}
		
		// store level values at proper positions in memory
		int zerosLeft = 0;
		if (TotalCoeff <= endIdx - startIdx)
			zerosLeft = CALL(parse_total_zeros, endIdx, TotalCoeff);
		int8_t *scan = ctx->scan + startIdx + zerosLeft + TotalCoeff - 1;
		ctx->c[*scan] = level[0];
		for (int i = 1, v, run_before; i < TotalCoeff; i++) {
			scan--;
			if (zerosLeft > 0) {
				int threeBits = msb_cache >> (SIZE_BIT - 3);
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
					v = clz(msb_cache) + 1;
					run_before = min(v + 3, zerosLeft);
				}
				scan -= run_before;
				zerosLeft -= run_before;
				msb_cache = lsd(msb_cache, lsb_cache, v);
				lsb_cache <<= v;
			}
			ctx->c[*scan] = level[i];
		}
		
		// trailing_ones_sign_flags+total_zeros+run_before consumed at most 31 bits, so we can delay refill here
		if (!lsb_cache)
			CALL(refill, 0);
	#else // CABAC
		// significant_coeff_flags are stored as a bit mask
		uint64_t significant_coeff_flags = 0;
		int i = startIdx;
		do {
			if (CALL(get_ae, ctx->ctxIdxOffsets[1] + ctx->sig_inc[i])) {
				significant_coeff_flags |= (uint64_t)1 << i;
				if (CALL(get_ae, ctx->ctxIdxOffsets[2] + ctx->last_inc[i]))
					break;
			}
		} while (++i < endIdx);
		significant_coeff_flags |= (uint64_t)1 << i;
		
		// Now loop on set bits to parse all non-zero coefficients.
		int ctxIdx0 = ctx->ctxIdxOffsets[3] + 1;
		int ctxIdx1 = ctx->ctxIdxOffsets[3] + 5;
		do {
			int coeff_level = 1;
			int ctxIdx = ctxIdx0;
			while (coeff_level < 15 && CALL(get_ae, ctxIdx))
				coeff_level++, ctxIdx = ctxIdx1;
			if (coeff_level >= 15) {
				#if SIZE_BIT == 32
					// the biggest value to encode is 2^(14+7)-14, for which k=20 (see 9.3.2.3)
					int k = 0;
					while (CALL(get_bypass) && k < 20)
						k++;
					coeff_level = 1;
					while (k--)
						coeff_level += coeff_level + CALL(get_bypass);
					coeff_level += 14;
				#elif SIZE_BIT == 64
					// we need at least 50 bits in codIOffset to get 41 bits with a division by 9 bits
					int zeros = clz(codIRange);
					if (zeros > 64 - 50) {
						codIOffset = lsd(codIOffset, CALL(get_bytes, zeros >> 3), zeros & -8);
						codIRange <<= zeros & -8;
						zeros = clz(codIRange);
					}
					codIRange >>= 64 - 9 - zeros;
					size_t quo = codIOffset / codIRange; // requested bits are in lsb and zeros+9 empty bits above
					size_t rem = codIOffset % codIRange;
					int k = min(clz(~quo << (zeros + 9)), 20);
					int unused = 64 - 9 - zeros - k * 2 - 1;
					coeff_level = 14 + (1 << k | (quo >> unused & (((size_t)1 << k) - 1)));
					codIOffset = (quo & (((size_t)1 << unused) - 1)) * codIRange + rem;
					codIRange <<= unused;
				#endif
			}
			
			// not the brightest part of spec (9.3.3.1.3), I did my best
			static const int8_t trans[5] = {0, 2, 3, 4, 4};
			int last_sig_offset = ctx->ctxIdxOffsets[3];
			int ctxIdxInc = trans[ctxIdx0 - last_sig_offset];
			ctxIdx0 = last_sig_offset + (coeff_level > 1 ? 0 : ctxIdxInc);
			ctxIdx1 = min(ctxIdx1 + (coeff_level > 1), (last_sig_offset == 257 ? last_sig_offset + 8 : last_sig_offset + 9));
		
			// scale and store
			int c = CALL(get_bypass) ? -coeff_level : coeff_level;
			int i = 63 - clz64(significant_coeff_flags);
			ctx->c[ctx->scan[i]] = c; // beware, scan is transposed already
			significant_coeff_flags &= ~((uint64_t)1 << i);
		} while (significant_coeff_flags != 0);
	#endif
	for (int i = startIdx; i <= endIdx; i++)
		fprintf(stderr, " %d", ctx->c[ctx->scan[i]]);
	fprintf(stderr, "\n");
}



/**
 * As its name says, parses mb_qp_delta (9.3.2.7 and 9.3.3.1.1.5).
 */
static void CAFUNC(parse_mb_qp_delta)
{
	#ifndef CABAC
		int mb_qp_delta = CALL(get_se16, -26, 25); // FIXME QpBdOffset
		if (mb_qp_delta) {
			int sum = ctx->QP[0] + mb_qp_delta;
			int QP_Y = (sum < 0) ? sum + 52 : (sum >= 52) ? sum - 52 : sum;
			mb->QP_s = ctx->QP_s = (v4qi){QP_Y, ctx->QP_C[0][QP_Y], ctx->QP_C[1][QP_Y]};
		}
	#else
		int mb_qp_delta_nz = CALL(get_ae, 60 + ctx->mb_qp_delta_nz);
		ctx->mb_qp_delta_nz = mb_qp_delta_nz;
		int mb_qp_delta = 0;
		if (mb_qp_delta_nz) {
			unsigned count = 1, ctxIdx = 62;
			while (CALL(get_ae, ctxIdx) && count < 52) // FIXME QpBdOffset
				count++, ctxIdx = 63;
			mb_qp_delta = count & 1 ? count / 2 + 1 : -(count / 2);
			int sum = ctx->QP[0] + mb_qp_delta;
			int QP_Y = (sum < 0) ? sum + 52 : (sum >= 52) ? sum - 52 : sum;
			mb->QP_s = ctx->QP_s = (v4qi){QP_Y, ctx->QP_C[0][QP_Y], ctx->QP_C[1][QP_Y]};
		}
	#endif
	fprintf(stderr, "mb_qp_delta: %d\n", mb_qp_delta);
}



/**
 * Parsing for chroma 4:2:0 and 4:2:2 is put in a separate function to be
 * tail-called from parse_NxN_residual and parse_Intra16x16_residual.
 */
static void CAFUNC(parse_chroma_residual)
{
	// As in Intra16x16, DC blocks are parsed to ctx->c[0..15], then transformed to ctx->c[16..31]
	if (mb->f.CodedBlockPatternChromaDC) { // valid also for 4:0:0
		#ifdef CABAC
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_chromaDC[0]; // FIXME 4:2:2
			ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_chromaDC[0];
		#endif
		ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (v4si){};
		int token_or_cbf_Cb = CACOND(
			CALL(parse_DC2x2_coeff_token_cavlc),
			CALL(get_ae, ctx->ctxIdxOffsets[0] + ctx->inc.coded_block_flags_16x16[1]));
		if (token_or_cbf_Cb) {
			#ifdef CABAC
				mb->f.coded_block_flags_16x16[1] = 1;
			#endif
			fprintf(stderr, "Cb DC coeffLevels:");
			ctx->scan_s = (v4qi){0, 4, 2, 6};
			CACALL(parse_residual_block, 0, 3, token_or_cbf_Cb);
		}
		int token_or_cbf_Cr = CACOND(
			CALL(parse_DC2x2_coeff_token_cavlc),
			CALL(get_ae, ctx->ctxIdxOffsets[0] + ctx->inc.coded_block_flags_16x16[2]));
		if (token_or_cbf_Cr) {
			#ifdef CABAC
				mb->f.coded_block_flags_16x16[2] = 1;
			#endif
			fprintf(stderr, "Cr DC coeffLevels:");
			ctx->scan_s = (v4qi){1, 5, 3, 7};
			CACALL(parse_residual_block, 0, 3, token_or_cbf_Cr);
		}
		CALL(transform_dc2x2);
		
		// Eight or sixteen 4x4 AC blocks for the Cb/Cr components
		if (mb->f.CodedBlockPatternChromaAC) {
			#ifdef CABAC
				ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
				ctx->ctxIdxOffsets_l = ctxIdxOffsets_chromaAC[0];
			#endif
			ctx->scan_v[0] = scan_4x4[0];
			for (int i4x4 = 0; i4x4 < 8; i4x4++) {
				int iYCbCr = 1 + (i4x4 >> 2);
				uint8_t *samples = ctx->samples_mb[iYCbCr] + y420[i4x4] * ctx->stride[1] + x420[i4x4];
				int nA = *(mb->nC[1] + ctx->ACbCr_int8[i4x4]);
				int nB = *(mb->nC[1] + ctx->BCbCr_int8[i4x4]);
				int token_or_cbf = CACOND(CALL(parse_coeff_token_cavlc, i4x4 << 2 & 15, nA, nB),
					CALL(get_ae, ctx->ctxIdxOffsets[0] + nA + nB * 2));
				if (token_or_cbf) {
					mb->nC[1][i4x4] = CACOND(token_or_cbf >> 2, 1);
					ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (v4si){};
					fprintf(stderr, "Chroma AC coeffLevels[%d]:", i4x4);
					CACALL(parse_residual_block, 1, 15, token_or_cbf);
					v16qu wS = ((v16qu *)ctx->ps.weightScale4x4)[iYCbCr + mb->f.mbIsInterFlag * 3];
					CALL(add_idct4x4, iYCbCr, ctx->QP[iYCbCr], wS, i4x4, samples);
				} else {
					CALL(add_dc4x4, iYCbCr, i4x4, samples);
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
	#ifdef CABAC
		ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
	#endif
	for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
		
		// Parse a DC block, then transform it to ctx->c[16..31]
		#ifdef CABAC
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_16x16DC[iYCbCr][0];
		#endif
		int token_or_cbf = CACOND(
			CALL(parse_coeff_token_cavlc, 0, mb[-1].nC[iYCbCr][5], ctx->mbB->nC[iYCbCr][10]),
			CALL(get_ae, ctx->ctxIdxOffsets[0] + ctx->inc.coded_block_flags_16x16[iYCbCr]));
		if (token_or_cbf) {
			#ifdef CABAC
				mb->f.coded_block_flags_16x16[iYCbCr] = 1;
			#endif
			ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (v4si){};
			fprintf(stderr, "16x16 DC coeffLevels[%d]:", iYCbCr);
			CACALL(parse_residual_block, 0, 15, token_or_cbf);
			CALL(transform_dc4x4, iYCbCr);
		} else {
			if (mb->bits[0] & 1 << 5)
				ctx->c_v[4] = ctx->c_v[5] = ctx->c_v[6] = ctx->c_v[7] = (v4si){};
		}
		
		// All AC blocks pick a DC coeff, then go to ctx->c[1..15]
		if (mb->bits[0] & 1 << 5) {
			#ifdef CABAC
				ctx->ctxIdxOffsets_l = ctxIdxOffsets_16x16AC[iYCbCr][0];
			#endif
			for (int i4x4 = 0; i4x4 < 16; i4x4++) {
				uint8_t *samples = ctx->samples_mb[iYCbCr] + y444[i4x4] * ctx->stride[iYCbCr] + x444[i4x4];
				int nA = *(mb->nC[iYCbCr] + ctx->A4x4_int8[i4x4]);
				int nB = *(mb->nC[iYCbCr] + ctx->B4x4_int8[i4x4]);
				int token_or_cbf = CACOND(CALL(parse_coeff_token_cavlc, i4x4, nA, nB),
					CALL(get_ae, ctx->ctxIdxOffsets[0] + nA + nB * 2));
				if (token_or_cbf) {
					mb->nC[iYCbCr][i4x4] = CACOND(token_or_cbf >> 2, 1);
					ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (v4si){};
					fprintf(stderr, "16x16 AC coeffLevels[%d]:", iYCbCr * 16 + i4x4);
					CACALL(parse_residual_block, 1, 15, token_or_cbf);
					CALL(add_idct4x4, iYCbCr, ctx->QP[0], ((v16qu *)ctx->ps.weightScale4x4)[iYCbCr], i4x4, samples);
				} else {
					CALL(add_dc4x4, iYCbCr, i4x4, samples);
				}
			}
		}
		
		// here is how we share the decoding of luma coefficients with 4:4:4 modes
		if (ctx->ps.ChromaArrayType <3)
			CAJUMP(parse_chroma_residual);
	}
}



/**
 * This block is dedicated to the parsing of Intra_NxN and Inter_NxN residual
 * blocks, since they share much in common.
 */
static void CAFUNC(parse_NxN_residual)
{
	static const int8_t intra4x4_modes[9][16] = {
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
	
	if (mb->f.CodedBlockPatternChromaDC | (mb->bits[0] & 0xac))
		CACALL(parse_mb_qp_delta);
	#ifdef CABAC
		else
			ctx->mb_qp_delta_nz = 0;
	#endif
	
	// next few blocks will share many parameters, so we cache them
	for (int iYCbCr = 0; iYCbCr < 3; iYCbCr++) {
		if (!mb->f.transform_size_8x8_flag) {
			#ifdef CABAC
				ctx->ctxIdxOffsets_l = ctxIdxOffsets_4x4[iYCbCr][0];
				ctx->sig_inc_v[0] = ctx->last_inc_v[0] = sig_inc_4x4;
			#endif
			ctx->scan_v[0] = scan_4x4[0];
			
			// Decoding directly follows parsing to avoid duplicate loops.
			for (int i4x4 = 0; i4x4 < 16; i4x4++) {
				size_t stride = ctx->stride[iYCbCr];
				uint8_t *samples = ctx->samples_mb[iYCbCr] + y444[i4x4] * stride + x444[i4x4];
				if (!mb->f.mbIsInterFlag)
					CALL(decode_intra4x4, intra4x4_modes[mb->Intra4x4PredMode[i4x4]][ctx->unavail4x4[i4x4]], samples, stride, ctx->clip[iYCbCr]);
				if (mb->bits[0] & 1 << bit8x8[i4x4 >> 2]) {
					int nA = *(mb->nC[iYCbCr] + ctx->A4x4_int8[i4x4]);
					int nB = *(mb->nC[iYCbCr] + ctx->B4x4_int8[i4x4]);
					int token_or_cbf = CACOND(CALL(parse_coeff_token_cavlc, i4x4, nA, nB),
						CALL(get_ae, ctx->ctxIdxOffsets[0] + nA + nB * 2));
					if (token_or_cbf) {
						mb->nC[iYCbCr][i4x4] = CACOND(token_or_cbf >> 2, 1);
						ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (v4si){};
						fprintf(stderr, "4x4 coeffLevels[%d]:", iYCbCr * 16 + i4x4);
						CACALL(parse_residual_block, 0, 15, token_or_cbf);
						
						// DC blocks are marginal here (about 16%) so we do not handle them separately
						v16qu wS = ((v16qu *)ctx->ps.weightScale4x4)[iYCbCr + mb->f.mbIsInterFlag * 3];
						CALL(add_idct4x4, iYCbCr, ctx->QP[0], wS, -1, samples);
					}
				}
			}
		} else {
			
			ctx->ctxIdxOffsets_l = ctxIdxOffsets_8x8[iYCbCr][0];
			ctx->sig_inc_v[0] = sig_inc_8x8[0][0];
			ctx->last_inc_v[0] = last_inc_8x8[0];
			ctx->scan_v[0] = scan_8x8[0][0];
			for (int i8x8 = 0; i8x8 < 4; i8x8++) {
				int BlkIdx = iYCbCr * 4 + i8x8;
				int coded_block_flag = mb->bits[0] & 1 << bit8x8[i8x8];
				//if (coded_block_flag && ctx->ps.ChromaArrayType == 3) {
				//	int cbfA = *(mb->coded_block_flags_8x8 + ctx->coded_block_flags_8x8_A[BlkIdx]);
				//	int cbfB = *(mb->coded_block_flags_8x8 + ctx->coded_block_flags_8x8_B[BlkIdx]);
				//	coded_block_flag = CALL(get_ae, ctx->ctxIdxOffsets[0] + cbfA + cbfB * 2);
				//}
				//mb->coded_block_flags_8x8[BlkIdx] = coded_block_flag;
				//mb->coded_block_flags_4x4_s[BlkIdx] = coded_block_flag ? 0x01010101 : 0;
				memset(ctx->c, 0, 256); // FIXME
				//CALL(parse_residual_block_cabac, coded_block_flag, 0, 63);
			}
		}
		
		// nice optimisation for 4:4:4 modes
		if (ctx->ps.ChromaArrayType <3)
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
	#ifndef CABAC
		int cbp = map_me[CALL(get_ue16, 47)];
		mb->bits[0] |= cbp & 0xac;
	#else
		int bits = mb->bits[0];
		bits |= CALL(get_ae, 76 - (bits      & 3)) << 5;
		bits |= CALL(get_ae, 76 - (bits >> 5 & 3)) << 3;
		bits |= CALL(get_ae, 76 - (bits >> 4 & 3)) << 2;
		bits |= CALL(get_ae, 76 - (bits >> 2 & 3)) << 7;
		mb->bits[0] = bits;
	#endif
	
	// Chroma suffix
	if ((ctx->ps.ChromaArrayType == 1 || ctx->ps.ChromaArrayType == 2) &&
		CACOND(cbp & 3, CALL(get_ae, 77 + ctx->inc.CodedBlockPatternChromaDC)))
	{
		mb->f.CodedBlockPatternChromaDC = 1;
		mb->f.CodedBlockPatternChromaAC = CACOND(cbp >> 1 & 1, CALL(get_ae, 81 + ctx->inc.CodedBlockPatternChromaAC));
	}
	
	fprintf(stderr, "coded_block_pattern: %u\n",
		(mb->bits[0] >> 5 & 1) | (mb->bits[0] >> 2 & 2) | (mb->bits[0] & 4) | (mb->bits[0] >> 4 & 8) |
		(mb->f.CodedBlockPatternChromaDC + mb->f.CodedBlockPatternChromaAC) << 4);
}



/**
 * Parses intra_chroma_pred_mode (9.3.2.2 and 9.3.3.1.1.8).
 */
static void CAFUNC(parse_intra_chroma_pred_mode)
{
	static const int8_t intraChroma_modes[4][4] = {
		{IC8x8_DC_8, IC8x8_DCA_8, IC8x8_DCB_8, IC8x8_DCAB_8},
		{IC8x8_H_8 , IC8x8_DCA_8, IC8x8_H_8  , IC8x8_DCAB_8},
		{IC8x8_V_8 , IC8x8_V_8  , IC8x8_DCB_8, IC8x8_DCAB_8},
		{IC8x8_P_8 , IC8x8_DCA_8, IC8x8_DCB_8, IC8x8_DCAB_8},
	};
	
	// Do not optimise too hard to keep the code understandable here.
	int type = ctx->ps.ChromaArrayType;
	if (type == 1 || type == 2) {
		#ifndef CABAC
			int mode = CALL(get_ue16, 3);
		#else
			int ctxIdx = 64 + ctx->inc.intra_chroma_pred_mode_non_zero;
			int mode = 0;
			while (mode <3 && CALL(get_ae, ctxIdx))
				mode++, ctxIdx = 67;
			mb->f.intra_chroma_pred_mode_non_zero = (mode > 0);
		#endif
		fprintf(stderr, "intra_chroma_pred_mode: %u\n", mode);
		CALL(decode_intraChroma, intraChroma_modes[mode][ctx->unavail16x16 & 3], ctx->samples_mb[1], ctx->samples_mb[2], ctx->stride[1], ctx->clip[1]);
	}
}



/**
 * Parses prev_intraNxN_pred_mode_flag and rem_intraNxN_pred_mode, and returns
 * the given intra_pred_mode (7.3.5.1, 7.4.5.1, 8.3.1.1 and table 9-34).
 */
static int CAFUNC(parse_intraNxN_pred_mode, int luma4x4BlkIdx)
{
	// dcPredModePredictedFlag is enforced by putting -2
	int intraMxMPredModeA = *(mb->Intra4x4PredMode + ctx->A4x4_int8[luma4x4BlkIdx]);
	int intraMxMPredModeB = *(mb->Intra4x4PredMode + ctx->B4x4_int8[luma4x4BlkIdx]);
	int mode = abs(min(intraMxMPredModeA, intraMxMPredModeB));
	int rem_intra_pred_mode = -1;
	if (CACOND(!CALL(get_u1), !CALL(get_ae, 68))) {
		#ifndef CABAC
			rem_intra_pred_mode = CALL(get_uv, 3);
		#else
			rem_intra_pred_mode = CALL(get_ae, 69);
			rem_intra_pred_mode += CALL(get_ae, 69) * 2;
			rem_intra_pred_mode += CALL(get_ae, 69) * 4;
		#endif
		mode = rem_intra_pred_mode + (rem_intra_pred_mode >= mode);
	}
	fprintf(stderr, " %d", rem_intra_pred_mode);
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
	static const Edge264_flags flags_PCM = {
		.CodedBlockPatternChromaDC = 1,
		.CodedBlockPatternChromaAC = 1,
		.coded_block_flags_16x16 = {1, 1, 1},
	};
	static const uint8_t me_intra[48] = {174, 173, 172, 0, 45, 169, 165, 141, 44, 168, 164, 140, 46, 170, 166, 142, 1, 40, 36, 136, 132, 41, 37, 137, 133, 42, 38, 138, 134, 32, 8, 4, 128, 33, 9, 5, 129, 12, 160, 13, 161, 2, 34, 10, 6, 130, 14, 162};
	
	// Intra-specific initialisations
	if (ctx->unavail16x16 & 1) {
		mb->bits[1] |= 0x111111; // FIXME 4:2:2
		#ifdef CABAC
			mb[-1].nC_v[0] = mb[-1].nC_v[1] = mb[-1].nC_v[2] = (v16qi){1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
		#endif
		ctx->inc.coded_block_flags_16x16_s |= 0x010101;
	}
	if (ctx->unavail16x16 & 2) {
		mb->bits[1] |= 0x424242;
		#ifdef CABAC
			ctx->mbB->nC_v[0] = ctx->mbB->nC_v[1] = ctx->mbB->nC_v[2] = (v16qi){1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
		#endif
		ctx->inc.coded_block_flags_16x16_s |= 0x020202;
	}
	mb->f.mbIsInterFlag = 0;
	mb->inter_eqs_s = little_endian32(0x1b5fbbff);
	mb->refIdx_l = -1;
	mb->refPic_l = -1;
	mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (v8hi){};
	
	// I_NxN
	if (CACOND(mb_type_or_ctxIdx == 0, !CALL(get_ae, mb_type_or_ctxIdx))) {
		#ifdef CABAC
			mb->f.mb_type_I_NxN = 1;
			fprintf(stderr, (mb_type_or_ctxIdx == 17) ? "mb_type: 5\n" : // in P slice
								 (mb_type_or_ctxIdx == 32) ? "mb_type: 23\n" : // in B slice
																	  "mb_type: 0\n"); // in I slice
		#endif
		
		// 7.3.5, 7.4.5, 9.3.3.1.1.10 and table 9-34
		int transform_size_8x8_flag = 0;
		if (ctx->ps.transform_8x8_mode_flag) {
			transform_size_8x8_flag = CACOND(CALL(get_u1), CALL(get_ae, 399 + ctx->inc.transform_size_8x8_flag));
			fprintf(stderr, "transform_size_8x8_flag: %x\n", transform_size_8x8_flag);
		}
		
		fprintf(stderr, "rem_intra_pred_modes:");
		if (transform_size_8x8_flag) {
			mb->f.transform_size_8x8_flag = transform_size_8x8_flag;
			for (int i = 0; i < 16; i += 4)
				mb->Intra4x4PredMode[i + 1] = mb->Intra4x4PredMode[i + 2] = mb->Intra4x4PredMode[i + 3] = CACALL(parse_intraNxN_pred_mode, i);
		} else {
			for (int i = 0; i < 16; i++)
				mb->Intra4x4PredMode[i] = CACALL(parse_intraNxN_pred_mode, i);
		}
		fprintf(stderr, "\n");
		
		CACALL(parse_intra_chroma_pred_mode);
		CACALL(parse_coded_block_pattern, me_intra);
		CAJUMP(parse_NxN_residual);
	
	// Intra_16x16
	} else if (CACOND(mb_type_or_ctxIdx < 25, !CALL(cabac_terminate))) {
		#ifndef CABAC
			int mb_type = mb_type_or_ctxIdx - 1;
			mb->bits[0] = mb_type > 11 ? 0xac : 0; // zeroes ref_idx_nz as byproduct
			mb_type = mb_type > 11 ? mb_type - 12 : mb_type;
			mb->f.CodedBlockPatternChromaDC = mb_type > 3;
			mb->f.CodedBlockPatternChromaAC = mb_type >> 3;
			int mode = mb_type & 3;
		#else
			int ctxIdx = max(mb_type_or_ctxIdx, 5);
			mb->bits[0] = CALL(get_ae, ctxIdx + 1) ? 0xac : 0; // zeroes ref_idx_nz as byproduct
			int CodedBlockPatternChromaDC = CALL(get_ae, ctxIdx + 2);
			ctxIdx = max(ctxIdx, 6);
			if (CodedBlockPatternChromaDC) {
				mb->f.CodedBlockPatternChromaDC = 1;
				mb->f.CodedBlockPatternChromaAC = CALL(get_ae, ctxIdx + 2);
			}
			int mode = CALL(get_ae, ctxIdx + 3) << 1;
			mode += CALL(get_ae, max(ctxIdx + 3, 10));
			fprintf(stderr, "mb_type: %u\n", (mb->bits[0] & 12) +
				(mb->f.CodedBlockPatternChromaDC + mb->f.CodedBlockPatternChromaAC) * 4 +
				mode + (ctxIdx == 17 ? 6 : ctxIdx == 32 ? 24 : 1));
		#endif
		
		// decode the samples before parsing residuals
		static const int8_t intra16x16_modes[4][4] = {
			{I16x16_V_8 , I16x16_V_8  , I16x16_DCB_8, I16x16_DCAB_8},
			{I16x16_H_8 , I16x16_DCA_8, I16x16_H_8  , I16x16_DCAB_8},
			{I16x16_DC_8, I16x16_DCA_8, I16x16_DCB_8, I16x16_DCAB_8},
			{I16x16_P_8 , I16x16_DCA_8, I16x16_DCB_8, I16x16_DCAB_8},
		};
		mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
		CALL(decode_intra16x16, intra16x16_modes[mode][ctx->unavail16x16 & 3], ctx->samples_mb[0], ctx->stride[0], ctx->clip[0]); // FIXME 4:4:4
		CACALL(parse_intra_chroma_pred_mode);
		CAJUMP(parse_Intra16x16_residual);
		
	// I_PCM
	} else {
		#ifndef CABAC
			unsigned bits = (SIZE_BIT - 1 - ctz(lsb_cache)) & 7;
			msb_cache = lsd(msb_cache, lsb_cache, bits);
			lsb_cache = lsb_cache << bits;
		#else
			fprintf(stderr, (mb_type_or_ctxIdx == 17) ? "mb_type: 30\n" : (mb_type_or_ctxIdx == 32) ? "mb_type: 48\n" : "mb_type: 25\n");
		#endif
		
		ctx->mb_qp_delta_nz = 0;
		mb->f.v |= flags_PCM.v; // FIXME reuse flags_twice
		mb->QP_s = (v4qi){0, ctx->QP_C[0][0], ctx->QP_C[1][0]};
		mb->bits_l = (uint64_t)(v2su){0xac, 0xacacac}; // FIXME 4:2:2
		mb->nC_v[0] = mb->nC_v[1] = mb->nC_v[2] = CACOND(
			((v16qi){16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16}),
			((v16qi){1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}));
		mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
		
		// PCM is so rare that it should be compact rather than fast
		uint8_t *p = ctx->samples_mb[0];
		for (int y = 16; y-- > 0; p += ctx->stride[0]) {
			for (int x = 0; x < 16; x++) {
				if (ctx->ps.BitDepth_Y == 8)
					p[x] = CALL(get_uv, 8);
				else
					((uint16_t *)p)[x] = CALL(get_uv, ctx->ps.BitDepth_Y);
			}
		}
		p = ctx->samples_mb[1];
		int MbWidthC = (ctx->ps.ChromaArrayType < 3) ? 8 : 16;
		static int8_t MbHeightC[4] = {0, 8, 16, 16};
		for (int y = MbHeightC[ctx->ps.ChromaArrayType]; y-- > 0; p += ctx->stride[1]) {
			for (int x = 0; x < MbWidthC; x++) {
				if (ctx->ps.BitDepth_Y == 8)
					p[x] = CALL(get_uv, 8);
				else
					((uint16_t *)p)[x] = CALL(get_uv, ctx->ps.BitDepth_Y);
			}
		}
		p = ctx->samples_mb[2];
		for (int y = MbHeightC[ctx->ps.ChromaArrayType]; y-- > 0; p += ctx->stride[1]) {
			for (int x = 0; x < MbWidthC; x++) {
				if (ctx->ps.BitDepth_Y == 8)
					p[x] = CALL(get_uv, 8);
				else
					((uint16_t *)p)[x] = CALL(get_uv, ctx->ps.BitDepth_Y);
			}
		}
		#ifdef CABAC
			CALL(cabac_start);
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
		mb->f.transform_size_8x8_flag = CACOND(CALL(get_u1), CALL(get_ae, 399 + ctx->inc.transform_size_8x8_flag));
		fprintf(stderr, "transform_size_8x8_flag: %x\n", mb->f.transform_size_8x8_flag);
	}
	
	if (ctx->unavail16x16 & 1) {
		#ifdef CABAC
			mb[-1].nC_v[0] = mb[-1].nC_v[1] = mb[-1].nC_v[2] = (v16qi){};
		#endif
		ctx->inc.coded_block_flags_16x16_s &= 0x020202;
	}
	if (ctx->unavail16x16 & 2) {
		#ifdef CABAC
			ctx->mbB->nC_v[0] = ctx->mbB->nC_v[1] = ctx->mbB->nC_v[2] = (v16qi){};
		#endif
		ctx->inc.coded_block_flags_16x16_s &= 0x010101;
	}
	CAJUMP(parse_NxN_residual);
}



/**
 * Parse both components of a motion vector (7.3.5.1, 7.4.5.1, 9.3.2.3,
 * 9.3.3.1.1.7 and tables 9-34 and 9-39).
 * 
 * As with residual coefficients, bypass bits can be extracted all at once
 * using a binary division. mvd expects at most 2^15-9, i.e 28 bits as
 * Exp-Golomb, so we need a single division on 64-bit machines and two on
 * 32-bit machines.
 */
static v8hi CAFUNC(parse_mvd_pair, const uint8_t *absMvd_lx, int i4x4) {
	#ifndef CABAC
		int x = CALL(get_se32, -32768, 32767);
		int y = CALL(get_se32, -32768, 32767);
		fprintf(stderr, "mvd[%lu]: %d,%d\n", absMvd_lx - mb->absMvd + i4x4, x, y);
		return (v8hi){x, y};
	#else
		v8hi res;
		for (int ctxBase = 40, i = 0;;) {
			int sum = absMvd_lx[ctx->absMvd_A[i4x4] + i] + absMvd_lx[ctx->absMvd_B[i4x4] + i];
			int ctxIdx = ctxBase + (sum >= 3) + (sum > 32);
			int mvd = 0;
			ctxBase += 3;
			while (mvd < 9 && CALL(get_ae, ctxIdx))
				ctxIdx = ctxBase + min(mvd++, 3);
			if (mvd >= 9) {
				// we need at least 37 (or 22) bits in codIOffset to get 28 (or 13) bypass bits
				int zeros = clz(codIRange);
				if (zeros > (SIZE_BIT == 64 ? 64 - 37 : 32 - 22)) {
					codIOffset = lsd(codIOffset, CALL(get_bytes, zeros >> 3), zeros & -8);
					codIRange <<= zeros & -8;
					zeros = clz(codIRange);
				}
				// for 64-bit we could shift codIOffset down to 37 bits to help iterative hardware dividers, but this code isn't critical enough
				codIRange >>= SIZE_BIT - 9 - zeros;
				size_t quo = codIOffset / codIRange; // requested bits are in lsb and zeros+9 empty bits above
				size_t rem = codIOffset % codIRange;
				int k = 3 + min(clz(~quo << (zeros + 9)), 12);
				int unused = SIZE_BIT - 9 - zeros - k * 2 + 2;
				#if SIZE_BIT == 32
					if (__builtin_expect(unused < 0, 0)) { // FIXME needs testing
						// refill codIOffset with 16 bits then make a new division
						codIOffset = lsd(rem, CALL(get_bytes, 2), 16);
						quo = lsd(quo, (codIOffset / codIRange) << (SIZE_BIT - 16), 16);
						rem = codIOffset % codIRange;
						unused += 16;
					}
				#endif
				mvd = 1 + (1 << k | (quo >> unused & (((size_t)1 << k) - 1)));
				codIOffset = (quo & (((size_t)1 << unused) - 1)) * codIRange + rem;
				codIRange <<= unused;
			}
			
			// Parse the sign flag.
			if (mvd > 0)
				mvd = CALL(get_bypass) ? -mvd : mvd;
			// fprintf(stderr, "mvd_l%x: %d\n", pos >> 1 & 1, mvd);
			
			if (++i == 2) {
				res[1] = mvd;
				fprintf(stderr, "mvd[%lu]: %d,%d\n", absMvd_lx - mb->absMvd + i4x4, res[0], mvd);
				return res;
			}
			ctxBase = 47;
			res = (v8hi){mvd};
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
	v16qu v = {f, f, f, f, f, f, f, f, f, f, f, f, f, f, f, f};
	v16qu bits = {1, 2, 4, 8, 16, 32, 64, 128};
	mb->refIdx_l = ((v2li){mb->refIdx_l} & ~(v2li)((v & bits) == bits))[0]; // set to 0 if parsed
	for (unsigned u = f & ctx->num_ref_idx_mask; u; u &= u - 1) {
		int i = __builtin_ctz(u);
		int ref_idx = 0;
		#ifndef CABAC
			if (ctx->clip_ref_idx[i] == 1)
				ref_idx = CALL(get_u1) ^ 1;
			else
				ref_idx = CALL(get_ue16, ctx->clip_ref_idx[i]);
		#else
			if (CALL(get_ae, 54 + (mb->bits[0] >> inc8x8[4 + i] & 3))) {
				ref_idx = 1;
				mb->bits[0] |= 1 << bit8x8[4 + i];
				int ctxIdx = 58;
				while (ref_idx < 32 && CALL(get_ae, ctxIdx))
					ref_idx++, ctxIdx = 59;
			}
		#endif
		mb->refIdx[i] = ref_idx;
	}
	
	// clip and broadcast the values
	v16qi refIdx_v = (v16qi)(v2li){mb->refIdx_l};
	#ifdef CABAC
		refIdx_v = min_v16qi(refIdx_v, (v16qi)(v2li){(int64_t)ctx->clip_ref_idx_v});
	#endif
	if (!(f & 0x122)) { // 16xN
		refIdx_v = shuffle(refIdx_v, (v16qi){0, 0, 2, 2, 4, 4, 6, 6, -1, -1, -1, -1, -1, -1, -1, -1});
		#ifdef CABAC
			mb->bits[0] |= (mb->bits[0] >> 2 & 0x080800) | (mb->bits[0] << 5 & 0x808000);
		#endif
	}
	if (!(f & 0x144)) { // Nx16
		refIdx_v = shuffle(refIdx_v, (v16qi){0, 1, 0, 1, 4, 5, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1});
		#ifdef CABAC
			mb->bits[0] |= (mb->bits[0] >> 3 & 0x040400) | (mb->bits[0] << 4 & 0x808000);
		#endif
	}
	mb->refIdx_l = ((v2li)refIdx_v)[0];
	fprintf(stderr, "ref_idx: %d %d %d %d %d %d %d %d\n", mb->refIdx[0], mb->refIdx[1], mb->refIdx[2], mb->refIdx[3], mb->refIdx[4], mb->refIdx[5], mb->refIdx[6], mb->refIdx[7]);
	
	// compute reference picture numbers
	mb->refPic_s[0] = ((v4si)ifelse_msb(refIdx_v, refIdx_v, shuffle(ctx->RefPicList_v[0], refIdx_v)))[0];
	mb->refPic_s[1] = ((v4si)ifelse_msb(refIdx_v, refIdx_v, shuffle(ctx->RefPicList_v[2], refIdx_v)))[1];
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
	fprintf(stderr, "sub_mb_types:");
	unsigned mvd_flags = 0;
	mb->inter_eqs_s = 0;
	for (int i8x8 = 0; i8x8 < 4; i8x8++) {
		int i4x4 = i8x8 * 4;
		#ifndef CABAC
			int sub_mb_type = CALL(get_ue16, 12);
			fprintf(stderr, " %u", sub_mb_type);
		#endif
		if (CACOND(sub_mb_type == 0, !CALL(get_ae, 36))) { // B_Direct_8x8
			mb->inter_eqs[i8x8] = 0x1b;
			#ifdef CABAC
				fprintf(stderr, " 0");
			#endif
		} else {
			#ifndef CABAC
				static const uint32_t sub_mb_type2flags[13] = {0, 0x00001, 0x10000,
					0x10001, 0x00005, 0x00003, 0x50000, 0x30000, 0x50005, 0x30003,
					0x0000f, 0xf0000, 0xf000f};
				static const uint8_t sub_mb_type2eqs[13] = {0, 0x1b, 0x1b, 0x1b,
					0x11, 0x0a, 0x11, 0x0a, 0x11, 0x0a, 0, 0, 0};
				mvd_flags |= sub_mb_type2flags[sub_mb_type] << i4x4;
				mb->inter_eqs[i8x8] = sub_mb_type2eqs[sub_mb_type];
			#else
				int sub = 2;
				if (!CALL(get_ae, 37) || (sub = CALL(get_ae, 38),
					sub += sub + CALL(get_ae, 39),
					sub += sub + CALL(get_ae, 39), sub - 4 < 2u))
				{
					sub += sub + CALL(get_ae, 39);
				}
				static const uint32_t sub2flags[12] = {0x10001, 0x00005, 0x00003, 0x50000,
					0x00001, 0x10000, 0xf0000, 0xf000f, 0x30000, 0x50005, 0x30003, 0x0000f};
				static const uint8_t sub2eqs[12] = {0x1b, 0x11, 0x0a, 0x11, 0x1b,
					0x1b, 0, 0, 0x0a, 0x11, 0x0a, 0};
				mvd_flags |= sub2flags[sub] << i4x4;
				mb->inter_eqs[i8x8] = sub2eqs[sub];
				static const uint8_t sub2mb_type[13] = {3, 4, 5, 6, 1, 2, 11, 12, 7, 8, 9, 10, 0};
				fprintf(stderr, " %u", sub2mb_type[sub]);
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
	fprintf(stderr, "\n");
	
	// initialize direct prediction then parse all ref_idx values
	unsigned direct_flags = (~((mvd_flags & 0xffff) | mvd_flags >> 16) & 0x1111) * 0xf000f;
	if (direct_flags != 0)
		CALL(decode_direct_mv_pred, direct_flags);
	if (mvd_flags == 0) // yes there are streams that use 4 Direct8x8 instead of one Direct16x16
		CAJUMP(parse_inter_residual);
	CACALL(parse_ref_idx, 0x100 | mvd_flags2ref_idx(mvd_flags));
	
	// load neighbouring refIdx values and shuffle them into A/B/C/D
	Edge264_macroblock *mbB = ctx->mbB;
	v16qi BC = (v16qi)(v2li){(int64_t)mbB->refIdx_l, (int64_t)mbB[1].refIdx_l};
	v16qi Ar = (v16qi)(v2li){(int64_t)mb[-1].refIdx_l, (int64_t)mb->refIdx_l};
	v16qi BCAr0 = (v16qi)__builtin_shufflevector((v4si)BC, (v4si)Ar, 0, 2, 4, 6);
	v16qi BCAr1 = (v16qi)__builtin_shufflevector((v4si)BC, (v4si)Ar, 1, 3, 5, 7);
	v16qi r0 = __builtin_shufflevector(BCAr0, BCAr0, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15);
	v16qi r1 = __builtin_shufflevector(BCAr1, BCAr1, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15);
	v16qi A0 = __builtin_shufflevector(BCAr0, BCAr0, 9, 12, 9, 12, 12, 13, 12, 13, 11, 14, 11, 14, 14, 15, 14, 15);
	v16qi A1 = __builtin_shufflevector(BCAr1, BCAr1, 9, 12, 9, 12, 12, 13, 12, 13, 11, 14, 11, 14, 14, 15, 14, 15);
	v16qi B0 = __builtin_shufflevector(BCAr0, BCAr0, 2, 2, 12, 12, 3, 3, 13, 13, 12, 12, 14, 14, 13, 13, 15, 15);
	v16qi B1 = __builtin_shufflevector(BCAr1, BCAr1, 2, 2, 12, 12, 3, 3, 13, 13, 12, 12, 14, 14, 13, 13, 15, 15);
	v16qi C0 = shuffle(BCAr0, ctx->refIdx4x4_C_v);
	v16qi C1 = shuffle(BCAr1, ctx->refIdx4x4_C_v);
	v16qi D0 = __builtin_shufflevector(BCAr0, BCAr0, -1, 2, 9, 12, 2, 3, 12, 13, 9, 12, 11, 14, 12, 13, 14, 15);
	v16qi D1 = __builtin_shufflevector(BCAr1, BCAr1, -1, 2, 9, 12, 2, 3, 12, 13, 9, 12, 11, 14, 12, 13, 14, 15);
	D0[0] = mbB->refIdx[3];
	D1[0] = mbB->refIdx[7];
	
	// combine them into a vector of 4-bit equality masks
	v16qi u = ctx->unavail4x4_v;
	v16qi uC = u & 4;
	ctx->refIdx4x4_eq_v[0] = (uC - ifelse_mask(uC==4, r0==D0, r0==C0) * 2 - (r0==B0)) * 2 - (r0==A0 | u==14);
	ctx->refIdx4x4_eq_v[1] = (uC - ifelse_mask(uC==4, r1==D1, r1==C1) * 2 - (r1==B1)) * 2 - (r1==A1 | u==14);
	
	// loop on mvs
	do {
		int i = __builtin_ctz(mvd_flags);
		int i4x4 = i & 15;
		uint8_t *absMvd_p = mb->absMvd + (i & 16) * 2;
		v8hi mvd = CACALL(parse_mvd_pair, absMvd_p, i4x4);
		
		// branch on equality mask
		int32_t *mvs_p = mb->mvs_s + (i & 16);
		v8hi mvp;
		int eq = ctx->refIdx4x4_eq[i];
		int mvs_DC = eq & 8 ? ctx->mvs_D[i4x4] : ctx->mvs_C[i4x4];
		if (__builtin_expect(0xe9e9 >> eq & 1, 1)) {
			v8hi mvA = (v8hi)(v4si){mvs_p[ctx->mvs_A[i4x4]]};
			v8hi mvB = (v8hi)(v4si){mvs_p[ctx->mvs_B[i4x4]]};
			v8hi mvDC = (v8hi)(v4si){mvs_p[mvs_DC]};
			mvp = median_v8hi(mvA, mvB, mvDC);
		} else {
			int mvs_AB = eq & 1 ? ctx->mvs_A[i4x4] : ctx->mvs_B[i4x4];
			mvp = (v8hi)(v4si){mvs_p[eq & 4 ? mvs_DC : mvs_AB]};
		}
		
		// broadcast absMvd and mvs to memory then call decoding
		static const int8_t masks[16] = {0, 15, 10, 5, 12, 3, 0, 0, 8, 0, 0, 0, 4, 0, 2, 1};
		static const int8_t widths[16] = {0, 8, 4, 4, 8, 8, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		static const int8_t heights[16] = {0, 8, 8, 8, 4, 4, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		int type = mvd_flags >> (i & -4) & 15;
		int m = masks[type];
		int i8x8 = i >> 2;
		v8hi bits = {1, 2, 4, 8};
		v8hi absMvd_mask = ((v8hi){m, m, m, m, m, m, m, m} & bits) == bits;
		v8hi absMvd_old = (v8hi)(v2li){mb->absMvd_l[i8x8]};
		v8hi mvs_mask = __builtin_shufflevector(absMvd_mask, (v8hi){}, 0, 0, 1, 1, 2, 2, 3, 3);
		v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
		mb->absMvd_l[i8x8] = ((v2li)ifelse_mask(absMvd_mask, (v8hi)pack_absMvd(mvd), absMvd_old))[0];
		mb->mvs_v[i8x8] = ifelse_mask(mvs_mask, mvs, mb->mvs_v[i8x8]);
		CALL(decode_inter, i, widths[type], heights[type]);
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
	mb->f.mbIsInterFlag = 1;
	mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
	#ifdef CABAC
		mb->absMvd_v[0] = mb->absMvd_v[1] = mb->absMvd_v[2] = mb->absMvd_v[3] = (v16qu){};
	#endif
	
	// parse mb_skip_run/flag
	#ifndef CABAC
		if (ctx->mb_skip_run < 0) {
			ctx->mb_skip_run = CALL(get_ue32, 139264);
			fprintf(stderr, "mb_skip_run: %u\n", ctx->mb_skip_run);
		}
		int mb_skip_flag = ctx->mb_skip_run-- > 0;
	#else
		int mb_skip_flag = CALL(get_ae, 26 - ctx->inc.mb_skip_flag);
		fprintf(stderr, "mb_skip_flag: %x\n", mb_skip_flag);
	#endif
	
	// earliest handling for B_Skip
	if (mb_skip_flag) {
		#ifdef CABAC
			mb->f.mb_skip_flag = 1;
			mb->f.mb_type_B_Direct = 1;
			ctx->mb_qp_delta_nz = 0;
		#endif
		mb->inter_eqs_s = 0;
		JUMP(decode_direct_mv_pred, 0xffffffff);
	}
		
	// B_Direct_16x16
	#ifndef CABAC
		int mb_type = CALL(get_ue16, 48);
	#endif
	if (CACOND(mb_type == 0, !CALL(get_ae, 29 - ctx->inc.mb_type_B_Direct))) {
		fprintf(stderr, "mb_type: 0\n");
		#ifdef CABAC
			mb->f.mb_type_B_Direct = 1;
		#endif
		ctx->transform_8x8_mode_flag = ctx->ps.transform_8x8_mode_flag & ctx->ps.direct_8x8_inference_flag;
		mb->inter_eqs_s = 0;
		CALL(decode_direct_mv_pred, 0xffffffff);
		CAJUMP(parse_inter_residual);
	}
	ctx->transform_8x8_mode_flag = ctx->ps.transform_8x8_mode_flag;
	
	// initializations and jumps for mb_type
	#ifndef CABAC
		fprintf(stderr, "mb_type: %u\n", mb_type);
		if (mb_type > 22)
			CAJUMP(parse_I_mb, mb_type - 23);
		mb->refIdx_l = -1;
		mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (v8hi){};
		if (mb_type == 22)
			CAJUMP(parse_B_sub_mb);
		static const uint8_t mb_type2flags[22] = {0, 0x01, 0x10, 0x11, 0x05, 0x03,
			0x50, 0x30, 0x41, 0x21, 0x14, 0x12, 0x45, 0x23, 0x54, 0x32, 0x15, 0x13,
			0x51, 0x31, 0x55, 0x33};
		int flags8x8 = mb_type2flags[mb_type];
	#else
		int str = 4;
		if (!CALL(get_ae, 30) || (str = CALL(get_ae, 31),
			str += str + CALL(get_ae, 32),
			str += str + CALL(get_ae, 32),
			str += str + CALL(get_ae, 32), str - 8 < 5u))
		{
			str += str + CALL(get_ae, 32);
		}
		if (str == 13)
			CAJUMP(parse_I_mb, 32);
		static const uint8_t str2mb_type[26] = {3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 0, 0,
			0, 0, 11, 22, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
		fprintf(stderr, "mb_type: %u\n", str2mb_type[str]);
		mb->refIdx_l = -1;
		mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (v8hi){};
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
		mb->inter_eqs_s = little_endian32(0x1b5fbbff);
		if (flags8x8 & 0x01) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd, 0);
			CALL(decode_inter_16x16, mvd, 0);
		}
		if (flags8x8 & 0x10) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd + 32, 0);
			CALL(decode_inter_16x16, mvd, 1);
		}
	} else if (!(flags8x8 & 0xcc)) { // 8x16
		mb->inter_eqs_s = little_endian32(0x1b1bbbbb);
		if (flags8x8 & 0x01) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd, 0);
			CALL(decode_inter_8x16_left, mvd, 0);
		}
		if (flags8x8 & 0x02) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd, 4);
			CALL(decode_inter_8x16_right, mvd, 0);
		}
		if (flags8x8 & 0x10) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd + 32, 0);
			CALL(decode_inter_8x16_left, mvd, 1);
		}
		if (flags8x8 & 0x20) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd + 32, 4);
			CALL(decode_inter_8x16_right, mvd, 1);
		}
	} else { // 16x8
		mb->inter_eqs_s = little_endian32(0x1b5f1b5f);
		if (flags8x8 & 0x01) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd, 0);
			CALL(decode_inter_16x8_top, mvd, 0);
		}
		if (flags8x8 & 0x04) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd, 8);
			CALL(decode_inter_16x8_bottom, mvd, 0);
		}
		if (flags8x8 & 0x10) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd + 32, 0);
			CALL(decode_inter_16x8_top, mvd, 1);
		}
		if (flags8x8 & 0x40) {
			v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd + 32, 8);
			CALL(decode_inter_16x8_bottom, mvd, 1);
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
	fprintf(stderr, "sub_mb_types:");
	unsigned mvd_flags = 0;
	mb->inter_eqs_s = 0;
	for (int i8x8 = 0; i8x8 < 4; i8x8++) {
		int i4x4 = i8x8 * 4;
		int flags = 1;
		int eqs = 0x1b;
		#ifndef CABAC
			int sub_mb_type = CALL(get_ue16, 3);
		#endif
		if (CACOND(sub_mb_type == 0, CALL(get_ae, 21)) || // 8x8
			(ctx->transform_8x8_mode_flag = 0, flags = 5, eqs = 0x11, CACOND(sub_mb_type == 1, !CALL(get_ae, 22)))) { // 8x4
			ctx->unavail4x4[i4x4] = (ctx->unavail4x4[i4x4] & 11) | (ctx->unavail4x4[i4x4 + 1] & 4);
			ctx->unavail4x4[i4x4 + 2] |= 4;
			ctx->refIdx4x4_C[i4x4] = 0x0d63 >> i4x4 & 15;
			ctx->mvs_C[i4x4] = ctx->mvs_C[i4x4 + 1];
		} else { // 4xN
			ctx->refIdx4x4_C[i4x4] = 0xdc32 >> i4x4 & 15;
			ctx->mvs_C[i4x4] = ctx->mvs_B[i4x4 + 1];
			if (CACOND(sub_mb_type == 2, CALL(get_ae, 23)))
				flags = 3, eqs = 0x0a;
			else
				flags = 15, eqs = 0;
		}
		mvd_flags |= flags << i4x4;
		mb->inter_eqs[i8x8] = eqs;
		fprintf(stderr, (flags == 1) ? " 0" : (flags == 5) ? " 1" : (flags == 3) ? " 2" : " 3");
	}
	fprintf(stderr, "\n");
	CACALL(parse_ref_idx, ref_idx_flags);
	
	// load neighbouring refIdx values and shuffle them into A/B/C/D
	Edge264_macroblock *mbB = ctx->mbB;
	v16qi BC = (v16qi)(v2li){(int64_t)mbB->refIdx_l, (int64_t)mbB[1].refIdx_l};
	v16qi Ar = (v16qi)(v2li){(int64_t)mb[-1].refIdx_l, (int64_t)mb->refIdx_l};
	v16qi BCAr0 = (v16qi)__builtin_shufflevector((v4si)BC, (v4si)Ar, 0, 2, 4, 6);
	v16qi r0 = __builtin_shufflevector(BCAr0, BCAr0, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15);
	v16qi A0 = __builtin_shufflevector(BCAr0, BCAr0, 9, 12, 9, 12, 12, 13, 12, 13, 11, 14, 11, 14, 14, 15, 14, 15);
	v16qi B0 = __builtin_shufflevector(BCAr0, BCAr0, 2, 2, 12, 12, 3, 3, 13, 13, 12, 12, 14, 14, 13, 13, 15, 15);
	v16qi C0 = shuffle(BCAr0, ctx->refIdx4x4_C_v);
	v16qi D0 = __builtin_shufflevector(BCAr0, BCAr0, -1, 2, 9, 12, 2, 3, 12, 13, 9, 12, 11, 14, 12, 13, 14, 15);
	D0[0] = mbB->refIdx[3];
	
	// combine them into a vector of 4-bit equality masks
	v16qi u = ctx->unavail4x4_v;
	v16qi uC = u & 4;
	ctx->refIdx4x4_eq_v[0] = (uC - ifelse_mask(uC==4, r0==D0, r0==C0) * 2 - (r0==B0)) * 2 - (r0==A0 | u==14);
	
	// loop on mvs
	do {
		int i = __builtin_ctz(mvd_flags);
		v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd, i);
		
		// branch on equality mask
		v8hi mvp;
		int eq = ctx->refIdx4x4_eq[i];
		int mvs_DC = eq & 8 ? ctx->mvs_D[i] : ctx->mvs_C[i];
		if (__builtin_expect(0xe9e9 >> eq & 1, 1)) {
			v8hi mvA = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_A[i])};
			v8hi mvB = (v8hi)(v4si){*(mb->mvs_s + ctx->mvs_B[i])};
			v8hi mvDC = (v8hi)(v4si){*(mb->mvs_s + mvs_DC)};
			mvp = median_v8hi(mvA, mvB, mvDC);
		} else {
			int mvs_AB = eq & 1 ? ctx->mvs_A[i] : ctx->mvs_B[i];
			mvp = (v8hi)(v4si){*(mb->mvs_s + (eq & 4 ? mvs_DC : mvs_AB))};
		}
		
		// broadcast absMvd and mvs to memory then call decoding
		static const int8_t masks[16] = {0, 15, 10, 5, 12, 3, 0, 0, 8, 0, 0, 0, 4, 0, 2, 1};
		static const int8_t widths[16] = {0, 8, 4, 4, 8, 8, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		static const int8_t heights[16] = {0, 8, 8, 8, 4, 4, 0, 0, 4, 0, 0, 0, 4, 0, 4, 4};
		int type = mvd_flags >> (i & -4) & 15;
		int m = masks[type];
		int i8x8 = i >> 2;
		v8hi bits = {1, 2, 4, 8};
		v8hi absMvd_mask = ((v8hi){m, m, m, m, m, m, m, m} & bits) == bits;
		v8hi absMvd_old = (v8hi)(v2li){mb->absMvd_l[i8x8]};
		v8hi mvs_mask = __builtin_shufflevector(absMvd_mask, (v8hi){}, 0, 0, 1, 1, 2, 2, 3, 3);
		v8hi mvs = (v8hi)__builtin_shufflevector((v4si)(mvp + mvd), (v4si){}, 0, 0, 0, 0);
		mb->absMvd_l[i8x8] = ((v2li)ifelse_mask(absMvd_mask, (v8hi)pack_absMvd(mvd), absMvd_old))[0];
		mb->mvs_v[i8x8] = ifelse_mask(mvs_mask, mvs, mb->mvs_v[i8x8]);
		CALL(decode_inter, i, widths[type], heights[type]);
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
 *   blocks since there is no dependency between blocks here
 * _ then for each block in sequence, fetch the correct mv(s) and compute their
 *   median based on the mask
 */
static inline void CAFUNC(parse_P_mb)
{
	// Inter initializations
	mb->f.mbIsInterFlag = 1;
	mb->Intra4x4PredMode_v = (v16qi){2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
	mb->refIdx_l = (int64_t)(v8qi){0, 0, 0, 0, -1, -1, -1, -1};
	
	// parse mb_skip_run/flag
	#ifndef CABAC
		if (ctx->mb_skip_run < 0) {
			ctx->mb_skip_run = CALL(get_ue32, 139264);
			fprintf(stderr, "mb_skip_run: %u\n", ctx->mb_skip_run);
		}
		int mb_skip_flag = ctx->mb_skip_run-- > 0;
	#else
		int mb_skip_flag = CALL(get_ae, 13 - ctx->inc.mb_skip_flag);
		fprintf(stderr, "mb_skip_flag: %x\n", mb_skip_flag);
	#endif
	
	// earliest handling for P_Skip
	if (mb_skip_flag) {
		#ifdef CABAC
			mb->f.mb_skip_flag = 1;
			ctx->mb_qp_delta_nz = 0;
		#endif
		mb->inter_eqs_s = little_endian32(0x1b5fbbff);
		v16qi refIdx_v = (v16qi)(v2li){mb->refIdx_l};
		mb->refPic_l = ((v2li)(shuffle(ctx->RefPicList_v[0], refIdx_v) | refIdx_v))[0];
		int refIdxA = mb[-1].refIdx[1];
		int refIdxB = ctx->mbB->refIdx[2];
		int mvA = *(mb->mvs_s + ctx->mvs_A[0]);
		int mvB = *(mb->mvs_s + ctx->mvs_B[0]);
		v8hi mv = {};
		if ((refIdxA | mvA) && (refIdxB | mvB) && !(ctx->unavail16x16 & 3)) {
			int refIdxC, mvs_C;
			if (__builtin_expect(ctx->unavail16x16 & 4, 0)) {
				refIdxC = ctx->mbB[-1].refIdx[3];
				mvs_C = ctx->mvs_D[0];
			} else {
				refIdxC = ctx->mbB[1].refIdx[2];
				mvs_C = ctx->mvs_C[5];
			}
			// B/C unavailability (->A) was ruled out, thus not tested here
			int eq = !refIdxA + !refIdxB * 2 + !refIdxC * 4;
			if (__builtin_expect(0xe9 >> eq & 1, 1)) {
				mv = median_v8hi((v8hi)(v4si){mvA}, (v8hi)(v4si){mvB}, (v8hi)(v4si){*(mb->mvs_s + mvs_C)});
			} else if (eq == 4) {
				mv = (v8hi)(v4si){*(mb->mvs_s + mvs_C)};
			} else {
				mv = (v8hi)(v4si){(eq == 1) ? mvA : mvB};
			}
		}
		v8hi mvs = (v8hi)__builtin_shufflevector((v4si)mv, (v4si){}, 0, 0, 0, 0);
		mb->mvs_v[0] = mb->mvs_v[1] = mb->mvs_v[2] = mb->mvs_v[3] = mvs;
		mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (v8hi){};
		#ifdef CABAC
			mb->absMvd_v[0] = mb->absMvd_v[1] = (v16qu){};
		#endif
		JUMP(decode_inter, 0, 16, 16);
	}
	
	// initializations and jumps for mb_type
	#ifndef CABAC
		int mb_type = CALL(get_ue16, 30);
		fprintf(stderr, "mb_type: %u\n", mb_type);
		if (mb_type > 4) {
			#ifdef CABAC
				mb->absMvd_v[0] = mb->absMvd_v[1] = (v16qu){};
			#endif
			CAJUMP(parse_I_mb, mb_type - 5);
		}
		mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (v8hi){};
		if (mb_type > 2)
			CAJUMP(parse_P_sub_mb, (mb_type + 12) & 15); // 3->15, 4->0
		CACALL(parse_ref_idx, 0x351 >> (mb_type << 2) & 15); // 0->1, 1->5, 2->3
	#else
		if (CALL(get_ae, 14)) { // Intra
			#ifdef CABAC
				mb->absMvd_v[0] = mb->absMvd_v[1] = (v16qu){};
			#endif
			CAJUMP(parse_I_mb, 17);
		}
		int mb_type = CALL(get_ae, 15); // actually 1 and 3 are swapped
		mb_type += mb_type + CALL(get_ae, 16 + mb_type);
		fprintf(stderr, "mb_type: %u\n", (4 - mb_type) & 3);
		mb->mvs_v[4] = mb->mvs_v[5] = mb->mvs_v[6] = mb->mvs_v[7] = (v8hi){};
		if (mb_type == 1)
			CAJUMP(parse_P_sub_mb, 15);
		CACALL(parse_ref_idx, (mb_type + 1) | 1); // 0->1, 2->3, 3->5
	#endif
	
	// decoding large blocks
	if (mb_type == 0) { // 16x16
		mb->inter_eqs_s = little_endian32(0x1b5fbbff);
		v8hi mvd = CACALL(parse_mvd_pair, mb->absMvd, 0);
		CALL(decode_inter_16x16, mvd, 0);
	} else if (mb_type == 2) { // 8x16
		mb->inter_eqs_s = little_endian32(0x1b1bbbbb);
		v8hi mvd0 = CACALL(parse_mvd_pair, mb->absMvd, 0);
		CALL(decode_inter_8x16_left, mvd0, 0);
		v8hi mvd1 = CACALL(parse_mvd_pair, mb->absMvd, 4);
		CALL(decode_inter_8x16_right, mvd1, 0);
	} else { // 16x8
		mb->inter_eqs_s = little_endian32(0x1b5f1b5f);
		v8hi mvd0 = CACALL(parse_mvd_pair, mb->absMvd, 0);
		CALL(decode_inter_16x8_top, mvd0, 0);
		v8hi mvd1 = CACALL(parse_mvd_pair, mb->absMvd, 8);
		CALL(decode_inter_16x8_bottom, mvd1, 0);
	}
	CAJUMP(parse_inter_residual);
}



/**
 * This function loops through the macroblocks of a slice, initialising their
 * data and calling parse_{I/P/B}_mb for each one.
 */
static noinline void CAFUNC(parse_slice_data)
{
	static const v16qi block_unavailability[16] = {
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
	
	while (1) {
		fprintf(stderr, "********** POC=%u MB=%u **********\n", ctx->PicOrderCnt, ctx->CurrMbAddr);
		
		// store temporary unavailable data on slice edge
		int unavail16x16 = mb->unavail16x16;
		int filter_edges = (ctx->disable_deblocking_filter_idc == 1) ? 0 : ~(mb->unavail16x16 << 1) & 7;
		v16qi fA = mb[-1].f.v;
		v16qi fB = ctx->mbB->f.v;
		uint64_t bitsA = mb[-1].bits_l;
		uint64_t bitsB = ctx->mbB->bits_l;
		if (ctx->first_mb_in_slice) {
			v16qi zero = {};
			if (ctx->CurrMbAddr == ctx->first_mb_in_slice) { // A is unavailable
				unavail16x16 |= 1;
				fA = unavail_mb.f.v;
				bitsA = unavail_mb.bits_l;
				ctx->refIdx_copyA = mb[-1].refIdx_l;
				ctx->nC_copyA[0] = mb[-1].nC_v[0];
				ctx->nC_copyA[1] = mb[-1].nC_v[1];
				ctx->nC_copyA[2] = mb[-1].nC_v[2];
				ctx->mvs_copyA[0] = mb[-1].mvs_v[1];
				ctx->mvs_copyA[1] = mb[-1].mvs_v[3];
				ctx->mvs_copyA[2] = mb[-1].mvs_v[5];
				ctx->mvs_copyA[3] = mb[-1].mvs_v[7];
				mb[-1].refIdx_l = -1;
				mb[-1].nC_v[0] = mb[-1].nC_v[1] = mb[-1].nC_v[2] = (v16qi)zero;
				mb[-1].Intra4x4PredMode_v = unavail_mb.Intra4x4PredMode_v;
				mb[-1].absMvd_v[0] = mb[-1].absMvd_v[1] = mb[-1].absMvd_v[2] = mb[-1].absMvd_v[3] = (v16qu)zero;
				mb[-1].mvs_v[1] = mb[-1].mvs_v[3] = mb[-1].mvs_v[5] = mb[-1].mvs_v[7] = (v8hi)zero;
				filter_edges &= ~ctx->disable_deblocking_filter_idc; // impacts only bit 1
			}
			if (ctx->CurrMbAddr <= ctx->first_mb_in_slice + ctx->ps.pic_width_in_mbs) { // B is unavailable
				unavail16x16 |= 2;
				fB = unavail_mb.f.v;
				bitsB = unavail_mb.bits_l;
				ctx->refIdx_copyB = ctx->mbB->refIdx_l;
				ctx->nC_copyB[0] = ctx->mbB->nC_l[1];
				ctx->nC_copyB[1] = ctx->mbB->nC_l[5];
				ctx->nC_copyB_v[1] = ctx->mbB->nC_v[1];
				ctx->mvs_copyB[0] = ctx->mbB->mvs_l[5];
				ctx->mvs_copyB[1] = ctx->mbB->mvs_l[7];
				ctx->mvs_copyB[2] = ctx->mbB->mvs_l[13];
				ctx->mvs_copyB[3] = ctx->mbB->mvs_l[15];
				ctx->mbB->refIdx_l = -1;
				ctx->mbB->nC_l[1] = ctx->mbB->nC_l[5] = 0;
				ctx->mbB->nC_v[1] = (v2li){};
				ctx->mbB->Intra4x4PredMode_v = unavail_mb.Intra4x4PredMode_v;
				ctx->mbB->absMvd_v[1] = ctx->mbB->absMvd_v[3] = (v16qu)zero;
				ctx->mbB->mvs_l[5] = ctx->mbB->mvs_l[7] = ctx->mbB->mvs_l[13] = ctx->mbB->mvs_l[15] = 0;
				filter_edges &= ~(ctx->disable_deblocking_filter_idc << 1); // impacts only bit 2
			}
			if (ctx->CurrMbAddr < ctx->first_mb_in_slice + ctx->ps.pic_width_in_mbs) { // C is unavailable
				unavail16x16 |= 4;
				ctx->refIdx_copyCD = ctx->mbB[1].refIdx_l;
				ctx->mvs_copyCD[0] = ctx->mbB[1].mvs_s[10];
				ctx->mvs_copyCD[1] = ctx->mbB[1].mvs_s[26];
				ctx->mbB[1].refIdx_l = -1;
				ctx->mbB[1].mvs_s[10] = ctx->mbB[1].mvs_s[26] = 0;
			}
		}
		
		// initialize current macroblock
		mb->filter_edges = filter_edges;
		ctx->unavail16x16 = unavail16x16;
		ctx->unavail4x4_v = block_unavailability[unavail16x16];
		ctx->inc.v = fA + fB + (fB & flags_twice.v);
		mb->f.v = (v16qi){};
		mb->QP_s = ctx->QP_s;
		if (ctx->ps.ChromaArrayType == 1) { // FIXME 4:2:2
			mb->bits_l = (bitsA >> 3 & 0x11111100111111) | (bitsB >> 1 & 0x42424200424242);
		}
		mb->nC_v[0] = mb->nC_v[1] = mb->nC_v[2] = (v16qi){};
		
		// Would it actually help to push this test outside the loop?
		if (ctx->slice_type == 0) {
			CACALL(parse_P_mb);
		} else if (ctx->slice_type == 1) {
			CACALL(parse_B_mb);
		} else {
			int mb_type_or_ctxIdx = CACOND(CALL(get_ue16, 25), 5 - ctx->inc.mb_type_I_NxN);
			#ifndef CABAC
				fprintf(stderr, "mb_type: %u\n", mb_type_or_ctxIdx);
			#endif
			CACALL(parse_I_mb, mb_type_or_ctxIdx);
		}
		
		// restore macroblock data on slice edge
		if (ctx->first_mb_in_slice) {
			if (ctx->CurrMbAddr == ctx->first_mb_in_slice) { // A is unavailable
				mb[-1].refIdx_l = ctx->refIdx_copyA;
				mb[-1].nC_v[0] = ctx->nC_copyA[0];
				mb[-1].nC_v[1] = ctx->nC_copyA[1];
				mb[-1].nC_v[2] = ctx->nC_copyA[2];
				mb[-1].mvs_v[1] = ctx->mvs_copyA[0];
				mb[-1].mvs_v[3] = ctx->mvs_copyA[1];
				mb[-1].mvs_v[5] = ctx->mvs_copyA[2];
				mb[-1].mvs_v[7] = ctx->mvs_copyA[3];
			}
			if (ctx->CurrMbAddr <= ctx->first_mb_in_slice + ctx->ps.pic_width_in_mbs) { // B is unavailable
				ctx->mbB->refIdx_l = ctx->refIdx_copyB;
				ctx->mbB->nC_l[1] = ctx->nC_copyB[0];
				ctx->mbB->nC_l[5] = ctx->nC_copyB[1];
				ctx->mbB->nC_v[1] = ctx->nC_copyB_v[1];
				ctx->mbB->mvs_l[5] = ctx->mvs_copyB[0];
				ctx->mbB->mvs_l[7] = ctx->mvs_copyB[1];
				ctx->mbB->mvs_l[13] = ctx->mvs_copyB[2];
				ctx->mbB->mvs_l[15] = ctx->mvs_copyB[3];
			}
			if (ctx->CurrMbAddr < ctx->first_mb_in_slice + ctx->ps.pic_width_in_mbs) { // C is unavailable
				ctx->mbB[1].refIdx_l = ctx->refIdx_copyCD;
				ctx->mbB[1].mvs_s[10] = ctx->mvs_copyCD[0];
				ctx->mbB[1].mvs_s[26] = ctx->mvs_copyCD[1];
			}
		}
		
		// break at end of slice
		ctx->CurrMbAddr++;
		#ifdef CABAC
			int end_of_slice_flag = CALL(cabac_terminate);
			fprintf(stderr, "end_of_slice_flag: %x\n\n", end_of_slice_flag);
		#endif
		if (CACOND(ctx->mb_skip_run < 0 && msb_cache >> (SIZE_BIT - 24) == 0x800000, end_of_slice_flag))
			break;
		
		// point to the next macroblock
		mb++;
		ctx->mbB++;
		ctx->mbCol++;
		ctx->samples_mb[0] += 16; // FIXME 16bit
		ctx->samples_mb[1] += 8; // FIXME 4:2:2, 16bit
		ctx->samples_mb[2] += 8;
		
		// end of row
		if (ctx->samples_mb[0] - ctx->samples_row[0] >= ctx->stride[0]) {
			mb++; // skip the empty macroblock at the edge
			ctx->mbB++;
			ctx->mbCol++;
			ctx->samples_mb[0] = ctx->samples_row[0] += ctx->stride[0] * 16;
			ctx->samples_mb[1] = ctx->samples_row[1] += ctx->stride[1] * 8; // FIXME 4:2:2
			ctx->samples_mb[2] = ctx->samples_row[2] += ctx->stride[1] * 8;
			if (ctx->samples_row[0] - ctx->samples_pic >= ctx->plane_size_Y)
				break;
		}
	}
}
