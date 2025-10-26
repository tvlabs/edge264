/**
 * Parsing CAVLC and CABAC values (for 32 or 64 bit machines).
 */
#include "edge264_internal.h"


/**
 * Extract nbytes from the bitstream and return them as big endian.
 * 
 * The process reads an unaligned 16-byte chunk and will not read past the last
 * aligned 16-byte chunk containing the last bytes before ctx->t.gb.end.
 */
static inline size_t get_bytes(Edge264GetBits *gb, int nbytes)
{
	static const i8x16 shuf[8] = {
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1},
		{0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1},
		{0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1},
		{0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1},
		{0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1},
		{0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1},
		{0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1},
		{0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, -1},
	};
	
	// load 16 bytes without reading past aligned buffer boundaries
	const uint8_t *CPB = gb->CPB;
	const uint8_t *end = gb->end;
	intptr_t diff = CPB - end;
	u8x16 v;
	if (__builtin_expect(diff <= -14, 1)) {
		v = load128(CPB - 2);
	} else if (diff < 0) {
		const uint8_t *p = minp(CPB - 2, (uint8_t *)((uintptr_t)(end - 1) & -16));
		v = shrv128(shlv128(load128(p), p + 16 - end), min(14 + diff, 16));
	} else {
		gb->CPB = CPB + nbytes;
		return 0;
	}
	
	// make a bitmask for the positions of 00n escape sequences and iterate on it
	u8x16 eq0 = v == 0;
	u8x16 x = shr128(v, 2);
	#if defined(__SSE2__)
		unsigned test = movemask(eq0 & shr128(eq0, 1) & (x <= 3));
		unsigned mask = (1 << nbytes) - 1;
		if (__builtin_expect(test & mask, 0)) {
			unsigned three = movemask(x == 3);
			unsigned stop = test & mask & ~three;
			if (stop) {
				int i = __builtin_ctz(stop);
				gb->end = CPB + i - 2;
				x &= ~shlv128(set8(-1), i);
			}
			for (unsigned esc = test & three; esc & mask; esc = (esc & (esc - 1)) >> 1) {
				int i = __builtin_ctz(esc);
				x = shuffle(x, shuf[i]);
				CPB++;
			}
		}
	#elif defined(__ARM_NEON)
		uint64_t test = (uint64_t)vshrn_n_u16(eq0 & shr128(eq0, 1) & (x <= 3), 4);
		uint64_t mask = (1ULL << (nbytes << 2)) - 1;
		if (__builtin_expect(test & mask, 0)) {
			uint64_t three = (uint64_t)vshrn_n_u16(x == 3, 4);
			uint64_t stop = test & mask & ~three;
			if (stop) {
				int i = __builtin_ctzll(stop) >> 2;
				gb->end = CPB + i - 2;
				x &= ~shlv128(set8(-1), i);
			}
			for (uint64_t esc = test & three & 0x1111111111111111ULL; esc & mask; esc = (esc & (esc - 1)) >> 4) {
				int i = __builtin_ctzll(esc);
				x = shuffle(x, shuf[i]);
				CPB++;
			}
		}
	#endif
	
	// increment CPB and return the requested bytes in upper part of the result
	gb->CPB = CPB + nbytes;
	#if SIZE_BIT == 32
		return big_endian32(((i32x4)x)[0]);
	#elif SIZE_BIT == 64
		return big_endian64(((i64x2)x)[0]);
	#endif
}



/**
 * Read Exp-Golomb codes and bit sequences.
 *
 * History of versions and updates:
 * _ Removing all emulation_prevention_three_bytes beforehand to a buffer, then
 *   making 1~3 aligned reads depending on expected code size. The code for
 *   reading Exp-Golomb values is branchless, although the functions for long
 *   codes are quite complex.
 * _ Same as above but making 1~2 unaligned reads. It makes the code shorter
 *   and probably faster, but more complex to understand.
 * _ Removing emulation_prevention_three_bytes on the fly to a uint64_t cache
 *   with a shift count. This adds a test for refill each time a value is read
 *   (predicted not taken), but spares a full read/write pass on memory and
 *   needs zero buffer allocation. However it incurs some complexity to
 *   reassemble codes bigger than 32 bits.
 * _ Same as above but with a size_t[2] cache with shift count. It doubles the
 *   storage for 64 bit machines, and allows them to read any code size without
 *   an intermediate refill (pushing it as a tail call).
 * _ Same as above but using a trailing set bit instead of a shift variable.
 *   The main context then fits in 2 variables, which are easier to store in
 *   Global Register Variables.
 */
static noinline int refill(Edge264GetBits *gb, int ret) {
	size_t bytes = get_bytes(gb, SIZE_BIT >> 3);
	int trailing_bit = ctz(gb->msb_cache); // [0..SIZE_BIT-1]
	gb->msb_cache = (gb->msb_cache ^ (size_t)1 << trailing_bit) | bytes >> (SIZE_BIT - 1 - trailing_bit);
	gb->lsb_cache = (bytes * 2 + 1) << trailing_bit;
	return ret;
}

static noinline int get_u1(Edge264GetBits *gb) {
	int ret = gb->msb_cache >> (SIZE_BIT - 1);
	gb->msb_cache = shld(gb->lsb_cache, gb->msb_cache, 1);
	if (gb->lsb_cache <<= 1)
		return ret;
	return refill(gb, ret);
}

// Parses a 1~32-bit fixed size code
static noinline unsigned get_uv(Edge264GetBits *gb, unsigned v) {
	unsigned ret = gb->msb_cache >> (SIZE_BIT - v);
	if (SIZE_BIT == 32 && __builtin_expect(v == 32, 0)) {
		gb->msb_cache = gb->lsb_cache;
		gb->lsb_cache = 0;
	} else {
		gb->msb_cache = shld(gb->lsb_cache, gb->msb_cache, v);
		gb->lsb_cache <<= v;
	}
	if (gb->lsb_cache)
		return ret;
	return refill(gb, ret);
}

// Parses a Exp-Golomb code in one read, up to 2^16-2 (2^32-2 on 64-bit machines)
static noinline unsigned get_ue16(Edge264GetBits *gb, unsigned upper) {
	unsigned v = clz(gb->msb_cache | (size_t)1 << (SIZE_BIT / 2)) * 2 + 1; // [1..SIZE_BIT-1]
	unsigned ret = minu((gb->msb_cache >> (SIZE_BIT - v)) - 1, upper);
	gb->msb_cache = shld(gb->lsb_cache, gb->msb_cache, v);
	if (gb->lsb_cache <<= v)
		return ret;
	return refill(gb, ret);
}

// Parses a signed Exp-Golomb code in one read, from -2^15+1 to 2^15-1 (-2^31+1 to 2^31-1 on 64-bit machines)
static noinline int get_se16(Edge264GetBits *gb, int lower, int upper) {
	unsigned v = clz(gb->msb_cache | (size_t)1 << (SIZE_BIT / 2)) * 2 + 1; // [1..SIZE_BIT-1]
	unsigned ue = (gb->msb_cache >> (SIZE_BIT - v)) - 1;
	int ret = min(max((ue & 1) ? ue / 2 + 1 : -(ue / 2), lower), upper);
	gb->msb_cache = shld(gb->lsb_cache, gb->msb_cache, v);
	if (gb->lsb_cache <<= v)
		return ret;
	return refill(gb, ret);
}

// Extensions to [0,2^32-2] and [-2^31+1,2^31-1] for 32-bit machines
#if SIZE_BIT == 32
	static noinline unsigned get_ue32(Edge264GetBits *gb, unsigned upper) {
		unsigned leadingZeroBits = clz(gb->msb_cache | 1); // [0..31]
		gb->msb_cache = shld(gb->lsb_cache, gb->msb_cache, leadingZeroBits);
		if (!(gb->lsb_cache <<= leadingZeroBits))
			refill(gb, 0);
		return minu(get_uv(gb, leadingZeroBits + 1) - 1, upper);
	}

	static noinline int get_se32(Edge264GetBits *gb, int lower, int upper) {
		unsigned leadingZeroBits = clz(gb->msb_cache | 1); // [0..31]
		gb->msb_cache = shld(gb->lsb_cache, gb->msb_cache, leadingZeroBits);
		if (!(gb->lsb_cache <<= leadingZeroBits))
			refill(gb, 0);
		unsigned ue = get_uv(gb, leadingZeroBits + 1) - 1;
		return min(max(ue & 1 ? ue / 2 + 1 : -(ue / 2), lower), upper);
	}
#endif



/**
 * Read CABAC bins (9.3.3.2).
 * 
 * History of versions and updates:
 * _ Storing codIOffset and codIRange in two size_t variables, along with a
 *   counter to indicate how many extra bits are present in codIOffset.
 *   codIRange corresponds with the spec, and codIOffset is shifted with extra
 *   least significant bits. It dramatically reduces the number of memory reads
 *   to refill codIOffset compared to the specification, at the expense of more
 *   complex shift math.
 * _ Same as above but using the leading set bit of codIRange instead of a
 *   variable to count the remaining bits in codIOffset. codIRange is now
 *   shifted along with codIOffset. It requires one less memory load and
 *   shortens the dependency chain of critical function get_ae, thus improving
 *   overall performance.
 * _ Using a division instruction to get bypass bits ahead of time, since the
 *   process described in 9.3.3.2.3 is actually equivalent with a binary
 *   division. It is unclear whether it actually improves performance, but it
 *   looks very cool!
 * _ Storing codIOffset and codIRange in Global Register Variables (possible
 *   with GCC), since they account for a lot of reads/writes otherwise. It
 *   improves performance overall, and was later followed by the storage of
 *   CAVLC's context in GRVs. However the renormalization is now quite complex
 *   as it involves swapping CABAC and CAVLC in GRVs to fetch bits before
 *   inserting them into codIOffset.
 * _ Increasing the codIRange lower bound to trigger a refill from 256 to 512,
 *   to spare the refill test when parsing coeff_sign_flag. It was later
 *   abandoned since it required additional comments to explain divergence from
 *   specification.
 * _ Removing the dependency on CAVLC for renormalization, such that it can
 *   fetch bytes without restoring CAVLC's context. It spared a test for
 *   calling CAVLC's refill (not well predicted), thus improving performance.
 *   However the bypass division trick is now unavailable on 32 bit machines,
 *   since renormalization will now leave 25~32 bits in codIOffset, but the
 *   algorithm needs at least 29 to remain simple.
 */
static const uint8_t rangeTabLPS[64 * 4] = {
	128, 176, 208, 240, 128, 167, 197, 227, 128, 158, 187, 216, 123, 150, 178, 205,
	116, 142, 169, 195, 111, 135, 160, 185, 105, 128, 152, 175, 100, 122, 144, 166,
	 95, 116, 137, 158,  90, 110, 130, 150,  85, 104, 123, 142,  81,  99, 117, 135,
	 77,  94, 111, 128,  73,  89, 105, 122,  69,  85, 100, 116,  66,  80,  95, 110,
	 62,  76,  90, 104,  59,  72,  86,  99,  56,  69,  81,  94,  53,  65,  77,  89,
	 51,  62,  73,  85,  48,  59,  69,  80,  46,  56,  66,  76,  43,  53,  63,  72,
	 41,  50,  59,  69,  39,  48,  56,  65,  37,  45,  54,  62,  35,  43,  51,  59,
	 33,  41,  48,  56,  32,  39,  46,  53,  30,  37,  43,  50,  29,  35,  41,  48,
	 27,  33,  39,  45,  26,  31,  37,  43,  24,  30,  35,  41,  23,  28,  33,  39,
	 22,  27,  32,  37,  21,  26,  30,  35,  20,  24,  29,  33,  19,  23,  27,  31,
	 18,  22,  26,  30,  17,  21,  25,  28,  16,  20,  23,  27,  15,  19,  22,  25,
	 14,  18,  21,  24,  14,  17,  20,  23,  13,  16,  19,  22,  12,  15,  18,  21,
	 12,  14,  17,  20,  11,  14,  16,  19,  11,  13,  15,  18,  10,  12,  15,  17,
	 10,  12,  14,  16,   9,  11,  13,  15,   9,  11,  12,  14,   8,  10,  12,  14,
	  8,   9,  11,  13,   7,   9,  11,  12,   7,   9,  10,  12,   7,   8,  10,  11,
	  6,   8,   9,  11,   6,   7,   9,  10,   6,   7,   8,   9,   2,   2,   2,   2,
};
static const uint8_t transIdx[256] = {
	  4,   5, 253, 252,   8,   9, 153, 152,  12,  13, 153, 152,  16,  17, 149, 148,
	 20,  21, 149, 148,  24,  25, 149, 148,  28,  29, 145, 144,  32,  33, 145, 144,
	 36,  37, 145, 144,  40,  41, 141, 140,  44,  45, 141, 140,  48,  49, 141, 140,
	 52,  53, 137, 136,  56,  57, 137, 136,  60,  61, 133, 132,  64,  65, 133, 132,
	 68,  69, 133, 132,  72,  73, 129, 128,  76,  77, 129, 128,  80,  81, 125, 124,
	 84,  85, 121, 120,  88,  89, 121, 120,  92,  93, 121, 120,  96,  97, 117, 116,
	100, 101, 117, 116, 104, 105, 113, 112, 108, 109, 109, 108, 112, 113, 109, 108,
	116, 117, 105, 104, 120, 121, 105, 104, 124, 125, 101, 100, 128, 129,  97,  96,
	132, 133,  97,  96, 136, 137,  93,  92, 140, 141,  89,  88, 144, 145,  89,  88,
	148, 149,  85,  84, 152, 153,  85,  84, 156, 157,  77,  76, 160, 161,  77,  76,
	164, 165,  73,  72, 168, 169,  73,  72, 172, 173,  65,  64, 176, 177,  65,  64,
	180, 181,  61,  60, 184, 185,  61,  60, 188, 189,  53,  52, 192, 193,  53,  52,
	196, 197,  49,  48, 200, 201,  45,  44, 204, 205,  45,  44, 208, 209,  37,  36,
	212, 213,  37,  36, 216, 217,  33,  32, 220, 221,  29,  28, 224, 225,  25,  24,
	228, 229,  21,  20, 232, 233,  17,  16, 236, 237,  17,  16, 240, 241,   9,   8,
	244, 245,   9,   8, 248, 249,   5,   4, 248, 249,   1,   0, 252, 253,   0,   1,
};

static noinline int get_ae(Edge264Context * restrict ctx, int ctxIdx) {
	assert(ctx->t.gb.range == ctx->t.gb.codIRange << clz(ctx->t.gb.codIRange));
	assert(ctx->t.gb.offset == (ctx->t.gb.codIOffset << 1 | 1) << (clz(ctx->t.gb.codIRange) - 1));
	size_t state = ctx->cabac[ctxIdx];
	size_t shift = SIZE_BIT - 3 - clz(ctx->t.gb.codIRange);\
	size_t idx = (state & -4) + (ctx->t.gb.codIRange >> shift);\
	size_t codIRangeLPS = (size_t)((uint8_t *)rangeTabLPS - 4)[idx] << (shift - 6);\
	ctx->t.gb.codIRange -= codIRangeLPS;\
	if (ctx->t.gb.codIOffset >= ctx->t.gb.codIRange) {\
		state ^= 255;\
		ctx->t.gb.codIOffset -= ctx->t.gb.codIRange;\
		ctx->t.gb.codIRange = codIRangeLPS;\
	}\
	ctx->cabac[ctxIdx] = transIdx[state];\
	int binVal = state & 1;\
	if (__builtin_expect(ctx->t.gb.codIRange < 256, 0)) {\
		ctx->t.gb.codIOffset = shld(get_bytes(&ctx->t.gb, SIZE_BIT / 8 - 2), ctx->t.gb.codIOffset, SIZE_BIT - 16);\
		ctx->t.gb.codIRange <<= SIZE_BIT - 16;\
	}\
	ctx->t.gb.range = ctx->t.gb.codIRange << clz(ctx->t.gb.codIRange);
	ctx->t.gb.offset = (ctx->t.gb.codIOffset << 1 | 1) << (clz(ctx->t.gb.codIRange) - 1);
	return binVal;
}

static inline int get_bypass(Edge264Context *ctx) {
	assert(ctx->t.gb.range == ctx->t.gb.codIRange << clz(ctx->t.gb.codIRange));
	assert(ctx->t.gb.offset == (ctx->t.gb.codIOffset << 1 | 1) << (clz(ctx->t.gb.codIRange) - 1));
	// FIXME move the refill out of get_bypass to the caller
	if (ctx->t.gb.codIRange < 512) {
		ctx->t.gb.codIOffset = shld(get_bytes(&ctx->t.gb, SIZE_BIT / 8 - 2), ctx->t.gb.codIOffset, SIZE_BIT - 16);
		ctx->t.gb.codIRange <<= SIZE_BIT - 16;
	}
	ctx->t.gb.codIRange >>= 1;
	size_t binVal = ctx->t.gb.codIOffset >= ctx->t.gb.codIRange;
	ctx->t.gb.codIOffset = binVal ? ctx->t.gb.codIOffset - ctx->t.gb.codIRange : ctx->t.gb.codIOffset;
	ctx->t.gb.range = ctx->t.gb.codIRange << clz(ctx->t.gb.codIRange);
	ctx->t.gb.offset = (ctx->t.gb.codIOffset << 1 | 1) << (clz(ctx->t.gb.codIRange) - 1);
	return binVal;
}

static int cabac_start(Edge264Context *ctx) {
	// reclaim bits from cache while realigning with CPB on a byte boundary
	int cached_bits = SIZE_BIT * 2 - 1 - ctz(ctx->t.gb.lsb_cache);
	int shift = cached_bits & 7;
	int ret = shift > 0 && (ssize_t)ctx->t.gb.msb_cache >> (SIZE_BIT - shift) != -1; // return 1 if not all alignment bits are ones
	ctx->t.gb.codIOffset = ctx->t.gb.msb_cache << shift >> 8;
	while (cached_bits >= SIZE_BIT) {
		int32_t i = 0;
		if ((intptr_t)(ctx->t.gb.end - ctx->t.gb.CPB) >= 0)
			memcpy(&i, ctx->t.gb.CPB - 4, 4);
		ctx->t.gb.CPB -= 1 + ((big_endian32(i) & 0xffffff) == 3);
		cached_bits -= 8;
	}
	ctx->t.gb.codIRange = (size_t)510 << (SIZE_BIT - 17);
	ctx->t.gb.codIOffset = (ctx->t.gb.codIOffset < ctx->t.gb.codIRange) ? ctx->t.gb.codIOffset : ctx->t.gb.codIRange - 1; // protection against invalid bitstream
	ctx->t.gb.range = ctx->t.gb.codIRange << clz(ctx->t.gb.codIRange);
	ctx->t.gb.offset = (ctx->t.gb.codIOffset << 1 | 1) << (clz(ctx->t.gb.codIRange) - 1);
	return ret;
}

static int cabac_terminate(Edge264Context *ctx) {
	assert(ctx->t.gb.range == ctx->t.gb.codIRange << clz(ctx->t.gb.codIRange));
	assert(ctx->t.gb.offset == (ctx->t.gb.codIOffset << 1 | 1) << (clz(ctx->t.gb.codIRange) - 1));
	int extra = SIZE_BIT - 9 - clz(ctx->t.gb.codIRange); // [0..SIZE_BIT-9]
	ctx->t.gb.codIRange -= (size_t)2 << extra;
	ctx->t.gb.range = ctx->t.gb.codIRange << clz(ctx->t.gb.codIRange);
	ctx->t.gb.offset = (ctx->t.gb.codIOffset << 1 | 1) << (clz(ctx->t.gb.codIRange) - 1);
	if (ctx->t.gb.codIOffset >= ctx->t.gb.codIRange) {
		// reclaim the extra bits minus alignment bits, then refill the cache
		ctx->t.gb.msb_cache = (ctx->t.gb.codIOffset * 2 + 1) << (SIZE_BIT - 1 - (extra & -8));
		return refill(&ctx->t.gb, 1);
	}
	if (__builtin_expect(ctx->t.gb.codIRange < 256, 0)) {
		ctx->t.gb.codIOffset = shld(get_bytes(&ctx->t.gb, SIZE_BIT / 8 - 2), ctx->t.gb.codIOffset, SIZE_BIT - 16);
		ctx->t.gb.codIRange <<= SIZE_BIT - 16;
		ctx->t.gb.range = ctx->t.gb.codIRange << clz(ctx->t.gb.codIRange);
		ctx->t.gb.offset = (ctx->t.gb.codIOffset << 1 | 1) << (clz(ctx->t.gb.codIRange) - 1);
	}
	return 0;
}



/**
 * Initialise CABAC's context variables using vector code (9.3.1.1).
 *
 * Considering the bottleneck of memory access, this is probably just as fast
 * as copying from precomputed values. Please refrain from providing a default,
 * unoptimised version.
 */
static void cabac_init(Edge264Context *ctx) {
	i8x16 mul = set16(max(ctx->t.QP[0], 0) + 4096);
	i8x16 c1 = set8(1), c64 = set8(64), c126 = set8(126);
	const i8x16 *src = (i8x16 *)cabac_context_init[ctx->t.cabac_init_idc];
	for (i8x16 *dst = ctx->cabac_v; dst < ctx->cabac_v + 64; dst++, src += 2) {
		#if defined(__SSE2__)
			i16x8 sum0 = maddubs(mul, src[0]) >> 4;
			i16x8 sum1 = maddubs(mul, src[1]) >> 4;
		#elif defined(__ARM_NEON)
			i16x8 l0 = vmull_s8(vget_low_s8(mul), vget_low_s8(src[0]));
			i16x8 h0 = vmull_high_s8(mul, src[0]);
			i16x8 l1 = vmull_s8(vget_low_s8(mul), vget_low_s8(src[1]));
			i16x8 h1 = vmull_high_s8(mul, src[1]);
			i16x8 sum0 = (i16x8)vpaddq_s16(l0, h0) >> 4;
			i16x8 sum1 = (i16x8)vpaddq_s16(l1, h1) >> 4;
		#endif
		i8x16 min = minu8(packus16(sum0, sum1), c126);
		i8x16 mask = c64 > min;
		i8x16 preCtxState = maxu8(min, c1);
		i8x16 pStateIdx = preCtxState ^ mask;
		i8x16 shift = pStateIdx + pStateIdx;
		*dst = shift + shift + mask + c1; // pStateIdx << 2 | valMPS (middle bit is for transIdxLPS/MPS)
	}
	ctx->cabac[276] = 252;
}
