#include "edge264_common.h"



#ifdef __SSSE3__
size_t refill(int shift, size_t ret) {
	typedef size_t v16u __attribute__((vector_size(16)));
	static const v16qi shuf[8] = {
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15},
		{0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 15},
	};
	
	// when shift overflows, read the next few bytes in both scalar and vector registers
	ctx->shift = shift;
	if ((shift -= SIZE_BIT) >= 0) {
		__m128i x;
		size_t bits;
		const uint8_t *CPB = ctx->CPB;
		ctx->RBSP[0] = ctx->RBSP[1];
		ctx->shift = shift;
		if (CPB <= ctx->end - 16) {
			x = _mm_loadu_si128((__m128i *)(CPB - 2));
			memcpy(&bits, CPB, sizeof(size_t));
			bits = big_endian(bits);
			CPB += sizeof(size_t);
		} else {
			x = _mm_srl_si128(_mm_loadu_si128((__m128i *)(ctx->end - 16)), CPB - (ctx->end - 16));
			bits = big_endian(((v16u)x)[0]);
			CPB += sizeof(size_t);
			CPB = CPB < ctx->end ? CPB : ctx->end;
		}
		
		// ignore words without a zero odd byte
		unsigned mask = _mm_movemask_epi8(_mm_cmpeq_epi8(x, _mm_setzero_si128()));
		if (mask & (SIZE_BIT == 32 ? 0xa : 0xaa)) {
			x = _mm_srli_si128(x, 2);
			mask &= mask >> 1 & _mm_movemask_epi8(_mm_cmpeq_epi8(x, _mm_set1_epi8(3)));
			
			// iterate on and remove every emulation_prevention_three_byte
			for (; mask & (SIZE_BIT == 32 ? 0xf : 0xff); mask = (mask & (mask - 1)) >> 1, CPB++) {
				int i = __builtin_ctz(mask);
				x = _mm_shuffle_epi8(x, (__m128i)shuf[i]);
				bits = big_endian(((v16u)x)[0]);
			}
		}
		ctx->RBSP[1] = bits;
		ctx->CPB = CPB;
	}
	return ret;
}
#endif



/**
 * Read Exp-Golomb codes and bit sequences.
 *
 * upper and lower are the bounds allowed by the spec, which get_ue and get_se
 * use both as hints to choose the fastest input routine, and as clipping
 * parameters such that values are always bounded no matter the input stream.
 * To keep your code branchless, upper and lower shall always be constants.
 * Use min/max with get_ueN/map_se to apply variable bounds.
 */
__attribute__((noinline)) size_t get_u1() {
	return refill(ctx->shift + 1, ctx->RBSP[0] << ctx->shift >> (SIZE_BIT - 1));
}
__attribute__((noinline)) size_t get_uv(unsigned v) {
	size_t bits = lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift);
	return refill(ctx->shift + v, bits >> (SIZE_BIT - v));
}
__attribute__((noinline)) size_t get_ue16() { // Parses Exp-Golomb codes up to 2^16-2
	size_t bits = lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift);
	unsigned v = clz(bits | (size_t)1 << (SIZE_BIT / 2)) * 2 + 1;
	return refill(ctx->shift + v, (bits >> (SIZE_BIT - v)) - 1);
}
#if SIZE_BIT == 32
__attribute__((noinline)) size_t get_ue32() { // Parses Exp-Golomb codes up to 2^32-2
	size_t bits = lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift);
	unsigned leadingZeroBits = clz(bits | 1);
	refill(ctx->shift + leadingZeroBits, 0);
	return get_uv(leadingZeroBits + 1) - 1;
}
#endif
