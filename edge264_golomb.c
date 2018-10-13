/**
 * Read Exp-Golomb codes and bit sequences.
 *
 * These critical functions are designed to contain minimal numbers of branches
 * and memory writes. Several approaches were considered:
 * _ Removing all emulation_prevention_three_bytes beforehand to an intermediate
 *   buffer, then making 1~3 aligned reads depending on expected code size.
 *   The first part incurs some complexity for the buffer management, but then
 *   the second part is simple and branchless.
 * _ The same as above but with unaligned reads, to make 1 read for small codes
 *   (which are the most frequent). This part is frustrating, because it adds
 *   complexity but is still useful.
 * _ Removing emulation_prevention_three_bytes on the fly to a size_t[2] buffer,
 *   and putting the refill code in a tail call. This approach incurs a branch
 *   in the refill function, but has been the simplest to maintain so far.
 * _ The same as above but with a single size_t buffer. This approach prevented
 *   refilling in a tail call, and generally needed more code.
 * _ The same as above but replacing shift by a trailing set bit in the buffer.
 *   This approach makes the fastest reads, but a cumbersome refill.
 */
#include "edge264_common.h"


#ifdef __SSSE3__
__attribute__((noinline)) size_t refill(int shift, size_t ret)
{
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
	
	// check for shift overflow
	ctx->shift = shift;
	if (__builtin_expect((shift -= SIZE_BIT) < 0, 1))
		return ret;
	ctx->shift = shift;
	
	// load 16 bytes and detect escape sequences
	const uint8_t *CPB = ctx->CPB;
	__m128i x = (CPB <= ctx->end - 14) ?
		_mm_loadu_si128((__m128i *)(CPB - 2)) :
		_mm_srl_si128(_mm_loadu_si128((__m128i *)(ctx->end - 16)), CPB - (ctx->end - 14));
	ctx->RBSP[0] = ctx->RBSP[1];
	unsigned eptb = _mm_movemask_epi8(_mm_cmpeq_epi8(x, _mm_setzero_si128()));
	x = _mm_srli_si128(x, 2);
	eptb &= eptb >> 1 & _mm_movemask_epi8(_mm_cmpgt_epi8(_mm_set1_epi8(4), x));
	
	// iterate on and remove every emulation_prevention_three_byte
	for (; eptb & (SIZE_BIT == 32 ? 0xf : 0xff); CPB++, eptb = (eptb & (eptb - 1)) >> 1) {
		int i = __builtin_ctz(eptb);
		
		// for a start code, stop at the last byte
		if (CPB[i] <3) {
			CPB += i - sizeof(size_t);
			break;
		}
		x = _mm_shuffle_epi8(x, (__m128i)shuf[i]);
	}
	CPB += sizeof(size_t);
	ctx->RBSP[1] = big_endian(((v16u)x)[0]); // FIXME: faster way than going through regs?
	ctx->CPB = CPB < ctx->end ? CPB : ctx->end;
	return ret;
}
#endif



__attribute__((noinline)) size_t get_u1() {
	return refill(ctx->shift + 1, ctx->RBSP[0] << ctx->shift >> (SIZE_BIT - 1));
}

__attribute__((noinline)) size_t get_uv(unsigned v) {
	size_t bits = lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift);
	return refill(ctx->shift + v, bits >> (SIZE_BIT - v));
}

// Parses Exp-Golomb codes up to 2^16-2
__attribute__((noinline)) size_t get_ue16() {
	size_t bits = lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift);
	unsigned v = clz(bits | (size_t)1 << (SIZE_BIT / 2)) * 2 + 1;
	return refill(ctx->shift + v, (bits >> (SIZE_BIT - v)) - 1);
}

// Parses Exp-Golomb codes up to 2^32-2
#if SIZE_BIT == 32
__attribute__((noinline)) size_t get_ue32() {
	size_t bits = lsd(ctx->RBSP[0], ctx->RBSP[1], ctx->shift);
	unsigned leadingZeroBits = clz(bits | 1);
	refill(ctx->shift + leadingZeroBits, 0);
	return get_uv(leadingZeroBits + 1) - 1;
}
#endif
