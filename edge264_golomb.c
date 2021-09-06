/**
 * Read Exp-Golomb codes and bit sequences.
 *
 * These critical functions are designed to contain minimal numbers of branches
 * and memory writes. Several approaches were tried before:
 * _ Removing all emulation_prevention_three_bytes beforehand to an
 *   intermediate buffer, then making 1~3 aligned reads. The first part incurs
 *   some complexity for the buffer management, but then the second part is
 *   simple and branchless (3 functions with fixed number of aligned reads for
 *   different maximum code sizes).
 * _ The same as above but with unaligned reads, to make 1 read for most codes.
 *   This part is frustrating, because it is more complex but still faster.
 * _ Removing emulation_prevention_three_bytes on the fly to a uint64_t cache
 *   with a shift count. This approach incurs a test for refill (predicted not
 *   taken), but spares a lot of buffer management code. However it incurs some
 *   complexity to reassemble codes bigger than 32 bits.
 * _ The same as above but with a size_t[2] cache with shift count. On 64-bit
 *   machines it allows reading ANY code size with very few instructions, and
 *   pushes the refill call at the end for tail call optimization.
 * _ The same size_t[2] buffer but with the shift replaced by a trailing set
 *   bit. It requires updating 2 variables instead of 1 (shift), but shortens
 *   all dependency chains and allows storing the whole context in two Global
 *   Register Variables.
 */

#include "edge264_common.h"


#ifdef __SSSE3__
noinline int FUNC(refill, int ret)
{
	static const uint8_t shr_data[32] = {
		 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	};
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
	
	// load 16 bytes without ever reading past the end
	const uint8_t *CPB = ctx->CPB;
	const uint8_t *end = ctx->end;
	const uint8_t *after;
	__m128i x;
	if (__builtin_expect(CPB + 14 <= end, 1)) {
		after = CPB + sizeof(size_t);
		x = _mm_loadu_si128((__m128i *)(CPB - 2));
	} else {
		after = CPB + sizeof(size_t) < end ? CPB + sizeof(size_t) : end;
		__m128i shr_mask = _mm_loadu_si128((__m128i *)(shr_data + (CPB + 14 - end)));
		x = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(end - 16)), shr_mask);
		x = vector_select(shr_mask, shr_mask, x); // turn every inserted 0 into -1
	}
	
	// create a bitmask for the positions of 00n escape sequences
	unsigned bytes0 = _mm_movemask_epi8(_mm_cmpeq_epi8(x, _mm_setzero_si128()));
	x = _mm_srli_si128(x, 2);
	unsigned bytes03 = _mm_movemask_epi8(_mm_cmpeq_epi8(x, _mm_min_epu8(x, _mm_set1_epi8(3))));
	unsigned esc = bytes0 & bytes0 >> 1 & bytes03;
	
	// iterate on escape sequences that fall inside the bytes to refill
	while (__builtin_expect(esc & (SIZE_BIT == 32 ? 0xf : 0xff), 0)) {
		int i = __builtin_ctz(esc);
		
		// when hitting a start code, point at the 3rd byte to stall future refills there
		if (CPB[i] <3) {
			after = CPB + i;
			break;
		}
		
		// otherwise this is an emulation_prevention_three_byte -> remove it
		x = _mm_shuffle_epi8(x, (__m128i)shuf[i]);
		esc = (esc & (esc - 1)) >> 1;
		CPB++;
		after = after + 1 < end ? after + 1 : end;
	}
	
	// increment CPB and insert the unescaped bytes into the cache
	ctx->CPB = after;
	int trailing_bit = ctz(msb_cache);
	int shift = trailing_bit + 1;
	size_t bytes = big_endian(((v16u)x)[0]);
	msb_cache = lsd(msb_cache >> shift, bytes, shift);
	lsb_cache = bytes << shift | (size_t)1 << trailing_bit;
	return ret;
}
#endif



noinline int FUNC(get_u1) {
	int ret = msb_cache >> (SIZE_BIT - 1);
	msb_cache = lsd(msb_cache, lsb_cache, 1);
	if (lsb_cache <<= 1)
		return ret;
	return CALL(refill, ret);
}

// Parses a 1~32-bit fixed size code
noinline unsigned FUNC(get_uv, unsigned v) {
	unsigned ret = msb_cache >> (SIZE_BIT - v);
	msb_cache = lsd(msb_cache, lsb_cache, v);
	if (lsb_cache <<= v)
		return ret;
	return CALL(refill, ret);
}

// Parses a Exp-Golomb code in one read, up to 2^16-2 (2^32-2 on 64-bit machines)
noinline unsigned FUNC(get_ue16, unsigned upper) {
	unsigned v = clz(msb_cache | (size_t)1 << (SIZE_BIT / 2)) * 2 + 1;
	unsigned ret = umin((msb_cache >> (SIZE_BIT - v)) - 1, upper);
	msb_cache = lsd(msb_cache, lsb_cache, v);
	if (lsb_cache <<= v)
		return ret;
	return CALL(refill, ret);
}

// Parses a signed Exp-Golomb code in one read, from -2^15+1 to 2^15-1 (-2^31+1 to 2^31-1 on 64-bit machines)
noinline int FUNC(get_se16, int lower, int upper) {
	unsigned v = clz(msb_cache | (size_t)1 << (SIZE_BIT / 2)) * 2 + 1;
	unsigned ue = (msb_cache >> (SIZE_BIT - v)) - 1;
	int ret = min(max((ue & 1) ? ue / 2 + 1 : -(ue / 2), lower), upper);
	msb_cache = lsd(msb_cache, lsb_cache, v);
	if (lsb_cache <<= v)
		return ret;
	return CALL(refill, ret);
}

// Extensions to [0,2^32-2] and [-2^31+1,2^31-1] for 32-bit machines
#if SIZE_BIT == 32
noinline unsigned FUNC(get_ue32, unsigned upper) {
	unsigned leadingZeroBits = clz(msb_cache | 1);
	msb_cache = lsd(msb_cache, lsb_cache, leadingZeroBits);
	if (!(lsb_cache <<= leadingZeroBits))
		CALL(refill, 0);
	return umin(CALL(get_uv, leadingZeroBits + 1) - 1, upper);
}

noinline int FUNC(get_se32, int lower, int upper) {
	unsigned leadingZeroBits = clz(msb_cache | 1);
	msb_cache = lsd(msb_cache, lsb_cache, leadingZeroBits);
	if (!(lsb_cache <<= leadingZeroBits))
		CALL(refill, 0);
	unsigned ue = CALL(get_uv, leadingZeroBits + 1) - 1;
	return min(max(ue & 1 ? ue / 2 + 1 : -(ue / 2), lower), upper);
}
#endif
