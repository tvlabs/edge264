#include "edge264_common.h"

static const v8hi bounds[6] = {
	{0x01ff, 0x01ff, 0x01ff, 0x01ff, 0x01ff, 0x01ff, 0x01ff, 0x01ff},
	{0x03ff, 0x03ff, 0x03ff, 0x03ff, 0x03ff, 0x03ff, 0x03ff, 0x03ff},
	{0x07ff, 0x07ff, 0x07ff, 0x07ff, 0x07ff, 0x07ff, 0x07ff, 0x07ff},
	{0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff},
	{0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff},
	{0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff},
};



/**
 * Inverse 4x4 transform
 *
 * 8/16bit share the same transform since an 8bit version would not bring much
 * speedup. Also the implementation matches the spec's pseudocode, avoiding
 * minor optimisations which would make it harder to understand.
 */
__attribute__((noinline)) int decode_Residual4x4(__m128i p0, __m128i p1)
{
	// scaling
	const __m128i *c = (__m128i *)ctx->residual_block;
	const __m128i *s = (__m128i *)ctx->LevelScale;
	__m128i s8 = _mm_set1_epi32(8);
	__m128i d0 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[0], s[0]), s8), 4);
	__m128i d1 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[1], s[1]), s8), 4);
	__m128i d2 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[2], s[2]), s8), 4);
	__m128i d3 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[3], s[3]), s8), 4);
	
	// horizontal 1D transform
	__m128i e0 = _mm_add_epi32(d0, d2);
	__m128i e1 = _mm_sub_epi32(d0, d2);
	__m128i e2 = _mm_sub_epi32(_mm_srai_epi32(d1, 1), d3);
	__m128i e3 = _mm_add_epi32(_mm_srai_epi32(d3, 1), d1);
	__m128i f0 = _mm_add_epi32(e0, e3);
	__m128i f1 = _mm_add_epi32(e1, e2);
	__m128i f2 = _mm_sub_epi32(e1, e2);
	__m128i f3 = _mm_sub_epi32(e0, e3);
	
	// matrix transposition
	__m128i x0 = _mm_unpacklo_epi32(f0, f1);
	__m128i x1 = _mm_unpacklo_epi32(f2, f3);
	__m128i x2 = _mm_unpackhi_epi32(f0, f1);
	__m128i x3 = _mm_unpackhi_epi32(f2, f3);
	f0 = _mm_add_epi32(_mm_unpacklo_epi64(x0, x1), _mm_set1_epi32(32));
	f1 = _mm_unpackhi_epi64(x0, x1);
	f2 = _mm_unpacklo_epi64(x2, x3);
	f3 = _mm_unpackhi_epi64(x2, x3);
	
	// vertical 1D transform
	__m128i g0 = _mm_add_epi32(f0, f2);
	__m128i g1 = _mm_sub_epi32(f0, f2);
	__m128i g2 = _mm_sub_epi32(_mm_srai_epi32(f1, 1), f3);
	__m128i g3 = _mm_add_epi32(_mm_srai_epi32(f3, 1), f1);
	__m128i h0 = _mm_add_epi32(g0, g3);
	__m128i h1 = _mm_add_epi32(g1, g2);
	__m128i h2 = _mm_sub_epi32(g1, g2);
	__m128i h3 = _mm_sub_epi32(g0, g3);
	
	// final residual values and addition to predicted samples
	__m128i r0 = _mm_packs_epi32(_mm_srai_epi32(h0, 6), _mm_srai_epi32(h1, 6));
	__m128i r1 = _mm_packs_epi32(_mm_srai_epi32(h2, 6), _mm_srai_epi32(h3, 6));
	__m128i x4 = _mm_adds_epi16(r0, p0);
	__m128i x5 = _mm_adds_epi16(r1, p1);
	
	// storage
	uint8_t *p = ctx->plane + ctx->plane_offsets[ctx->BlkIdx];
	size_t stride = ctx->stride;
	if (__builtin_expect(ctx->BitDepth == 8, 1)) {
		v4si u = (v4si)_mm_packus_epi16(x4, x5);
		*(int32_t *)(p + stride * 0) = u[0];
		*(int32_t *)(p + stride * 1) = u[1];
		*(int32_t *)(p + stride * 2) = u[2];
		*(int32_t *)(p + stride * 3) = u[3];
	} else {
		__m128i zero = _mm_setzero_si128();
		__m128i clip = (__m128i)bounds[ctx->BitDepth - 9];
		v2li u0 = (v2li)_mm_min_epi16(_mm_max_epi16(x4, zero), clip);
		v2li u1 = (v2li)_mm_min_epi16(_mm_max_epi16(x5, zero), clip);
		*(int64_t *)(p + stride * 0) = u0[0];
		*(int64_t *)(p + stride * 1) = u0[1];
		*(int64_t *)(p + stride * 2) = u1[0];
		*(int64_t *)(p + stride * 3) = u1[1];
	}
	return 0;
}



/**
 * Inverse 8x8 transform
 */
__attribute__((noinline)) int decode_Residual8x8(__m128i p0, __m128i p1,
	__m128i p2, __m128i p3, __m128i p4, __m128i p5, __m128i p6, __m128i p7)
{
	// scaling (if only pmulhrsw had variable shift...)
	const __m128i s32 = _mm_set1_epi32(32);
	const __m128i *c = (__m128i *)ctx->residual_block;
	const __m128i *s = (__m128i *)ctx->LevelScale;
	__m128i d8 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[1], s[1]), s32), 6);
	__m128i d9 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[3], s[3]), s32), 6);
	__m128i dA = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[5], s[5]), s32), 6);
	__m128i dB = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[7], s[7]), s32), 6);
	__m128i dC = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[9], s[9]), s32), 6);
	__m128i dD = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[11], s[11]), s32), 6);
	__m128i dE = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[13], s[13]), s32), 6);
	__m128i dF = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[15], s[15]), s32), 6);
	__m128i d0 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[0], s[0]), s32), 6);
	__m128i d1 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[2], s[2]), s32), 6);
	__m128i d2 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[4], s[4]), s32), 6);
	__m128i d3 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[6], s[6]), s32), 6);
	__m128i d4 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[8], s[8]), s32), 6);
	__m128i d5 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[10], s[10]), s32), 6);
	__m128i d6 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[12], s[12]), s32), 6);
	__m128i d7 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[14], s[14]), s32), 6);
	
	// The 8bit version can hold the whole transform in registers.
	uint8_t *p = ctx->plane + ctx->plane_offsets[ctx->BlkIdx];
	size_t stride = ctx->stride;
	if (__builtin_expect(ctx->BitDepth == 8, 1)) {
		d0 = _mm_packs_epi32(d0, d8);
		d1 = _mm_packs_epi32(d1, d9);
		d2 = _mm_packs_epi32(d2, dA);
		d3 = _mm_packs_epi32(d3, dB);
		d4 = _mm_packs_epi32(d4, dC);
		d5 = _mm_packs_epi32(d5, dD);
		d6 = _mm_packs_epi32(d6, dE);
		d7 = _mm_packs_epi32(d7, dF);
		
		for (int i = 2;;) {
			// 1D transform
			__m128i e0 = _mm_add_epi16(d0, d4);
			__m128i e1 = _mm_sub_epi16(_mm_sub_epi16(d5, d3), _mm_add_epi16(_mm_srai_epi16(d7, 1), d7));
			__m128i e2 = _mm_sub_epi16(d0, d4);
			__m128i e3 = _mm_sub_epi16(_mm_add_epi16(d1, d7), _mm_add_epi16(_mm_srai_epi16(d3, 1), d3));
			__m128i e4 = _mm_sub_epi16(_mm_srai_epi16(d2, 1), d6);
			__m128i e5 = _mm_add_epi16(_mm_sub_epi16(d7, d1), _mm_add_epi16(_mm_srai_epi16(d5, 1), d5));
			__m128i e6 = _mm_add_epi16(_mm_srai_epi16(d6, 1), d2);
			__m128i e7 = _mm_add_epi16(_mm_add_epi16(d3, d5), _mm_add_epi16(_mm_srai_epi16(d1, 1), d1));
			__m128i f0 = _mm_add_epi16(e0, e6);
			__m128i f1 = _mm_add_epi16(_mm_srai_epi16(e7, 2), e1);
			__m128i f2 = _mm_add_epi16(e2, e4);
			__m128i f3 = _mm_add_epi16(_mm_srai_epi16(e5, 2), e3);
			__m128i f4 = _mm_sub_epi16(e2, e4);
			__m128i f5 = _mm_sub_epi16(_mm_srai_epi16(e3, 2), e5);
			__m128i f6 = _mm_sub_epi16(e0, e6);
			__m128i f7 = _mm_sub_epi16(e7, _mm_srai_epi16(e1, 2));
			
			// Compilers freak out whenever output uses other registers.
			d0 = _mm_add_epi16(f0, f7);
			d1 = _mm_add_epi16(f2, f5);
			d2 = _mm_add_epi16(f4, f3);
			d3 = _mm_add_epi16(f6, f1);
			d4 = _mm_sub_epi16(f6, f1);
			d5 = _mm_sub_epi16(f4, f3);
			d6 = _mm_sub_epi16(f2, f5);
			d7 = _mm_sub_epi16(f0, f7);
			
			// matrix transposition
			if (--i == 0)
				break;
			__m128i x0 = _mm_unpacklo_epi16(d0, d1);
			__m128i x1 = _mm_unpacklo_epi16(d2, d3);
			__m128i x2 = _mm_unpacklo_epi16(d4, d5);
			__m128i x3 = _mm_unpacklo_epi16(d6, d7);
			__m128i x4 = _mm_unpackhi_epi16(d0, d1);
			__m128i x5 = _mm_unpackhi_epi16(d2, d3);
			__m128i x6 = _mm_unpackhi_epi16(d4, d5);
			__m128i x7 = _mm_unpackhi_epi16(d6, d7);
			__m128i x8 = _mm_unpacklo_epi32(x0, x1);
			__m128i x9 = _mm_unpacklo_epi32(x2, x3);
			__m128i xA = _mm_unpacklo_epi32(x4, x5);
			__m128i xB = _mm_unpacklo_epi32(x6, x7);
			__m128i xC = _mm_unpackhi_epi32(x0, x1);
			__m128i xD = _mm_unpackhi_epi32(x2, x3);
			__m128i xE = _mm_unpackhi_epi32(x4, x5);
			__m128i xF = _mm_unpackhi_epi32(x6, x7);
			d0 = _mm_add_epi16(_mm_unpacklo_epi64(x8, x9), _mm_set1_epi16(32));
			d1 = _mm_unpackhi_epi64(x8, x9);
			d2 = _mm_unpacklo_epi64(xA, xB);
			d3 = _mm_unpackhi_epi64(xA, xB);
			d4 = _mm_unpacklo_epi64(xC, xD);
			d5 = _mm_unpackhi_epi64(xC, xD);
			d6 = _mm_unpacklo_epi64(xE, xF);
			d7 = _mm_unpackhi_epi64(xE, xF);
		}
		
		// final residual values and addition to predicted samples
		__m128i r0 = _mm_srai_epi16(d0, 6);
		__m128i r1 = _mm_srai_epi16(d1, 6);
		__m128i r2 = _mm_srai_epi16(d2, 6);
		__m128i r3 = _mm_srai_epi16(d3, 6);
		__m128i r4 = _mm_srai_epi16(d4, 6);
		__m128i r5 = _mm_srai_epi16(d5, 6);
		__m128i r6 = _mm_srai_epi16(d6, 6);
		__m128i r7 = _mm_srai_epi16(d7, 6);
		v2li u0 = (v2li)_mm_packus_epi16(_mm_adds_epi16(r0, p0), _mm_adds_epi16(r1, p1));
		v2li u1 = (v2li)_mm_packus_epi16(_mm_adds_epi16(r2, p2), _mm_adds_epi16(r3, p3));
		v2li u2 = (v2li)_mm_packus_epi16(_mm_adds_epi16(r4, p4), _mm_adds_epi16(r5, p5));
		v2li u3 = (v2li)_mm_packus_epi16(_mm_adds_epi16(r6, p6), _mm_adds_epi16(r7, p7));
		
		// storage
		*(int64_t *)(p + stride * 0) = u0[0];
		*(int64_t *)(p + stride * 1) = u0[1];
		*(int64_t *)(p + stride * 2) = u1[0];
		*(int64_t *)(p + stride * 3) = u1[1];
		*(int64_t *)(p + stride * 4) = u2[0];
		*(int64_t *)(p + stride * 5) = u2[1];
		*(int64_t *)(p + stride * 6) = u3[0];
		*(int64_t *)(p + stride * 7) = u3[1];
	
	// The 16bit version uses a nested loop trick to reduce code size
	} else {
		for (int i = 2;; ) {
			for (int j = 2;; ) {
				// 1D transform
				__m128i e0 = _mm_add_epi32(d0, d4);
				__m128i e1 = _mm_sub_epi32(_mm_sub_epi32(d5, d3), _mm_add_epi32(_mm_srai_epi32(d7, 1), d7));
				__m128i e2 = _mm_sub_epi32(d0, d4);
				__m128i e3 = _mm_sub_epi32(_mm_add_epi32(d1, d7), _mm_add_epi32(_mm_srai_epi32(d3, 1), d3));
				__m128i e4 = _mm_sub_epi32(_mm_srai_epi32(d2, 1), d6);
				__m128i e5 = _mm_add_epi32(_mm_sub_epi32(d7, d1), _mm_add_epi32(_mm_srai_epi32(d5, 1), d5));
				__m128i e6 = _mm_add_epi32(_mm_srai_epi32(d6, 1), d2);
				__m128i e7 = _mm_add_epi32(_mm_add_epi32(d3, d5), _mm_add_epi32(_mm_srai_epi32(d1, 1), d1));
				__m128i f0 = _mm_add_epi32(e0, e6);
				__m128i f1 = _mm_add_epi32(_mm_srai_epi32(e7, 2), e1);
				__m128i f2 = _mm_add_epi32(e2, e4);
				__m128i f3 = _mm_add_epi32(_mm_srai_epi32(e5, 2), e3);
				__m128i f4 = _mm_sub_epi32(e2, e4);
				__m128i f5 = _mm_sub_epi32(_mm_srai_epi32(e3, 2), e5);
				__m128i f6 = _mm_sub_epi32(e0, e6);
				__m128i f7 = _mm_sub_epi32(e7, _mm_srai_epi32(e1, 2));
				d0 = _mm_add_epi32(f0, f7);
				d1 = _mm_add_epi32(f2, f5);
				d2 = _mm_add_epi32(f4, f3);
				d3 = _mm_add_epi32(f6, f1);
				d4 = _mm_sub_epi32(f6, f1);
				d5 = _mm_sub_epi32(f4, f3);
				d6 = _mm_sub_epi32(f2, f5);
				d7 = _mm_sub_epi32(f0, f7);
				
				// matrix swap
				if (--j == 0)
					break;
				__m128i x0 = d8; d8 = d0; d0 = x0;
				__m128i x1 = d9; d9 = d1; d1 = x1;
				__m128i x2 = dA; dA = d2; d2 = x2;
				__m128i x3 = dB; dB = d3; d3 = x3;
				__m128i x4 = dC; dC = d4; d4 = x4;
				__m128i x5 = dD; dD = d5; d5 = x5;
				__m128i x6 = dE; dE = d6; d6 = x6;
				__m128i x7 = dF; dF = d7; d7 = x7;
			}
			
			// matrix swap, transpose and swap again
			if (--i == 0)
				break;
			__m128i x8 = _mm_unpacklo_epi32(d0, d1);
			__m128i x9 = _mm_unpacklo_epi32(d2, d3);
			__m128i xA = _mm_unpackhi_epi32(d0, d1);
			__m128i xB = _mm_unpackhi_epi32(d2, d3);
			__m128i xC = _mm_unpacklo_epi32(d4, d5);
			__m128i xD = _mm_unpacklo_epi32(d6, d7);
			__m128i xE = _mm_unpackhi_epi32(d4, d5);
			__m128i xF = _mm_unpackhi_epi32(d6, d7);
			__m128i x0 = _mm_unpacklo_epi32(d8, d9);
			__m128i x1 = _mm_unpacklo_epi32(dA, dB);
			__m128i x2 = _mm_unpackhi_epi32(d8, d9);
			__m128i x3 = _mm_unpackhi_epi32(dA, dB);
			__m128i x4 = _mm_unpacklo_epi32(dC, dD);
			__m128i x5 = _mm_unpacklo_epi32(dE, dF);
			__m128i x6 = _mm_unpackhi_epi32(dC, dD);
			__m128i x7 = _mm_unpackhi_epi32(dE, dF);
			d8 = _mm_add_epi32(_mm_unpacklo_epi64(x0, x1), s32);
			d9 = _mm_unpackhi_epi64(x0, x1);
			dA = _mm_unpacklo_epi64(x2, x3);
			dB = _mm_unpackhi_epi64(x2, x3);
			dC = _mm_unpacklo_epi64(x8, x9);
			dD = _mm_unpackhi_epi64(x8, x9);
			dE = _mm_unpacklo_epi64(xA, xB);
			dF = _mm_unpackhi_epi64(xA, xB);
			d0 = _mm_add_epi32(_mm_unpacklo_epi64(x4, x5), s32);
			d1 = _mm_unpackhi_epi64(x4, x5);
			d2 = _mm_unpacklo_epi64(x6, x7);
			d3 = _mm_unpackhi_epi64(x6, x7);
			d4 = _mm_unpacklo_epi64(xC, xD);
			d5 = _mm_unpackhi_epi64(xC, xD);
			d6 = _mm_unpacklo_epi64(xE, xF);
			d7 = _mm_unpackhi_epi64(xE, xF);
		}
		
		// final constructed residual sample values
		__m128i r0 = _mm_packs_epi32(_mm_srai_epi32(d0, 6), _mm_srai_epi32(d8, 6));
		__m128i r1 = _mm_packs_epi32(_mm_srai_epi32(d1, 6), _mm_srai_epi32(d9, 6));
		__m128i r2 = _mm_packs_epi32(_mm_srai_epi32(d2, 6), _mm_srai_epi32(dA, 6));
		__m128i r3 = _mm_packs_epi32(_mm_srai_epi32(d3, 6), _mm_srai_epi32(dB, 6));
		__m128i r4 = _mm_packs_epi32(_mm_srai_epi32(d4, 6), _mm_srai_epi32(dC, 6));
		__m128i r5 = _mm_packs_epi32(_mm_srai_epi32(d5, 6), _mm_srai_epi32(dD, 6));
		__m128i r6 = _mm_packs_epi32(_mm_srai_epi32(d6, 6), _mm_srai_epi32(dE, 6));
		__m128i r7 = _mm_packs_epi32(_mm_srai_epi32(d7, 6), _mm_srai_epi32(dF, 6));
		
		// addition to predicted values, clipping and storage
		__m128i zero = _mm_setzero_si128();
		__m128i clip = (__m128i)bounds[ctx->BitDepth - 9];
		__m128i u0 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(r0, p0), zero), clip);
		__m128i u1 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(r1, p1), zero), clip);
		__m128i u2 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(r2, p2), zero), clip);
		__m128i u3 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(r3, p3), zero), clip);
		__m128i u4 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(r4, p4), zero), clip);
		__m128i u5 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(r5, p5), zero), clip);
		__m128i u6 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(r6, p6), zero), clip);
		__m128i u7 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(r7, p7), zero), clip);
		*(__m128i *)(p + stride * 0) = u0;
		*(__m128i *)(p + stride * 1) = u1;
		*(__m128i *)(p + stride * 2) = u2;
		*(__m128i *)(p + stride * 3) = u3;
		*(__m128i *)(p + stride * 4) = u4;
		*(__m128i *)(p + stride * 5) = u5;
		*(__m128i *)(p + stride * 6) = u6;
		*(__m128i *)(p + stride * 7) = u7;
	}
	return 0;
}
