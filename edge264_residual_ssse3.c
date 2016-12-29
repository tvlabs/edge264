#include "edge264_common.h"



/**
 * Inverse 4x4 transform
 *
 * 8/16bit share the same transform since an 8bit version would not bring much
 * speedup. Also the implementation matches the spec's pseudocode, avoiding
 * minor optimisations which would make it harder to understand.
 */
__attribute__((noinline)) int decode_Residual4x4(uint8_t *p, size_t stride, __m128i p0, __m128i p1)
{
	static const v8hi bounds[6] = {
		{0x01ff, 0x01ff, 0x01ff, 0x01ff, 0x01ff, 0x01ff, 0x01ff, 0x01ff},
		{0x03ff, 0x03ff, 0x03ff, 0x03ff, 0x03ff, 0x03ff, 0x03ff, 0x03ff},
		{0x07ff, 0x07ff, 0x07ff, 0x07ff, 0x07ff, 0x07ff, 0x07ff, 0x07ff},
		{0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff, 0x0fff},
		{0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff, 0x1fff},
		{0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff, 0x3fff},
	};
	
	__m128i c0 = (__m128i)ctx->residual_block[0];
	__m128i c1 = (__m128i)ctx->residual_block[1];
	__m128i c2 = (__m128i)ctx->residual_block[2];
	__m128i c3 = (__m128i)ctx->residual_block[3];
	
	// scaling
	__m128i x0 = _mm_mullo_epi32(c0, (__m128i)ctx->LevelScale[0]);
	__m128i x1 = _mm_mullo_epi32(c1, (__m128i)ctx->LevelScale[1]);
	__m128i x2 = _mm_mullo_epi32(c2, (__m128i)ctx->LevelScale[2]);
	__m128i x3 = _mm_mullo_epi32(c3, (__m128i)ctx->LevelScale[3]);
	__m128i s8 = _mm_set1_epi32(8);
	__m128i d0 = _mm_srai_epi32(_mm_add_epi32(x0, s8), 4);
	__m128i d1 = _mm_srai_epi32(_mm_add_epi32(x1, s8), 4);
	__m128i d2 = _mm_srai_epi32(_mm_add_epi32(x2, s8), 4);
	__m128i d3 = _mm_srai_epi32(_mm_add_epi32(x3, s8), 4);
	
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
	__m128i x4 = _mm_unpacklo_epi32(f0, f1);
	__m128i x5 = _mm_unpacklo_epi32(f2, f3);
	__m128i x6 = _mm_unpackhi_epi32(f0, f1);
	__m128i x7 = _mm_unpackhi_epi32(f2, f3);
	f0 = _mm_add_epi32(_mm_unpacklo_epi64(x4, x5), _mm_set1_epi32(32));
	f1 = _mm_unpackhi_epi64(x4, x5);
	f2 = _mm_unpacklo_epi64(x6, x7);
	f3 = _mm_unpackhi_epi64(x6, x7);
	
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
	__m128i r0 = _mm_srai_epi32(h0, 6);
	__m128i r1 = _mm_srai_epi32(h1, 6);
	__m128i r2 = _mm_srai_epi32(h2, 6);
	__m128i r3 = _mm_srai_epi32(h3, 6);
	__m128i x8 = _mm_adds_epi16(_mm_packs_epi32(r0, r1), p0);
	__m128i x9 = _mm_adds_epi16(_mm_packs_epi32(r2, r3), p1);
	
	// storage
	if (__builtin_expect(ctx->BitDepth == 8, 1)) {
		v4si u = (v4si)_mm_packus_epi16(x8, x9);
		*(int32_t *)(p + stride * 0) = u[0];
		*(int32_t *)(p + stride * 1) = u[1];
		*(int32_t *)(p + stride * 2) = u[2];
		*(int32_t *)(p + stride * 3) = u[3];
	} else {
		__m128i zero = _mm_setzero_si128();
		__m128i clip = (__m128i)bounds[ctx->BitDepth - 9];
		v2li u0 = (v2li)_mm_min_epi16(_mm_max_epi16(x8, zero), clip);
		v2li u1 = (v2li)_mm_min_epi16(_mm_max_epi16(x9, zero), clip);
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
/*static void decode_Residual8x8(const Decode_ctx *d, const Part_ctx *p,
	__m128i p0, __m128i p1, __m128i p2, __m128i p3, __m128i p4, __m128i p5,
	__m128i p6, __m128i p7)
{
	const __m128i *c = (__m128i *)p->c, *s = (__m128i *)d->scale;
	__m128i r0, r1, r2, r3, r4, r5, r6, r7;
	if (__builtin_expect(d->TransformBypassModeFlag, 0)) {
		r0 = _mm_packs_epi32(c[0], c[1]);
		r1 = _mm_packs_epi32(c[2], c[3]);
		r2 = _mm_packs_epi32(c[4], c[5]);
		r3 = _mm_packs_epi32(c[6], c[7]);
		r4 = _mm_packs_epi32(c[8], c[9]);
		r5 = _mm_packs_epi32(c[10], c[11]);
		r6 = _mm_packs_epi32(c[12], c[13]);
		r7 = _mm_packs_epi32(c[14], c[15]);
	} else {
		
		// scaling (if only pmulhrsw had variable shift...)
		__m128i s32 = _mm_set1_epi32(32);
		__m128i A0 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[1], s[1]), s32), 6);
		__m128i A1 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[3], s[3]), s32), 6);
		__m128i A2 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[5], s[5]), s32), 6);
		__m128i A3 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[7], s[7]), s32), 6);
		__m128i A4 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[9], s[9]), s32), 6);
		__m128i A5 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[11], s[11]), s32), 6);
		__m128i A6 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[13], s[13]), s32), 6);
		__m128i A7 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[15], s[15]), s32), 6);
		__m128i a0 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[0], s[0]), s32), 6);
		__m128i a1 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[2], s[2]), s32), 6);
		__m128i a2 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[4], s[4]), s32), 6);
		__m128i a3 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[6], s[6]), s32), 6);
		__m128i a4 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[8], s[8]), s32), 6);
		__m128i a5 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[10], s[10]), s32), 6);
		__m128i a6 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[12], s[12]), s32), 6);
		__m128i a7 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(c[14], s[14]), s32), 6);
		
		// The 8bit version can hold the whole transform in registers.
		if (d->BitDepth == 8) {
			__m128i d0 = _mm_packs_epi32(a0, A0);
			__m128i d1 = _mm_packs_epi32(a1, A1);
			__m128i d2 = _mm_packs_epi32(a2, A2);
			__m128i d3 = _mm_packs_epi32(a3, A3);
			__m128i d4 = _mm_packs_epi32(a4, A4);
			__m128i d5 = _mm_packs_epi32(a5, A5);
			__m128i d6 = _mm_packs_epi32(a6, A6);
			__m128i d7 = _mm_packs_epi32(a7, A7);
			
			for (int i = 2; i-- > 0; ) {
				// 1D transform
				__m128i e0 = _mm_add_epi16(d0, d4);
				__m128i e1 = _mm_sub_epi16(_mm_sub_epi16(d5, d3),
					_mm_add_epi16(_mm_srai_epi16(d7, 1), d7));
				__m128i e2 = _mm_sub_epi16(d0, d4);
				__m128i e3 = _mm_sub_epi16(_mm_add_epi16(d1, d7),
					_mm_add_epi16(_mm_srai_epi16(d3, 1), d3));
				__m128i e4 = _mm_sub_epi16(_mm_srai_epi16(d2, 1), d6);
				__m128i e5 = _mm_add_epi16(_mm_sub_epi16(d7, d1),
					_mm_add_epi16(_mm_srai_epi16(d5, 1), d5));
				__m128i e6 = _mm_add_epi16(_mm_srai_epi16(d6, 1), d2);
				__m128i e7 = _mm_add_epi16(_mm_add_epi16(d3, d5),
					_mm_add_epi16(_mm_srai_epi16(d1, 1), d1));
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
				
				// transposition
				if (i > 0) {
					__m128i b0 = _mm_unpacklo_epi16(d0, d1);
					__m128i b1 = _mm_unpacklo_epi16(d2, d3);
					__m128i b2 = _mm_unpacklo_epi16(d4, d5);
					__m128i b3 = _mm_unpacklo_epi16(d6, d7);
					__m128i b4 = _mm_unpackhi_epi16(d0, d1);
					__m128i b5 = _mm_unpackhi_epi16(d2, d3);
					__m128i b6 = _mm_unpackhi_epi16(d4, d5);
					__m128i b7 = _mm_unpackhi_epi16(d6, d7);
					__m128i c0 = _mm_unpacklo_epi32(b0, b1);
					__m128i c1 = _mm_unpacklo_epi32(b2, b3);
					__m128i c2 = _mm_unpacklo_epi32(b4, b5);
					__m128i c3 = _mm_unpacklo_epi32(b6, b7);
					__m128i c4 = _mm_unpackhi_epi32(b0, b1);
					__m128i c5 = _mm_unpackhi_epi32(b2, b3);
					__m128i c6 = _mm_unpackhi_epi32(b4, b5);
					__m128i c7 = _mm_unpackhi_epi32(b6, b7);
					d0 = _mm_add_epi16(_mm_unpacklo_epi64(c0, c1), _mm_set1_epi16(32));
					d1 = _mm_unpackhi_epi64(c0, c1);
					d2 = _mm_unpacklo_epi64(c2, c3);
					d3 = _mm_unpackhi_epi64(c2, c3);
					d4 = _mm_unpacklo_epi64(c4, c5);
					d5 = _mm_unpackhi_epi64(c4, c5);
					d6 = _mm_unpacklo_epi64(c6, c7);
					d7 = _mm_unpackhi_epi64(c6, c7);
				}
			}
			
			// final constructed residual sample values
			r0 = _mm_srai_epi16(d0, 6);
			r1 = _mm_srai_epi16(d1, 6);
			r2 = _mm_srai_epi16(d2, 6);
			r3 = _mm_srai_epi16(d3, 6);
			r4 = _mm_srai_epi16(d4, 6);
			r5 = _mm_srai_epi16(d5, 6);
			r6 = _mm_srai_epi16(d6, 6);
			r7 = _mm_srai_epi16(d7, 6);
		} else {
			for (int i = 2; i-- > 0; ) {
				for (int j = 2; j-- > 0; ) {
					// 1D transform
					__m128i e0 = _mm_add_epi32(a0, a4);
					__m128i e1 = _mm_sub_epi32(_mm_sub_epi32(a5, a3),
						_mm_add_epi32(_mm_srai_epi32(a7, 1), a7));
					__m128i e2 = _mm_sub_epi32(a0, a4);
					__m128i e3 = _mm_sub_epi32(_mm_add_epi32(a1, a7),
						_mm_add_epi32(_mm_srai_epi32(a3, 1), a3));
					__m128i e4 = _mm_sub_epi32(_mm_srai_epi32(a2, 1), a6);
					__m128i e5 = _mm_add_epi32(_mm_sub_epi32(a7, a1),
						_mm_add_epi32(_mm_srai_epi32(a5, 1), a5));
					__m128i e6 = _mm_add_epi32(_mm_srai_epi32(a6, 1), a2);
					__m128i e7 = _mm_add_epi32(_mm_add_epi32(a3, a5),
						_mm_add_epi32(_mm_srai_epi32(a1, 1), a1));
					__m128i f0 = _mm_add_epi32(e0, e6);
					__m128i f1 = _mm_add_epi32(_mm_srai_epi32(e7, 2), e1);
					__m128i f2 = _mm_add_epi32(e2, e4);
					__m128i f3 = _mm_add_epi32(_mm_srai_epi32(e5, 2), e3);
					__m128i f4 = _mm_sub_epi32(e2, e4);
					__m128i f5 = _mm_sub_epi32(_mm_srai_epi32(e3, 2), e5);
					__m128i f6 = _mm_sub_epi32(e0, e6);
					__m128i f7 = _mm_sub_epi32(e7, _mm_srai_epi32(e1, 2));
					a0 = _mm_add_epi32(f0, f7);
					a1 = _mm_add_epi32(f2, f5);
					a2 = _mm_add_epi32(f4, f3);
					a3 = _mm_add_epi32(f6, f1);
					a4 = _mm_sub_epi32(f6, f1);
					a5 = _mm_sub_epi32(f4, f3);
					a6 = _mm_sub_epi32(f2, f5);
					a7 = _mm_sub_epi32(f0, f7);
					
					// swap sides
					if (j > 0) {
						__m128i b0 = A0; A0 = a0; a0 = b0;
						__m128i b1 = A1; A1 = a1; a1 = b1;
						__m128i b2 = A2; A2 = a2; a2 = b2;
						__m128i b3 = A3; A3 = a3; a3 = b3;
						__m128i b4 = A4; A4 = a4; a4 = b4;
						__m128i b5 = A5; A5 = a5; a5 = b5;
						__m128i b6 = A6; A6 = a6; a6 = b6;
						__m128i b7 = A7; A7 = a7; a7 = b7;
					}
				}
				
				// transposition
				if (i > 0) {
					__m128i B0 = _mm_unpacklo_epi32(A4, A5);
					__m128i B1 = _mm_unpacklo_epi32(A6, A7);
					__m128i B2 = _mm_unpackhi_epi32(A4, A5);
					__m128i B3 = _mm_unpackhi_epi32(A6, A7);
					__m128i B4 = _mm_unpacklo_epi32(a4, a5);
					__m128i B5 = _mm_unpacklo_epi32(a6, a7);
					__m128i B6 = _mm_unpackhi_epi32(a4, a5);
					__m128i B7 = _mm_unpackhi_epi32(a6, a7);
					__m128i b0 = _mm_unpacklo_epi32(A0, A1);
					__m128i b1 = _mm_unpacklo_epi32(A2, A3);
					__m128i b2 = _mm_unpackhi_epi32(A0, A1);
					__m128i b3 = _mm_unpackhi_epi32(A2, A3);
					__m128i b4 = _mm_unpacklo_epi32(a0, a1);
					__m128i b5 = _mm_unpacklo_epi32(a2, a3);
					__m128i b6 = _mm_unpackhi_epi32(a0, a1);
					__m128i b7 = _mm_unpackhi_epi32(a2, a3);
					A0 = _mm_add_epi32(_mm_unpacklo_epi64(B0, B1), s32);
					A1 = _mm_unpackhi_epi64(B0, B1);
					A2 = _mm_unpacklo_epi64(B2, B3);
					A3 = _mm_unpackhi_epi64(B2, B3);
					A4 = _mm_unpacklo_epi64(B4, B5);
					A5 = _mm_unpackhi_epi64(B4, B5);
					A6 = _mm_unpacklo_epi64(B6, B7);
					A7 = _mm_unpackhi_epi64(B6, B7);
					a0 = _mm_add_epi32(_mm_unpacklo_epi64(b0, b1), s32);
					a1 = _mm_unpackhi_epi64(b0, b1);
					a2 = _mm_unpacklo_epi64(b2, b3);
					a3 = _mm_unpackhi_epi64(b2, b3);
					a4 = _mm_unpacklo_epi64(b4, b5);
					a5 = _mm_unpackhi_epi64(b4, b5);
					a6 = _mm_unpacklo_epi64(b6, b7);
					a7 = _mm_unpackhi_epi64(b6, b7);
				}
			}
			
			// final constructed residual sample values
			r0 = _mm_packs_epi32(_mm_srai_epi32(a0, 6), _mm_srai_epi32(A0, 6));
			r1 = _mm_packs_epi32(_mm_srai_epi32(a1, 6), _mm_srai_epi32(A1, 6));
			r2 = _mm_packs_epi32(_mm_srai_epi32(a2, 6), _mm_srai_epi32(A2, 6));
			r3 = _mm_packs_epi32(_mm_srai_epi32(a3, 6), _mm_srai_epi32(A3, 6));
			r4 = _mm_packs_epi32(_mm_srai_epi32(a4, 6), _mm_srai_epi32(A4, 6));
			r5 = _mm_packs_epi32(_mm_srai_epi32(a5, 6), _mm_srai_epi32(A5, 6));
			r6 = _mm_packs_epi32(_mm_srai_epi32(a6, 6), _mm_srai_epi32(A6, 6));
			r7 = _mm_packs_epi32(_mm_srai_epi32(a7, 6), _mm_srai_epi32(A7, 6));
		}
	}
	
	// addition to predicted values, clipping and storage
	__m128i c0 = _mm_adds_epi16(r0, p0);
	__m128i c1 = _mm_adds_epi16(r1, p1);
	__m128i c2 = _mm_adds_epi16(r2, p2);
	__m128i c3 = _mm_adds_epi16(r3, p3);
	__m128i c4 = _mm_adds_epi16(r4, p4);
	__m128i c5 = _mm_adds_epi16(r5, p5);
	__m128i c6 = _mm_adds_epi16(r6, p6);
	__m128i c7 = _mm_adds_epi16(r7, p7);
	__m128i clip = _mm_set1_epi32((0x00010001 << d->BitDepth) - 0x00010001);
	__m128i zero = _mm_setzero_si128();
	uint16_t (*q)[d->stride] = (uint16_t (*)[d->stride])p->dst;
	*(__m128i *)q[0] = _mm_max_epi16(_mm_min_epi16(c0, clip), zero);
	*(__m128i *)q[1] = _mm_max_epi16(_mm_min_epi16(c1, clip), zero);
	*(__m128i *)q[2] = _mm_max_epi16(_mm_min_epi16(c2, clip), zero);
	*(__m128i *)q[3] = _mm_max_epi16(_mm_min_epi16(c3, clip), zero);
	*(__m128i *)q[4] = _mm_max_epi16(_mm_min_epi16(c4, clip), zero);
	*(__m128i *)q[5] = _mm_max_epi16(_mm_min_epi16(c5, clip), zero);
	*(__m128i *)q[6] = _mm_max_epi16(_mm_min_epi16(c6, clip), zero);
	*(__m128i *)q[7] = _mm_max_epi16(_mm_min_epi16(c7, clip), zero);
}*/
