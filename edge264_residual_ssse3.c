#include "edge264_common.h"



/**
 * These functions are not executed often enough to justify maintaining AVX2
 * versions.
 */
static const v16qi normAdjust4x4[6] = { // shared with decode_ResidualDC2x4
	10, 13, 10, 13, 13, 16, 13, 16, 10, 13, 10, 13, 13, 16, 13, 16,
	11, 14, 11, 14, 14, 18, 14, 18, 11, 14, 11, 14, 14, 18, 14, 18,
	13, 16, 13, 16, 16, 20, 16, 20, 13, 16, 13, 16, 16, 20, 16, 20,
	14, 18, 14, 18, 18, 23, 18, 23, 14, 18, 14, 18, 18, 23, 18, 23,
	16, 20, 16, 20, 20, 25, 20, 25, 16, 20, 16, 20, 20, 25, 20, 25,
	18, 23, 18, 23, 23, 29, 23, 29, 18, 23, 18, 23, 23, 29, 23, 29,
};

__attribute__((noinline)) void FUNC(compute_LevelScale4x4, int iYCbCr) {
	// This part is very unsatisfactory and will have to be optimised some day.
	int BitDepth = (iYCbCr == 0) ? ctx->ps.BitDepth_Y : ctx->ps.BitDepth_C;
	__m128i zero = _mm_setzero_si128();
	__m128i shift = _mm_cvtsi32_si128((unsigned)mb->QP[iYCbCr] / 6 + BitDepth - 6); // QP' / 6 + 2
	__m128i nA = (__m128i)normAdjust4x4[(unsigned)mb->QP[iYCbCr] % 6];
	__m128i w = ((__m128i *)ctx->ps.weightScale4x4)[iYCbCr + mb->f.mbIsInterFlag * 3];
	__m128i x0 = _mm_mullo_epi16(_mm_unpacklo_epi8(w, zero), _mm_unpacklo_epi8(nA, zero));
	__m128i x1 = _mm_mullo_epi16(_mm_unpackhi_epi8(w, zero), _mm_unpackhi_epi8(nA, zero));
	ctx->LevelScale_v[0] = (v4si)_mm_sll_epi32(_mm_unpacklo_epi16(x0, zero), shift);
	ctx->LevelScale_v[1] = (v4si)_mm_sll_epi32(_mm_unpackhi_epi16(x0, zero), shift);
	ctx->LevelScale_v[2] = (v4si)_mm_sll_epi32(_mm_unpacklo_epi16(x1, zero), shift);
	ctx->LevelScale_v[3] = (v4si)_mm_sll_epi32(_mm_unpackhi_epi16(x1, zero), shift);
}

__attribute__((noinline)) void FUNC(compute_LevelScale8x8, int iYCbCr) {
	static const v16qi normAdjust8x8[6][4] = {
		20, 19, 25, 19, 20, 19, 25, 19, 19, 18, 24, 18, 19, 18, 24, 18,
		25, 24, 32, 24, 25, 24, 32, 24, 19, 18, 24, 18, 19, 18, 24, 18,
		20, 19, 25, 19, 20, 19, 25, 19, 19, 18, 24, 18, 19, 18, 24, 18,
		25, 24, 32, 24, 25, 24, 32, 24, 19, 18, 24, 18, 19, 18, 24, 18,
		22, 21, 28, 21, 22, 21, 28, 21, 21, 19, 26, 19, 21, 19, 26, 19,
		28, 26, 35, 26, 28, 26, 35, 26, 21, 19, 26, 19, 21, 19, 26, 19,
		22, 21, 28, 21, 22, 21, 28, 21, 21, 19, 26, 19, 21, 19, 26, 19,
		28, 26, 35, 26, 28, 26, 35, 26, 21, 19, 26, 19, 21, 19, 26, 19,
		26, 24, 33, 24, 26, 24, 33, 24, 24, 23, 31, 23, 24, 23, 31, 23,
		33, 31, 42, 31, 33, 31, 42, 31, 24, 23, 31, 23, 24, 23, 31, 23,
		26, 24, 33, 24, 26, 24, 33, 24, 24, 23, 31, 23, 24, 23, 31, 23,
		33, 31, 42, 31, 33, 31, 42, 31, 24, 23, 31, 23, 24, 23, 31, 23,
		28, 26, 35, 26, 28, 26, 35, 26, 26, 25, 33, 25, 26, 25, 33, 25,
		35, 33, 45, 33, 35, 33, 45, 33, 26, 25, 33, 25, 26, 25, 33, 25,
		28, 26, 35, 26, 28, 26, 35, 26, 26, 25, 33, 25, 26, 25, 33, 25,
		35, 33, 45, 33, 35, 33, 45, 33, 26, 25, 33, 25, 26, 25, 33, 25,
		32, 30, 40, 30, 32, 30, 40, 30, 30, 28, 38, 28, 30, 28, 38, 28,
		40, 38, 51, 38, 40, 38, 51, 38, 30, 28, 38, 28, 30, 28, 38, 28,
		32, 30, 40, 30, 32, 30, 40, 30, 30, 28, 38, 28, 30, 28, 38, 28,
		40, 38, 51, 38, 40, 38, 51, 38, 30, 28, 38, 28, 30, 28, 38, 28,
		36, 34, 46, 34, 36, 34, 46, 34, 34, 32, 43, 32, 34, 32, 43, 32,
		46, 43, 58, 43, 46, 43, 58, 43, 34, 32, 43, 32, 34, 32, 43, 32,
		36, 34, 46, 34, 36, 34, 46, 34, 34, 32, 43, 32, 34, 32, 43, 32,
		46, 43, 58, 43, 46, 43, 58, 43, 34, 32, 43, 32, 34, 32, 43, 32,
	};
	
	int BitDepth = (iYCbCr == 0) ? ctx->ps.BitDepth_Y : ctx->ps.BitDepth_C;
	__m128i zero = _mm_setzero_si128();
	__m128i shift = _mm_cvtsi32_si128((unsigned)mb->QP[iYCbCr] / 6 + BitDepth - 8);
	__m128i *nA = (__m128i *)normAdjust8x8[(unsigned)mb->QP[iYCbCr] % 6];
	__m128i *w = (__m128i *)ctx->ps.weightScale8x8[iYCbCr * 2 + mb->f.mbIsInterFlag];
	for (int i = 0; i < 4; i++) {
		__m128i x0 = _mm_mullo_epi16(_mm_unpacklo_epi8(w[i], zero), _mm_unpacklo_epi8(nA[i], zero));
		__m128i x1 = _mm_mullo_epi16(_mm_unpacklo_epi8(w[i], zero), _mm_unpacklo_epi8(nA[i], zero));
		ctx->LevelScale_v[i * 4 + 0] = (v4si)_mm_sll_epi32(_mm_unpacklo_epi16(x0, zero), shift);
		ctx->LevelScale_v[i * 4 + 1] = (v4si)_mm_sll_epi32(_mm_unpackhi_epi16(x0, zero), shift);
		ctx->LevelScale_v[i * 4 + 2] = (v4si)_mm_sll_epi32(_mm_unpacklo_epi16(x1, zero), shift);
		ctx->LevelScale_v[i * 4 + 3] = (v4si)_mm_sll_epi32(_mm_unpackhi_epi16(x1, zero), shift);
	}
}



/**
 * Inverse 4x4 transform
 *
 * 8/16bit share the same transform since an 8bit version would not bring much
 * speedup. Also the implementation matches the spec's pseudocode, avoiding
 * minor optimisations which would make it harder to understand.
 */
__attribute__((noinline)) void FUNC(add_idct4x4)
{
	// shortcut for blocks without AC coefficients
	__m128i r0, r1;
	if (__builtin_expect(ctx->significant_coeff_flags <= 1, 1)) {
		r0 = r1 = _mm_set1_epi16((ctx->d[0] + 32) >> 6);
	} else {
		
		// loading
		__m128i d0 = (__m128i)ctx->d_v[0];
		__m128i d1 = (__m128i)ctx->d_v[1];
		__m128i d2 = (__m128i)ctx->d_v[2];
		__m128i d3 = (__m128i)ctx->d_v[3];
		
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
		
		// final residual values
		r0 = _mm_packs_epi32(_mm_srai_epi32(h0, 6), _mm_srai_epi32(h1, 6));
		r1 = _mm_packs_epi32(_mm_srai_epi32(h2, 6), _mm_srai_epi32(h3, 6));
	}
	
	// addition to values in place, clipping and storage
	uint8_t *p = ctx->frame + ctx->frame_offsets_x[ctx->BlkIdx2i4x4[ctx->BlkIdx]] + ctx->frame_offsets_y[ctx->BlkIdx2i4x4[ctx->BlkIdx]];
	size_t stride = ctx->stride;
	if (__builtin_expect(ctx->clip == 255, 1)) {
		__m128i zero = _mm_setzero_si128();
		__m128i p0 = load4x2_8bit(p + stride * 0, p + stride * 1, zero);
		__m128i p1 = load4x2_8bit(p + stride * 2, p + stride * 3, zero);
		v4si u = (v4si)_mm_packus_epi16(_mm_adds_epi16(p0, r0), _mm_adds_epi16(p1, r1));
		*(int32_t *)(p             ) = u[0];
		*(int32_t *)(p + stride    ) = u[1];
		*(int32_t *)(p + stride * 2) = u[2];
		*(int32_t *)(p + stride * 3) = u[3];
	} else {
		__m128i p0 = _mm_setr_epi64(*(__m64 *)(p + stride * 0), *(__m64 *)(p + stride * 1));
		__m128i p1 = _mm_setr_epi64(*(__m64 *)(p + stride * 2), *(__m64 *)(p + stride * 3));
		__m128i zero = _mm_setzero_si128();
		__m128i clip = (__m128i)ctx->clip_v;
		v2li u0 = (v2li)_mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(p0, r0), zero), clip);
		v2li u1 = (v2li)_mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(p1, r1), zero), clip);
		*(int64_t *)(p             ) = u0[0];
		*(int64_t *)(p + stride    ) = u0[1];
		*(int64_t *)(p + stride * 2) = u1[0];
		*(int64_t *)(p + stride * 3) = u1[1];
	}
}

// legacy function
__attribute__((noinline)) void FUNC(decode_Residual4x4, __m128i p0, __m128i p1)
{
	uint8_t *p = ctx->frame + ctx->frame_offsets_x[ctx->BlkIdx2i4x4[ctx->BlkIdx]] + ctx->frame_offsets_y[ctx->BlkIdx2i4x4[ctx->BlkIdx]];
	size_t stride = ctx->stride;
	if (__builtin_expect(ctx->clip == 255, 1)) {
		v4si u = (v4si)_mm_packus_epi16(p0, p1);
		*(int32_t *)(p             ) = u[0];
		*(int32_t *)(p + stride    ) = u[1];
		*(int32_t *)(p + stride * 2) = u[2];
		*(int32_t *)(p + stride * 3) = u[3];
	} else {
		*(int64_t *)(p             ) = ((v2li)p0)[0];
		*(int64_t *)(p + stride    ) = ((v2li)p0)[1];
		*(int64_t *)(p + stride * 2) = ((v2li)p1)[0];
		*(int64_t *)(p + stride * 3) = ((v2li)p1)[1];
	}
	JUMP(add_idct4x4);
}



/**
 * Inverse 8x8 transform
 *
 * 8/16bit versions are kept separate since the first is much faster.
 * For the second, an AVX2 version is maintained that brings a decent speedup.
 */
__attribute__((noinline)) void FUNC(add_idct8x8)
{
	if (__builtin_expect(ctx->clip == 255, 1)) {
		
		// blocks without AC coefficients can get away with a few adds
		__m128i r0, r1, r2, r3, r4, r5, r6, r7;
		if (__builtin_expect(ctx->significant_coeff_flags <= 1, 1)) {
			r0 = r1 = r2 = r3 = r4 = r5 = r6 = r7 = _mm_set1_epi16((ctx->d[0] + 32) >> 6);
		} else {
			
			// loading
			__m128i d0 = _mm_packs_epi32((__m128i)ctx->d_v[0], (__m128i)ctx->d_v[1]);
			__m128i d1 = _mm_packs_epi32((__m128i)ctx->d_v[2], (__m128i)ctx->d_v[3]);
			__m128i d2 = _mm_packs_epi32((__m128i)ctx->d_v[4], (__m128i)ctx->d_v[5]);
			__m128i d3 = _mm_packs_epi32((__m128i)ctx->d_v[6], (__m128i)ctx->d_v[7]);
			__m128i d4 = _mm_packs_epi32((__m128i)ctx->d_v[8], (__m128i)ctx->d_v[9]);
			__m128i d5 = _mm_packs_epi32((__m128i)ctx->d_v[10], (__m128i)ctx->d_v[11]);
			__m128i d6 = _mm_packs_epi32((__m128i)ctx->d_v[12], (__m128i)ctx->d_v[13]);
			__m128i d7 = _mm_packs_epi32((__m128i)ctx->d_v[14], (__m128i)ctx->d_v[15]);
			
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
				if (--i == 0)
					break;
				
				// matrix transposition
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
			
			// final residual values
			r0 = _mm_srai_epi16(d0, 6);
			r1 = _mm_srai_epi16(d1, 6);
			r2 = _mm_srai_epi16(d2, 6);
			r3 = _mm_srai_epi16(d3, 6);
			r4 = _mm_srai_epi16(d4, 6);
			r5 = _mm_srai_epi16(d5, 6);
			r6 = _mm_srai_epi16(d6, 6);
			r7 = _mm_srai_epi16(d7, 6);
		}
		
		// addition to values in place, clipping and storage
		size_t stride = ctx->stride;
		size_t stride3 = stride * 3;
		uint8_t *p = ctx->frame + ctx->frame_offsets_x[ctx->BlkIdx2i4x4[ctx->BlkIdx]] + ctx->frame_offsets_y[ctx->BlkIdx2i4x4[ctx->BlkIdx]];
		uint8_t *q = p + stride * 4;
		__m128i zero = _mm_setzero_si128();
		__m128i p0 = load8x1_8bit(p             , zero);
		__m128i p1 = load8x1_8bit(p + stride    , zero);
		__m128i p2 = load8x1_8bit(p + stride * 2, zero);
		__m128i p3 = load8x1_8bit(p + stride3   , zero);
		__m128i p4 = load8x1_8bit(q             , zero);
		__m128i p5 = load8x1_8bit(q + stride    , zero);
		__m128i p6 = load8x1_8bit(q + stride * 2, zero);
		__m128i p7 = load8x1_8bit(q + stride3   , zero);
		v2li u0 = (v2li)_mm_packus_epi16(_mm_adds_epi16(p0, r0), _mm_adds_epi16(p1, r1));
		v2li u1 = (v2li)_mm_packus_epi16(_mm_adds_epi16(p2, r2), _mm_adds_epi16(p3, r3));
		v2li u2 = (v2li)_mm_packus_epi16(_mm_adds_epi16(p4, r4), _mm_adds_epi16(p5, r5));
		v2li u3 = (v2li)_mm_packus_epi16(_mm_adds_epi16(p6, r6), _mm_adds_epi16(p7, r7));
		*(int64_t *)(p             ) = u0[0];
		*(int64_t *)(p + stride    ) = u0[1];
		*(int64_t *)(p + stride * 2) = u1[0];
		*(int64_t *)(p + stride3   ) = u1[1];
		*(int64_t *)(q             ) = u2[0];
		*(int64_t *)(q + stride    ) = u2[1];
		*(int64_t *)(q + stride * 2) = u3[0];
		*(int64_t *)(q + stride3   ) = u3[1];
	} else {
		
#ifdef __AVX2__
		// shortcut for blocks without AC coefficients
		__m256i r0, r1, r2, r3;
		if (__builtin_expect(ctx->significant_coeff_flags <= 1, 1)) {
			r0 = r1 = r2 = r3 = _mm256_set1_epi16((ctx->d[0] + 32) >> 6);
		} else {
			
			// loading
			__m256i d0 = (__m256i)ctx->d_V[0];
			__m256i d1 = (__m256i)ctx->d_V[1];
			__m256i d2 = (__m256i)ctx->d_V[2];
			__m256i d3 = (__m256i)ctx->d_V[3];
			__m256i d4 = (__m256i)ctx->d_V[4];
			__m256i d5 = (__m256i)ctx->d_V[5];
			__m256i d6 = (__m256i)ctx->d_V[6];
			__m256i d7 = (__m256i)ctx->d_V[7];
			
			for (int i = 2;;) {
				// 1D transform
				__m256i e0 = _mm256_add_epi32(d0, d4);
				__m256i e1 = _mm256_sub_epi32(_mm256_sub_epi32(d5, d3), _mm256_add_epi32(_mm256_srai_epi32(d7, 1), d7));
				__m256i e2 = _mm256_sub_epi32(d0, d4);
				__m256i e3 = _mm256_sub_epi32(_mm256_add_epi32(d1, d7), _mm256_add_epi32(_mm256_srai_epi32(d3, 1), d3));
				__m256i e4 = _mm256_sub_epi32(_mm256_srai_epi32(d2, 1), d6);
				__m256i e5 = _mm256_add_epi32(_mm256_sub_epi32(d7, d1), _mm256_add_epi32(_mm256_srai_epi32(d5, 1), d5));
				__m256i e6 = _mm256_add_epi32(_mm256_srai_epi32(d6, 1), d2);
				__m256i e7 = _mm256_add_epi32(_mm256_add_epi32(d3, d5), _mm256_add_epi32(_mm256_srai_epi32(d1, 1), d1));
				__m256i f0 = _mm256_add_epi32(e0, e6);
				__m256i f1 = _mm256_add_epi32(_mm256_srai_epi32(e7, 2), e1);
				__m256i f2 = _mm256_add_epi32(e2, e4);
				__m256i f3 = _mm256_add_epi32(_mm256_srai_epi32(e5, 2), e3);
				__m256i f4 = _mm256_sub_epi32(e2, e4);
				__m256i f5 = _mm256_sub_epi32(_mm256_srai_epi32(e3, 2), e5);
				__m256i f6 = _mm256_sub_epi32(e0, e6);
				__m256i f7 = _mm256_sub_epi32(e7, _mm256_srai_epi32(e1, 2));
				
				// Compilers freak out whenever output uses other registers.
				d0 = _mm256_add_epi32(f0, f7);
				d1 = _mm256_add_epi32(f2, f5);
				d2 = _mm256_add_epi32(f4, f3);
				d3 = _mm256_add_epi32(f6, f1);
				d4 = _mm256_sub_epi32(f6, f1);
				d5 = _mm256_sub_epi32(f4, f3);
				d6 = _mm256_sub_epi32(f2, f5);
				d7 = _mm256_sub_epi32(f0, f7);
				if (--i == 0)
					break;
				
				// matrix transposition
				__m256i y0 = _mm256_unpacklo_epi32(d0, d1);
				__m256i y1 = _mm256_unpacklo_epi32(d2, d3);
				__m256i y2 = _mm256_unpacklo_epi32(d4, d5);
				__m256i y3 = _mm256_unpacklo_epi32(d6, d7);
				__m256i y4 = _mm256_unpackhi_epi32(d0, d1);
				__m256i y5 = _mm256_unpackhi_epi32(d2, d3);
				__m256i y6 = _mm256_unpackhi_epi32(d4, d5);
				__m256i y7 = _mm256_unpackhi_epi32(d6, d7);
				__m256i y8 = _mm256_unpacklo_epi64(y0, y1);
				__m256i y9 = _mm256_unpacklo_epi64(y2, y3);
				__m256i yA = _mm256_unpacklo_epi64(y4, y5);
				__m256i yB = _mm256_unpacklo_epi64(y6, y7);
				__m256i yC = _mm256_unpackhi_epi64(y0, y1);
				__m256i yD = _mm256_unpackhi_epi64(y2, y3);
				__m256i yE = _mm256_unpackhi_epi64(y4, y5);
				__m256i yF = _mm256_unpackhi_epi64(y6, y7);
				d0 = _mm256_add_epi32(_mm256_permute2x128_si256(y8, y9, _MM_SHUFFLE(0, 2, 0, 0)), _mm256_set1_epi32(32));
				d1 = _mm256_permute2x128_si256(yC, yD, _MM_SHUFFLE(0, 2, 0, 0));
				d2 = _mm256_permute2x128_si256(yA, yB, _MM_SHUFFLE(0, 2, 0, 0));
				d3 = _mm256_permute2x128_si256(yE, yF, _MM_SHUFFLE(0, 2, 0, 0));
				d4 = _mm256_permute2x128_si256(y8, y9, _MM_SHUFFLE(0, 3, 0, 1));
				d5 = _mm256_permute2x128_si256(yC, yD, _MM_SHUFFLE(0, 3, 0, 1));
				d6 = _mm256_permute2x128_si256(yA, yB, _MM_SHUFFLE(0, 3, 0, 1));
				d7 = _mm256_permute2x128_si256(yE, yF, _MM_SHUFFLE(0, 3, 0, 1));
			}
			
			// final residual values
			__m256i yG = _mm256_packs_epi32(_mm256_srai_epi32(d0, 6), _mm256_srai_epi32(d1, 6));
			__m256i yH = _mm256_packs_epi32(_mm256_srai_epi32(d2, 6), _mm256_srai_epi32(d3, 6));
			__m256i yI = _mm256_packs_epi32(_mm256_srai_epi32(d4, 6), _mm256_srai_epi32(d5, 6));
			__m256i yJ = _mm256_packs_epi32(_mm256_srai_epi32(d6, 6), _mm256_srai_epi32(d7, 6));
			r0 = _mm256_permute4x64_epi64(yG, _MM_SHUFFLE(3, 1, 2, 0));
			r1 = _mm256_permute4x64_epi64(yH, _MM_SHUFFLE(3, 1, 2, 0));
			r2 = _mm256_permute4x64_epi64(yI, _MM_SHUFFLE(3, 1, 2, 0));
			r3 = _mm256_permute4x64_epi64(yJ, _MM_SHUFFLE(3, 1, 2, 0));
		}
		
		// addition to values in place, clipping and storage
		size_t stride = ctx->stride;
		size_t stride3 = stride * 3;
		uint8_t *p = ctx->frame + ctx->frame_offsets_x[ctx->BlkIdx2i4x4[ctx->BlkIdx]] + ctx->frame_offsets_y[ctx->BlkIdx2i4x4[ctx->BlkIdx]];
		uint8_t *q = p + stride * 4;
		__m256i zero = _mm256_setzero_si256();
		__m256i clip = _mm256_broadcastsi128_si256((__m128i)ctx->clip_v);
		__m256i p0 = _mm256_setr_m128i(*(__m128i *)(p             ), *(__m128i *)(p + stride ));
		__m256i p1 = _mm256_setr_m128i(*(__m128i *)(p + stride * 2), *(__m128i *)(p + stride3));
		__m256i p2 = _mm256_setr_m128i(*(__m128i *)(q             ), *(__m128i *)(q + stride ));
		__m256i p3 = _mm256_setr_m128i(*(__m128i *)(q + stride * 2), *(__m128i *)(q + stride3));
		__m256i u0 = _mm256_min_epi16(_mm256_max_epi16(_mm256_adds_epi16(p0, r0), zero), clip);
		__m256i u1 = _mm256_min_epi16(_mm256_max_epi16(_mm256_adds_epi16(p1, r1), zero), clip);
		__m256i u2 = _mm256_min_epi16(_mm256_max_epi16(_mm256_adds_epi16(p2, r2), zero), clip);
		__m256i u3 = _mm256_min_epi16(_mm256_max_epi16(_mm256_adds_epi16(p3, r3), zero), clip);
		*(__m128i *)(p             ) = _mm256_extracti128_si256(u0, 0);
		*(__m128i *)(p + stride    ) = _mm256_extracti128_si256(u0, 1);
		*(__m128i *)(p + stride * 2) = _mm256_extracti128_si256(u1, 0);
		*(__m128i *)(p + stride3   ) = _mm256_extracti128_si256(u1, 1);
		*(__m128i *)(q             ) = _mm256_extracti128_si256(u2, 0);
		*(__m128i *)(q + stride    ) = _mm256_extracti128_si256(u2, 1);
		*(__m128i *)(q + stride * 2) = _mm256_extracti128_si256(u3, 0);
		*(__m128i *)(q + stride3   ) = _mm256_extracti128_si256(u3, 1);

#else // !defined(__AVX2__)
		// shortcut for blocks without AC coefficients
		__m128i r0, r1, r2, r3, r4, r5, r6, r7;
		if (__builtin_expect(ctx->significant_coeff_flags <= 1, 1)) {
			r0 = r1 = r2 = r3 = r4 = r5 = r6 = r7 = _mm_set1_epi16((ctx->d[0] + 32) >> 6);
		} else {
			
			// load half of samples
			__m128i d0 = (__m128i)ctx->d_v[0];
			__m128i d1 = (__m128i)ctx->d_v[2];
			__m128i d2 = (__m128i)ctx->d_v[4];
			__m128i d3 = (__m128i)ctx->d_v[6];
			__m128i d4 = (__m128i)ctx->d_v[8];
			__m128i d5 = (__m128i)ctx->d_v[10];
			__m128i d6 = (__m128i)ctx->d_v[12];
			__m128i d7 = (__m128i)ctx->d_v[14];
			
			// This (crappy) version uses a nested loop trick to reduce code size
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
					if (--j == 0)
						break;
					
					// load other half of samples
					ctx->d_v[0] = (v4si)d0;
					d0 = (__m128i)ctx->d_v[1];
					ctx->d_v[2] = (v4si)d1;
					d1 = (__m128i)ctx->d_v[3];
					ctx->d_v[4] = (v4si)d2;
					d2 = (__m128i)ctx->d_v[5];
					ctx->d_v[6] = (v4si)d3;
					d3 = (__m128i)ctx->d_v[7];
					ctx->d_v[8] = (v4si)d4;
					d4 = (__m128i)ctx->d_v[9];
					ctx->d_v[10] = (v4si)d5;
					d5 = (__m128i)ctx->d_v[11];
					ctx->d_v[12] = (v4si)d6;
					d6 = (__m128i)ctx->d_v[13];
					ctx->d_v[14] = (v4si)d7;
					d7 = (__m128i)ctx->d_v[15];
				}
				if (--i == 0)
					break;
				
				// transpose the half matrix going to memory
				__m128i x0 = _mm_unpacklo_epi32((__m128i)ctx->d_v[8], (__m128i)ctx->d_v[10]);
				__m128i x1 = _mm_unpacklo_epi32((__m128i)ctx->d_v[12], (__m128i)ctx->d_v[14]);
				__m128i x2 = _mm_unpackhi_epi32((__m128i)ctx->d_v[8], (__m128i)ctx->d_v[10]);
				__m128i x3 = _mm_unpackhi_epi32((__m128i)ctx->d_v[12], (__m128i)ctx->d_v[14]);
				__m128i x4 = _mm_unpacklo_epi32(d4, d5);
				__m128i x5 = _mm_unpacklo_epi32(d6, d7);
				__m128i x6 = _mm_unpackhi_epi32(d4, d5);
				__m128i x7 = _mm_unpackhi_epi32(d6, d7);
				ctx->d_v[1] = (v4si)_mm_add_epi32(_mm_unpacklo_epi64(x0, x1), _mm_set1_epi32(32));
				ctx->d_v[3] = (v4si)_mm_unpackhi_epi64(x0, x1);
				ctx->d_v[5] = (v4si)_mm_unpacklo_epi64(x2, x3);
				ctx->d_v[7] = (v4si)_mm_unpackhi_epi64(x2, x3);
				ctx->d_v[9] = (v4si)_mm_unpacklo_epi64(x4, x5);
				ctx->d_v[11] = (v4si)_mm_unpackhi_epi64(x4, x5);
				ctx->d_v[13] = (v4si)_mm_unpacklo_epi64(x6, x7);
				ctx->d_v[15] = (v4si)_mm_unpackhi_epi64(x6, x7);
				
				// transpose the half matrix staying in registers
				__m128i x8 = _mm_unpacklo_epi32((__m128i)ctx->d_v[0], (__m128i)ctx->d_v[2]);
				__m128i x9 = _mm_unpacklo_epi32((__m128i)ctx->d_v[4], (__m128i)ctx->d_v[6]);
				__m128i xA = _mm_unpackhi_epi32((__m128i)ctx->d_v[0], (__m128i)ctx->d_v[2]);
				__m128i xB = _mm_unpackhi_epi32((__m128i)ctx->d_v[4], (__m128i)ctx->d_v[6]);
				__m128i xC = _mm_unpacklo_epi32(d0, d1);
				__m128i xD = _mm_unpacklo_epi32(d2, d3);
				__m128i xE = _mm_unpackhi_epi32(d0, d1);
				__m128i xF = _mm_unpackhi_epi32(d2, d3);
				d0 = _mm_add_epi32(_mm_unpacklo_epi64(x8, x9), _mm_set1_epi32(32));
				d1 = _mm_unpackhi_epi64(x8, x9);
				d2 = _mm_unpacklo_epi64(xA, xB);
				d3 = _mm_unpackhi_epi64(xA, xB);
				d4 = _mm_unpacklo_epi64(xC, xD);
				d5 = _mm_unpackhi_epi64(xC, xD);
				d6 = _mm_unpacklo_epi64(xE, xF);
				d7 = _mm_unpackhi_epi64(xE, xF);
			}
			
			// final residual values
			r0 = _mm_packs_epi32(_mm_srai_epi32((__m128i)ctx->d_v[0], 6), _mm_srai_epi32(d0, 6));
			r1 = _mm_packs_epi32(_mm_srai_epi32((__m128i)ctx->d_v[2], 6), _mm_srai_epi32(d1, 6));
			r2 = _mm_packs_epi32(_mm_srai_epi32((__m128i)ctx->d_v[4], 6), _mm_srai_epi32(d2, 6));
			r3 = _mm_packs_epi32(_mm_srai_epi32((__m128i)ctx->d_v[6], 6), _mm_srai_epi32(d3, 6));
			r4 = _mm_packs_epi32(_mm_srai_epi32((__m128i)ctx->d_v[8], 6), _mm_srai_epi32(d4, 6));
			r5 = _mm_packs_epi32(_mm_srai_epi32((__m128i)ctx->d_v[10], 6), _mm_srai_epi32(d5, 6));
			r6 = _mm_packs_epi32(_mm_srai_epi32((__m128i)ctx->d_v[12], 6), _mm_srai_epi32(d6, 6));
			r7 = _mm_packs_epi32(_mm_srai_epi32((__m128i)ctx->d_v[14], 6), _mm_srai_epi32(d7, 6));
		}
		
		// addition to values in place, clipping and storage
		size_t stride = ctx->stride;
		size_t stride3 = stride * 3;
		uint8_t *p = ctx->frame + ctx->frame_offsets_x[ctx->BlkIdx2i4x4[ctx->BlkIdx]] + ctx->frame_offsets_y[ctx->BlkIdx2i4x4[ctx->BlkIdx]];
		uint8_t *q = p + stride * 4;
		__m128i zero = _mm_setzero_si128();
		__m128i clip = (__m128i)ctx->clip_v;
		__m128i p0 = *(__m128i *)(p             );
		__m128i p1 = *(__m128i *)(p + stride    );
		__m128i p2 = *(__m128i *)(p + stride * 2);
		__m128i p3 = *(__m128i *)(p + stride3   );
		__m128i p4 = *(__m128i *)(q             );
		__m128i p5 = *(__m128i *)(q + stride    );
		__m128i p6 = *(__m128i *)(q + stride * 2);
		__m128i p7 = *(__m128i *)(q + stride3   );
		__m128i u0 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(p0, r0), zero), clip);
		__m128i u1 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(p1, r1), zero), clip);
		__m128i u2 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(p2, r2), zero), clip);
		__m128i u3 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(p3, r3), zero), clip);
		__m128i u4 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(p4, r4), zero), clip);
		__m128i u5 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(p5, r5), zero), clip);
		__m128i u6 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(p6, r6), zero), clip);
		__m128i u7 = _mm_min_epi16(_mm_max_epi16(_mm_adds_epi16(p7, r7), zero), clip);
		*(__m128i *)(p             ) = u0;
		*(__m128i *)(p + stride    ) = u1;
		*(__m128i *)(p + stride * 2) = u2;
		*(__m128i *)(p + stride3   ) = u3;
		*(__m128i *)(q             ) = u4;
		*(__m128i *)(q + stride    ) = u5;
		*(__m128i *)(q + stride * 2) = u6;
		*(__m128i *)(q + stride3   ) = u7;
#endif // __AVX2__
	}
}

// legacy function
__attribute__((noinline)) void FUNC(decode_Residual8x8, __m128i p0,
	__m128i p1, __m128i p2, __m128i p3, __m128i p4, __m128i p5, __m128i p6,
	__m128i p7)
{
	size_t stride = ctx->stride;
	size_t stride3 = stride * 3;
	uint8_t *p = ctx->frame + ctx->frame_offsets_x[ctx->BlkIdx2i4x4[ctx->BlkIdx]] + ctx->frame_offsets_y[ctx->BlkIdx2i4x4[ctx->BlkIdx]];
	uint8_t *q = p + stride * 4;
	if (__builtin_expect(ctx->clip == 255, 1)) {
		v2li u0 = (v2li)_mm_packus_epi16(p0, p1);
		v2li u1 = (v2li)_mm_packus_epi16(p2, p3);
		v2li u2 = (v2li)_mm_packus_epi16(p4, p5);
		v2li u3 = (v2li)_mm_packus_epi16(p6, p7);
		*(int64_t *)(p             ) = u0[0];
		*(int64_t *)(p + stride    ) = u0[1];
		*(int64_t *)(p + stride * 2) = u1[0];
		*(int64_t *)(p + stride3   ) = u1[1];
		*(int64_t *)(q             ) = u2[0];
		*(int64_t *)(q + stride    ) = u2[1];
		*(int64_t *)(q + stride * 2) = u3[0];
		*(int64_t *)(q + stride3   ) = u3[1];
	} else {
		*(__m128i *)(p             ) = p0;
		*(__m128i *)(p + stride    ) = p1;
		*(__m128i *)(p + stride * 2) = p2;
		*(__m128i *)(p + stride3   ) = p3;
		*(__m128i *)(q             ) = p4;
		*(__m128i *)(q + stride    ) = p5;
		*(__m128i *)(q + stride * 2) = p6;
		*(__m128i *)(q + stride3   ) = p7;
	}
	JUMP(add_idct8x8);
}
#define decode_Residual8x8_8bit decode_Residual8x8



/**
 * DC coefficients transform
 */
__attribute__((noinline)) void FUNC(transform_dc4x4) {
	CALL(compute_LevelScale4x4, ctx->BlkIdx >> 4);
	
	// loading
	__m128i c0 = (__m128i)ctx->d_v[0];
	__m128i c1 = (__m128i)ctx->d_v[1];
	__m128i c2 = (__m128i)ctx->d_v[2];
	__m128i c3 = (__m128i)ctx->d_v[3];
	
	// left matrix multiplication
	__m128i x0 = _mm_add_epi32(c0, c1);
	__m128i x1 = _mm_add_epi32(c2, c3);
	__m128i x2 = _mm_sub_epi32(c0, c1);
	__m128i x3 = _mm_sub_epi32(c2, c3);
	__m128i x4 = _mm_add_epi32(x0, x1);
	__m128i x5 = _mm_sub_epi32(x0, x1);
	__m128i x6 = _mm_sub_epi32(x2, x3);
	__m128i x7 = _mm_add_epi32(x2, x3);
	
	// transposition
	__m128i x8 = _mm_unpacklo_epi32(x4, x5);
	__m128i x9 = _mm_unpacklo_epi32(x6, x7);
	__m128i xA = _mm_unpackhi_epi32(x4, x5);
	__m128i xB = _mm_unpackhi_epi32(x6, x7);
	__m128i xC = _mm_unpacklo_epi64(x8, x9);
	__m128i xD = _mm_unpackhi_epi64(x8, x9);
	__m128i xE = _mm_unpacklo_epi64(xA, xB);
	__m128i xF = _mm_unpackhi_epi64(xA, xB);
	
	// right matrix multiplication
	__m128i xG = _mm_add_epi32(xC, xD);
	__m128i xH = _mm_add_epi32(xE, xF);
	__m128i xI = _mm_sub_epi32(xC, xD);
	__m128i xJ = _mm_sub_epi32(xE, xF);
	__m128i f0 = _mm_add_epi32(xG, xH);
	__m128i f1 = _mm_sub_epi32(xG, xH);
	__m128i f2 = _mm_sub_epi32(xI, xJ);
	__m128i f3 = _mm_add_epi32(xI, xJ);
	
	// scaling and storing
	__m128i s = _mm_set1_epi32(ctx->LevelScale[0]);
	__m128i s128 = _mm_set1_epi32(128);
	__m128i dc0 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(f0, s), s128), 8);
	__m128i dc1 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(f1, s), s128), 8);
	__m128i dc2 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(f2, s), s128), 8);
	__m128i dc3 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(f3, s), s128), 8);
	ctx->d_v[4] = (v4si)_mm_unpacklo_epi64(dc0, dc1);
	ctx->d_v[5] = (v4si)_mm_unpackhi_epi64(dc0, dc1);
	ctx->d_v[6] = (v4si)_mm_unpacklo_epi64(dc2, dc3);
	ctx->d_v[7] = (v4si)_mm_unpackhi_epi64(dc2, dc3);
}

__attribute__((noinline)) void FUNC(transform_dc2x2) {
	int iYCbCr = (ctx->BlkIdx - 12) >> 2; // BlkIdx is 16 or 20
	unsigned qP = mb->QP[iYCbCr];
	unsigned w = ctx->ps.weightScale4x4[iYCbCr + mb->f.mbIsInterFlag * 3][0];
	unsigned nA = normAdjust4x4[qP % 6][0];
	int LevelScale = (w * nA) << (qP / 6 + ctx->ps.BitDepth_C - 8);
	int i0 = ctx->d[0] + ctx->d[2];
	int i1 = ctx->d[1] + ctx->d[3];
	int i2 = ctx->d[0] - ctx->d[2];
	int i3 = ctx->d[1] - ctx->d[3];
	int32_t *d = ctx->d + ctx->BlkIdx;
	d[0] = ((i0 + i1) * LevelScale) >> 5;
	d[1] = ((i0 - i1) * LevelScale) >> 5;
	d[2] = ((i2 + i3) * LevelScale) >> 5;
	d[3] = ((i2 - i3) * LevelScale) >> 5;
}

__attribute__((noinline)) void FUNC(transform_dc2x4) {
	int iYCbCr = (ctx->BlkIdx - 8) >> 3; // BlkIdx is 16 or 24
	unsigned qP_DC = mb->QP[iYCbCr] + 3;
	int w = ctx->ps.weightScale4x4[iYCbCr + mb->f.mbIsInterFlag * 3][0];
	int nA = normAdjust4x4[qP_DC % 6][0];
	__m128i x0 = (__m128i)ctx->d_v[0]; // {c00, c01, c10, c11} as per 8.5.11.1
	__m128i x1 = (__m128i)ctx->d_v[1]; // {c20, c21, c30, c31}
	__m128i x2 = _mm_add_epi32(x0, x1); // {c00+c20, c01+c21, c10+c30, c11+c31}
	__m128i x3 = _mm_sub_epi32(x0, x1); // {c00-c20, c01-c21, c10-c30, c11-c31}
	__m128i x4 = _mm_unpacklo_epi64(x2, x3); // {c00+c20, c01+c21, c00-c20, c01-c21}
	__m128i x5 = _mm_unpackhi_epi64(x2, x3); // {c10+c30, c11+c31, c10-c30, c11-c31}
	__m128i x6 = _mm_add_epi32(x4, x5); // {d00, d01, d10, d11}
	__m128i x7 = _mm_sub_epi32(x4, x5); // {d30, d31, d20, d21}
	__m128i x8 = _mm_hadd_epi32(x6, x7); // {f00, f10, f30, f20}
	__m128i x9 = _mm_hsub_epi32(x6, x7); // {f01, f11, f31, f21}
	__m128i s = _mm_set1_epi32((w * nA) << (qP_DC / 6 + ctx->ps.BitDepth_C - 8));
	__m128i s32 = _mm_set1_epi32(32);
	__m128i dc0 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(x8, s), s32), 6);
	__m128i dc1 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(x9, s), s32), 6);
	__m128i *d = (__m128i *)ctx->d_v + 2 + iYCbCr * 2;
	d[0] = _mm_unpacklo_epi32(dc0, dc1);
	d[1] = _mm_shuffle_epi32(_mm_unpackhi_epi64(dc0, dc1), _MM_SHUFFLE(2, 0, 3, 1));
}
