/**
 * Intra decoding involves so many shuffling tricks that it is better expressed
 * as native code, where each architecture can give its best.
 * 
 * Each decoding function is a large switch with a downward tree structure for
 * sharing code. Cases start at the leaves and branch towards the exit root.
 * While there is no way to make compilers generate proper code with vanilla
 * switches or functions, goto instructions are used for internal branches.
 * 
 * Choosing between the different possibilities of a same function is tricky,
 * in general I favor in order:
 * _ the fastest code, obviously (http://www.agner.org/optimize/#manual_instr_tab),
 * _ a short dependency chain (more freedom for compilers to reorder),
 * _ smaller code+data (avoid excessive use of pshufb),
 * _ readable code (helped by Intel's astounding instrinsics naming...).
 * 
 * My thumb rules:
 * _ Reads are ordered downwards in case prefetch loads some useful future data.
 * _ Aligned reads are favored if they incur no additional instructions.
 * _ pshufb is used iff doing otherwise would require 3+ instructions.
 * _ Favor vector over scalar code to avoid callee-save conventions.
 */

#include "edge264_internal.h"

static inline __m128i lowpass_8bit(__m128i left, __m128i mid, __m128i right) {
	__m128i bit = _mm_and_si128(_mm_xor_si128(left, right), _mm_set1_epi8(1));
	return _mm_avg_epu8(_mm_subs_epu8(_mm_avg_epu8(left, right), bit), mid);
}
static inline __m128i lowpass_16bit(__m128i left, __m128i mid, __m128i right) {
	return _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(left, right), 1), mid);
}

// all functions using _mm_cvtepu8_epi16 should contain a variable zero=_mm_setzero_si128()
#ifndef __SSE4_1__
	#define _mm_cvtepu8_epi16(a) _mm_unpacklo_epi8(a, (__m128i)zero)
#endif


/**
 * Intra4x4
 */
void _decode_intra4x4(int mode, uint8_t *px1, size_t stride, ssize_t nstride, v8hi clip, v16qi zero) {
	__m128i dc, top;
	switch (mode) {
	
	case I4x4_V_8:
		*(int32_t *)(px1 + nstride) = *(int32_t *)px1 = *(int32_t *)(px1 + stride) = *(int32_t *)(px1 + stride * 2) = *(int32_t *)(px1 + nstride * 2);
		return;
	
	case I4x4_H_8: {
		__m128i x0 = _mm_set1_epi8(3);
		*(int32_t *)(px1 + nstride    ) = ((v4si)_mm_shuffle_epi8(_mm_loadu_si32(px1 + nstride     - 4), x0))[0];
		*(int32_t *)(px1              ) = ((v4si)_mm_shuffle_epi8(_mm_loadu_si32(px1               - 4), x0))[0];
		*(int32_t *)(px1 +  stride    ) = ((v4si)_mm_shuffle_epi8(_mm_loadu_si32(px1 +  stride     - 4), x0))[0];
		*(int32_t *)(px1 +  stride * 2) = ((v4si)_mm_shuffle_epi8(_mm_loadu_si32(px1 +  stride * 2 - 4), x0))[0];
		return; }
	
	case I4x4_DC_8: {
		__m128i x0 = _mm_loadu_si32(px1 + nstride * 2);
		__m128i x1 = _mm_loadu_si32(px1 + nstride     - 4);
		__m128i x2 = _mm_loadu_si32(px1               - 4);
		__m128i x3 = _mm_loadu_si32(px1 +  stride     - 4);
		__m128i x4 = _mm_loadu_si32(px1 +  stride * 2 - 4);
		__m128i x5 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(x1, x2), _mm_unpacklo_epi8(x3, x4));
		dc = _mm_srli_epi16(_mm_sad_epu8(_mm_alignr_epi8(x0, x5, 12), (__m128i)zero), 2); }
	dc_4x4: {
		v4si pred = (v4si)_mm_shuffle_epi8(_mm_avg_epu16(dc, (__m128i)zero), (__m128i)zero);
		*(int32_t *)(px1 + nstride) = *(int32_t *)px1 = *(int32_t *)(px1 + stride) = *(int32_t *)(px1 + stride * 2) = pred[0];
		return; }
	case I4x4_DCA_8:
		dc = _mm_srli_epi16(_mm_sad_epu8(_mm_loadu_si32(px1 + nstride * 2), (__m128i)zero), 1);
		goto dc_4x4;
	case I4x4_DCB_8: {
		__m128i x0 = _mm_loadu_si32(px1 + nstride     - 4);
		__m128i x1 = _mm_loadu_si32(px1               - 4);
		__m128i x2 = _mm_loadu_si32(px1 +  stride     - 4);
		__m128i x3 = _mm_loadu_si32(px1 +  stride * 2 - 4);
		__m128i x4 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(x0, x1), _mm_unpacklo_epi8(x2, x3));
		dc = _mm_srli_epi16(_mm_sad_epu8(_mm_srli_si128(x4, 12), (__m128i)zero), 1);
		goto dc_4x4; }
	case I4x4_DCAB_8:
		dc = _mm_broadcastw_epi16((__m128i)clip);
		goto dc_4x4;
	
	case I4x4_DDL_8:
		top = load8x1_8bit(px1 + nstride * 2, (__m128i)zero);
	diagonal_down_left_4x4: {
		__m128i x1 = _mm_srli_si128(top, 2);
		__m128i x2 = _mm_shufflehi_epi16(_mm_shuffle_epi32(top, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
		__m128i x3 = _mm_packus_epi16(lowpass_16bit(top, x1, x2), (__m128i)zero);
		*(int32_t *)(px1 + nstride    ) = ((v4si)x3)[0];
		*(int32_t *)(px1              ) = ((v4si)_mm_srli_epi64(x3, 8))[0];
		*(int32_t *)(px1 +  stride    ) = ((v4si)_mm_srli_epi64(x3, 16))[0];
		*(int32_t *)(px1 +  stride * 2) = ((v4si)_mm_srli_epi64(x3, 24))[0];
		return; }
	case I4x4_DDLC_8:
		top = _mm_shuffle_epi8(_mm_loadu_si32(px1 + nstride * 2), _mm_setr_epi8(0, -1, 1, -1, 2, -1, 3, -1, 3, -1, 3, -1, 3, -1, 3, -1));
		goto diagonal_down_left_4x4;
	
	case I4x4_DDR_8: {
		__m128i x0 = load8x1_8bit(px1 + nstride * 2 - 1, (__m128i)zero); // 45678...
		__m128i x1 = _mm_loadu_si32(px1 + nstride     - 4); // ...3............
		__m128i x2 = _mm_loadu_si32(px1               - 4); // ...2............
		__m128i x3 = _mm_loadu_si32(px1 +  stride     - 4); // ...1............
		__m128i x4 = _mm_loadu_si32(px1 +  stride * 2 - 4); // ...0............
		__m128i x5 = _mm_unpackhi_epi8(_mm_unpacklo_epi16(_mm_unpacklo_epi8(x4, x3), _mm_unpacklo_epi8(x2, x1)), (__m128i)zero); // ....0123
		__m128i x6 = lowpass_16bit(_mm_alignr_epi8(x0, x5, 8), _mm_alignr_epi8(x0, x5, 10), _mm_alignr_epi8(x0, x5, 12));
		__m128i x7 = _mm_packus_epi16(x6, (__m128i)zero);
		*(int32_t *)(px1 + nstride    ) = ((v4si)_mm_srli_epi64(x7, 24))[0];
		*(int32_t *)(px1              ) = ((v4si)_mm_srli_epi64(x7, 16))[0];
		*(int32_t *)(px1 +  stride    ) = ((v4si)_mm_srli_epi64(x7, 8))[0];
		*(int32_t *)(px1 +  stride * 2) = ((v4si)x7)[0];
		return; }
	
	case I4x4_VR_8: {
		__m128i x0 = _mm_loadu_si64(px1 + nstride * 2 - 1); // 34567...........
		__m128i x1 = _mm_loadu_si32(px1 + nstride     - 4); // ...2............
		__m128i x2 = _mm_loadu_si32(px1               - 4); // ...1............
		__m128i x3 = _mm_loadu_si32(px1 +  stride     - 4); // ...0............
		__m128i x4 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(x3, x3), _mm_unpacklo_epi8(x2, x1)); // .............012
		__m128i x5 = _mm_cvtepu8_epi16(_mm_alignr_epi8(x0, x4, 13));
		__m128i x6 = _mm_slli_si128(x5, 2);
		__m128i x7 = lowpass_16bit(x5, x6, _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 1, 0, 0)));
		__m128i x8 = _mm_packus_epi16(_mm_avg_epu16(x5, x6), x7);
		v4si pred = (v4si)_mm_shuffle_epi8(x8, _mm_setr_epi8(4, 5, 6, 7, 12, 13, 14, 15, 11, 4, 5, 6, 10, 12, 13, 14));
		*(int32_t *)(px1 + nstride    ) = pred[0];
		*(int32_t *)(px1              ) = pred[1];
		*(int32_t *)(px1 +  stride    ) = pred[2];
		*(int32_t *)(px1 +  stride * 2) = pred[3];
		return; }
	
	case I4x4_HD_8: {
		__m128i x0 = _mm_loadu_si32(px1 + nstride * 2 - 1); // 4567............
		__m128i x1 = _mm_loadu_si32(px1 + nstride     - 4); // ...3............
		__m128i x2 = _mm_loadu_si32(px1               - 4); // ...2............
		__m128i x3 = _mm_loadu_si32(px1 +  stride     - 4); // ...1............
		__m128i x4 = _mm_loadu_si32(px1 +  stride * 2 - 4); // ...0............
		__m128i x5 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(x4, x3), _mm_unpacklo_epi8(x2, x1)); // ............0123
		__m128i x6 = _mm_cvtepu8_epi16(_mm_alignr_epi8(x0, x5, 12));
		__m128i x7 = _mm_srli_si128(x6, 2);
		__m128i x8 = lowpass_16bit(x6, x7, _mm_shuffle_epi32(x6, _MM_SHUFFLE(3, 3, 2, 1)));
		__m128i x9 = _mm_packus_epi16(_mm_avg_epu16(x6, x7), x8);
		v4si pred = (v4si)_mm_shuffle_epi8(x9, _mm_setr_epi8(3, 11, 12, 13, 2, 10, 3, 11, 1, 9, 2, 10, 0, 8, 1, 9));
		*(int32_t *)(px1 + nstride    ) = pred[0];
		*(int32_t *)(px1              ) = pred[1];
		*(int32_t *)(px1 +  stride    ) = pred[2];
		*(int32_t *)(px1 +  stride * 2) = pred[3];
		return; }
	
	case I4x4_VL_8:
		top = load8x1_8bit(px1 + nstride * 2, (__m128i)zero);
	vertical_left_4x4: {
		__m128i x0 = _mm_srli_si128(top, 2);
		__m128i x1 = _mm_shufflehi_epi16(_mm_shuffle_epi32(top, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
		__m128i x2 = _mm_packus_epi16(_mm_avg_epu16(top, x0), lowpass_16bit(top, x0, x1));
		*(int32_t *)(px1 + nstride    ) = ((v4si)x2)[0];
		*(int32_t *)(px1              ) = ((v4si)x2)[2];
		*(int32_t *)(px1 +  stride    ) = ((v4si)_mm_srli_si128(x2, 1))[0];
		*(int32_t *)(px1 +  stride * 2) = ((v4si)_mm_srli_si128(x2, 9))[0];
		return; }
	case I4x4_VLC_8:
		top = _mm_shuffle_epi8(_mm_loadu_si32(px1 + nstride * 2), _mm_setr_epi8(0, -1, 1, -1, 2, -1, 3, -1, 3, -1, 3, -1, 3, -1, 3, -1));
		goto vertical_left_4x4;
	
	case I4x4_HU_8: {
		__m128i x0 = _mm_loadu_si32(px1 + nstride     - 4); // ...0............
		__m128i x1 = _mm_loadu_si32(px1               - 4); // ...1............
		__m128i x2 = _mm_loadu_si32(px1 +  stride     - 4); // ...2............
		__m128i x3 = _mm_loadu_si32(px1 +  stride * 2 - 4); // ...3............
		__m128i x4 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(x0, x1), _mm_unpacklo_epi8(x2, x3)); // ............0123
		__m128i x5 = _mm_unpackhi_epi8(x4, (__m128i)zero);
		__m128i x6 = _mm_shufflehi_epi16(x5, _MM_SHUFFLE(3, 3, 2, 1));
		__m128i x7 = _mm_shufflehi_epi16(x5, _MM_SHUFFLE(3, 3, 3, 2));
		__m128i x8 = _mm_packus_epi16(_mm_avg_epu16(x5, x6), lowpass_16bit(x5, x6, x7));
		v4si pred = (v4si)_mm_shuffle_epi8(x8, _mm_setr_epi8(4, 12, 5, 13, 5, 13, 6, 14, 6, 14, 7, 7, 7, 7, 7, 7));
		*(int32_t *)(px1 + nstride    ) = pred[0];
		*(int32_t *)(px1              ) = pred[1];
		*(int32_t *)(px1 +  stride    ) = pred[2];
		*(int32_t *)(px1 +  stride * 2) = pred[3];
		return; }
	
	default: __builtin_unreachable();
	}
}



/**
 * Intra8x8
 * 
 * Neighbouring samples are named a to z from bottom left to top right, with
 * i being p[-1,-1] or p[-1,0] if unavailable, and j being p[-1,-1] or p[0,-1].
 */
static __m128i filter_h2a_8bit(ssize_t dstride, uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride) {
	__m128i i = _mm_loadu_si32(px0 + dstride     - 4);
	__m128i h = _mm_loadu_si32(px0               - 4);
	__m128i g = _mm_loadu_si32(px0 +  stride     - 4);
	__m128i f = _mm_loadu_si32(px0 +  stride * 2 - 4);
	__m128i e = _mm_loadu_si32(px7 + nstride * 4 - 4);
	__m128i d = _mm_loadu_si32(px0 +  stride * 4 - 4);
	__m128i c = _mm_loadu_si32(px7 + nstride * 2 - 4);
	__m128i b = _mm_loadu_si32(px7 + nstride     - 4);
	__m128i a = _mm_loadu_si32(px7               - 1);
	// unpack samples in downward direction
	__m128i i2f = _mm_unpacklo_epi16(_mm_unpacklo_epi8(i, h), _mm_unpacklo_epi8(g, f));
	__m128i e2b = _mm_unpacklo_epi16(_mm_unpacklo_epi8(e, d), _mm_unpacklo_epi8(c, b));
	__m128i i2b = _mm_unpackhi_epi32(i2f, e2b);
	__m128i h2a = _mm_alignr_epi8(a, i2b, 1);
	__m128i g2a = _mm_alignr_epi8(a, h2a, 1);
	return _mm_srli_si128(lowpass_8bit(i2b, h2a, g2a), 8);
}

void _decode_intra8x8(int mode, uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride, v8hi clip) {
	static const v16qi shr8_8bit = {1, 2, 3, 4, 5, 6, 7, 7, -1, -1, -1, -1, -1, -1};
	static const v16qi shl_8bit = {0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
	static const v16qi shr16_8bit = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15};
	static const v16qi C8_8bit = {0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8, 8, 8, 8};
	static const v16qi C7_8bit = {0, 1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7};
	__m128i j2q, k2r, l2s, j2y, k2z, j2s, p0, p1, p2, p3, p4, p5, p6, p7;
	ssize_t dstride = nstride;
	switch (mode) {
	
	case I8x8_V_8:
		j2q = _mm_loadu_si64(px0 + nstride     - 1);
		l2s = _mm_loadu_si64(px0 + nstride     + 1);
		k2r = _mm_loadu_si64(px0 + nstride        );
	vertical_8x8: {
		v2li p = (v2li)lowpass_8bit(j2q, k2r, l2s);
		*(int64_t *)(px0              ) = p[0];
		*(int64_t *)(px0 +  stride    ) = p[0];
		*(int64_t *)(px0 +  stride * 2) = p[0];
		*(int64_t *)(px7 + nstride * 4) = p[0];
		*(int64_t *)(px0 +  stride * 4) = p[0];
		*(int64_t *)(px7 + nstride * 2) = p[0];
		*(int64_t *)(px7 + nstride    ) = p[0];
		*(int64_t *)(px7              ) = p[0];
		return; }
	case I8x8_V_C_8:
		k2r = _mm_loadu_si64(px0 + nstride        );
		j2q = _mm_loadu_si64(px0 + nstride     - 1);
		l2s = _mm_shuffle_epi8(k2r, (__m128i)shr8_8bit);
		goto vertical_8x8;
	case I8x8_V_D_8:
		k2r = _mm_loadu_si64(px0 + nstride        );
		l2s = _mm_loadu_si64(px0 + nstride     + 1);
		j2q = _mm_shuffle_epi8(k2r, (__m128i)shl_8bit);
		goto vertical_8x8;
	case I8x8_V_CD_8:
		k2r = _mm_loadu_si64(px0 + nstride        );
		l2s = _mm_shuffle_epi8(k2r, (__m128i)shr8_8bit);
		j2q = _mm_shuffle_epi8(k2r, (__m128i)shl_8bit);
		goto vertical_8x8;
	
	case I8x8_H_D_8:
		dstride = 0;
		// PASSTHROUGH
	case I8x8_H_8: {
		__m128i h2a = filter_h2a_8bit(dstride, px0, px7, stride, nstride);
		__m128i x0 = _mm_unpacklo_epi8(h2a, h2a);
		__m128i x1 = _mm_unpacklo_epi16(x0, x0);
		__m128i x2 = _mm_unpackhi_epi16(x0, x0);
		p0 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(0, 0, 0, 0));
		p1 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 1, 1, 1));
		p2 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(2, 2, 2, 2));
		p3 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(3, 3, 3, 3));
		p4 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(0, 0, 0, 0));
		p5 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(1, 1, 1, 1));
		p6 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 2, 2, 2));
		p7 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 3, 3, 3));
		break; }
	
	case I8x8_DC_8: {
		j2q = _mm_loadu_si64(px0 + nstride     - 1);
		l2s = _mm_loadu_si64(px0 + nstride     + 1);
		k2r = _mm_loadu_si64(px0 + nstride        );
	} dc_8x8: {
		__m128i i = _mm_loadu_si32(px0 + dstride     - 4);
		__m128i h = _mm_loadu_si32(px0               - 4);
		__m128i g = _mm_loadu_si32(px0 +  stride     - 4);
		__m128i f = _mm_loadu_si32(px0 +  stride * 2 - 4);
		__m128i e = _mm_loadu_si32(px7 + nstride * 4 - 4);
		__m128i d = _mm_loadu_si32(px0 +  stride * 4 - 4);
		__m128i c = _mm_loadu_si32(px7 + nstride * 2 - 4);
		__m128i b = _mm_loadu_si32(px7 + nstride     - 4);
		__m128i a = _mm_loadu_si32(px7               - 1);
		__m128i i2f = _mm_unpacklo_epi16(_mm_unpacklo_epi8(i, h), _mm_unpacklo_epi8(g, f));
		__m128i e2b = _mm_unpacklo_epi16(_mm_unpacklo_epi8(e, d), _mm_unpacklo_epi8(c, b));
		__m128i i2b = _mm_unpackhi_epi32(i2f, e2b);
		__m128i h2a = _mm_alignr_epi8(a, i2b, 1);
		__m128i g2a = _mm_alignr_epi8(a, h2a, 1);
		__m128i i2b_l2s = _mm_alignr_epi8(l2s, i2b, 8);
		__m128i h2a_k2r = _mm_alignr_epi8(k2r, h2a, 8);
		__m128i g2a_j2q = _mm_alignr_epi8(j2q, g2a, 8);
		__m128i zero = _mm_setzero_si128();
		__m128i x0 = _mm_sad_epu8(lowpass_8bit(i2b_l2s, h2a_k2r, g2a_j2q), zero);
		__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 2, 3, 2)));
		v2li p = (v2li)_mm_broadcastb_epi8(_mm_avg_epu16(_mm_srli_epi16(x1, 3), zero));
		*(int64_t *)(px0              ) = p[0];
		*(int64_t *)(px0 +  stride    ) = p[0];
		*(int64_t *)(px0 +  stride * 2) = p[0];
		*(int64_t *)(px7 + nstride * 4) = p[0];
		*(int64_t *)(px0 +  stride * 4) = p[0];
		*(int64_t *)(px7 + nstride * 2) = p[0];
		*(int64_t *)(px7 + nstride    ) = p[0];
		*(int64_t *)(px7              ) = p[0];
		return; }
	case I8x8_DC_C_8:
		k2r = _mm_loadu_si64(px0 + nstride        );
		j2q = _mm_loadu_si64(px0 + nstride     - 1);
		l2s = _mm_shuffle_epi8(k2r, (__m128i)shr8_8bit);
		goto dc_8x8;
	case I8x8_DC_D_8:
		k2r = _mm_loadu_si64(px0 + nstride        );
		l2s = _mm_loadu_si64(px0 + nstride     + 1);
		j2q = _mm_shuffle_epi8(k2r, (__m128i)shl_8bit);
		dstride = 0;
		goto dc_8x8;
	case I8x8_DC_CD_8:
		k2r = _mm_loadu_si64(px0 + nstride        );
		l2s = _mm_shuffle_epi8(k2r, (__m128i)shr8_8bit);
		j2q = _mm_shuffle_epi8(k2r, (__m128i)shl_8bit);
		dstride = 0;
		goto dc_8x8;
	case I8x8_DC_A_8:
		j2q = _mm_loadu_si64(px0 + nstride     - 1);
		l2s = _mm_loadu_si64(px0 + nstride     + 1);
		k2r = _mm_loadu_si64(px0 + nstride        );
	dc_top_8x8: {
		__m128i zero = _mm_setzero_si128();
		__m128i x0 = _mm_sad_epu8(lowpass_8bit(j2q, k2r, l2s), zero);
		v2li p = (v2li)_mm_broadcastb_epi8(_mm_avg_epu16(_mm_srli_epi16(x0, 2), zero));
		*(int64_t *)(px0              ) = p[0];
		*(int64_t *)(px0 +  stride    ) = p[0];
		*(int64_t *)(px0 +  stride * 2) = p[0];
		*(int64_t *)(px7 + nstride * 4) = p[0];
		*(int64_t *)(px0 +  stride * 4) = p[0];
		*(int64_t *)(px7 + nstride * 2) = p[0];
		*(int64_t *)(px7 + nstride    ) = p[0];
		*(int64_t *)(px7              ) = p[0];
		return; }
	case I8x8_DC_AC_8:
		k2r = _mm_loadu_si64(px0 + nstride        );
		j2q = _mm_loadu_si64(px0 + nstride     - 1);
		l2s = _mm_shuffle_epi8(k2r, (__m128i)shr8_8bit);
		goto dc_top_8x8;
	case I8x8_DC_AD_8:
		k2r = _mm_loadu_si64(px0 + nstride        );
		l2s = _mm_loadu_si64(px0 + nstride     + 1);
		j2q = _mm_shuffle_epi8(k2r, (__m128i)shl_8bit);
		goto dc_top_8x8;
	case I8x8_DC_ACD_8:
		k2r = _mm_loadu_si64(px0 + nstride        );
		l2s = _mm_shuffle_epi8(k2r, (__m128i)shr8_8bit);
		j2q = _mm_shuffle_epi8(k2r, (__m128i)shl_8bit);
		goto dc_top_8x8;
	case I8x8_DC_BD_8:
		dstride = 0;
		// PASSTHROUGH
	case I8x8_DC_B_8: {
		__m128i zero = _mm_setzero_si128();
		__m128i x0 = _mm_sad_epu8(filter_h2a_8bit(dstride, px0, px7, stride, nstride), zero);
		v2li p = (v2li)_mm_broadcastb_epi8(_mm_avg_epu16(_mm_srli_epi16(x0, 2), zero));
		*(int64_t *)(px0              ) = p[0];
		*(int64_t *)(px0 +  stride    ) = p[0];
		*(int64_t *)(px0 +  stride * 2) = p[0];
		*(int64_t *)(px7 + nstride * 4) = p[0];
		*(int64_t *)(px0 +  stride * 4) = p[0];
		*(int64_t *)(px7 + nstride * 2) = p[0];
		*(int64_t *)(px7 + nstride    ) = p[0];
		*(int64_t *)(px7              ) = p[0];
		return; }
	case I8x8_DC_AB_8: {
		*(int64_t *)(px0              ) =
		*(int64_t *)(px0 +  stride    ) =
		*(int64_t *)(px0 +  stride * 2) =
		*(int64_t *)(px7 + nstride * 4) =
		*(int64_t *)(px0 +  stride * 4) =
		*(int64_t *)(px7 + nstride * 2) =
		*(int64_t *)(px7 + nstride    ) =
		*(int64_t *)(px7              ) = 0x8080808080808080ull;
		return; }
	
	case I8x8_DDL_8:
		j2y = _mm_loadu_si128((__m128i *)(px0 + nstride     - 1));
		k2z = _mm_loadu_si128((__m128i *)(px0 + nstride        ));
	diagonal_down_left_8x8: {
		__m128i l2z = _mm_shuffle_epi8(k2z, (__m128i)shr16_8bit);
		__m128i x0 = lowpass_8bit(j2y, k2z, l2z);
		__m128i x1 = _mm_shuffle_epi8(x0, (__m128i)shr16_8bit);
		__m128i x2 = _mm_srli_si128(x1, 1);
		p0 = lowpass_8bit(x0, x1, x2);
		p1 = _mm_srli_si128(p0, 1);
		p2 = _mm_srli_si128(p0, 2);
		p3 = _mm_srli_si128(p0, 3);
		p4 = _mm_srli_si128(p0, 4);
		p5 = _mm_srli_si128(p0, 5);
		p6 = _mm_srli_si128(p0, 6);
		p7 = _mm_srli_si128(p0, 7);
		break; }
	case I8x8_DDL_C_8:
		j2y = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(px0 + nstride     - 1)), (__m128i)C8_8bit);
		k2z = _mm_shuffle_epi8(j2y, (__m128i)shr16_8bit);
		goto diagonal_down_left_8x8;
	case I8x8_DDL_D_8:
		k2z = _mm_loadu_si128((__m128i *)(px0 + nstride        ));
		j2y = _mm_shuffle_epi8(k2z, (__m128i)shl_8bit);
		goto diagonal_down_left_8x8;
	case I8x8_DDL_CD_8:
		k2z = _mm_shuffle_epi8(_mm_loadu_si64(px0 + nstride        ), (__m128i)C7_8bit);
		j2y = _mm_shuffle_epi8(k2z, (__m128i)shl_8bit);
		goto diagonal_down_left_8x8;
	
	case I8x8_DDR_C_8:
		j2s = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(px0 + nstride     - 1)), (__m128i)C8_8bit);
		goto diagonal_down_right_8x8;
	case I8x8_DDR_8:
		j2s = _mm_loadu_si128((__m128i *)(px0 + nstride     - 1));
	diagonal_down_right_8x8: {
		__m128i h = _mm_loadu_si32(px0               - 4);
		__m128i g = _mm_loadu_si32(px0 +  stride     - 4);
		__m128i f = _mm_loadu_si32(px0 +  stride * 2 - 4);
		__m128i e = _mm_loadu_si32(px7 + nstride * 4 - 4);
		__m128i d = _mm_loadu_si32(px0 +  stride * 4 - 4);
		__m128i c = _mm_loadu_si32(px7 + nstride * 2 - 4);
		__m128i b = _mm_loadu_si32(px7 + nstride     - 4);
		__m128i a = _mm_loadu_si32(px7               - 4);
		__m128i e2h = _mm_unpacklo_epi16(_mm_unpacklo_epi8(e, f), _mm_unpacklo_epi8(g, h));
		__m128i a2d = _mm_unpacklo_epi16(_mm_unpacklo_epi8(a, b), _mm_unpacklo_epi8(c, d));
		__m128i a2h = _mm_unpackhi_epi32(a2d, e2h);
		__m128i a2q = _mm_alignr_epi8(j2s, a2h, 8);
		__m128i b2r = _mm_alignr_epi8(j2s, a2h, 9);
		__m128i x0 = lowpass_8bit(_mm_shuffle_epi8(a2q, (__m128i)shl_8bit), a2q, b2r);
		__m128i x1 = lowpass_8bit(a2q, b2r, _mm_alignr_epi8(j2s, a2h, 10));
		p7 = lowpass_8bit(x0, x1, _mm_srli_si128(x1, 1));
		p6 = _mm_srli_si128(p7, 1);
		p5 = _mm_srli_si128(p7, 2);
		p4 = _mm_srli_si128(p7, 3);
		p3 = _mm_srli_si128(p7, 4);
		p2 = _mm_srli_si128(p7, 5);
		p1 = _mm_srli_si128(p7, 6);
		p0 = _mm_srli_si128(p7, 7);
		break; }
	
	case I8x8_VR_C_8:
		j2s = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(px0 + nstride     - 1)), (__m128i)C8_8bit);
		goto vertical_right_8x8;
	case I8x8_VR_8:
		j2s = _mm_loadu_si128((__m128i *)(px0 + nstride     - 1));
	vertical_right_8x8: {
		__m128i h = _mm_loadu_si32(px0               - 4);
		__m128i g = _mm_loadu_si32(px0 +  stride     - 4);
		__m128i f = _mm_loadu_si32(px0 +  stride * 2 - 4);
		__m128i e = _mm_loadu_si32(px7 + nstride * 4 - 4);
		__m128i d = _mm_loadu_si32(px0 +  stride * 4 - 4);
		__m128i c = _mm_loadu_si32(px7 + nstride * 2 - 4);
		__m128i b = _mm_loadu_si32(px7 + nstride     - 4);
		__m128i a = _mm_loadu_si32(px7               - 4);
		__m128i e2h = _mm_unpacklo_epi16(_mm_unpacklo_epi8(e, f), _mm_unpacklo_epi8(g, h));
		__m128i a2d = _mm_unpacklo_epi16(_mm_unpacklo_epi8(a, b), _mm_unpacklo_epi8(c, d));
		__m128i a2h = _mm_unpackhi_epi32(a2d, e2h);
		__m128i a2q = _mm_alignr_epi8(j2s, a2h, 8);
		__m128i b2r = _mm_alignr_epi8(j2s, a2h, 9);
		__m128i c2s = _mm_alignr_epi8(j2s, a2h, 10);
		__m128i x0 = lowpass_8bit(a2q, b2r, c2s);
		__m128i x1 = _mm_slli_si128(x0, 1);
		__m128i x2 = lowpass_8bit(x0, x1, _mm_slli_si128(x0, 2));
		p0 = _mm_srli_si128(_mm_avg_epu8(x0, x1), 8);
		p1 = _mm_srli_si128(x2, 8);
		p2 = _mm_alignr_epi8(p0, _mm_slli_si128(x2, 8), 15);
		p3 = _mm_alignr_epi8(p1, _mm_slli_si128(x2, 9), 15);
		p4 = _mm_alignr_epi8(p2, _mm_slli_si128(x2, 10), 15);
		p5 = _mm_alignr_epi8(p3, _mm_slli_si128(x2, 11), 15);
		p6 = _mm_alignr_epi8(p4, _mm_slli_si128(x2, 12), 15);
		p7 = _mm_alignr_epi8(p5, _mm_slli_si128(x2, 13), 15);
		break; }
	
	case I8x8_HD_8: {
		j2s = _mm_loadu_si128((__m128i *)(px0 + nstride     - 1));
		__m128i h = _mm_loadu_si32(px0               - 4);
		__m128i g = _mm_loadu_si32(px0 +  stride     - 4);
		__m128i f = _mm_loadu_si32(px0 +  stride * 2 - 4);
		__m128i e = _mm_loadu_si32(px7 + nstride * 4 - 4);
		__m128i d = _mm_loadu_si32(px0 +  stride * 4 - 4);
		__m128i c = _mm_loadu_si32(px7 + nstride * 2 - 4);
		__m128i b = _mm_loadu_si32(px7 + nstride     - 4);
		__m128i a = _mm_loadu_si32(px7               - 4);
		__m128i e2h = _mm_unpacklo_epi16(_mm_unpacklo_epi8(e, f), _mm_unpacklo_epi8(g, h));
		__m128i a2d = _mm_unpacklo_epi16(_mm_unpacklo_epi8(a, b), _mm_unpacklo_epi8(c, d));
		__m128i a2h = _mm_unpackhi_epi32(a2d, e2h);
		__m128i a2q = _mm_alignr_epi8(j2s, a2h, 8);
		__m128i b2r = _mm_alignr_epi8(j2s, a2h, 9);
		__m128i x0 = lowpass_8bit(_mm_shuffle_epi8(a2q, (__m128i)shl_8bit), a2q, b2r);
		__m128i x1 = _mm_srli_si128(x0, 1);
		__m128i x2 = lowpass_8bit(x0, x1, _mm_srli_si128(x0, 2));
		p7 = _mm_unpacklo_epi8(_mm_avg_epu8(x0, x1), x2);
		p6 = _mm_srli_si128(p7, 2);
		p5 = _mm_srli_si128(p7, 4);
		p4 = _mm_srli_si128(p7, 6);
		p3 = _mm_unpackhi_epi64(p7, x2);
		p2 = _mm_srli_si128(p3, 2);
		p1 = _mm_srli_si128(p3, 4);
		p0 = _mm_srli_si128(p3, 6);
		break; }
	
	case I8x8_VL_8:
		j2y = _mm_loadu_si128((__m128i *)(px0 + nstride     - 1));
		k2z = _mm_loadu_si128((__m128i *)(px0 + nstride        ));
	vertical_left_8x8: {
		__m128i x0 = lowpass_8bit(j2y, k2z, _mm_srli_si128(k2z, 1));
		__m128i x1 = _mm_srli_si128(x0, 1);
		p0 = _mm_avg_epu8(x0, x1);
		p1 = lowpass_8bit(x0, x1, _mm_srli_si128(x0, 2));
		p2 = _mm_srli_si128(p0, 1);
		p3 = _mm_srli_si128(p1, 1);
		p4 = _mm_srli_si128(p2, 1);
		p5 = _mm_srli_si128(p3, 1);
		p6 = _mm_srli_si128(p4, 1);
		p7 = _mm_srli_si128(p5, 1);
		break; }
	case I8x8_VL_C_8:
		j2y = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(px0 + nstride     - 1)), (__m128i)C8_8bit);
		k2z = _mm_srli_si128(j2y, 1);
		goto vertical_left_8x8;
	case I8x8_VL_D_8:
		k2z = _mm_loadu_si128((__m128i *)(px0 + nstride        ));
		j2y = _mm_shuffle_epi8(k2z, (__m128i)shl_8bit);
		goto vertical_left_8x8;
	case I8x8_VL_CD_8:
		k2z = _mm_shuffle_epi8(_mm_loadu_si64(px0 + nstride        ), (__m128i)C7_8bit);
		j2y = _mm_shuffle_epi8(k2z, (__m128i)shl_8bit);
		goto vertical_left_8x8;
	
	case I8x8_HU_D_8:
		dstride = 0;
		// PASSTHROUGH
	case I8x8_HU_8: {
		__m128i x0 = _mm_shuffle_epi8(filter_h2a_8bit(dstride, px0, px7, stride, nstride), (__m128i)C7_8bit);
		__m128i x1 = _mm_srli_si128(x0, 1);
		__m128i x2 = lowpass_8bit(x0, x1, _mm_srli_si128(x0, 2));
		p0 = _mm_unpacklo_epi8(_mm_avg_epu8(x0, x1), x2);
		p1 = _mm_srli_si128(p0, 2);
		p2 = _mm_srli_si128(p0, 4);
		p3 = _mm_srli_si128(p0, 6);
		p4 = _mm_unpackhi_epi64(p0, x0);
		p5 = _mm_srli_si128(p4, 2);
		p6 = _mm_srli_si128(p4, 4);
		p7 = _mm_srli_si128(p4, 6);
		break; }
	
	default: __builtin_unreachable();
	}
	*(int64_t *)(px0              ) = ((v2li)p0)[0];
	*(int64_t *)(px0 +  stride    ) = ((v2li)p1)[0];
	*(int64_t *)(px0 +  stride * 2) = ((v2li)p2)[0];
	*(int64_t *)(px7 + nstride * 4) = ((v2li)p3)[0];
	*(int64_t *)(px0 +  stride * 4) = ((v2li)p4)[0];
	*(int64_t *)(px7 + nstride * 2) = ((v2li)p5)[0];
	*(int64_t *)(px7 + nstride    ) = ((v2li)p6)[0];
	*(int64_t *)(px7              ) = ((v2li)p7)[0];
}



/**
 * Intra16x16
 */
void _decode_intra16x16(int mode, uint8_t *px0, uint8_t *px7, uint8_t *pxE, size_t stride, ssize_t nstride, v8hi clip) {
	__m128i top, left, pred;
	switch (mode) {
	
	case I16x16_V_8:
		pred = *(__m128i *)(px0 + nstride    );
		break;
	
	case I16x16_H_8: {
		__m128i x0 = _mm_set1_epi8(3);
		*(__m128i *)(px0              ) = _mm_shuffle_epi8(_mm_loadu_si32(px0               - 4), x0);
		*(__m128i *)(px0 +  stride    ) = _mm_shuffle_epi8(_mm_loadu_si32(px0 +  stride     - 4), x0);
		*(__m128i *)(px0 +  stride * 2) = _mm_shuffle_epi8(_mm_loadu_si32(px0 +  stride * 2 - 4), x0);
		*(__m128i *)(px7 + nstride * 4) = _mm_shuffle_epi8(_mm_loadu_si32(px7 + nstride * 4 - 4), x0);
		*(__m128i *)(px0 +  stride * 4) = _mm_shuffle_epi8(_mm_loadu_si32(px0 +  stride * 4 - 4), x0);
		*(__m128i *)(px7 + nstride * 2) = _mm_shuffle_epi8(_mm_loadu_si32(px7 + nstride * 2 - 4), x0);
		*(__m128i *)(px7 + nstride    ) = _mm_shuffle_epi8(_mm_loadu_si32(px7 + nstride     - 4), x0);
		*(__m128i *)(px7              ) = _mm_shuffle_epi8(_mm_loadu_si32(px7               - 4), x0);
		*(__m128i *)(px7 +  stride    ) = _mm_shuffle_epi8(_mm_loadu_si32(px7 +  stride     - 4), x0);
		*(__m128i *)(px7 +  stride * 2) = _mm_shuffle_epi8(_mm_loadu_si32(px7 +  stride * 2 - 4), x0);
		*(__m128i *)(pxE + nstride * 4) = _mm_shuffle_epi8(_mm_loadu_si32(pxE + nstride * 4 - 4), x0);
		*(__m128i *)(px7 +  stride * 4) = _mm_shuffle_epi8(_mm_loadu_si32(px7 +  stride * 4 - 4), x0);
		*(__m128i *)(pxE + nstride * 2) = _mm_shuffle_epi8(_mm_loadu_si32(pxE + nstride * 2 - 4), x0);
		*(__m128i *)(pxE + nstride    ) = _mm_shuffle_epi8(_mm_loadu_si32(pxE + nstride     - 4), x0);
		*(__m128i *)(pxE              ) = _mm_shuffle_epi8(_mm_loadu_si32(pxE               - 4), x0);
		*(__m128i *)(pxE +  stride    ) = _mm_shuffle_epi8(_mm_loadu_si32(pxE +  stride     - 4), x0);
		} return;
	
	case I16x16_DC_8: {
		top = *(__m128i *)(px0 + nstride    );
		__m128i l0 = *(__m128i *)(px0               - 16);
		__m128i l1 = *(__m128i *)(px0 +  stride     - 16);
		__m128i l2 = *(__m128i *)(px0 +  stride * 2 - 16);
		__m128i l3 = *(__m128i *)(px7 + nstride * 4 - 16);
		__m128i l4 = *(__m128i *)(px0 +  stride * 4 - 16);
		__m128i l5 = *(__m128i *)(px7 + nstride * 2 - 16);
		__m128i l6 = *(__m128i *)(px7 + nstride     - 16);
		__m128i l7 = *(__m128i *)(px7               - 16);
		__m128i l8 = *(__m128i *)(px7 +  stride     - 16);
		__m128i l9 = *(__m128i *)(px7 +  stride * 2 - 16);
		__m128i lA = *(__m128i *)(pxE + nstride * 4 - 16);
		__m128i lB = *(__m128i *)(px7 +  stride * 4 - 16);
		__m128i lC = *(__m128i *)(pxE + nstride * 2 - 16);
		__m128i lD = *(__m128i *)(pxE + nstride     - 16);
		__m128i lE = *(__m128i *)(pxE               - 16);
		__m128i lF = *(__m128i *)(pxE +  stride     - 16);
		__m128i x0 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l0, l1), _mm_unpackhi_epi8(l2, l3));
		__m128i x1 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l4, l5), _mm_unpackhi_epi8(l6, l7));
		__m128i x2 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l8, l9), _mm_unpackhi_epi8(lA, lB));
		__m128i x3 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(lC, lD), _mm_unpackhi_epi8(lE, lF));
		left = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
	} i16x16_dc_8: {
		__m128i zero = _mm_setzero_si128();
		__m128i x0 = _mm_add_epi16(_mm_sad_epu8(top, zero), _mm_sad_epu8(left, zero));
		__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)));
		pred = _mm_broadcastb_epi8(_mm_avg_epu16(_mm_srai_epi16(x1, 4), zero));
		} break;
	case I16x16_DCA_8:
		top = left = *(__m128i *)(px0 + nstride    );
		goto i16x16_dc_8;
	case I16x16_DCB_8: {
		__m128i l0 = *(__m128i *)(px0               - 16);
		__m128i l1 = *(__m128i *)(px0 +  stride     - 16);
		__m128i l2 = *(__m128i *)(px0 +  stride * 2 - 16);
		__m128i l3 = *(__m128i *)(px7 + nstride * 4 - 16);
		__m128i l4 = *(__m128i *)(px0 +  stride * 4 - 16);
		__m128i l5 = *(__m128i *)(px7 + nstride * 2 - 16);
		__m128i l6 = *(__m128i *)(px7 + nstride     - 16);
		__m128i l7 = *(__m128i *)(px7               - 16);
		__m128i l8 = *(__m128i *)(px7 +  stride     - 16);
		__m128i l9 = *(__m128i *)(px7 +  stride * 2 - 16);
		__m128i lA = *(__m128i *)(pxE + nstride * 4 - 16);
		__m128i lB = *(__m128i *)(px7 +  stride * 4 - 16);
		__m128i lC = *(__m128i *)(pxE + nstride * 2 - 16);
		__m128i lD = *(__m128i *)(pxE + nstride     - 16);
		__m128i lE = *(__m128i *)(pxE               - 16);
		__m128i lF = *(__m128i *)(pxE +  stride     - 16);
		__m128i x0 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l0, l1), _mm_unpackhi_epi8(l2, l3));
		__m128i x1 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l4, l5), _mm_unpackhi_epi8(l6, l7));
		__m128i x2 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(l8, l9), _mm_unpackhi_epi8(lA, lB));
		__m128i x3 = _mm_unpackhi_epi16(_mm_unpackhi_epi8(lC, lD), _mm_unpackhi_epi8(lE, lF));
		top = left = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
		} goto i16x16_dc_8;
	case I16x16_DCAB_8: 
		pred = _mm_set1_epi8(-128);
		break;
	
	case I16x16_P_8: {
		// load neighbouring values in vector registers
		top = _mm_setr_epi64(*(__m64 *)(px0 + nstride     - 1), *(__m64 *)(px0 + nstride     + 8));
		__m128i l0 = _mm_loadu_si32(px0               - 4);
		__m128i l1 = _mm_loadu_si32(px0 +  stride     - 4);
		__m128i l2 = _mm_loadu_si32(px0 +  stride * 2 - 4);
		__m128i l3 = _mm_loadu_si32(px7 + nstride * 4 - 4);
		__m128i l4 = _mm_loadu_si32(px0 +  stride * 4 - 4);
		__m128i l5 = _mm_loadu_si32(px7 + nstride * 2 - 4);
		__m128i l6 = _mm_loadu_si32(px7 + nstride     - 4);
		__m128i l8 = _mm_loadu_si32(px7 +  stride     - 4);
		__m128i l9 = _mm_loadu_si32(px7 +  stride * 2 - 4);
		__m128i lA = _mm_loadu_si32(pxE + nstride * 4 - 4);
		__m128i lB = _mm_loadu_si32(px7 +  stride * 4 - 4);
		__m128i lC = _mm_loadu_si32(pxE + nstride * 2 - 4);
		__m128i lD = _mm_loadu_si32(pxE + nstride     - 4);
		__m128i lE = _mm_loadu_si32(pxE               - 4);
		__m128i lF = _mm_loadu_si32(pxE +  stride     - 4);
		__m128i lG = _mm_unpacklo_epi16(_mm_unpacklo_epi8(_mm_slli_si128(top, 3), l0), _mm_unpacklo_epi8(l1, l2));
		__m128i lH = _mm_unpacklo_epi16(_mm_unpacklo_epi8(l3, l4), _mm_unpacklo_epi8(l5, l6));
		__m128i lI = _mm_unpacklo_epi16(_mm_unpacklo_epi8(l8, l9), _mm_unpacklo_epi8(lA, lB));
		__m128i lJ = _mm_unpacklo_epi16(_mm_unpacklo_epi8(lC, lD), _mm_unpacklo_epi8(lE, lF));
		left = _mm_unpackhi_epi64(_mm_unpackhi_epi32(lG, lH), _mm_unpackhi_epi32(lI, lJ));
		
		// sum the samples and compute a, b, c (with care for overflow)
		__m128i mul = _mm_setr_epi8(-8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8);
		__m128i x0 = _mm_maddubs_epi16(top, mul);
		__m128i x1 = _mm_maddubs_epi16(left, mul);
		__m128i x2 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(1, 0, 3, 2)));
		__m128i x3 = _mm_add_epi16(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 0, 3, 2)));
		__m128i x4 = _mm_add_epi16(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
		__m128i x5 = _mm_add_epi16(x3, _mm_shuffle_epi32(x3, _MM_SHUFFLE(2, 3, 0, 1)));
		__m128i HV = _mm_hadd_epi16(x4, x5); // H in lower half, V in upper half, both in [-9180,9180]
		__m128i x6 = _mm_add_epi16(HV, _mm_srai_epi16(HV, 2)); // (5 * HV) >> 2, in [-11475,11475]
		__m128i x7 = _mm_srai_epi16(_mm_sub_epi16(x6, _mm_set1_epi16(-8)), 4); // (5 * HV + 32) >> 6
		__m128i x8 = _mm_add_epi16(_mm_srli_si128(top, 15), _mm_srli_si128(left, 15));
		__m128i a = _mm_slli_epi16(_mm_sub_epi16(_mm_broadcastw_epi16(x8), _mm_set1_epi16(-1)), 4); // in [16,8176]
		__m128i b = _mm_shuffle_epi32(x7, _MM_SHUFFLE(1, 0, 1, 0)); // in [-717,717]
		__m128i c = _mm_shuffle_epi32(x7, _MM_SHUFFLE(3, 2, 3, 2)); // in [-717,717]
		
		// compute prediction vectors and store them in memory
		__m128i x9 = _mm_sub_epi16(_mm_add_epi16(a, c), _mm_slli_epi16(c, 3));
		__m128i p1 = _mm_add_epi16(x9, _mm_mullo_epi16(_mm_unpackhi_epi8(mul, _mm_setzero_si128()), b));
		__m128i p0 = _mm_sub_epi16(p1, _mm_slli_epi16(b, 3));
		*(__m128i *)(px0              ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(px0 +  stride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(px0 +  stride * 2) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(px7 + nstride * 4) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(px0 +  stride * 4) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(px7 + nstride * 2) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(px7 + nstride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(px7              ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(px7 +  stride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(px7 +  stride * 2) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(pxE + nstride * 4) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(px7 +  stride * 4) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(pxE + nstride * 2) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(pxE + nstride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(pxE              ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		p0 = _mm_add_epi16(p0, c), p1 = _mm_add_epi16(p1, c);
		*(__m128i *)(pxE +  stride    ) = _mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
		} return;
	
	default: __builtin_unreachable();
	}
	*(__m128i *)(px0              ) = pred;
	*(__m128i *)(px0 +  stride    ) = pred;
	*(__m128i *)(px0 +  stride * 2) = pred;
	*(__m128i *)(px7 + nstride * 4) = pred;
	*(__m128i *)(px0 +  stride * 4) = pred;
	*(__m128i *)(px7 + nstride * 2) = pred;
	*(__m128i *)(px7 + nstride    ) = pred;
	*(__m128i *)(px7              ) = pred;
	*(__m128i *)(px7 +  stride    ) = pred;
	*(__m128i *)(px7 +  stride * 2) = pred;
	*(__m128i *)(pxE + nstride * 4) = pred;
	*(__m128i *)(px7 +  stride * 4) = pred;
	*(__m128i *)(pxE + nstride * 2) = pred;
	*(__m128i *)(pxE + nstride    ) = pred;
	*(__m128i *)(pxE              ) = pred;
	*(__m128i *)(pxE +  stride    ) = pred;
}



/**
 * Intra chroma
 * 
 * Since each mode is to be called twice, we inline them inside a switch.
 */
static always_inline void chroma8x8_DC_8bit(uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride) {
	__m128i top = _mm_loadu_si64(px0 + nstride    );
	__m128i l0 = _mm_loadu_si32(px0               - 4);
	__m128i l1 = _mm_loadu_si32(px0 +  stride     - 4);
	__m128i l2 = _mm_loadu_si32(px0 +  stride * 2 - 4);
	__m128i l3 = _mm_loadu_si32(px7 + nstride * 4 - 4);
	__m128i l4 = _mm_loadu_si32(px0 +  stride * 4 - 4);
	__m128i l5 = _mm_loadu_si32(px7 + nstride * 2 - 4);
	__m128i l6 = _mm_loadu_si32(px7 + nstride     - 4);
	__m128i l7 = _mm_loadu_si32(px7               - 4);
	__m128i x0 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(l0, l1), _mm_unpacklo_epi8(l2, l3));
	__m128i x1 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(l4, l5), _mm_unpacklo_epi8(l6, l7));
	__m128i x2 = _mm_alignr_epi8(top, _mm_unpackhi_epi32(x0, x1), 8);
	__m128i dc01 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 3, 2, 0));
	__m128i dc23 = _mm_shuffle_epi32(x2, _MM_SHUFFLE(3, 1, 1, 1));
	__m128i zero = _mm_setzero_si128();
	__m128i x3 = _mm_packs_epi32(_mm_sad_epu8(dc01, zero), _mm_sad_epu8(dc23, zero));
	__m128i x4 = _mm_avg_epu16(_mm_srli_epi16(x3, 2), zero);
	v2li dc = (v2li)_mm_shuffle_epi8(x4, _mm_setr_epi8(0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12));
	*(int64_t *)(px0              ) = *(int64_t *)(px0 +  stride    ) = *(int64_t *)(px0 +  stride * 2) = *(int64_t *)(px7 + nstride * 4) = dc[0];
	*(int64_t *)(px0 +  stride * 4) = *(int64_t *)(px7 + nstride * 2) = *(int64_t *)(px7 + nstride    ) = *(int64_t *)(px7              ) = dc[1];
}

static always_inline void chroma8x8_DC_A_8bit(uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride) {
	__m128i top = _mm_loadu_si64((__m128i *)(px0 + nstride    ));
	__m128i dc01 = _mm_unpacklo_epi32(top, top);
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_avg_epu16(_mm_srli_epi16(_mm_sad_epu8(dc01, zero), 2), zero);
	v2li dc = (v2li)_mm_shuffle_epi8(x0, _mm_setr_epi8(0, 0, 0, 0, 8, 8, 8, 8, -1, -1, -1, -1, -1, -1, -1, -1));
	*(int64_t *)(px0              ) = *(int64_t *)(px0 +  stride    ) = *(int64_t *)(px0 +  stride * 2) = *(int64_t *)(px7 + nstride * 4) =
	*(int64_t *)(px0 +  stride * 4) = *(int64_t *)(px7 + nstride * 2) = *(int64_t *)(px7 + nstride    ) = *(int64_t *)(px7              ) = dc[0];
}

static always_inline void chroma8x8_DC_B_8bit(uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride) {
	__m128i l0 = _mm_loadu_si32(px0               - 4);
	__m128i l1 = _mm_loadu_si32(px0 +  stride     - 4);
	__m128i l2 = _mm_loadu_si32(px0 +  stride * 2 - 4);
	__m128i l3 = _mm_loadu_si32(px7 + nstride * 4 - 4);
	__m128i l4 = _mm_loadu_si32(px0 +  stride * 4 - 4);
	__m128i l5 = _mm_loadu_si32(px7 + nstride * 2 - 4);
	__m128i l6 = _mm_loadu_si32(px7 + nstride     - 4);
	__m128i l7 = _mm_loadu_si32(px7               - 4);
	__m128i x0 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(l0, l1), _mm_unpacklo_epi8(l2, l3));
	__m128i x1 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(l4, l5), _mm_unpacklo_epi8(l6, l7));
	__m128i dc02 = (__m128i)_mm_shuffle_ps((__m128)x0, (__m128)x1, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i zero = _mm_setzero_si128();
	__m128i x2 = _mm_avg_epu16(_mm_srli_epi16(_mm_sad_epu8(dc02, zero), 2), zero);
	v2li dc = (v2li)_mm_shuffle_epi8(x2, _mm_setr_epi8(0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8, 8, 8));
	*(int64_t *)(px0              ) = *(int64_t *)(px0 +  stride    ) = *(int64_t *)(px0 +  stride * 2) = *(int64_t *)(px7 + nstride * 4) = dc[0];
	*(int64_t *)(px0 +  stride * 4) = *(int64_t *)(px7 + nstride * 2) = *(int64_t *)(px7 + nstride    ) = *(int64_t *)(px7              ) = dc[1];
}

static always_inline void chroma8x8_DC_AB_8bit(uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride) {
	int64_t dc = 0x8080808080808080LL;
	*(int64_t *)(px0              ) = *(int64_t *)(px0 +  stride    ) = *(int64_t *)(px0 +  stride * 2) = *(int64_t *)(px7 + nstride * 4) = dc;
	*(int64_t *)(px0 +  stride * 4) = *(int64_t *)(px7 + nstride * 2) = *(int64_t *)(px7 + nstride    ) = *(int64_t *)(px7              ) = dc;
}

static always_inline void chroma8x8_horizontal_8bit(uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride) {
	__m128i shuf = _mm_set1_epi8(3);
	*(int64_t *)(px0              ) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(px0               - 4), shuf))[0];
	*(int64_t *)(px0 +  stride    ) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(px0 +  stride     - 4), shuf))[0];
	*(int64_t *)(px0 +  stride * 2) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(px0 +  stride * 2 - 4), shuf))[0];
	*(int64_t *)(px7 + nstride * 4) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(px7 + nstride * 4 - 4), shuf))[0];
	*(int64_t *)(px0 +  stride * 4) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(px0 +  stride * 4 - 4), shuf))[0];
	*(int64_t *)(px7 + nstride * 2) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(px7 + nstride * 2 - 4), shuf))[0];
	*(int64_t *)(px7 + nstride    ) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(px7 + nstride     - 4), shuf))[0];
	*(int64_t *)(px7              ) = ((v2li)_mm_shuffle_epi8(_mm_loadu_si32(px7               - 4), shuf))[0];
}

static always_inline void chroma8x8_vertical_8bit(uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride) {
	int64_t top = *(int64_t *)(px0 + nstride    );
	*(int64_t *)(px0              ) = *(int64_t *)(px0 +  stride    ) = top;
	*(int64_t *)(px0 +  stride * 2) = *(int64_t *)(px7 + nstride * 4) = top;
	*(int64_t *)(px0 +  stride * 4) = *(int64_t *)(px7 + nstride * 2) = top;
	*(int64_t *)(px7 + nstride    ) = *(int64_t *)(px7              ) = top;
}

static always_inline void chroma8x8_plane_8bit(uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride) {
	// load neighbouring values in a single vector register
	__m128i t0 = _mm_loadu_si32(px0 + nstride     - 1); // unaligned
	__m128i t1 = _mm_loadu_si32(px0 + nstride     + 4);
	__m128i l1 = _mm_loadu_si32(px0               - 4);
	__m128i l2 = _mm_loadu_si32(px0 +  stride     - 4);
	__m128i l3 = _mm_loadu_si32(px0 +  stride * 2 - 4);
	__m128i l4 = _mm_loadu_si32(px0 +  stride * 4 - 4);
	__m128i l5 = _mm_loadu_si32(px7 + nstride * 2 - 4);
	__m128i l6 = _mm_loadu_si32(px7 + nstride     - 4);
	__m128i l7 = _mm_loadu_si64(px7               - 8);
	__m128i x0 = _mm_unpacklo_epi16(l7, _mm_unpacklo_epi8(l6, l5));
	__m128i x1 = _mm_unpacklo_epi16(_mm_unpacklo_epi8(l4, l3), _mm_unpacklo_epi8(l2, l1));
	__m128i x2 = _mm_alignr_epi8(t0, _mm_unpackhi_epi32(x0, x1), 1);
	__m128i x3 = _mm_alignr_epi8(_mm_unpacklo_epi32(t0, t1), x2, 8); // lllllllltttttttt
	
	// sum the samples and compute a, b, c (with care for overflow)
	__m128i x4 = _mm_shuffle_epi32(x3, _MM_SHUFFLE(3, 1, 2, 0)); // llllttttlllltttt (spares a phadd later)
	__m128i x5 = _mm_maddubs_epi16(x4, _mm_setr_epi8(4, 3, 2, 1, -4, -3, -2, -1, -1, -2, -3, -4, 1, 2, 3, 4));
	__m128i x6 = _mm_add_epi16(x5, _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i VH = _mm_add_epi16(x6, _mm_shufflelo_epi16(x6, _MM_SHUFFLE(2, 3, 0, 1))); // V in 1st quarter, H in 2nd, both in [-2550,2550]
	__m128i x7 = _mm_add_epi16(VH, _mm_srai_epi16(VH, 4)); // (17 * VH) >> 4, in [-2710,2709]
	__m128i x8 = _mm_srai_epi16(_mm_sub_epi16(x7, _mm_set1_epi16(-1)), 1); // (17 * VH + 16) >> 5
	__m128i x9 = _mm_add_epi16(_mm_srli_si128(l7, 7), _mm_srli_si128(t1, 3));
	__m128i a = _mm_slli_epi16(_mm_sub_epi16(_mm_broadcastw_epi16(x9), _mm_set1_epi16(-1)), 4); // in [16,8176]
	__m128i b = _mm_shuffle_epi32(x8, _MM_SHUFFLE(1, 1, 1, 1));
	__m128i c = _mm_shuffle_epi32(x8, _MM_SHUFFLE(0, 0, 0, 0));
	
	// compute prediction vectors and store them in memory
	__m128i xA = _mm_mullo_epi16(b, _mm_setr_epi16(-3, -2, -1, 0, 1, 2, 3, 4));
	__m128i c1 = _mm_add_epi16(c, c);
	__m128i p1 = _mm_add_epi16(_mm_sub_epi16(a, c1), xA);
	__m128i p0 = _mm_sub_epi16(p1, c);
	v2li xB = (v2li)_mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
	*(int64_t *)(px0              ) = xB[0];
	*(int64_t *)(px0 +  stride    ) = xB[1];
	p0 = _mm_add_epi16(p0, c1);
	p1 = _mm_add_epi16(p1, c1);
	v2li xC = (v2li)_mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
	*(int64_t *)(px0 +  stride * 2) = xC[0];
	*(int64_t *)(px7 + nstride * 4) = xC[1];
	p0 = _mm_add_epi16(p0, c1);
	p1 = _mm_add_epi16(p1, c1);
	v2li xD = (v2li)_mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
	*(int64_t *)(px0 +  stride * 4) = xD[0];
	*(int64_t *)(px7 + nstride * 2) = xD[1];
	p0 = _mm_add_epi16(p0, c1);
	p1 = _mm_add_epi16(p1, c1);
	v2li xE = (v2li)_mm_packus_epi16(_mm_srai_epi16(p0, 5), _mm_srai_epi16(p1, 5));
	*(int64_t *)(px7 + nstride    ) = xE[0];
	*(int64_t *)(px7              ) = xE[1];
}

void _decode_intraChroma(int mode, uint8_t *Cb0, uint8_t *Cb7, uint8_t *Cr0, uint8_t *Cr7, size_t stride, ssize_t nstride, v8hi clip) {
	switch (mode) {
	case IC8x8_DC_8:
		chroma8x8_DC_8bit(Cb0, Cb7, stride, nstride);
		chroma8x8_DC_8bit(Cr0, Cr7, stride, nstride);
		break;
	case IC8x8_DCA_8:
		chroma8x8_DC_A_8bit(Cb0, Cb7, stride, nstride);
		chroma8x8_DC_A_8bit(Cr0, Cr7, stride, nstride);
		break;
	case IC8x8_DCB_8:
		chroma8x8_DC_B_8bit(Cb0, Cb7, stride, nstride);
		chroma8x8_DC_B_8bit(Cr0, Cr7, stride, nstride);
		break;
	case IC8x8_DCAB_8:
		chroma8x8_DC_AB_8bit(Cb0, Cb7, stride, nstride);
		chroma8x8_DC_AB_8bit(Cr0, Cr7, stride, nstride);
		break;
	case IC8x8_H_8:
		chroma8x8_horizontal_8bit(Cb0, Cb7, stride, nstride);
		chroma8x8_horizontal_8bit(Cr0, Cr7, stride, nstride);
		break;
	case IC8x8_V_8:
		chroma8x8_vertical_8bit(Cb0, Cb7, stride, nstride);
		chroma8x8_vertical_8bit(Cr0, Cr7, stride, nstride);
		break;
	case IC8x8_P_8:
		chroma8x8_plane_8bit(Cb0, Cb7, stride, nstride);
		chroma8x8_plane_8bit(Cr0, Cr7, stride, nstride);
		break;
	default: __builtin_unreachable();
	}
}



/**
 * Legacy functions kept to help implement 16-bit and 4:2:2.
static noinline __m128i FUNC(filter8_left_16bit, size_t stride, ssize_t nstride, uint8_t *p, ssize_t lt) {
	uint8_t *q = p + stride * 4;
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + nstride     - 16), *(__m128i *)(p               - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p +  stride * 2 - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(q + nstride     - 16), *(__m128i *)(q               - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(q +  stride     - 16), *(__m128i *)(q +  stride * 2 - 16));
	__m128i x4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x0, x1), _mm_unpackhi_epi32(x2, x3));
	__m128i x5 = _mm_alignr_epi8(x4, *(__m128i *)(p + lt - 16), 14);
	__m128i x6 = _mm_shufflehi_epi16(_mm_srli_si128(x4, 2), _MM_SHUFFLE(2, 2, 1, 0));
	return lowpass_16bit(x5, x4, x6);
}
static noinline __m128i FUNC(filter8_top_left_16bit, size_t stride, ssize_t nstride, uint8_t *p, __m128i zero, ssize_t lt, __m128i tr, __m128i tl) {
	uint8_t *q = p + stride * 4;
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + nstride     - 16), *(__m128i *)(p +      lt     - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p +  stride     - 16), *(__m128i *)(p +             - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(q + nstride     - 16), *(__m128i *)(p +  stride * 2 - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(q +  stride     - 16), *(__m128i *)(q               - 16));
	__m128i x4 = _mm_unpackhi_epi64(_mm_unpackhi_epi32(x3, x2), _mm_unpackhi_epi32(x1, x0));
	__m128i x5 = _mm_alignr_epi8(x4, *(__m128i *)(q +  stride * 2 - 16), 14);
	__m128i x6 = _mm_alignr_epi8(x5, *(__m128i *)(q +  stride * 2 - 16), 14);
	//ctx->pred_buffer_v[0] = (v8hi)lowpass_16bit(x4, x5, x6);
	//ctx->pred_buffer[8] = (*(uint16_t *)(p + nstride - 2) + *(uint16_t *)(p + nstride * 2 - 2) * 2 + *(uint16_t *)(p + nstride * 2) + 2) >> 2;
	return lowpass_16bit(tl, *(__m128i *)(p + nstride * 2), tr);
}

static void FUNC(decode_Horizontal4x4_16bit, size_t stride, ssize_t nstride, uint8_t *p) {
	__m128i x0 = _mm_set_epi64(*(__m64 *)(p +             - 8), *(__m64 *)(p + nstride     - 8));
	__m128i x1 = _mm_set_epi64(*(__m64 *)(p +  stride * 2 - 8), *(__m64 *)(p +  stride     - 8));
	__m128i x2 = _mm_shufflelo_epi16(x0, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x3 = _mm_shufflelo_epi16(x1, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x4 = _mm_shufflehi_epi16(x2, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i x5 = _mm_shufflehi_epi16(x3, _MM_SHUFFLE(3, 3, 3, 3));
}

static void FUNC(decode_HorizontalUp4x4_16bit, size_t stride, ssize_t nstride, uint8_t *p) {
	__m64 m0 = _mm_shuffle_pi16(*(__m64 *)(p +  stride * 2 - 8), _MM_SHUFFLE(3, 3, 3, 3));
	__m64 m1 = _mm_alignr_pi8(m0, *(__m64 *)(p +  stride     - 8), 6);
	__m64 m2 = _mm_alignr_pi8(m1, *(__m64 *)(p               - 8), 6);
	__m64 m3 = _mm_alignr_pi8(m2, *(__m64 *)(p + nstride     - 8), 6);
	__m64 m4 = _mm_avg_pu16(m2, m3);
	__m64 m5 =  _mm_avg_pu16(_mm_srli_pi16(_mm_add_pi16(m1, m3), 1), m2);
	__m128i x0 = _mm_unpacklo_epi16(_mm_movpi64_epi64(m4), _mm_movpi64_epi64(m5));
	__m128i x1 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 1, 1, 0));
	__m128i x2 = _mm_shuffle_epi32(x0, _MM_SHUFFLE(3, 3, 3, 2));
}

static void FUNC(decode_DC16x16_16bit, __m128i topr, __m128i topl, __m128i leftt, __m128i leftb) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_adds_epu16(_mm_add_epi16(topr, topl), _mm_add_epi16(leftt, leftb));
	__m128i x1 = _mm_add_epi32(_mm_unpacklo_epi16(x0, zero), _mm_unpackhi_epi16(x0, zero));
	__m128i x2 = _mm_add_epi32(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i x3 = _mm_add_epi32(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x4 = _mm_srli_epi32(x3, 4);
	__m128i DC = _mm_avg_epu16(_mm_packs_epi32(x4, x4), zero);
	//ctx->pred_buffer_v[0] = (v8hi)DC;
}

static void FUNC(decode_Plane16x16_16bit, __m128i topr, __m128i topl, __m128i leftt, __m128i leftb) {
	// sum the samples and compute a, b, c
	__m128i mul0 = (__m128i)(v8hi){5, 10, 15, 20, 25, 30, 35, 40};
	__m128i mul1 = (__m128i)(v8hi){-40, -35, -30, -25, -20, -15, -10, -5};
	__m128i x0 = _mm_add_epi32(_mm_madd_epi16(topr, mul0), _mm_madd_epi16(topl, mul1));
	__m128i x1 = _mm_add_epi32(_mm_madd_epi16(leftb, mul0), _mm_madd_epi16(leftt, mul1));
	__m128i x2 = _mm_hadd_epi32(x0, x1);
	__m128i HV = _mm_add_epi32(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1))); // HHVV
	__m128i x3 = _mm_srai_epi32(_mm_add_epi32(HV, _mm_set1_epi32(32)), 6); // (5 * HV + 32) >> 6
	__m128i x4 = _mm_shuffle_epi32(_mm_srli_si128(_mm_add_epi16(topr, leftb), 14), 0);
	__m128i a = _mm_slli_epi32(_mm_sub_epi32(x4, _mm_set1_epi32(-1)), 4);
	__m128i b = _mm_unpacklo_epi64(x3, x3);
	__m128i c = _mm_unpackhi_epi64(x3, x3);
	
	// compute the first row of prediction vectors
	//ctx->pred_buffer_v[16] = (v8hi)c;
	__m128i x5 = _mm_sub_epi32(_mm_add_epi32(a, c), _mm_slli_epi32(c, 3)); // a - c * 7 + 16
	__m128i x6 = _mm_add_epi32(b, _mm_slli_si128(b, 4));
	__m128i x7 = _mm_add_epi32(x6, _mm_slli_si128(x6, 8));
	__m128i b2 = _mm_shuffle_epi32(x7, _MM_SHUFFLE(3, 3, 3, 3));
	__m128i p2 = _mm_add_epi32(x5, x7);
	__m128i p1 = _mm_sub_epi32(p2, b2);
	__m128i p3 = _mm_add_epi32(p2, b2);
	__m128i p0 = _mm_sub_epi32(p1, b2);
	
	// store them
	__m128i c2 = _mm_slli_epi32(c, 2);
	//ctx->pred_buffer_v[0] = (v8hi)p0;
	//ctx->pred_buffer_v[1] = (v8hi)p1;
	//ctx->pred_buffer_v[4] = (v8hi)p2;
	//ctx->pred_buffer_v[5] = (v8hi)p3;
	//ctx->pred_buffer_v[2] = (v8hi)(p0 = _mm_add_epi32(p0, c2));
	//ctx->pred_buffer_v[3] = (v8hi)(p1 = _mm_add_epi32(p1, c2));
	//ctx->pred_buffer_v[6] = (v8hi)(p2 = _mm_add_epi32(p2, c2));
	//ctx->pred_buffer_v[7] = (v8hi)(p3 = _mm_add_epi32(p3, c2));
	//ctx->pred_buffer_v[8] = (v8hi)(p0 = _mm_add_epi32(p0, c2));
	//ctx->pred_buffer_v[9] = (v8hi)(p1 = _mm_add_epi32(p1, c2));
	//ctx->pred_buffer_v[12] = (v8hi)(p2 = _mm_add_epi32(p2, c2));
	//ctx->pred_buffer_v[13] = (v8hi)(p3 = _mm_add_epi32(p3, c2));
	//ctx->pred_buffer_v[10] = (v8hi)_mm_add_epi32(p0, c2);
	//ctx->pred_buffer_v[11] = (v8hi)_mm_add_epi32(p1, c2);
	//ctx->pred_buffer_v[14] = (v8hi)_mm_add_epi32(p2, c2);
	//ctx->pred_buffer_v[15] = (v8hi)_mm_add_epi32(p3, c2);
}

static void FUNC(decode_ChromaDC8x8_16bit, __m128i top03, __m128i left03, __m128i dc12) {
	__m128i x0 = _mm_add_epi16(top03, left03);
	__m128i x1 = _mm_add_epi16(x0, _mm_shuffle_epi32(x0, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x2 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 3, 0, 1)), _MM_SHUFFLE(2, 3, 0, 1));
	__m128i x3 = _mm_srli_epi16(_mm_avg_epu16(_mm_add_epi16(x1, _mm_set1_epi16(3)), x2), 2);
	__m128i x4 = _mm_add_epi16(dc12, _mm_shuffle_epi32(dc12, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x5 = _mm_avg_epu16(_mm_srli_epi16(_mm_hadd_epi16(x4, x4), 1), _mm_setzero_si128());
	//__m128i *buf = (__m128i *)&ctx->pred_buffer_v[BlkIdx & 15];
	//buf[0] = _mm_unpacklo_epi64(x3, x3);
	//buf[1] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	//buf[2] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	//buf[3] = _mm_unpackhi_epi64(x3, x3);
}

static void FUNC(decode_ChromaPlane8x8_16bit, size_t stride, ssize_t nstride, uint8_t *p, __m128i *buf) {
	static const v8hi mul = {17, 34, 51, 68, 68, 51, 34, 17};
	uint8_t *q = p + stride * 4;
	__m128i x0 = _mm_unpackhi_epi16(*(__m128i *)(p + nstride * 2 - 16), *(__m128i *)(p + nstride     - 16));
	__m128i x1 = _mm_unpackhi_epi16(*(__m128i *)(p               - 16), *(__m128i *)(p +  stride     - 16));
	__m128i x2 = _mm_unpackhi_epi16(*(__m128i *)(q +  stride * 2 - 16), *(__m128i *)(q +  stride     - 16));
	__m128i x3 = _mm_unpackhi_epi16(*(__m128i *)(q               - 16), *(__m128i *)(q + nstride     - 16));
	__m128i x4 = (__m128i)_mm_loadl_pi((__m128)_mm_unpackhi_epi32(x0, x1), (__m64 *)(p + nstride * 2 - 2));
	__m128i x5 = (__m128i)_mm_loadl_pi((__m128)_mm_unpackhi_epi32(x2, x3), (__m64 *)(p + nstride * 2 + 8));
	__m128i x6 = _mm_shufflelo_epi16(x4, _MM_SHUFFLE(0, 1, 2, 3));
	
	// sum the samples and compute a, b, c
	__m128i x7 = _mm_madd_epi16(_mm_sub_epi16(x5, x6), (__m128i)mul);
	__m128i HV = _mm_add_epi32(x7, _mm_shuffle_epi32(x7, _MM_SHUFFLE(2, 3, 0, 1))); // HHVV
	__m128i x8 = _mm_srai_epi32(_mm_add_epi32(HV, _mm_set1_epi32(16)), 5);
	__m128i a = _mm_set1_epi32((*(uint16_t *)(p + nstride * 2 + 14) + *(uint16_t *)(q + stride * 2 - 2) + 1) * 16);
	__m128i b = _mm_unpacklo_epi64(x8, x8);
	__m128i c = _mm_unpackhi_epi64(x8, x8);
	
	// compute and store the first row of prediction vectors
	//ctx->pred_buffer_v[17] = (v8hi)c;
	__m128i c2 = _mm_slli_epi32(c, 2);
	__m128i x9 = _mm_sub_epi32(_mm_add_epi32(a, c), c2); // a - c * 3 + 16
	__m128i xA = _mm_add_epi32(b, _mm_slli_si128(b, 4));
	__m128i xB = _mm_add_epi32(xA, _mm_slli_si128(xA, 8));
	__m128i p1 = _mm_add_epi32(x9, xB);
	__m128i p0 = _mm_sub_epi32(p1, _mm_shuffle_epi32(xB, _MM_SHUFFLE(3, 3, 3, 3)));
	buf[0] = p0;
	buf[1] = p1;
	buf[2] = _mm_add_epi32(p0, c2);
	buf[3] = _mm_add_epi32(p1, c2);
}

static void FUNC(decode_ChromaDC8x16_8bit, __m128i dc03, __m128i dc2146, __m128i dc57) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_unpacklo_epi32(dc2146, dc2146);
	__m128i x1 = _mm_unpackhi_epi32(dc2146, dc2146);
	__m128i x2 = _mm_packs_epi32(_mm_sad_epu8(dc03, zero), _mm_sad_epu8(dc57, zero));
	__m128i x3 = _mm_packs_epi32(_mm_sad_epu8(x0, zero), _mm_sad_epu8(x1, zero));
	__m128i x4 = _mm_avg_epu16(_mm_srli_epi16(_mm_packs_epi32(x2, x3), 2), zero);
	__m128i x5 = _mm_unpacklo_epi16(x4, x4);
	__m128i x6 = _mm_unpackhi_epi16(x4, x4);
	//__m128i *buf = (__m128i *)&ctx->pred_buffer_v[BlkIdx & 15];
	//buf[0] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	//buf[1] = _mm_shuffle_epi32(x6, _MM_SHUFFLE(1, 1, 1, 1));
	//buf[2] = _mm_shuffle_epi32(x6, _MM_SHUFFLE(0, 0, 0, 0));
	//buf[3] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	//buf[4] = _mm_shuffle_epi32(x6, _MM_SHUFFLE(2, 2, 2, 2));
	//buf[5] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 2, 2, 2));
	//buf[6] = _mm_shuffle_epi32(x6, _MM_SHUFFLE(3, 3, 3, 3));
	//buf[7] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
	JUMP(transform_dc2x4);
}

static void FUNC(decode_ChromaDC8x16_16bit, __m128i top03, __m128i left03, __m128i top57, __m128i left57, __m128i dc12, __m128i dc46) {
	__m128i x0 = _mm_hadd_epi16(_mm_add_epi16(top03, left03), _mm_add_epi16(top57, left57));
	__m128i x1 = _mm_hadd_epi16(dc12, dc46);
	__m128i x2 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(x0, _MM_SHUFFLE(2, 3, 0, 1)), _MM_SHUFFLE(2, 3, 0, 1));
	__m128i x3 = _mm_shufflelo_epi16(_mm_shufflehi_epi16(x1, _MM_SHUFFLE(2, 3, 0, 1)), _MM_SHUFFLE(2, 3, 0, 1));
	__m128i x4 = _mm_srli_epi16(_mm_avg_epu16(_mm_add_epi16(x0, _mm_set1_epi16(3)), x2), 2);
	__m128i x5 = _mm_avg_epu16(_mm_srli_epi16(_mm_add_epi16(x1, x3), 1), _mm_setzero_si128());
	//__m128i *buf = (__m128i *)&ctx->pred_buffer_v[BlkIdx & 15];
	//buf[0] = _mm_shuffle_epi32(x4, _MM_SHUFFLE(0, 0, 0, 0));
	//buf[1] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(0, 0, 0, 0));
	//buf[2] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(1, 1, 1, 1));
	//buf[3] = _mm_shuffle_epi32(x4, _MM_SHUFFLE(1, 1, 1, 1));
	//buf[4] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(2, 2, 2, 2));
	//buf[5] = _mm_shuffle_epi32(x4, _MM_SHUFFLE(2, 2, 2, 2));
	//buf[6] = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
	//buf[7] = _mm_shuffle_epi32(x4, _MM_SHUFFLE(3, 3, 3, 3));
	JUMP(transform_dc2x4);
}

static void FUNC(decode_ChromaHorizontal8x16, __m128i leftt, __m128i leftb) {
	//__m128i *buf = (__m128i *)&ctx->pred_buffer_v[BlkIdx & 15];
	//buf[0] = buf[1] = _mm_unpacklo_epi16(leftt, leftt);
	//buf[2] = buf[3] = _mm_unpackhi_epi16(leftt, leftt);
	//buf[4] = buf[5] = _mm_unpacklo_epi16(leftb, leftb);
	//buf[6] = buf[7] = _mm_unpackhi_epi16(leftb, leftb);
	JUMP(transform_dc2x4);
}

static void FUNC(decode_ChromaPlane8x16, __m128i top, __m128i leftt, __m128i leftb) {
	static const v8hi mulT = {-136, -102, -68, -34, 34, 68, 102, 136};
	static const v8hi mulLT = {-40, -35, -30, -25, -20, -15, -10, -5};
	static const v8hi mulLB = {5, 10, 15, 20, 25, 30, 35, 40};
	
	// sum the samples and compute a, b, c
	__m128i x0 = _mm_madd_epi16(top, (__m128i)mulT);
	__m128i x1 = _mm_madd_epi16(leftt, (__m128i)mulLT);
	__m128i x2 = _mm_madd_epi16(leftb, (__m128i)mulLB);
	__m128i x3 = _mm_hadd_epi32(x0, _mm_add_epi32(x1, x2));
	__m128i HV = _mm_add_epi32(x3, _mm_shuffle_epi32(x3, _MM_SHUFFLE(2, 3, 0, 1))); // HHVV
	__m128i x4 = _mm_srai_epi32(_mm_add_epi32(HV, _mm_set1_epi32(32)), 6); // (HV * {34,5} + 32) >> 6
	__m128i x5 = _mm_shuffle_epi32(_mm_srli_si128(_mm_add_epi16(top, leftb), 14), 0);
	__m128i a = _mm_slli_epi32(_mm_sub_epi32(x5, _mm_set1_epi32(-1)), 4);
	__m128i b = _mm_unpacklo_epi64(x4, x4);
	__m128i c = _mm_unpackhi_epi64(x4, x4);
	
	// compute the first row of prediction vectors
	//ctx->pred_buffer_v[17] = (v8hi)c;
	__m128i x6 = _mm_sub_epi32(_mm_add_epi32(a, c), _mm_slli_epi32(c, 3)); // a - c * 7 + 16
	__m128i x7 = _mm_add_epi32(b, _mm_slli_si128(b, 4));
	__m128i x8 = _mm_add_epi32(x7, _mm_slli_si128(x7, 8));
	__m128i p1 = _mm_add_epi32(x6, x8);
	__m128i p0 = _mm_sub_epi32(p1, _mm_shuffle_epi32(x8, _MM_SHUFFLE(3, 3, 3, 3)));
	__m128i c2 = _mm_slli_epi32(c, 2);
	
	// 8bit mode can use the same add-and-store sequence
	if (ctx->ps.BitDepth_C == 8) {
		//ctx->pred_buffer_v[16] = (v8hi)_mm_slli_epi16(_mm_packs_epi32(c, c), 1);
		p0 = _mm_packs_epi32(p0, _mm_add_epi32(p0, c));
		p1 = _mm_packs_epi32(p1, _mm_add_epi32(p1, c));
		c2 = _mm_packs_epi32(c2, c2);
	}
	//__m128i *buf = (__m128i *)&ctx->pred_buffer_v[BlkIdx & 15];
	//buf[0] = p0;
	//buf[1] = p1;
	//buf[2] = (p0 = _mm_add_epi32(p0, c2));
	//buf[3] = (p1 = _mm_add_epi32(p1, c2));
	//buf[4] = (p0 = _mm_add_epi32(p0, c2));
	//buf[5] = (p1 = _mm_add_epi32(p1, c2));
	//buf[6] = _mm_add_epi32(p0, c2);
	//buf[7] = _mm_add_epi32(p1, c2);
	JUMP(transform_dc2x4);
}

static void FUNC(decode_DC8x8_16bit, __m128i top, __m128i left) {
	__m128i zero = _mm_setzero_si128();
	__m128i x0 = _mm_add_epi16(top, left);
	__m128i x1 = _mm_add_epi32(_mm_unpacklo_epi16(x0, zero), _mm_unpackhi_epi16(x0, zero));
	__m128i x2 = _mm_add_epi32(x1, _mm_shuffle_epi32(x1, _MM_SHUFFLE(1, 0, 3, 2)));
	__m128i x3 = _mm_add_epi32(x2, _mm_shuffle_epi32(x2, _MM_SHUFFLE(2, 3, 0, 1)));
	__m128i x4 = _mm_srli_epi32(x3, 3);
	__m128i DC = _mm_avg_epu16(_mm_packs_epi32(x4, x4), zero);
	JUMP(decode_Residual8x8, DC, DC, DC, DC, DC, DC, DC, DC);
}

static void FUNC(decode_DiagonalDownLeft8x8, __m128i right, __m128i top, __m128i topl) {
	__m128i topr = _mm_alignr_epi8(right, top, 2);
	__m128i rightl = _mm_alignr_epi8(right, top, 14);
	__m128i rightr = _mm_shufflehi_epi16(_mm_srli_si128(right, 2), _MM_SHUFFLE(2, 2, 1, 0));
	__m128i x0 = lowpass_16bit(topl, top, topr);
	__m128i x1 = lowpass_16bit(rightl, right, rightr);
	__m128i x2 = _mm_alignr_epi8(x1, x0, 2);
	__m128i x3 = _mm_alignr_epi8(x1, x0, 4);
	__m128i x4 = _mm_srli_si128(x1, 2);
	__m128i x5 = _mm_shufflehi_epi16(_mm_shuffle_epi32(x1, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x6 = lowpass_16bit(x0, x2, x3);
	__m128i x7 = lowpass_16bit(x1, x4, x5);
	__m128i x8 = _mm_alignr_epi8(x7, x6, 2);
	__m128i x9 = _mm_alignr_epi8(x7, x6, 4);
	__m128i xA = _mm_alignr_epi8(x7, x6, 6);
	__m128i xB = _mm_alignr_epi8(x7, x6, 8);
	__m128i xC = _mm_alignr_epi8(x7, x6, 10);
	__m128i xD = _mm_alignr_epi8(x7, x6, 12);
	__m128i xE = _mm_alignr_epi8(x7, x6, 14);
	JUMP(decode_Residual8x8, x6, x8, x9, xA, xB, xC, xD, xE);
}

static void FUNC(decode_DiagonalDownRight8x8, __m128i top) {
	__m128i left = top;//(__m128i)ctx->pred_buffer_v[0];
	__m128i lt = top;//_mm_loadu_si128((__m128i *)(ctx->pred_buffer + 1));
	__m128i lb = _mm_slli_si128(left, 2);
	__m128i tl = _mm_alignr_epi8(top, lt, 14);
	__m128i tll = _mm_alignr_epi8(top, lt, 12);
	__m128i x0 = lowpass_16bit(lb, left, lt);
	__m128i x1 = lowpass_16bit(tll, tl, top);
	__m128i x2 = _mm_alignr_epi8(x1, x0, 14);
	__m128i x3 = _mm_alignr_epi8(x1, x0, 12);
	__m128i x4 = _mm_alignr_epi8(x1, x0, 10);
	__m128i x5 = _mm_alignr_epi8(x1, x0, 8);
	__m128i x6 = _mm_alignr_epi8(x1, x0, 6);
	__m128i x7 = _mm_alignr_epi8(x1, x0, 4);
	__m128i x8 = _mm_alignr_epi8(x1, x0, 2);
	JUMP(decode_Residual8x8, x1, x2, x3, x4, x5, x6, x7, x8);
}

static void FUNC(decode_VerticalRight8x8, __m128i top) {
	__m128i lt = top;//_mm_loadu_si128((__m128i *)(ctx->pred_buffer + 1));
	__m128i x0 = _mm_slli_si128(lt, 2);
	__m128i x1 = _mm_shuffle_epi32(lt, _MM_SHUFFLE(2, 1, 0, 0));
	__m128i x2 = _mm_alignr_epi8(top, lt, 14);
	__m128i x3 = _mm_alignr_epi8(top, lt, 12);
	__m128i x4 = _mm_avg_epu16(top, x2);
	__m128i x5 = lowpass_16bit(lt, x0, x1);
	__m128i x6 = lowpass_16bit(top, x2, x3);
	__m128i x7 = _mm_alignr_epi8(x4, x5, 14);
	__m128i x8 = _mm_alignr_epi8(x6, x5 = _mm_slli_si128(x5, 2), 14);
	__m128i x9 = _mm_alignr_epi8(x7, x5 = _mm_slli_si128(x5, 2), 14);
	__m128i xA = _mm_alignr_epi8(x8, x5 = _mm_slli_si128(x5, 2), 14);
	__m128i xB = _mm_alignr_epi8(x9, x5 = _mm_slli_si128(x5, 2), 14);
	__m128i xC = _mm_alignr_epi8(xA, _mm_slli_si128(x5, 2), 14);
	JUMP(decode_Residual8x8, x4, x6, x7, x8, x9, xA, xB, xC);
}

static void FUNC(decode_HorizontalDown8x8, __m128i top) {
	__m128i left = top;//(__m128i)ctx->pred_buffer_v[0];
	__m128i topl = top;//_mm_alignr_epi8(top, _mm_loadu_si128((__m128i *)(ctx->pred_buffer + 1)), 14);
	__m128i x0 = _mm_alignr_epi8(topl, left, 2);
	__m128i x1 = _mm_alignr_epi8(topl, left, 4);
	__m128i x2 = _mm_srli_si128(topl, 2);
	__m128i x3 = _mm_shuffle_epi32(topl, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x4 = _mm_avg_epu16(left, x0);
	__m128i x5 = lowpass_16bit(left, x0, x1);
	__m128i x6 = lowpass_16bit(topl, x2, x3);
	__m128i x7 = _mm_unpackhi_epi16(x4, x5);
	__m128i x8 = _mm_unpacklo_epi16(x4, x5);
	__m128i x9 = _mm_alignr_epi8(x6, x7, 12);
	__m128i xA = _mm_alignr_epi8(x6, x7, 8);
	__m128i xB = _mm_alignr_epi8(x6, x7, 4);
	__m128i xC = _mm_alignr_epi8(x7, x8, 12);
	__m128i xD = _mm_alignr_epi8(x7, x8, 8);
	__m128i xE = _mm_alignr_epi8(x7, x8, 4);
	JUMP(decode_Residual8x8, x9, xA, xB, x7, xC, xD, xE, x8);
}

static void FUNC(decode_VerticalLeft8x8, __m128i right, __m128i top, __m128i topl) {
	__m128i topr = _mm_alignr_epi8(right, top, 2);
	__m128i rightl = _mm_alignr_epi8(right, top, 14);
	__m128i rightr = _mm_shufflehi_epi16(_mm_srli_si128(right, 2), _MM_SHUFFLE(2, 2, 1, 0));
	__m128i x0 = lowpass_16bit(topl, top, topr);
	__m128i x1 = lowpass_16bit(rightl, right, rightr);
	__m128i x2 = _mm_alignr_epi8(x1, x0, 2);
	__m128i x3 = _mm_alignr_epi8(x1, x0, 4);
	__m128i x4 = _mm_srli_si128(x1, 2);
	__m128i x5 = _mm_shuffle_epi32(x1, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i x6 = _mm_avg_epu16(x0, x2);
	__m128i x7 = _mm_avg_epu16(x1, x4);
	__m128i x8 = lowpass_16bit(x0, x2, x3);
	__m128i x9 = lowpass_16bit(x1, x4, x5);
	__m128i xA = _mm_alignr_epi8(x7, x6, 2);
	__m128i xB = _mm_alignr_epi8(x9, x8, 2);
	__m128i xC = _mm_alignr_epi8(x7, x6, 4);
	__m128i xD = _mm_alignr_epi8(x9, x8, 4);
	__m128i xE = _mm_alignr_epi8(x7, x6, 6);
	__m128i xF = _mm_alignr_epi8(x9, x8, 6);
	JUMP(decode_Residual8x8, x6, x8, xA, xB, xC, xD, xE, xF);
}

static void FUNC(decode_HorizontalUp8x8, __m128i left) {
	__m128i x0 = _mm_shufflehi_epi16(_mm_srli_si128(left, 2), _MM_SHUFFLE(2, 2, 1, 0));
	__m128i x1 = _mm_shufflehi_epi16(_mm_shuffle_epi32(left, _MM_SHUFFLE(3, 3, 2, 1)), _MM_SHUFFLE(1, 1, 1, 0));
	__m128i x2 = _mm_avg_epu16(left, x0);
	__m128i x3 = lowpass_16bit(left, x0, x1);
	__m128i x4 = _mm_unpacklo_epi16(x2, x3);
	__m128i x5 = _mm_unpackhi_epi16(x2, x3);
	__m128i x6 = _mm_alignr_epi8(x5, x4, 4);
	__m128i x7 = _mm_alignr_epi8(x5, x4, 8);
	__m128i x8 = _mm_alignr_epi8(x5, x4, 12);
	__m128i x9 = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 2, 1));
	__m128i xA = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 2));
	__m128i xB = _mm_shuffle_epi32(x5, _MM_SHUFFLE(3, 3, 3, 3));
	JUMP(decode_Residual8x8, x4, x6, x7, x8, x5, x9, xA, xB);
}
 */
