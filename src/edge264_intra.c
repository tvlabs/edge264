/**
 * Intra decoding involves a lot of suffling for which I use compatible
 * instructions from both SSE and NEON, with a few exceptions where a common
 * ground would be too suboptimal.
 * 
 * Each decoding function is a large switch with a downward tree structure for
 * sharing code. Cases start at the leaves and branch towards the exit root.
 * While there is no way to make compilers generate proper code with vanilla
 * switches or functions, goto instructions are used for internal branches.
 * 
 * Choosing between the different possibilities of a same function is tricky,
 * in general I favor in order:
 * _ the fastest code, obviously
 * _ a short dependency chain (more freedom for compilers to reorder),
 * _ less instructions (while avoiding excessive use of shuffle for ARM),
 * _ readable code.
 * 
 * My thumb rules:
 * _ Aligned reads are favored if they incur no additional instructions.
 * _ shuffle is used iff doing otherwise would require 3+ instructions.
 * _ Favor vector over scalar code to avoid callee-save conventions.
 */

#include "edge264_internal.h"

#if defined(__wasm_simd128__)
	#define spreadh8(a) (i8x16)__builtin_shufflevector((i8x16)(a), (i8x16){}, 0, 1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7)
	#define spreadq8(a) (i8x16)__builtin_shufflevector((i8x16)(a), (i8x16){}, 0, 1, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3)
	static i8x16 lowpass8(i8x16 l, i8x16 m, i8x16 r) {return avgu8(subu8(avgu8(l, r), (l ^ r) & set8(1)), m);}
#elif defined(__SSE2__)
	#define spreadh8(a) shuffle(a, (i8x16){0, 1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7})
	#define spreadq8(a) shuffle(a, (i8x16){0, 1, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3})
	static always_inline i8x16 lowpass8(i8x16 l, i8x16 m, i8x16 r) {return avgu8(subu8(avgu8(l, r), (l ^ r) & set8(1)), m);}
#elif defined(__ARM_NEON)
	#define addlou8(a, b) (i16x8)vaddl_u8(vget_low_s8(a), vget_low_s8(b))
	#define lowpass8(l, m, r) (i8x16)vrhaddq_u8(vhaddq_u8(l, r), m)
	#define sublou8(a, b) (i16x8)vsubl_u8(vget_low_s8(a), vget_low_s8(b))
	static always_inline i8x16 spreadh8(i8x16 a) {return vzip1q_s64(a, vdupq_laneq_s8(a, 7));}
	static always_inline i8x16 spreadq8(i8x16 a) {return vextq_s8(vextq_s8(a, a, 4), vdupq_laneq_s8(a, 3), 12);}
#endif

#if defined(__wasm_simd128__) || defined(__ARM_NEON)
	static i8x16 ldleft4x4(const uint8_t *p, size_t stride) {
		i8x16 v = {*(p -= 1)};
		v[1] = *(p += stride);
		v[2] = *(p += stride);
		v[3] = *(p + stride);
		return v;
	}
	static i8x16 ldedge4x4(const uint8_t *p, size_t stride) {
		i8x16 v = loadu128(p - 5 - stride);
		v[3] = *(p -= 1);
		v[2] = *(p += stride);
		v[1] = *(p += stride);
		v[0] = *(p + stride);
		return v;
	}
	static i8x16 ldleft8x8(const uint8_t *p, size_t stride, i8x16 v0) {
		v0[1] = *(p -= 1);
		v0[2] = *(p += stride);
		v0[3] = *(p += stride);
		v0[4] = *(p += stride);
		v0[5] = *(p += stride);
		v0[6] = *(p += stride);
		v0[7] = *(p += stride);
		i8x16 v1 = ziplo64(v0, set8(*(p + stride)));
		return lowpass8(shr128(v1, 2), shr128(v1, 1), v1);
	}
	static i8x16 ldedge8x8(const uint8_t *p, size_t stride) {
		i8x16 v = {};
		v[15] = *(p -= 1);
		v[14] = *(p += stride);
		v[13] = *(p += stride);
		v[12] = *(p += stride);
		v[11] = *(p += stride);
		v[10] = *(p += stride);
		v[9] = *(p += stride);
		v[8] = v[7] = *(p + stride);
		return v;
	}
	static i8x16 ldleftC(const uint8_t *p, size_t stride) {
		i8x16 v = {*(p -= 1)};
		v[8] = *(p += stride);
		v[1] = *(p += stride);
		v[9] = *(p += stride);
		v[2] = *(p += stride);
		v[10] = *(p += stride);
		v[3] = *(p += stride);
		v[11] = *(p += stride);
		v[4] = *(p += stride);
		v[12] = *(p += stride);
		v[5] = *(p += stride);
		v[13] = *(p += stride);
		v[6] = *(p += stride);
		v[14] = *(p += stride);
		v[7] = *(p += stride);
		v[15] = *(p + stride);
		return v;
	}
#elif defined(__SSE4_1__)
	static i8x16 ldleft4x4(const uint8_t *p, size_t stride) {
		i8x16 v = {};
		v[0] = p[-1];
		v[1] = p[stride - 1];
		v[2] = p[stride * 2 - 1];
		v[3] = p[stride * 3 - 1];
		return v;
	}
	static i8x16 ldedge4x4(const uint8_t *p, size_t stride) {
		i8x16 v = loadu128(p - stride - 5);
		v[3] = p[-1];
		v[2] = p[stride - 1];
		v[1] = p[stride * 2 - 1];
		v[0] = p[stride * 3 - 1];
		return v;
	}
	static i8x16 ldleft8x8(const uint8_t *p, size_t stride, i8x16 v) {
		const uint8_t *p0 = p - 1;
		const uint8_t *p4 = p0 + stride * 4;
		size_t stride3 = stride * 3;
		v[1] = *p0;
		v[2] = p0[stride];
		v[3] = p0[stride * 2];
		v[4] = p0[stride3];
		v[5] = *p4;
		v[6] = p4[stride];
		v[7] = p4[stride * 2];
		v[8] = v[9] = p4[stride3];
		return lowpass8(shr128(v, 2), shr128(v, 1), v);
	}
	static i8x16 ldedge8x8(const uint8_t *p, size_t stride) {
		const uint8_t *p0 = p - 1;
		const uint8_t *p4 = p + stride * 4 - 1;
		size_t stride3 = stride * 3;
		i8x16 a2h = set8(p4[stride3]);
		a2h[15] = *p0;
		a2h[14] = p0[stride];
		a2h[13] = p0[stride * 2];
		a2h[12] = p0[stride3];
		a2h[11] = *p4;
		a2h[10] = p4[stride];
		a2h[9] = p4[stride * 2];
		return a2h;
	}
	static i8x16 ldleftC(const uint8_t *p, size_t stride) {
		size_t stride3 = stride * 3;
		const uint8_t *p0 = p - 1;
		const uint8_t *p4 = p0 + stride * 4;
		const uint8_t *p8 = p0 + stride * 8;
		const uint8_t *pC = p4 + stride * 8;
		return (i8x16){*p0, p0[stride * 2], *p4, p4[stride * 2], *p8, p8[stride * 2], *pC, pC[stride * 2], p0[stride], p0[stride3], p4[stride], p4[stride3], p8[stride], p8[stride3], pC[stride], pC[stride3]};
	}
#elif defined(__SSE2__)
	static i8x16 ldleft4x4(const uint8_t *p, size_t stride) {
		p -= 4;
		i8x16 v0 = ziplo8(loada32(p             ), loada32(p + stride    ));
		i8x16 v1 = ziplo8(loada32(p + stride * 2), loada32(p + stride * 3));
		return shr128(ziplo16(v0, v1), 12);
	}
	static i8x16 ldedge4x4(const uint8_t *p, size_t stride) {
		i8x16 v0 = loadu64(p - stride     - 1);
		i8x16 v1 = loada32(p              - 4);
		i8x16 v2 = loada32(p + stride     - 4);
		i8x16 v3 = loada32(p + stride * 2 - 4);
		i8x16 v4 = loada32(p + stride * 3 - 4);
		return shrd128(ziplo16(ziplo8(v4, v3), ziplo8(v2, v1)), v0, 12);
	}
	static i8x16 ldleft8x8(const uint8_t *p, size_t stride, i8x16 v) {
		const uint8_t *p0 = p - 4;
		const uint8_t *p4 = p0 + stride * 4;
		size_t stride3 = stride * 3;
		i8x16 v0 = ziplo8((u32x4)v << 24, loada32(p0));
		i8x16 v1 = ziplo8(loada32(p0 + stride), loada32(p0 + stride * 2));
		i8x16 v2 = ziplo8(loada32(p0 + stride3), loada32(p4));
		i8x16 v3 = ziplo8(loada32(p4 + stride), loada32(p4 + stride * 2));
		i8x16 v4 = loada32(p4 + stride3 + 3);
		i8x16 v5 = ziphi32(ziplo16(v0, v1), ziplo16(v2, v3));
		i8x16 v6 = ziplo8(v4, v4);
		return lowpass8(shrd128(v5, v6, 8), shrd128(v5, v6, 9), shrd128(v5, v6, 10));
	}
	static i8x16 ldedge8x8(const uint8_t *p, size_t stride) {
		const uint8_t *p0 = p - 4;
		const uint8_t *p4 = p + stride * 4 - 8;
		size_t stride3 = stride * 3;
		i8x16 gh = ziplo8(loada32(p0 + stride), loada32(p0));
		i8x16 ef = ziplo8(loada32(p0 + stride3), loada32(p0 + stride * 2));
		i8x16 cd = ziplo8(loada64(p4 + stride), loada64(p4));
		i8x16 a = loada64(p4 + stride3);
		i8x16 ab = ziplo8(a, loada64(p4 + stride * 2));
		return shuffleps(a, ziphi32(ziphi16(ab, cd), ziplo16(ef, gh)), 0, 1, 2, 3);
	}
	static i8x16 ldleftC(const uint8_t *p, size_t stride) {
		size_t stride3 = stride * 3;
		const uint8_t *p0 = p - 4;
		const uint8_t *p4 = p0 + stride * 4;
		const uint8_t *p8 = p0 + stride * 8;
		const uint8_t *pC = p4 + stride * 8;
		i8x16 v0 = ziplo8(loada32(p0), loada32(p0 + stride * 2));
		i8x16 v1 = ziplo8(loada32(p4), loada32(p4 + stride * 2));
		i8x16 v2 = ziplo8(loada32(p8), loada32(p8 + stride * 2));
		i8x16 v3 = ziplo8(loada32(pC), loada32(pC + stride * 2));
		i8x16 v4 = ziplo8(loada32(p0 + stride), loada32(p0 + stride3));
		i8x16 v5 = ziplo8(loada32(p4 + stride), loada32(p4 + stride3));
		i8x16 v6 = ziplo8(loada32(p8 + stride), loada32(p8 + stride3));
		i8x16 v7 = ziplo8(loada32(pC + stride), loada32(pC + stride3));
		i8x16 v8 = ziphi32(ziplo16(v0, v1), ziplo16(v2, v3));
		i8x16 v9 = ziphi32(ziplo16(v4, v5), ziplo16(v6, v7));
		return ziphi64(v8, v9);
	}
#endif



/**
 * Intra 4x4
 */
static cold noinline void decode_intra4x4(uint8_t * restrict p, size_t stride, int mode, i16x8 clip) {
	static const i8x16 shuf[6] = {
		{0, 1, 2, 3, 1, 2, 3, 4, 2, 3, 4, 5, 3, 4, 5, 6},
		{3, 4, 5, 6, 2, 3, 4, 5, 1, 2, 3, 4, 0, 1, 2, 3},
		{12, 13, 14, 15, 3, 4, 5, 6, 2, 12, 13, 14, 1, 3, 4, 5},
		{11, 3, 4, 5, 10, 2, 11, 3, 9, 1, 10, 2, 8, 0, 9, 1},
		{8, 9, 10, 11, 0, 1, 2, 3, 9, 10, 11, 12, 1, 2, 3, 4},
		{8, 0, 9, 1, 9, 1, 10, 2, 10, 2, 11, 3, 11, 3, 12, 4},
	};
	const uint8_t *pT = p - stride;
	int idx;
	i32x4 v;
	switch (mode) {
	default: __builtin_unreachable();
	
	case I4x4_V_8:
		v = set32(*(int32_t *)pT);
		break;
	
	case I4x4_H_8: {
		i8x16 w = ldleft4x4(p, stride);
		v = shuffle(w, (i8x16){0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3});
		} break;
	
	case I4x4_DC_8:
		v = ziplo32(ldleft4x4(p, stride), loada32(pT));
	dc_4x4:
		v = broadcast8(shrru16(sumh8(v), 3), __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
		break;
	case I4x4_DC_A_8: {
		i8x16 w = loada32(pT);
		v = ziplo32(w, w);
		} goto dc_4x4;
	case I4x4_DC_B_8: {
		i8x16 w = ldleft4x4(p, stride);
		v = ziplo32(w, w);
		} goto dc_4x4;
	case I4x4_DC_AB_8:
		v = set8(-128);
		break;
	
	case I4x4_DDL_8:
		v = spreadh8(loada64(pT));
		idx = 0;
		goto lowpass_4x4;
	case I4x4_DDL_C_8:
		v = spreadq8(loada32(pT));
		idx = 0;
		goto lowpass_4x4;
	case I4x4_DDR_8:
		idx = 1;
		goto down_right_4x4;
	case I4x4_VR_8:
		idx = 2;
		goto down_right_4x4;
	case I4x4_HD_8:
		idx = 3;
		goto down_right_4x4;
	case I4x4_VL_8:
		v = loada64(pT);
		idx = 4;
		goto lowpass_4x4;
	case I4x4_VL_C_8:
		v = spreadq8(loada32(pT));
		idx = 4;
		goto lowpass_4x4;
	case I4x4_HU_8:
		v = spreadq8(ldleft4x4(p, stride));
		idx = 5;
		goto lowpass_4x4;
	down_right_4x4:
		v = ldedge4x4(p, stride);
	lowpass_4x4: {
		i8x16 w = shr128(v, 1);
		i8x16 x = shr128(v, 2);
		v = shuffle(lowpass8(ziplo64(v, v), ziplo64(w, w), ziplo64(x, v)), shuf[idx]);
		} break;
	}
	*(int32_t *)(p             ) = v[0];
	*(int32_t *)(p + stride    ) = v[1];
	*(int32_t *)(p + stride * 2) = v[2];
	*(int32_t *)(p + stride * 3) = v[3];
}



/**
 * Intra 8x8
 * 
 * Neighbouring samples are named a to z from bottom left to top right, with
 * i being p[-1,-1] or p[-1,0] if unavailable, and j being p[-1,-1] or p[0,-1].
 */
static cold noinline void decode_intra8x8(uint8_t * restrict p, size_t stride, int mode, i16x8 clip) {
	const uint8_t *pT = p - stride;
	i8x16 i2a, j2s, j2y, k2z, k2r, h2a;
	i64x2 p0, p1, p2, p3, p4, p5, p6, p7;
	switch (mode) {
	default: __builtin_unreachable();
	
	case I8x8_V_8:
		j2s = loadu128(pT - 1);
	vertical_8x8_lowpass:
		p0 = lowpass8(shr128(j2s, 2), shr128(j2s, 1), j2s);
	store1_8x8:
		for (int i = 0; i < 8; i++, p += stride)
			*(int64_t *)p = p0[0];
		return;
	case I8x8_V_C_8:
		j2s = shrc128(loadu128(pT - 8), 7);
		goto vertical_8x8_lowpass;
	case I8x8_V_D_8:
		j2s = shlc128(loadu128(pT), 1);
		goto vertical_8x8_lowpass;
	case I8x8_V_CD_8:
		j2s = shuffle(loada64(pT), (i8x16){0, 0, 1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 7, 7, 7, 7});
		goto vertical_8x8_lowpass;
	
	case I8x8_H_8:
		i2a = (i8x16){pT[-1]};
	horizontal_8x8_load_left: {
		i8x16 v0 = ldleft8x8(p, stride, i2a);
		p0 = broadcast8(v0, 0);
		p1 = broadcast8(v0, 1);
		p2 = broadcast8(v0, 2);
		p3 = broadcast8(v0, 3);
		p4 = broadcast8(v0, 4);
		p5 = broadcast8(v0, 5);
		p6 = broadcast8(v0, 6);
		p7 = broadcast8(v0, 7);
		} break;
	case I8x8_H_D_8:
		i2a = (i8x16){p[-1]};
		goto horizontal_8x8_load_left;
	
	case I8x8_DC_8:
		i2a = j2s = loadu128(pT - 1);
	dc_8x8_load_left:
		h2a = ldleft8x8(p, stride, i2a);
		k2r = lowpass8(shr128(j2s, 2), shr128(j2s, 1), j2s);
	dc_8x8_sum:
		p0 = broadcast8(shrru16(sum8(ziplo64(k2r, h2a)), 4), __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
		goto store1_8x8;
	case I8x8_DC_C_8:
		i2a = j2s = shrc128(loadu128(pT - 8), 7);
		goto dc_8x8_load_left;
	case I8x8_DC_D_8:
		j2s = shlc128(loadu128(pT), 1);
		i2a = (i8x16){p[-1]};
		goto dc_8x8_load_left;
	case I8x8_DC_CD_8:
		j2s = shlc128(spreadh8(loada64(pT)), 1);
		i2a = (i8x16){p[-1]};
		goto dc_8x8_load_left;
	case I8x8_DC_A_8:
		j2s = loadu128(pT - 1);
	dc_8x8_dup_top:
		h2a = k2r = lowpass8(shr128(j2s, 2), shr128(j2s, 1), j2s);
		goto dc_8x8_sum;
	case I8x8_DC_AC_8:
		i2a = j2s = shrc128(loadu128(pT - 8), 7);
		goto dc_8x8_dup_top;
	case I8x8_DC_AD_8:
		i2a = j2s = shlc128(loadu128(pT), 1);
		goto dc_8x8_dup_top;
	case I8x8_DC_ACD_8:
		i2a = j2s = shlc128(spreadh8(loada64(pT)), 1);
		goto dc_8x8_dup_top;
	case I8x8_DC_B_8:
		i2a = (i8x16){pT[-1]};
	dc_8x8_dup_left:
		k2r = h2a = ldleft8x8(p, stride, i2a);
		goto dc_8x8_sum;
	case I8x8_DC_BD_8:
		i2a = (i8x16){p[-1]};
		goto dc_8x8_dup_left;
	case I8x8_DC_AB_8:
		p0 = set8(-128);
		goto store1_8x8;
	
	case I8x8_DDL_8:
		j2y = loadu128(pT - 1);
		k2z = loadu128(pT);
	diagonal_down_left_8x8_lowpass: {
		i8x16 v0 = lowpass8(j2y, k2z, shrc128(k2z, 1));
		p0 = lowpass8(v0, shr128(v0, 1), shrc128(v0, 2));
		p1 = shr128(p0, 1);
		p2 = shr128(p0, 2);
		p3 = shr128(p0, 3);
		p4 = shr128(p0, 4);
		p5 = shr128(p0, 5);
		p6 = shr128(p0, 6);
		p7 = shr128(p0, 7);
		} break;
	case I8x8_DDL_C_8: {
		i8x16 j2r = loadu128(pT - 8);
		j2y = shrc128(j2r, 7);
		k2z = shrc128(j2r, 8);
		} goto diagonal_down_left_8x8_lowpass;
	case I8x8_DDL_D_8:
		k2z = loadu128(pT);
		j2y = shlc128(k2z, 1);
		goto diagonal_down_left_8x8_lowpass;
	case I8x8_DDL_CD_8:
		k2z = spreadh8(loada64(pT));
		j2y = shlc128(k2z, 1);
		goto diagonal_down_left_8x8_lowpass;
	
	case I8x8_DDR_8:
		j2s = loadu128(pT - 1);
	diagonal_down_right_8x8_load_left: {
		i8x16 a2h = ldedge8x8(p, stride);
		i8x16 a2q = shrd128(a2h, j2s, 8);
		i8x16 b2r = shrd128(a2h, j2s, 9);
		i8x16 x0 = lowpass8(shrd128(a2h, j2s, 7), a2q, b2r);
		i8x16 x1 = lowpass8(a2q, b2r, shrd128(a2h, j2s, 10));
		p7 = lowpass8(x0, x1, shr128(x1, 1));
		p6 = shr128(p7, 1);
		p5 = shr128(p7, 2);
		p4 = shr128(p7, 3);
		p3 = shr128(p7, 4);
		p2 = shr128(p7, 5);
		p1 = shr128(p7, 6);
		p0 = shr128(p7, 7);
		} break;
	case I8x8_DDR_C_8:
		j2s = shrc128(loadu128(pT - 8), 7);
		goto diagonal_down_right_8x8_load_left;
	
	case I8x8_VR_8:
		j2s = loadu128(pT - 1);
	vertical_right_8x8_load_left: {
		i8x16 a2h = ldedge8x8(p, stride);
		i8x16 a2q = shrd128(a2h, j2s, 8);
		i8x16 b2r = shrd128(a2h, j2s, 9);
		i8x16 c2s = shrd128(a2h, j2s, 10);
		i8x16 v0 = lowpass8(a2q, b2r, c2s);
		i8x16 v1 = shl128(v0, 1);
		i8x16 v2 = lowpass8(v0, v1, shl128(v0, 2));
		p0 = shr128(avgu8(v0, v1), 8);
		p1 = shr128(v2, 8);
		p2 = shrd128(shl128(v2, 8), p0, 15);
		p3 = shrd128(shl128(v2, 9), p1, 15);
		p4 = shrd128(shl128(v2, 10), p2, 15);
		p5 = shrd128(shl128(v2, 11), p3, 15);
		p6 = shrd128(shl128(v2, 12), p4, 15);
		p7 = shrd128(shl128(v2, 13), p5, 15);
		} break;
	case I8x8_VR_C_8:
		j2s = shrc128(loadu128(pT - 8), 7);
		goto vertical_right_8x8_load_left;
	
	case I8x8_HD_8: {
		j2s = loadu128(pT - 1);
		i8x16 a2h = ldedge8x8(p, stride);
		i8x16 a2p = shrd128(a2h, j2s, 7);
		i8x16 a2q = shrd128(a2h, j2s, 8);
		i8x16 b2r = shrd128(a2h, j2s, 9);
		i8x16 v0 = lowpass8(a2p, a2q, b2r);
		i8x16 v1 = shr128(v0, 1);
		i8x16 v2 = lowpass8(v0, v1, shr128(v0, 2));
		p7 = ziplo8(avgu8(v0, v1), v2);
		p6 = shr128(p7, 2);
		p5 = shr128(p7, 4);
		p4 = shr128(p7, 6);
		p3 = ziphi64(p7, v2);
		p2 = shr128(p3, 2);
		p1 = shr128(p3, 4);
		p0 = shr128(p3, 6);
		} break;
	
	case I8x8_VL_8:
		j2y = loadu128(pT - 1);
		k2z = loadu128(pT);
	vertical_left_8x8_lowpass: {
		i8x16 v0 = lowpass8(j2y, k2z, shr128(k2z, 1));
		i8x16 v1 = shr128(v0, 1);
		p0 = avgu8(v0, v1);
		p1 = lowpass8(v0, v1, shr128(v0, 2));
		p2 = shr128(p0, 1);
		p3 = shr128(p1, 1);
		p4 = shr128(p2, 1);
		p5 = shr128(p3, 1);
		p6 = shr128(p4, 1);
		p7 = shr128(p5, 1);
		} break;
	case I8x8_VL_C_8: {
		i8x16 j2r = loadu128(pT - 8);
		j2y = shrc128(j2r, 7);
		k2z = shrc128(j2r, 8);
		} goto vertical_left_8x8_lowpass;
	case I8x8_VL_D_8:
		k2z = loadu128(pT);
		j2y = shlc128(k2z, 1);
		goto vertical_left_8x8_lowpass;
	case I8x8_VL_CD_8:
		k2z = spreadh8(loada64(pT));
		j2y = shlc128(k2z, 1);
		goto vertical_left_8x8_lowpass;
	
	case I8x8_HU_8:
		i2a = (i8x16){pT[-1]};
	horizontal_up_8x8_load_left: {
		i8x16 v0 = spreadh8(ldleft8x8(p, stride, i2a));
		i8x16 v1 = shr128(v0, 1);
		p0 = ziplo8(avgu8(v0, v1), lowpass8(v0, v1, shr128(v0, 2)));
		p1 = shr128(p0, 2);
		p2 = shr128(p0, 4);
		p3 = shr128(p0, 6);
		p4 = ziphi64(p0, v0);
		p5 = shr128(p4, 2);
		p6 = shr128(p4, 4);
		p7 = shr128(p4, 6);
		} break;
	case I8x8_HU_D_8:
		i2a = (i8x16){p[-1]};
		goto horizontal_up_8x8_load_left;
	}
	*(int64_t *)p = p0[0];
	*(int64_t *)(p += stride) = p1[0];
	*(int64_t *)(p += stride) = p2[0];
	*(int64_t *)(p += stride) = p3[0];
	*(int64_t *)(p += stride) = p4[0];
	*(int64_t *)(p += stride) = p5[0];
	*(int64_t *)(p += stride) = p6[0];
	*(int64_t *)(p + stride) = p7[0];
}



/**
 * Intra 16x16
 */
static cold noinline void decode_intra16x16(uint8_t * restrict p, size_t stride, int mode, i16x8 clip) {
	const uint8_t *pT = p - stride;
	i8x16 pred;
	switch (mode) {
	default: __builtin_unreachable();
	
	case I16x16_V_8:
		pred = loada128(pT);
		break;
	
	case I16x16_H_8:
		for (int i = 0; i < 16; i++, p += stride)
			*(i8x16 *)p = set8(p[-1]);
		return;
	
	case I16x16_DC_8: {
		i8x16 t = loada128(pT);
		const uint8_t *q = p - 1;
		int l = 0;
		for (int i = 0; i < 16; i++, q += stride)
			l += *q;
		pred = broadcast8(shrru16(sum8(t) + (i16x8){l}, 5), __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
		} break;
	case I16x16_DC_A_8:
		pred = broadcast8(shrru16(sum8(loada128(pT)), 4), __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
		break;
	case I16x16_DC_B_8: {
		const uint8_t *q = p - 1;
		int l = 0;
		for (int i = 0; i < 16; i++, q += stride)
			l += *q;
		pred = set8((l + 8) >> 4);
		} break;
	case I16x16_DC_AB_8:
		pred = set8(-128);
		break;
	
	case I16x16_P_8: {
		#if defined(__SSE2__)
			size_t nstride = -stride;
			#ifdef __SSE4_1__
				const uint8_t *p0 = p - 1;
				const uint8_t *p7 = p0 + stride * 7;
				const uint8_t *pE = p7 + stride * 7;
				i64x2 t = loadu64(p0 + nstride);
				i8x16 l = t;
				t[1] = *(int64_t*)(p0 + nstride + 9);
				l[1] = *p0;
				l[2] = p0[stride];
				l[3] = p0[stride * 2];
				l[4] = p7[nstride * 4];
				l[5] = p0[stride * 4];
				l[6] = p7[nstride * 2];
				l[7] = p7[nstride];
				l[8] = p7[stride];
				l[9] = p7[stride * 2];
				l[10] = pE[nstride * 4];
				l[11] = p7[stride * 4];
				l[12] = pE[nstride * 2];
				l[13] = pE[nstride];
				l[14] = *pE;
				l[15] = pE[stride];
			#else
				const uint8_t *p0 = p - 16;
				const uint8_t *p7 = p0 + stride * 7;
				const uint8_t *pE = p7 + stride * 7;
				i8x16 t = loadu64x2(p0 + nstride + 15, p0 + nstride + 24);
				i8x16 l0 = ziphi8(shl128(t, 15), loada128(p0));
				i8x16 l1 = ziphi8(loada128(p0 + stride), loada128(p0 + stride * 2));
				i8x16 l2 = ziphi8(loada128(p7 + nstride * 4), loada128(p0 + stride * 4));
				i8x16 l3 = ziphi8(loada128(p7 + nstride * 2), loada128(p7 + nstride));
				i8x16 l4 = ziphi8(loada128(p7 + stride), loada128(p7 + stride * 2));
				i8x16 l5 = ziphi8(loada128(pE + nstride * 4), loada128(p7 + stride * 4));
				i8x16 l6 = ziphi8(loada128(pE + nstride * 2), loada128(pE + nstride));
				i8x16 l7 = ziphi8(loada128(pE), loada128(pE + stride));
				i8x16 l = ziphi64(ziphi32(ziphi16(l0, l1), ziphi16(l2, l3)), ziphi32(ziphi16(l4, l5), ziphi16(l6, l7)));
			#endif
			i8x16 m = {-8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8};
			i16x8 mul = ziphi8(m, (i8x16){});
			i16x8 v0 = maddubs(t, m);
			i16x8 v1 = maddubs(l, m);
			i16x8 v2 = (i16x8)ziplo32(v0, v1) + (i16x8)ziphi32(v0, v1);
			i16x8 v3 = v2 + (i16x8)shr128(v2, 8);
			i16x8 HV = v3 + shufflelo(v3, 1, 0, 3, 2); // H, H, V, V
			i16x8 a = (broadcast16((i16x8)shr128(t, 15) + (i16x8)shr128(l, 15), 0) - -1) << 4;
		#elif defined(__ARM_NEON)
			i8x16 tl = loadu64(pT - 1);
			i8x16 tr = loada64(pT + 8);
			const uint8_t *q = p - 1;
			i8x16 lt = tl, lb = {};
			lt[1] = *q;
			lt[2] = *(q += stride);
			lt[3] = *(q += stride);
			lt[4] = *(q += stride);
			lt[5] = *(q += stride);
			lt[6] = *(q += stride);
			lt[7] = *(q += stride);
			lb[0] = *(q += stride * 2);
			lb[1] = *(q += stride);
			lb[2] = *(q += stride);
			lb[3] = *(q += stride);
			lb[4] = *(q += stride);
			lb[5] = *(q += stride);
			lb[6] = *(q += stride);
			lb[7] = *(q + stride);
			i16x8 mul = {1, 2, 3, 4, 5, 6, 7, 8};
			i16x8 v0 = sublou8(tr, vrev64q_s8(tl)) * mul;
			i16x8 v1 = sublou8(lb, vrev64q_s8(lt)) * mul;
			i16x8 v2 = vpaddq_s16(v0, v1);
			i16x8 v3 = vpaddq_s16(v2, v2);
			i16x8 HV = (i16x8)vtrn1q_s16(v3, v3) + (i16x8)vtrn2q_s16(v3, v3); // {H, H, V, V, 0, 0, 0, 0}, -9180..9180
			i16x8 a = (broadcast16(addlou8(tr, lb) - -1, 7)) << 4;
		#endif
		i16x8 v4 = shrrs16(HV + (HV >> 2), 4); // (5 * HV + 32) >> 6, -717..717
		i16x8 b = broadcast32(v4, 0);
		i16x8 c = broadcast32(v4, 1);
		i16x8 v5 = (a + c) - (c << 3) + (b * mul);
		i16x8 v6 = v5 - (b << 3);
		for (int i = 16; i--; v6 += c, v5 += c, p += stride)
			*(i8x16 *)p = shrpus16(v6, v5, 5);
		} return;
	}
	for (int i = 0; i < 16; i++, p += stride)
		*(i8x16 *)p = pred;
}



/**
 * Intra Chroma
 */
static cold noinline void decode_intraChroma(uint8_t * restrict p, size_t stride, int mode, i16x8 clip) {
	#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
		static const i8x16 shufDC = {0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12};
		static const i8x16 shufDCA = {0, 0, 0, 0, 12, 12, 12, 12, 0, 0, 0, 0, 12, 12, 12, 12};
		static const i8x16 shufDCB = {0, 0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 12, 12, 12, 12, 12};
	#else
		static const i8x16 shufDC = {1, 1, 1, 1, 5, 5, 5, 5, 9, 9, 9, 9, 13, 13, 13, 13};
		static const i8x16 shufDCA = {1, 1, 1, 1, 13, 13, 13, 13, 1, 1, 1, 1, 13, 13, 13, 13};
		static const i8x16 shufDCB = {1, 1, 1, 1, 1, 1, 1, 1, 13, 13, 13, 13, 13, 13, 13, 13};
	#endif
	const uint8_t *pT = p - stride;
	const uint8_t *pU = p - stride * 2;
	i8x16 bt, rt, l, shuf;
	i64x2 bpred, rpred;
	switch (mode) {
	default: __builtin_unreachable();
	
	case IC8x8_DC_8: {
		bt = loada64(pU);
		rt = loada64(pT);
		l = ldleftC(p, stride);
		shuf = shufDC;
	} chroma_dc_8x8_sum: {
		#if defined(__SSE2__)
			i8x16 b = ziplo64(bt, l);
			i8x16 r = shrd128(l, rt, 8);
			i16x8 b01 = sumh8(shuffle32(b, 0, 2, 1, 1));
			i16x8 r01 = sumh8(shuffle32(r, 2, 0, 3, 3));
			i16x8 b23 = sumh8(shuffle32(b, 3, 3, 1, 3));
			i16x8 r23 = sumh8(shuffle32(r, 1, 1, 3, 1));
			bpred = shuffle(shrru16(packs32(b01, b23), 3), shuf);
			rpred = shuffle(shrru16(packs32(r01, r23), 3), shuf);
		#elif defined(__ARM_NEON)
			i8x16 t = ziplo64(bt, rt);
			i16x8 v0 = vpaddlq_u8(vtrn1q_s32(t, l)); // top-left sums
			i16x8 v1 = vpaddlq_u8(vtrn2q_s32(t, t)); // top-right sums
			i16x8 v2 = vpaddlq_u8(vtrn2q_s32(l, l)); // bottom-left sums
			i16x8 v3 = vpaddlq_u8(vtrn2q_s32(t, l)); // bottom-right sums
			i8x16 v4 = shrru16(vpaddq_u16(vpaddq_u16(v0, v1), vpaddq_u16(v2, v3)), 3);
			bpred = shuffle(v4, shuf);
			rpred = shuffle(shr128(v4, 2), shuf);
		#endif
		} break;
	case IC8x8_DC_A_8:
		bt = loada64(pU);
		rt = loada64(pT);
		l = ziplo64(bt, rt);
		shuf = shufDCA;
		goto chroma_dc_8x8_sum;
	case IC8x8_DC_B_8:
		bt = l = ldleftC(p, stride);
		rt = shr128(l, 8);
		shuf = shufDCB;
		goto chroma_dc_8x8_sum;
	case IC8x8_DC_AB_8:
		bpred = rpred = set8(-128);
		break;
	
	case IC8x8_H_8:
		for (int i = 0; i < 16; i++, p += stride)
			*(int64_t *)p = ((i64x2)set8(p[-1]))[0];
		return;
	
	case IC8x8_V_8: {
		bpred = set64(*(int64_t *)pU);
		rpred = set64(*(int64_t *)pT);
		} break;
	
	case IC8x8_P_8: {
		i8x16 btl = loadu128(pU - 1);
		i8x16 rtl = loadu128(pT - 1);
		i8x16 tl = ziplo32(btl, rtl);
		i8x16 tr = ziplo32(shr128(btl, 5), shr128(rtl, 5));
		#if defined(__SSE2__)
			size_t stride3 = stride * 3;
			const uint8_t *p0 = p - 1;
			const uint8_t *p4 = p0 + stride * 4;
			const uint8_t *p8 = p0 + stride * 8;
			const uint8_t *pC = p4 + stride * 8;
			i8x16 lt = {tl[0], *p0, p0[stride * 2], *p4, tl[4], p0[stride], p0[stride3], p4[stride]};
			i8x16 lb = {*p8, p8[stride * 2], *pC, pC[stride * 2], p8[stride], p8[stride3], pC[stride], pC[stride3]};
			i8x16 n = {-4, -3, -2, -1, -4, -3, -2, -1, -4, -3, -2, -1, -4, -3, -2, -1};
			i8x16 m = {1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4};
			i16x8 v0 = maddubs(ziplo64(tl, lt), n) + maddubs(ziplo64(tr, lb), m);
			i16x8 v1 = (((u32x4)tr >> 24) + ((u32x4)lb >> 24) - -1) << 4;
			i16x8 HV = v0 + shufflelo(shufflehi(v0, 1, 0, 3, 2), 1, 0, 3, 2); // Hb,Hb,Hr,Hr,Vb,Vb,Vr,Vr
			i16x8 ba = broadcast16(v1, 0); // 16..8176
			i16x8 ra = broadcast16(v1, 2); // 16..8176
		#elif defined(__ARM_NEON)
			const uint8_t *q = p - 1;
			i8x16 lt = tl, lb;
			lt[1] = *q;
			lt[5] = *(q += stride);
			lt[2] = *(q += stride);
			lt[6] = *(q += stride);
			lt[3] = *(q += stride);
			lt[7] = *(q += stride);
			lb[0] = *(q += stride * 3);
			lb[4] = *(q += stride);
			lb[1] = *(q += stride);
			lb[5] = *(q += stride);
			lb[2] = *(q += stride);
			lb[6] = *(q += stride);
			lb[3] = *(q += stride);
			lb[7] = *(q += stride);
			i16x8 v0 = sublou8(tr, vrev32q_s8(tl));
			i16x8 v1 = sublou8(lb, vrev32q_s8(lt));
			i16x8 m = {1, 2, 3, 4, 1, 2, 3, 4};
			i16x8 v2 = vpaddq_s16(v0 * m, v1 * m);
			i16x8 HV = (i16x8)vtrn1q_s16(v2, v2) + (i16x8)vtrn2q_s16(v2, v2);
			i16x8 v3 = (addlou8(tr, lb) - -1) << 4;
			i16x8 ba = broadcast16(v3, 3);
			i16x8 ra = broadcast16(v3, 7);
		#endif
		i16x8 hv = shrrs16(HV + (HV >> 4), 1); // (17 * HV + 16) >> 5
		i16x8 bb = broadcast32(hv, 0);
		i16x8 rb = broadcast32(hv, 1);
		i16x8 bc = broadcast32(hv, 2);
		i16x8 rc = broadcast32(hv, 3);
		i16x8 bp = ba - bc - bc - bc + bb * (i16x8){-3, -2, -1, 0, 1, 2, 3, 4};
		i16x8 rp = ra - rc - rc - rc + rb * (i16x8){-3, -2, -1, 0, 1, 2, 3, 4};
		for (int i = 8; i--; bp += bc, rp += rc, p += stride * 2) {
			i64x2 v6 = shrpus16(bp, rp, 5);
			*(int64_t *)p = v6[0];
			*(int64_t *)(p + stride) = v6[1];
		}
		} return;
	}
	for (int i = 0; i < 4; i++, p += stride * 2) {
		*(int64_t *)p = bpred[0];
		*(int64_t *)(p + stride) = rpred[0];
		*(int64_t *)(p + stride * 8) = bpred[1];
		*(int64_t *)(p + stride * 9) = rpred[1];
	}
}
