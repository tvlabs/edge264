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
	#define sum8h8(a, b, c, d) (i16x8)packs32(packs32(sumh8(a), sumh8(b)), packs32(sumh8(c), sumh8(d)))
	static always_inline i8x16 lowpass8(i8x16 l, i8x16 m, i8x16 r) {return avgu8(subu8(avgu8(l, r), (l ^ r) & set8(1)), m);}
#elif defined(__ARM_NEON)
	#define lowpass8(l, m, r) (i8x16)vrhaddq_u8(vhaddq_u8(l, r), m)
	static always_inline i8x16 spreadh8(i8x16 a) {return vcombine_s8(vget_low_s8(a), vdup_lane_s8(vget_low_s8(a), 7));}
	static always_inline i8x16 spreadq8(i8x16 a) {return vextq_s8(vextq_s8(a, a, 4), broadcast8(a, 3), 12);}
	#ifdef __aarch64__
		#define sum8h8(a, b, c, d) (i16x8)vpaddq_s16(vpaddq_s16(vpaddlq_u8(a), vpaddlq_u8(b)), vpaddq_s16(vpaddlq_u8(c), vpaddlq_u8(d)))
	#else
		static always_inline i16x8 sum8h8(u8x16 a, u8x16 b, u8x16 c, u8x16 d) {i16x8 e = vpaddlq_u8(a), f = vpaddlq_u8(b), g = vpaddlq_u8(c), h = vpaddlq_u8(d); i16x8 i = vcombine_s16(vpadd_s16(vget_low_s16(e), vget_high_s16(e)), vpadd_s16(vget_low_s16(f), vget_high_s16(f))); i16x8 j = vcombine_s16(vpadd_s16(vget_low_s16(g), vget_high_s16(g)), vpadd_s16(vget_low_s16(h), vget_high_s16(h))); return vcombine_s16(vpadd_s16(vget_low_s16(i), vget_high_s16(i)), vpadd_s16(vget_low_s16(j), vget_high_s16(j)));}
	#endif
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
	static i8x16 ldleftC(const uint8_t *p, size_t stride, size_t mstride) {
		i8x16 v = {*(p -= 1)};
		v[8] = *(p += stride);
		v[1] = *(p += stride);
		v[9] = *(p += stride);
		v[2] = *(p += stride);
		v[10] = *(p += stride);
		v[3] = *(p += stride);
		v[11] = *(p += stride);
		v[4] = *(p += stride + mstride);
		v[12] = *(p += stride);
		v[5] = *(p += stride);
		v[13] = *(p += stride);
		v[6] = *(p += stride);
		v[14] = *(p += stride);
		v[7] = *(p += stride);
		v[15] = *(p + stride);
		return v;
	}
	static i8x16 ldleftP16(const uint8_t *p, size_t stride, i8x16 v) {
		v[1] = *(p -= 1);
		v[2] = *(p += stride);
		v[3] = *(p += stride);
		v[4] = *(p += stride);
		v[5] = *(p += stride);
		v[6] = *(p += stride);
		v[7] = *(p += stride);
		v[8] = *(p += stride * 2);
		v[9] = *(p += stride);
		v[10] = *(p += stride);
		v[11] = *(p += stride);
		v[12] = *(p += stride);
		v[13] = *(p += stride);
		v[14] = *(p += stride);
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
	static i8x16 ldleftC(const uint8_t *p, size_t stride, size_t mstride) {
		size_t stride3 = stride * 3;
		const uint8_t *p0 = p - 1;
		const uint8_t *p4 = p0 + stride * 4;
		const uint8_t *p8 = p0 + stride * 8 + mstride;
		const uint8_t *pC = p8 + stride * 4;
		return (i8x16){*p0, p0[stride * 2], *p4, p4[stride * 2], *p8, p8[stride * 2], *pC, pC[stride * 2], p0[stride], p0[stride3], p4[stride], p4[stride3], p8[stride], p8[stride3], pC[stride], pC[stride3]};
	}
	static i8x16 ldleftP16(const uint8_t *p, size_t stride, i8x16 v) {
		size_t stride3 = stride * 3;
		const uint8_t *p0 = p - 1;
		const uint8_t *p4 = p0 + stride * 4;
		const uint8_t *p8 = p0 + stride * 8;
		const uint8_t *pC = p4 + stride * 8;
		v[1] = *p0;
		v[2] = p0[stride];
		v[3] = p0[stride * 2];
		v[4] = p0[stride3];
		v[5] = *p4;
		v[6] = p4[stride];
		v[7] = p4[stride * 2];
		v[8] = *p8;
		v[9] = p8[stride];
		v[10] = p8[stride * 2];
		v[11] = p8[stride3];
		v[12] = *pC;
		v[13] = pC[stride];
		v[14] = pC[stride * 2];
		v[15] = pC[stride3];
		return v;
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
	static i8x16 ldleftC(const uint8_t *p, size_t stride, size_t mstride) {
		size_t stride3 = stride * 3;
		const uint8_t *p0 = p - 4;
		const uint8_t *p4 = p0 + stride * 4;
		const uint8_t *p8 = p0 + stride * 8 + mstride;
		const uint8_t *pC = p8 + stride * 4;
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
	static i8x16 ldleftP16(const uint8_t *p, size_t stride, i8x16 v) {
		size_t stride3 = stride * 3;
		const uint8_t *p0 = p - 16;
		const uint8_t *p4 = p0 + stride * 4;
		const uint8_t *p8 = p0 + stride * 8;
		const uint8_t *pC = p4 + stride * 8;
		i8x16 l0 = ziplo8((u64x2)v << 56, loada64(p0 + 8));
		i8x16 l1 = ziphi8(loada128(p0 + stride), loada128(p0 + stride * 2));
		i8x16 l2 = ziphi8(loada128(p0 + stride3), loada128(p4));
		i8x16 l3 = ziphi8(loada128(p4 + stride), loada128(p4 + stride * 2));
		i8x16 l4 = ziphi8(loada128(p8), loada128(p8 + stride));
		i8x16 l5 = ziphi8(loada128(p8 + stride * 2), loada128(p8 + stride3));
		i8x16 l6 = ziphi8(loada128(pC), loada128(pC + stride));
		i8x16 l7 = ziphi8(loada128(pC + stride * 2), loada128(pC + stride3));
		i8x16 l8 = ziphi32(ziphi16(l0, l1), ziphi16(l2, l3));
		i8x16 l9 = ziphi32(ziphi16(l4, l5), ziphi16(l6, l7));
		return ziphi64(l8, l9);
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
		v = broadcast8(shrru16(sumh8(v), 3), 0);
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
		p0 = broadcast8(shrru16(sum8(ziplo64(k2r, h2a)), 4), 0);
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
		pred = broadcast8(shrru16(sum8(t) + (i16x8){l}, 5), 0);
		} break;
	case I16x16_DC_A_8:
		pred = broadcast8(shrru16(sum8(loada128(pT)), 4), 0);
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
		// The optimal codes differ wildly between ISAs, but this one is not too critical.
		size_t nstride = -stride;
		i8x16 t = loadu64x2(pT - 1, pT + 8);
		i8x16 l = ldleftP16(p, stride, t);
		i8x16 m = {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8};
		i16x8 mul = cvthi8u16(m);
		i16x8 v0 = hadd16(maddubx(t, m), maddubx(l, m));
		i16x8 v1 = (i16x8)((u32x4)v0 >> 16) + v0;
		i16x8 HV = (i16x8)((u64x2)v1 >> 32) - v1;
		i16x8 v2 = shrrs16(HV + (HV >> 2), 4);
		i16x8 a = broadcast16((((u16x8)t >> 8) + ((u16x8)l >> 8) + 1) << 4, 7);
		i16x8 b = broadcast16(v2, 0);
		i16x8 c = broadcast16(v2, 4);
		i16x8 w0 = (a + c) - (c << 3) + (b * mul);
		i16x8 w1 = w0 - (b << 3);
		for (int i = 16; i--; w1 += c, w0 += c, p += stride)
			*(i8x16 *)p = shrpus16(w1, w0, 5);
		} return;
	}
	for (int i = 0; i < 16; i++, p += stride)
		*(i8x16 *)p = pred;
}



/**
 * Intra Chroma
 */
static cold noinline void decode_intraChroma(uint8_t * restrict p, size_t stride, int mode, i16x8 clip) {
	static const i8x16 shufDC = {0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12};
	static const i8x16 shufDCA = {0, 0, 0, 0, 12, 12, 12, 12, 0, 0, 0, 0, 12, 12, 12, 12};
	static const i8x16 shufDCB = {0, 0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 12, 12, 12, 12, 12};
	const uint8_t *pT = p - stride;
	const uint8_t *pU = p - stride * 2;
	i8x16 t, l, shuf;
	i64x2 bpred, rpred;
	switch (mode) {
	default: __builtin_unreachable();
	
	case IC8x8_DC_8: {
		t = loada64x2(pU, pT);
		l = ldleftC(p, stride, 0);
		shuf = shufDC;
	} chroma_dc_8x8_sum: {
		i16x8 br0 = trnlo32(t, l); // top-left samples to sum
		i16x8 br1 = trnhi32(t, t); // top-right samples to sum
		i16x8 br2 = trnhi32(l, l); // bottom-left samples to sum
		i16x8 br3 = trnhi32(t, l); // bottom-right samples to sum
		u32x4 v0 = shrru16(sum8h8(br0, br1, br2, br3), 3);
		bpred = shuffle(v0, shuf);
		rpred = shuffle(v0 >> 16, shuf);
		} break;
	case IC8x8_DC_A_8:
		l = t = loada64x2(pU, pT);
		shuf = shufDCA;
		goto chroma_dc_8x8_sum;
	case IC8x8_DC_B_8:
		t = l = ldleftC(p, stride, 0);
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
		i8x16 s = {7, 8, 9, 10, 12, 13, 14, 15};
		i8x16 t = ziplo64(shuffle(loadu128(pU - 8), s), shuffle(loadu128(pT - 8), s));
		i8x16 l = ldleftC(pU, stride, stride * 2);
		i8x16 m = {4, 3, 2, 1, 1, 2, 3, 4, 4, 3, 2, 1, 1, 2, 3, 4};
		i16x8 v0 = hadd16(maddubx(t, m), maddubx(l, m));
		i16x8 HV = (i16x8)((u32x4)v0 >> 16) - v0; // Hb, _, Hr, _, Vb, _, Vr, _
		i16x8 v1 = shrrs16(HV + (HV >> 4), 1); // (17 * HV + 16) >> 5
		i16x8 v2 = (((u16x8)t >> 8) + ((u16x8)l >> 8) - -1) << 4;
		i16x8 ba = broadcast16(v2, 3);
		i16x8 ra = broadcast16(v2, 7);
		i16x8 bb = broadcast16(v1, 0);
		i16x8 rb = broadcast16(v1, 2);
		i16x8 bc = broadcast16(v1, 4);
		i16x8 rc = broadcast16(v1, 6);
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
