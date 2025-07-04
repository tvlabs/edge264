#include "edge264_internal.h"

#if defined(__SSE2__)
	#define addlou8s16(a, b) (cvtlo8u16(a) + (i16x8)b)
	#define addhiu8s16(a, b) ((i16x8)ziphi8(a, (i8x16){}) + (i16x8)b)
	#define mullou8(a, b) (cvtlo8u16(a) * cvtlo8u16(b))
	#define mulhiu8(a, b) ((u16x8)ziphi8(a, (i8x16){}) * (u16x8)ziphi8(b, (i8x16){}))
	#define shlrrs32(a, l, r, off) ((((i32x4)(a) << (l)) + (off)) >> (r))
	#define shrrs32(a, i, off) (((i32x4)a + off) >> i)
	#define shrps32(a, b, i) (i16x8)_mm_packs_epi32((i32x4)(a) >> i, (i32x4)(b) >> i)
	#define unziplo32(a, b) shuffleps(a, b, 0, 2, 0, 2)
	#define unziphi32(a, b) shuffleps(a, b, 1, 3, 1, 3)
	static always_inline i16x8 scale32(i32x4 c0, i32x4 c1, u16x8 ls, int mul, i32x4 off, i32x4 sh) {return packs32(_mm_sra_epi32((i32x4)_mm_madd_epi16(cvtlo16u32(ls), c0) + off, sh), _mm_sra_epi32((i32x4)_mm_madd_epi16(ziphi16(ls, (i8x16){}), c1) + off, sh));}
#elif defined(__ARM_NEON)
	#define addlou8s16(a, b) (i16x8)vaddw_u8(b, vget_low_u8(a))
	#define addhiu8s16(a, b) (i16x8)vaddw_high_u8(b, a)
	#define mullou8(a, b) (u16x8)vmull_u8(vget_low_u8(a), vget_low_u8(b))
	#define mulhiu8(a, b) (u16x8)vmull_high_u8(a, b)
	#define shlrrs32(a, l, r, off) (i32x4)vrshlq_s32(a, vdupq_n_s32((l) - (r)))
	#define shrrs32(a, i, off) (i32x4)vrshrq_n_s32(a, i)
	#define shrps32(a, b, i) (u8x16)vqshrn_high_n_s32(vqshrn_n_s32(a, i), b, i)
	#define unziplo32(a, b) (i32x4)vuzp1q_s32(a, b)
	#define unziphi32(a, b) (i32x4)vuzp2q_s32(a, b)
	static always_inline i16x8 scale32(i32x4 c0, i32x4 c1, u16x8 ls, int mul, i32x4 off, i32x4 sh) {return vrshrn_high_n_s32(vrshrn_n_s32((i32x4)vmull_n_s16(vget_low_s16(ls), mul) * c0, 6), (i32x4)vmull_high_n_s16(ls, mul) * c1, 6);}
#endif

static const i8x4 normAdjust4x4[6] = {
	{10, 16, 13},
	{11, 18, 14},
	{13, 20, 16},
	{14, 23, 18},
	{16, 25, 20},
	{18, 29, 23}
};
static const i8x8 normAdjust8x8[6] = {
	{20, 18, 32, 19, 25, 24},
	{22, 19, 35, 21, 28, 26},
	{26, 23, 42, 24, 33, 31},
	{28, 25, 45, 26, 35, 33},
	{32, 28, 51, 30, 40, 38},
	{36, 32, 58, 34, 46, 43}
};



/**
 * Inverse 4x4 transform
 *
 * Here we try to stay close to the spec's pseudocode, avoiding minor
 * optimisations that would make the code hard to understand.
 */
static noinline void add_idct4x4(Edge264Context *ctx, int iYCbCr, int DCidx, uint8_t *p)
{
	// loading and scaling
	unsigned qP = ctx->t.QP[iYCbCr];
	int sh = qP / 6;
	i8x16 vm = load32(&normAdjust4x4[qP % 6]);
	i8x16 nA = shuffle(vm, (i8x16){0, 2, 0, 2, 2, 1, 2, 1, 0, 2, 0, 2, 2, 1, 2, 1});
	i8x16 wS = ctx->t.pps.weightScale4x4_v[iYCbCr + mb->mbIsInterFlag * 3];
	i16x8 LS0 = mullou8(wS, nA);
	i16x8 LS1 = mulhiu8(wS, nA);
	i32x4 s8 = set32(8); // for SSE
	i32x4 d0 = shlrrs32(ctx->c_v[0] * cvtlo16u32(LS0), sh, 4, s8);
	i32x4 d1 = shlrrs32(ctx->c_v[1] * cvthi16u32(LS0), sh, 4, s8);
	i32x4 d2 = shlrrs32(ctx->c_v[2] * cvtlo16u32(LS1), sh, 4, s8);
	i32x4 d3 = shlrrs32(ctx->c_v[3] * cvthi16u32(LS1), sh, 4, s8);
	ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (i8x16){};
	if (DCidx >= 0)
		d0[0] = ctx->c[16 + DCidx];
	
	// horizontal 1D transform
	i32x4 e0 = d0 + d2;
	i32x4 e1 = d0 - d2;
	i32x4 e2 = (d1 >> 1) - d3;
	i32x4 e3 = (d3 >> 1) + d1;
	i32x4 f0 = e0 + e3;
	i32x4 f1 = e1 + e2;
	i32x4 f2 = e1 - e2;
	i32x4 f3 = e0 - e3;
	
	// matrix transposition
	i32x4 x0 = ziplo32(f0, f1);
	i32x4 x1 = ziplo32(f2, f3);
	i32x4 x2 = ziphi32(f0, f1);
	i32x4 x3 = ziphi32(f2, f3);
	f0 = (i32x4)ziplo64(x0, x1) + 32;
	f1 = ziphi64(x0, x1);
	f2 = ziplo64(x2, x3);
	f3 = ziphi64(x2, x3);
	
	// vertical 1D transform
	i32x4 g0 = f0 + f2;
	i32x4 g1 = f0 - f2;
	i32x4 g2 = (f1 >> 1) - f3;
	i32x4 g3 = (f3 >> 1) + f1;
	i32x4 h0 = g0 + g3;
	i32x4 h1 = g1 + g2;
	i32x4 h2 = g1 - g2;
	i32x4 h3 = g0 - g3;
	
	// final residual values
	i16x8 r0 = shrps32(h0, h1, 6);
	i16x8 r1 = shrps32(h2, h3, 6);
	
	// addition to values in place, clipping and storage
	size_t stride = ctx->t.stride[iYCbCr];
	INIT_P();
	if (ctx->t.samples_clip[iYCbCr][0] == 255) {
		i8x16 p0 = (i32x4){*(int32_t *)P(0, 0), *(int32_t *)P(0, 1)};
		i8x16 p1 = (i32x4){*(int32_t *)P(0, 2), *(int32_t *)P(0, 3)};
		i32x4 u = packus16(addlou8s16(p0, r0), addlou8s16(p1, r1));
		*(int32_t *)P(0, 0) = u[0];
		*(int32_t *)P(0, 1) = u[1];
		*(int32_t *)P(0, 2) = u[2];
		*(int32_t *)P(0, 3) = u[3];
	}
}

static void add_dc4x4(Edge264Context *ctx, int iYCbCr, int DCidx, uint8_t *p) {
	i32x4 r = set16((ctx->c[16 + DCidx] + 32) >> 6);
	size_t stride = ctx->t.stride[iYCbCr];
	INIT_P();
	if (ctx->t.samples_clip[iYCbCr][0] == 255) {
		i8x16 p0 = (i32x4){*(int32_t *)P(0, 0), *(int32_t *)P(0, 1)};
		i8x16 p1 = (i32x4){*(int32_t *)P(0, 2), *(int32_t *)P(0, 3)};
		i32x4 u = packus16(addlou8s16(p0, r), addlou8s16(p1, r));
		*(int32_t *)P(0, 0) = u[0];
		*(int32_t *)P(0, 1) = u[1];
		*(int32_t *)P(0, 2) = u[2];
		*(int32_t *)P(0, 3) = u[3];
	}
}



/**
 * Inverse 8x8 transform
 */
static void add_idct8x8(Edge264Context *ctx, int iYCbCr, uint8_t *p)
{
	// loading and scaling
	unsigned qP = ctx->t.QP[iYCbCr];
	if (ctx->t.samples_clip[iYCbCr][0] == 255) {
		int div = qP / 6;
		i8x16 vm = load64(&normAdjust8x8[qP % 6]);
		i8x16 *wS = ctx->t.pps.weightScale8x8_v + (iYCbCr * 2 + mb->mbIsInterFlag) * 4;
		i8x16 nA0 = shuffle(vm, (i8x16){0, 3, 4, 3, 0, 3, 4, 3, 3, 1, 5, 1, 3, 1, 5, 1});
		i8x16 nA1 = shuffle(vm, (i8x16){4, 5, 2, 5, 4, 5, 2, 5, 3, 1, 5, 1, 3, 1, 5, 1});
		i16x8 LS0 = mullou8(wS[0], nA0);
		i16x8 LS1 = mulhiu8(wS[0], nA0);
		i16x8 LS2 = mullou8(wS[1], nA1);
		i16x8 LS3 = mulhiu8(wS[1], nA1);
		i16x8 LS4 = mullou8(wS[2], nA0);
		i16x8 LS5 = mulhiu8(wS[2], nA0);
		i16x8 LS6 = mullou8(wS[3], nA1);
		i16x8 LS7 = mulhiu8(wS[3], nA1);
		i32x4 *c = ctx->c_v;
		i16x8 d0, d1, d2, d3, d4, d5, d6, d7;
		if (__builtin_expect(div < 6, 1)) {
			int mul = 1 << div; // for NEON
			i32x4 off = set32(1 << (5 - div)); // for SSE
			i32x4 sh = {6 - div}; // for SSE
			d0 = scale32(c[0], c[1], LS0, mul, off, sh);
			d1 = scale32(c[2], c[3], LS1, mul, off, sh);
			d2 = scale32(c[4], c[5], LS2, mul, off, sh);
			d3 = scale32(c[6], c[7], LS3, mul, off, sh);
			d4 = scale32(c[8], c[9], LS4, mul, off, sh);
			d5 = scale32(c[10], c[11], LS5, mul, off, sh);
			d6 = scale32(c[12], c[13], LS6, mul, off, sh);
			d7 = scale32(c[14], c[15], LS7, mul, off, sh);
		} else {
			int sh = div - 6;
			// FIXME use shift function for SSE
			d0 = packs32(c[0], c[1]) * (LS0 << sh);
			d1 = packs32(c[2], c[3]) * (LS1 << sh);
			d2 = packs32(c[4], c[5]) * (LS2 << sh);
			d3 = packs32(c[6], c[7]) * (LS3 << sh);
			d4 = packs32(c[8], c[9]) * (LS4 << sh);
			d5 = packs32(c[10], c[11]) * (LS5 << sh);
			d6 = packs32(c[12], c[13]) * (LS6 << sh);
			d7 = packs32(c[14], c[15]) * (LS7 << sh);
		}
		c[0] = c[1] = c[2] = c[3] = c[4] = c[5] = c[6] = c[7] = c[8] = c[9] = c[10] = c[11] = c[12] = c[13] = c[14] = c[15] = (i8x16){};
		
		for (int i = 2;;) {
			// 1D transform
			i16x8 e0 = d0 + d4;
			i16x8 e1 = d5 - d3 - ((d7 >> 1) + d7);
			i16x8 e2 = d0 - d4;
			i16x8 e3 = d1 + d7 - ((d3 >> 1) + d3);
			i16x8 e4 = (d2 >> 1) - d6;
			i16x8 e5 = d7 - d1 + ((d5 >> 1) + d5);
			i16x8 e6 = (d6 >> 1) + d2;
			i16x8 e7 = d3 + d5 + ((d1 >> 1) + d1);
			i16x8 f0 = e0 + e6;
			i16x8 f1 = (e7 >> 2) + e1;
			i16x8 f2 = e2 + e4;
			i16x8 f3 = (e5 >> 2) + e3;
			i16x8 f4 = e2 - e4;
			i16x8 f5 = (e3 >> 2) - e5;
			i16x8 f6 = e0 - e6;
			i16x8 f7 = e7 - (e1 >> 2);
			
			// Compilers freak out whenever output uses other registers.
			d0 = f0 + f7;
			d1 = f2 + f5;
			d2 = f4 + f3;
			d3 = f6 + f1;
			d4 = f6 - f1;
			d5 = f4 - f3;
			d6 = f2 - f5;
			d7 = f0 - f7;
			if (--i == 0)
				break;
			
			// matrix transposition
			i16x8 x0 = ziplo16(d0, d1);
			i16x8 x1 = ziplo16(d2, d3);
			i16x8 x2 = ziplo16(d4, d5);
			i16x8 x3 = ziplo16(d6, d7);
			i16x8 x4 = ziphi16(d0, d1);
			i16x8 x5 = ziphi16(d2, d3);
			i16x8 x6 = ziphi16(d4, d5);
			i16x8 x7 = ziphi16(d6, d7);
			i16x8 x8 = ziplo32(x0, x1);
			i16x8 x9 = ziplo32(x2, x3);
			i16x8 xA = ziplo32(x4, x5);
			i16x8 xB = ziplo32(x6, x7);
			i16x8 xC = ziphi32(x0, x1);
			i16x8 xD = ziphi32(x2, x3);
			i16x8 xE = ziphi32(x4, x5);
			i16x8 xF = ziphi32(x6, x7);
			d0 = (i16x8)ziplo64(x8, x9) + 32;
			d1 = ziphi64(x8, x9);
			d2 = ziplo64(xC, xD);
			d3 = ziphi64(xC, xD);
			d4 = ziplo64(xA, xB);
			d5 = ziphi64(xA, xB);
			d6 = ziplo64(xE, xF);
			d7 = ziphi64(xE, xF);
		}
		
		// final residual values
		i16x8 r0 = d0 >> 6;
		i16x8 r1 = d1 >> 6;
		i16x8 r2 = d2 >> 6;
		i16x8 r3 = d3 >> 6;
		i16x8 r4 = d4 >> 6;
		i16x8 r5 = d5 >> 6;
		i16x8 r6 = d6 >> 6;
		i16x8 r7 = d7 >> 6;
		
		// addition to values in place, clipping and storage
		size_t stride = ctx->t.stride[iYCbCr];
		INIT_P();
		i8x16 p0 = addlou8s16(load64(P(0, 0)), r0);
		i8x16 p1 = addlou8s16(load64(P(0, 1)), r1);
		i8x16 p2 = addlou8s16(load64(P(0, 2)), r2);
		i8x16 p3 = addlou8s16(load64(P(0, 3)), r3);
		i8x16 p4 = addlou8s16(load64(P(0, 4)), r4);
		i8x16 p5 = addlou8s16(load64(P(0, 5)), r5);
		i8x16 p6 = addlou8s16(load64(P(0, 6)), r6);
		i8x16 p7 = addlou8s16(load64(P(0, 7)), r7);
		i64x2 u0 = packus16(p0, p1);
		i64x2 u1 = packus16(p2, p3);
		i64x2 u2 = packus16(p4, p5);
		i64x2 u3 = packus16(p6, p7);
		*(int64_t *)P(0, 0) = u0[0];
		*(int64_t *)P(0, 1) = u0[1];
		*(int64_t *)P(0, 2) = u1[0];
		*(int64_t *)P(0, 3) = u1[1];
		*(int64_t *)P(0, 4) = u2[0];
		*(int64_t *)P(0, 5) = u2[1];
		*(int64_t *)P(0, 6) = u3[0];
		*(int64_t *)P(0, 7) = u3[1];
	} // FIXME else 16bit
}



/**
 * DC transforms
 * 
 * These functions do not gain enough from 8bit to justify distinct versions.
 */
static void transform_dc4x4(Edge264Context *ctx, int iYCbCr)
{
	// load matrix in column order and multiply right
	i32x4 x0 = ctx->c_v[0] + ctx->c_v[1];
	i32x4 x1 = ctx->c_v[2] + ctx->c_v[3];
	i32x4 x2 = ctx->c_v[0] - ctx->c_v[1];
	i32x4 x3 = ctx->c_v[2] - ctx->c_v[3];
	ctx->c_v[0] = ctx->c_v[1] = ctx->c_v[2] = ctx->c_v[3] = (i8x16){};
	i32x4 x4 = x0 + x1;
	i32x4 x5 = x0 - x1;
	i32x4 x6 = x2 - x3;
	i32x4 x7 = x2 + x3;
	
	// transpose
	i32x4 x8 = ziplo32(x4, x5);
	i32x4 x9 = ziplo32(x6, x7);
	i32x4 xA = ziphi32(x4, x5);
	i32x4 xB = ziphi32(x6, x7);
	i32x4 xC = ziplo64(x8, x9);
	i32x4 xD = ziphi64(x8, x9);
	i32x4 xE = ziplo64(xA, xB);
	i32x4 xF = ziphi64(xA, xB);
	
	// multiply left
	i32x4 xG = xC + xD;
	i32x4 xH = xE + xF;
	i32x4 xI = xC - xD;
	i32x4 xJ = xE - xF;
	i32x4 f0 = xG + xH;
	i32x4 f1 = xG - xH;
	i32x4 f2 = xI - xJ;
	i32x4 f3 = xI + xJ;
	
	// scale
	unsigned qP = ctx->t.QP[0]; // FIXME 4:4:4
	i32x4 s32 = set32(32);
	i32x4 LS = set32((ctx->t.pps.weightScale4x4[iYCbCr][0] * normAdjust4x4[qP % 6][0]) << (qP / 6));
	i32x4 dc0 = shrrs32(f0 * LS, 6, s32);
	i32x4 dc1 = shrrs32(f1 * LS, 6, s32);
	i32x4 dc2 = shrrs32(f2 * LS, 6, s32);
	i32x4 dc3 = shrrs32(f3 * LS, 6, s32);
	
	// store in zigzag order if needed later ...
	if (mb->bits[0] & 1 << 5) {
		ctx->c_v[4] = ziplo64(dc0, dc1);
		ctx->c_v[5] = ziphi64(dc0, dc1);
		ctx->c_v[6] = ziplo64(dc2, dc3);
		ctx->c_v[7] = ziphi64(dc2, dc3);
		
	// ... or prepare for storage in place
	} else {
		i32x4 r0 = (dc0 + s32) >> 6;
		i32x4 r1 = (dc1 + s32) >> 6;
		i32x4 r2 = (dc2 + s32) >> 6;
		i32x4 r3 = (dc3 + s32) >> 6;
		#if defined(__SSE2__)
			i16x8 lo0 = shuffle32(shufflelo(r0, 0, 0, 2, 2), 0, 0, 1, 1);
			i16x8 lo1 = shuffle32(shufflelo(r1, 0, 0, 2, 2), 0, 0, 1, 1);
			i16x8 lo2 = shuffle32(shufflelo(r2, 0, 0, 2, 2), 0, 0, 1, 1);
			i16x8 lo3 = shuffle32(shufflelo(r3, 0, 0, 2, 2), 0, 0, 1, 1);
			i16x8 hi0 = shuffle32(shufflehi(r0, 0, 0, 2, 2), 2, 2, 3, 3);
			i16x8 hi1 = shuffle32(shufflehi(r1, 0, 0, 2, 2), 2, 2, 3, 3);
			i16x8 hi2 = shuffle32(shufflehi(r2, 0, 0, 2, 2), 2, 2, 3, 3);
			i16x8 hi3 = shuffle32(shufflehi(r3, 0, 0, 2, 2), 2, 2, 3, 3);
		#elif defined(__ARM_NEON)
			i16x8 v0 = vtrn1q_s16(r0, r0);
			i16x8 v1 = vtrn1q_s16(r1, r1);
			i16x8 v2 = vtrn1q_s16(r2, r2);
			i16x8 v3 = vtrn1q_s16(r3, r3);
			i16x8 lo0 = ziplo32(v0, v0);
			i16x8 lo1 = ziplo32(v1, v1);
			i16x8 lo2 = ziplo32(v2, v2);
			i16x8 lo3 = ziplo32(v3, v3);
			i16x8 hi0 = ziphi32(v0, v0);
			i16x8 hi1 = ziphi32(v1, v1);
			i16x8 hi2 = ziphi32(v2, v2);
			i16x8 hi3 = ziphi32(v3, v3);
		#endif
		
		// add to predicted samples
		size_t stride = ctx->t.stride[iYCbCr];
		uint8_t *p = ctx->samples_mb[iYCbCr];
		if (ctx->t.samples_clip[iYCbCr][0] == 255) {
			i8x16 p0 = *(i8x16 *)(p             );
			i8x16 p1 = *(i8x16 *)(p + stride    );
			i8x16 p2 = *(i8x16 *)(p + stride * 2);
			i8x16 p3 = *(i8x16 *)(p + stride * 3);
			*(i8x16 *)(p             ) = packus16(addlou8s16(p0, lo0), addhiu8s16(p0, hi0));
			*(i8x16 *)(p + stride    ) = packus16(addlou8s16(p1, lo0), addhiu8s16(p1, hi0));
			*(i8x16 *)(p + stride * 2) = packus16(addlou8s16(p2, lo0), addhiu8s16(p2, hi0));
			*(i8x16 *)(p + stride * 3) = packus16(addlou8s16(p3, lo0), addhiu8s16(p3, hi0));
			i8x16 p4 = *(i8x16 *)(p += stride * 4);
			i8x16 p5 = *(i8x16 *)(p + stride    );
			i8x16 p6 = *(i8x16 *)(p + stride * 2);
			i8x16 p7 = *(i8x16 *)(p + stride * 3);
			*(i8x16 *)(p             ) = packus16(addlou8s16(p4, lo1), addhiu8s16(p4, hi1));
			*(i8x16 *)(p + stride    ) = packus16(addlou8s16(p5, lo1), addhiu8s16(p5, hi1));
			*(i8x16 *)(p + stride * 2) = packus16(addlou8s16(p6, lo1), addhiu8s16(p6, hi1));
			*(i8x16 *)(p + stride * 3) = packus16(addlou8s16(p7, lo1), addhiu8s16(p7, hi1));
			i8x16 p8 = *(i8x16 *)(p += stride * 4);
			i8x16 p9 = *(i8x16 *)(p + stride    );
			i8x16 pA = *(i8x16 *)(p + stride * 2);
			i8x16 pB = *(i8x16 *)(p + stride * 3);
			*(i8x16 *)(p             ) = packus16(addlou8s16(p8, lo2), addhiu8s16(p8, hi2));
			*(i8x16 *)(p + stride    ) = packus16(addlou8s16(p9, lo2), addhiu8s16(p9, hi2));
			*(i8x16 *)(p + stride * 2) = packus16(addlou8s16(pA, lo2), addhiu8s16(pA, hi2));
			*(i8x16 *)(p + stride * 3) = packus16(addlou8s16(pB, lo2), addhiu8s16(pB, hi2));
			i8x16 pC = *(i8x16 *)(p += stride * 4);
			i8x16 pD = *(i8x16 *)(p + stride    );
			i8x16 pE = *(i8x16 *)(p + stride * 2);
			i8x16 pF = *(i8x16 *)(p + stride * 3);
			*(i8x16 *)(p             ) = packus16(addlou8s16(pC, lo3), addhiu8s16(pC, hi3));
			*(i8x16 *)(p + stride    ) = packus16(addlou8s16(pD, lo3), addhiu8s16(pD, hi3));
			*(i8x16 *)(p + stride * 2) = packus16(addlou8s16(pE, lo3), addhiu8s16(pE, hi3));
			*(i8x16 *)(p + stride * 3) = packus16(addlou8s16(pF, lo3), addhiu8s16(pF, hi3));
		}
	}
}

static void transform_dc2x2(Edge264Context *ctx)
{
	// load both matrices interlaced+transposed and multiply right
	i32x4 d0 = ctx->c_v[0] + ctx->c_v[1];
	i32x4 d1 = ctx->c_v[0] - ctx->c_v[1];
	ctx->c_v[0] = ctx->c_v[1] = (i8x16){};
	
	// transpose and multiply left
	i32x4 e0 = ziplo64(d0, d1);
	i32x4 e1 = ziphi64(d0, d1);
	i32x4 f0 = e0 + e1;
	i32x4 f1 = e0 - e1;
	
	// deinterlace and scale
	unsigned qPb = ctx->t.QP[1];
	unsigned qPr = ctx->t.QP[2];
	i32x4 LSb = set32((ctx->t.pps.weightScale4x4[1 + mb->mbIsInterFlag * 3][0] * normAdjust4x4[qPb % 6][0]) << (qPb / 6));
	i32x4 LSr = set32((ctx->t.pps.weightScale4x4[2 + mb->mbIsInterFlag * 3][0] * normAdjust4x4[qPr % 6][0]) << (qPr / 6));
	i32x4 dcCb = ((i32x4)unziplo32(f0, f1) * LSb) >> 5;
	i32x4 dcCr = ((i32x4)unziphi32(f0, f1) * LSr) >> 5;
	
	// store if needed later ...
	if (mb->f.CodedBlockPatternChromaAC) {
		ctx->c_v[4] = dcCb;
		ctx->c_v[5] = dcCr;
		
	// ... or prepare for storage in place
	} else {
		i32x4 s32 = set32(32); // for SSE
		i32x4 rb = shrrs32(dcCb, 6, s32);
		i32x4 rr = shrrs32(dcCr, 6, s32);
		#if defined(__SSE2__)
			i16x8 lob = shuffle32(shufflelo(rb, 0, 0, 2, 2), 0, 0, 1, 1);
			i16x8 lor = shuffle32(shufflelo(rr, 0, 0, 2, 2), 0, 0, 1, 1);
			i16x8 hib = shuffle32(shufflehi(rb, 0, 0, 2, 2), 2, 2, 3, 3);
			i16x8 hir = shuffle32(shufflehi(rr, 0, 0, 2, 2), 2, 2, 3, 3);
		#elif defined(__ARM_NEON)
			i16x8 vb = vtrn1q_s16(rb, rb);
			i16x8 vr = vtrn1q_s16(rr, rr);
			i16x8 lob = ziplo32(vb, vb);
			i16x8 lor = ziplo32(vr, vr);
			i16x8 hib = ziphi32(vb, vb);
			i16x8 hir = ziphi32(vr, vr);
		#endif
		
		// add to predicted samples
		uint8_t *p = ctx->samples_mb[1];
		size_t stride = ctx->t.stride[1] >> 1;
		if (ctx->t.samples_clip[1][0] == 255) {
			i16x8 b0 = addlou8s16(load64(p             ), lob);
			i16x8 r0 = addlou8s16(load64(p + stride    ), lor);
			i16x8 b1 = addlou8s16(load64(p + stride * 2), lob);
			i16x8 r1 = addlou8s16(load64(p + stride * 3), lor);
			i64x2 b8 = packus16(b0, b1);
			i64x2 r8 = packus16(r0, r1);
			*(int64_t *)(p             ) = b8[0];
			*(int64_t *)(p + stride    ) = r8[0];
			*(int64_t *)(p + stride * 2) = b8[1];
			*(int64_t *)(p + stride * 3) = r8[1];
			i16x8 b2 = addlou8s16(load64(p += stride * 4), lob);
			i16x8 r2 = addlou8s16(load64(p + stride    ), lor);
			i16x8 b3 = addlou8s16(load64(p + stride * 2), lob);
			i16x8 r3 = addlou8s16(load64(p + stride * 3), lor);
			i64x2 b9 = packus16(b2, b3);
			i64x2 r9 = packus16(r2, r3);
			*(int64_t *)(p             ) = b9[0];
			*(int64_t *)(p + stride    ) = r9[0];
			*(int64_t *)(p + stride * 2) = b9[1];
			*(int64_t *)(p + stride * 3) = r9[1];
			i16x8 b4 = addlou8s16(load64(p += stride * 4), hib);
			i16x8 r4 = addlou8s16(load64(p + stride    ), hir);
			i16x8 b5 = addlou8s16(load64(p + stride * 2), hib);
			i16x8 r5 = addlou8s16(load64(p + stride * 3), hir);
			i64x2 bA = packus16(b4, b5);
			i64x2 rA = packus16(r4, r5);
			*(int64_t *)(p             ) = bA[0];
			*(int64_t *)(p + stride    ) = rA[0];
			*(int64_t *)(p + stride * 2) = bA[1];
			*(int64_t *)(p + stride * 3) = rA[1];
			i16x8 b6 = addlou8s16(load64(p += stride * 4), hib);
			i16x8 r6 = addlou8s16(load64(p + stride    ), hir);
			i16x8 b7 = addlou8s16(load64(p + stride * 2), hib);
			i16x8 r7 = addlou8s16(load64(p + stride * 3), hir);
			i64x2 bB = packus16(b6, b7);
			i64x2 rB = packus16(r6, r7);
			*(int64_t *)(p             ) = bB[0];
			*(int64_t *)(p + stride    ) = rB[0];
			*(int64_t *)(p + stride * 2) = bB[1];
			*(int64_t *)(p + stride * 3) = rB[1];
		}
	}
}



// legacy code for 16-bit 8x8 transform
#if 0
void FUNC_CTX(transform_dc2x4)
{
	int iYCbCr = (0/*BlkIdx*/ - 8) >> 3; // BlkIdx is 16 or 24
	unsigned qP_DC = 0; //mb->QP[iYCbCr] + 3;
	int w = ctx->t.pps.weightScale4x4[iYCbCr + mb->mbIsInterFlag * 3][0];
	int nA = normAdjust4x4[qP_DC % 6][0];
	__m128i x0 = (__m128i)ctx->c_v[0]; // {c00, c01, c10, c11} as per 8.5.11.1
	__m128i x1 = (__m128i)ctx->c_v[1]; // {c20, c21, c30, c31}
	__m128i x2 = _mm_add_epi32(x0, x1); // {c00+c20, c01+c21, c10+c30, c11+c31}
	__m128i x3 = _mm_sub_epi32(x0, x1); // {c00-c20, c01-c21, c10-c30, c11-c31}
	__m128i x4 = ziplo64(x2, x3); // {c00+c20, c01+c21, c00-c20, c01-c21}
	__m128i x5 = ziphi64(x2, x3); // {c10+c30, c11+c31, c10-c30, c11-c31}
	__m128i x6 = _mm_add_epi32(x4, x5); // {d00, d01, d10, d11}
	__m128i x7 = _mm_sub_epi32(x4, x5); // {d30, d31, d20, d21}
	__m128i x8 = _mm_hadd_epi32(x6, x7); // {f00, f10, f30, f20}
	__m128i x9 = _mm_hsub_epi32(x6, x7); // {f01, f11, f31, f21}
	__m128i s = set32((w * nA) << (qP_DC / 6 + ctx->ps.BitDepth_C - 8));
	__m128i s32 = set32(32);
	__m128i dc0 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(x8, s), s32), 6);
	__m128i dc1 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(x9, s), s32), 6);
	__m128i *c = (__m128i *)ctx->c_v + 2 + iYCbCr * 2;
	c[0] = ziplo32(dc0, dc1);
	c[1] = _mm_shuffle_epi32(ziphi64(dc0, dc1), _MM_SHUFFLE(2, 0, 3, 1));
}

#ifdef __AVX2__
		// loading
		// FIXME scaling
		__m256i d0 = (__m256i)ctx->c_V[0];
		__m256i d1 = (__m256i)ctx->c_V[1];
		__m256i d2 = (__m256i)ctx->c_V[2];
		__m256i d3 = (__m256i)ctx->c_V[3];
		__m256i d4 = (__m256i)ctx->c_V[4];
		__m256i d5 = (__m256i)ctx->c_V[5];
		__m256i d6 = (__m256i)ctx->c_V[6];
		__m256i d7 = (__m256i)ctx->c_V[7];
		
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
			__m256i y0 = _mm256_ziplo_epi32(d0, d1);
			__m256i y1 = _mm256_ziplo_epi32(d2, d3);
			__m256i y2 = _mm256_ziplo_epi32(d4, d5);
			__m256i y3 = _mm256_ziplo_epi32(d6, d7);
			__m256i y4 = _mm256_ziphi_epi32(d0, d1);
			__m256i y5 = _mm256_ziphi_epi32(d2, d3);
			__m256i y6 = _mm256_ziphi_epi32(d4, d5);
			__m256i y7 = _mm256_ziphi_epi32(d6, d7);
			__m256i y8 = _mm256_ziplo_epi64(y0, y1);
			__m256i y9 = _mm256_ziplo_epi64(y2, y3);
			__m256i yA = _mm256_ziplo_epi64(y4, y5);
			__m256i yB = _mm256_ziplo_epi64(y6, y7);
			__m256i yC = _mm256_ziphi_epi64(y0, y1);
			__m256i yD = _mm256_ziphi_epi64(y2, y3);
			__m256i yE = _mm256_ziphi_epi64(y4, y5);
			__m256i yF = _mm256_ziphi_epi64(y6, y7);
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
		__m256i r0 = _mm256_permute4x64_epi64(yG, _MM_SHUFFLE(3, 1, 2, 0));
		__m256i r1 = _mm256_permute4x64_epi64(yH, _MM_SHUFFLE(3, 1, 2, 0));
		__m256i r2 = _mm256_permute4x64_epi64(yI, _MM_SHUFFLE(3, 1, 2, 0));
		__m256i r3 = _mm256_permute4x64_epi64(yJ, _MM_SHUFFLE(3, 1, 2, 0));
		
		// addition to values in place, clipping and storage
		size_t stride = ctx->t.stride[0]; // FIXME 4:4:4
		size_t stride3 = stride * 3;
		uint8_t *p = NULL; //ctx->frame + ctx->frame_offsets_x[0/*BlkIdx2i4x4[BlkIdx]*/] + ctx->frame_offsets_y[0/*BlkIdx2i4x4[BlkIdx]*/];
		uint8_t *q = p + stride * 4;
		__m256i zero = _mm256_setzero_si256();
		__m256i clip = _mm256_set1_epi16(ctx->t.samples_clip[0]); // FIXME 4:4:4
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
		// load half of samples
		// FIXME scaline
		__m128i d0 = (__m128i)ctx->c_v[0];
		__m128i d1 = (__m128i)ctx->c_v[2];
		__m128i d2 = (__m128i)ctx->c_v[4];
		__m128i d3 = (__m128i)ctx->c_v[6];
		__m128i d4 = (__m128i)ctx->c_v[8];
		__m128i d5 = (__m128i)ctx->c_v[10];
		__m128i d6 = (__m128i)ctx->c_v[12];
		__m128i d7 = (__m128i)ctx->c_v[14];
		
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
				ctx->c_v[0] = (v4si)d0;
				d0 = (__m128i)ctx->c_v[1];
				ctx->c_v[2] = (v4si)d1;
				d1 = (__m128i)ctx->c_v[3];
				ctx->c_v[4] = (v4si)d2;
				d2 = (__m128i)ctx->c_v[5];
				ctx->c_v[6] = (v4si)d3;
				d3 = (__m128i)ctx->c_v[7];
				ctx->c_v[8] = (v4si)d4;
				d4 = (__m128i)ctx->c_v[9];
				ctx->c_v[10] = (v4si)d5;
				d5 = (__m128i)ctx->c_v[11];
				ctx->c_v[12] = (v4si)d6;
				d6 = (__m128i)ctx->c_v[13];
				ctx->c_v[14] = (v4si)d7;
				d7 = (__m128i)ctx->c_v[15];
			}
			if (--i == 0)
				break;
			
			// transpose the half matrix going to memory
			__m128i x0 = ziplo32((__m128i)ctx->c_v[8], (__m128i)ctx->c_v[10]);
			__m128i x1 = ziplo32((__m128i)ctx->c_v[12], (__m128i)ctx->c_v[14]);
			__m128i x2 = ziphi32((__m128i)ctx->c_v[8], (__m128i)ctx->c_v[10]);
			__m128i x3 = ziphi32((__m128i)ctx->c_v[12], (__m128i)ctx->c_v[14]);
			__m128i x4 = ziplo32(d4, d5);
			__m128i x5 = ziplo32(d6, d7);
			__m128i x6 = ziphi32(d4, d5);
			__m128i x7 = ziphi32(d6, d7);
			ctx->c_v[1] = (v4si)_mm_add_epi32(ziplo64(x0, x1), set32(32));
			ctx->c_v[3] = (v4si)ziphi64(x0, x1);
			ctx->c_v[5] = (v4si)ziplo64(x2, x3);
			ctx->c_v[7] = (v4si)ziphi64(x2, x3);
			ctx->c_v[9] = (v4si)ziplo64(x4, x5);
			ctx->c_v[11] = (v4si)ziphi64(x4, x5);
			ctx->c_v[13] = (v4si)ziplo64(x6, x7);
			ctx->c_v[15] = (v4si)ziphi64(x6, x7);
			
			// transpose the half matrix staying in registers
			__m128i x8 = ziplo32((__m128i)ctx->c_v[0], (__m128i)ctx->c_v[2]);
			__m128i x9 = ziplo32((__m128i)ctx->c_v[4], (__m128i)ctx->c_v[6]);
			__m128i xA = ziphi32((__m128i)ctx->c_v[0], (__m128i)ctx->c_v[2]);
			__m128i xB = ziphi32((__m128i)ctx->c_v[4], (__m128i)ctx->c_v[6]);
			__m128i xC = ziplo32(d0, d1);
			__m128i xD = ziplo32(d2, d3);
			__m128i xE = ziphi32(d0, d1);
			__m128i xF = ziphi32(d2, d3);
			d0 = _mm_add_epi32(ziplo64(x8, x9), set32(32));
			d1 = ziphi64(x8, x9);
			d2 = ziplo64(xA, xB);
			d3 = ziphi64(xA, xB);
			d4 = ziplo64(xC, xD);
			d5 = ziphi64(xC, xD);
			d6 = ziplo64(xE, xF);
			d7 = ziphi64(xE, xF);
		}
		
		// final residual values
		__m128i r0 = packs32(_mm_srai_epi32((__m128i)ctx->c_v[0], 6), _mm_srai_epi32(d0, 6));
		__m128i r1 = packs32(_mm_srai_epi32((__m128i)ctx->c_v[2], 6), _mm_srai_epi32(d1, 6));
		__m128i r2 = packs32(_mm_srai_epi32((__m128i)ctx->c_v[4], 6), _mm_srai_epi32(d2, 6));
		__m128i r3 = packs32(_mm_srai_epi32((__m128i)ctx->c_v[6], 6), _mm_srai_epi32(d3, 6));
		__m128i r4 = packs32(_mm_srai_epi32((__m128i)ctx->c_v[8], 6), _mm_srai_epi32(d4, 6));
		__m128i r5 = packs32(_mm_srai_epi32((__m128i)ctx->c_v[10], 6), _mm_srai_epi32(d5, 6));
		__m128i r6 = packs32(_mm_srai_epi32((__m128i)ctx->c_v[12], 6), _mm_srai_epi32(d6, 6));
		__m128i r7 = packs32(_mm_srai_epi32((__m128i)ctx->c_v[14], 6), _mm_srai_epi32(d7, 6));
		
		// addition to values in place, clipping and storage
		size_t stride = ctx->t.stride[0]; // FIXME 4:4:4
		size_t stride3 = stride * 3;
		uint8_t *p = NULL; //ctx->frame + ctx->frame_offsets_x[0/*BlkIdx2i4x4[BlkIdx]*/] + ctx->frame_offsets_y[0/*BlkIdx2i4x4[BlkIdx]*/];
		uint8_t *q = p + stride * 4;
		__m128i zero = _mm_setzero_si128();
		__m128i clip = set16(ctx->t.samples_clip[0]); // FIXME 4:4:4
		__m128i p0 = *(__m128i *)(p             );
		__m128i p1 = *(__m128i *)(p + stride    );
		__m128i p2 = *(__m128i *)(p + stride * 2);
		__m128i p3 = *(__m128i *)(p + stride3   );
		__m128i p4 = *(__m128i *)(q             );
		__m128i p5 = *(__m128i *)(q + stride    );
		__m128i p6 = *(__m128i *)(q + stride * 2);
		__m128i p7 = *(__m128i *)(q + stride3   );
		__m128i u0 = min16(max16(adds16(p0, r0), zero), clip);
		__m128i u1 = min16(max16(adds16(p1, r1), zero), clip);
		__m128i u2 = min16(max16(adds16(p2, r2), zero), clip);
		__m128i u3 = min16(max16(adds16(p3, r3), zero), clip);
		__m128i u4 = min16(max16(adds16(p4, r4), zero), clip);
		__m128i u5 = min16(max16(adds16(p5, r5), zero), clip);
		__m128i u6 = min16(max16(adds16(p6, r6), zero), clip);
		__m128i u7 = min16(max16(adds16(p7, r7), zero), clip);
		*(__m128i *)(p             ) = u0;
		*(__m128i *)(p + stride    ) = u1;
		*(__m128i *)(p + stride * 2) = u2;
		*(__m128i *)(p + stride3   ) = u3;
		*(__m128i *)(q             ) = u4;
		*(__m128i *)(q + stride    ) = u5;
		*(__m128i *)(q + stride * 2) = u6;
		*(__m128i *)(q + stride3   ) = u7;
#endif // __AVX2__
#endif
