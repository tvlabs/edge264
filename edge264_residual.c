#include "edge264_internal.h"


static const i8x16 normAdjust4x4[6] = {
	{10, 13, 10, 13, 13, 16, 13, 16, 10, 13, 10, 13, 13, 16, 13, 16},
	{11, 14, 11, 14, 14, 18, 14, 18, 11, 14, 11, 14, 14, 18, 14, 18},
	{13, 16, 13, 16, 16, 20, 16, 20, 13, 16, 13, 16, 16, 20, 16, 20},
	{14, 18, 14, 18, 18, 23, 18, 23, 14, 18, 14, 18, 18, 23, 18, 23},
	{16, 20, 16, 20, 20, 25, 20, 25, 16, 20, 16, 20, 20, 25, 20, 25},
	{18, 23, 18, 23, 23, 29, 23, 29, 18, 23, 18, 23, 23, 29, 23, 29},
};
static const i8x16 normAdjust8x8[24] = {
	{20, 19, 25, 19, 20, 19, 25, 19, 19, 18, 24, 18, 19, 18, 24, 18},
	{25, 24, 32, 24, 25, 24, 32, 24, 19, 18, 24, 18, 19, 18, 24, 18},
	{20, 19, 25, 19, 20, 19, 25, 19, 19, 18, 24, 18, 19, 18, 24, 18},
	{25, 24, 32, 24, 25, 24, 32, 24, 19, 18, 24, 18, 19, 18, 24, 18},
	{22, 21, 28, 21, 22, 21, 28, 21, 21, 19, 26, 19, 21, 19, 26, 19},
	{28, 26, 35, 26, 28, 26, 35, 26, 21, 19, 26, 19, 21, 19, 26, 19},
	{22, 21, 28, 21, 22, 21, 28, 21, 21, 19, 26, 19, 21, 19, 26, 19},
	{28, 26, 35, 26, 28, 26, 35, 26, 21, 19, 26, 19, 21, 19, 26, 19},
	{26, 24, 33, 24, 26, 24, 33, 24, 24, 23, 31, 23, 24, 23, 31, 23},
	{33, 31, 42, 31, 33, 31, 42, 31, 24, 23, 31, 23, 24, 23, 31, 23},
	{26, 24, 33, 24, 26, 24, 33, 24, 24, 23, 31, 23, 24, 23, 31, 23},
	{33, 31, 42, 31, 33, 31, 42, 31, 24, 23, 31, 23, 24, 23, 31, 23},
	{28, 26, 35, 26, 28, 26, 35, 26, 26, 25, 33, 25, 26, 25, 33, 25},
	{35, 33, 45, 33, 35, 33, 45, 33, 26, 25, 33, 25, 26, 25, 33, 25},
	{28, 26, 35, 26, 28, 26, 35, 26, 26, 25, 33, 25, 26, 25, 33, 25},
	{35, 33, 45, 33, 35, 33, 45, 33, 26, 25, 33, 25, 26, 25, 33, 25},
	{32, 30, 40, 30, 32, 30, 40, 30, 30, 28, 38, 28, 30, 28, 38, 28},
	{40, 38, 51, 38, 40, 38, 51, 38, 30, 28, 38, 28, 30, 28, 38, 28},
	{32, 30, 40, 30, 32, 30, 40, 30, 30, 28, 38, 28, 30, 28, 38, 28},
	{40, 38, 51, 38, 40, 38, 51, 38, 30, 28, 38, 28, 30, 28, 38, 28},
	{36, 34, 46, 34, 36, 34, 46, 34, 34, 32, 43, 32, 34, 32, 43, 32},
	{46, 43, 58, 43, 46, 43, 58, 43, 34, 32, 43, 32, 34, 32, 43, 32},
	{36, 34, 46, 34, 36, 34, 46, 34, 34, 32, 43, 32, 34, 32, 43, 32},
	{46, 43, 58, 43, 46, 43, 58, 43, 34, 32, 43, 32, 34, 32, 43, 32},
};



/**
 * Inverse 4x4 transform
 *
 * Here we try to stay close to the spec's pseudocode, avoiding minor
 * optimisations that would make the code hard to understand.
 */
void FUNC(add_idct4x4, int iYCbCr, int qP, i8x16 wS, int DCidx, uint8_t *samples)
{
	// loading and scaling
	i8x16 zero = {};
	i32x4 sh = {qP / 6};
	i8x16 nA = normAdjust4x4[qP % 6];
	i16x8 LS0 = cvt8zx16(wS) * cvt8zx16(nA);
	i16x8 LS1 = (i16x8)unpackhi8(wS, zero) * (i16x8)unpackhi8(nA, zero);
	i32x4 mul0 = shl32(cvt16zx32(LS0), sh);
	i32x4 mul1 = shl32(unpackhi16(LS0, zero), sh);
	i32x4 mul2 = shl32(cvt16zx32(LS1), sh);
	i32x4 mul3 = shl32(unpackhi16(LS1, zero), sh);
	i32x4 s8 = set32(8);
	i32x4 d0 = (mul0 * ctx->c_v[0] + s8) >> 4;
	i32x4 d1 = (mul1 * ctx->c_v[1] + s8) >> 4;
	i32x4 d2 = (mul2 * ctx->c_v[2] + s8) >> 4;
	i32x4 d3 = (mul3 * ctx->c_v[3] + s8) >> 4;
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
	i32x4 x0 = unpacklo32(f0, f1);
	i32x4 x1 = unpacklo32(f2, f3);
	i32x4 x2 = unpackhi32(f0, f1);
	i32x4 x3 = unpackhi32(f2, f3);
	f0 = (i32x4)unpacklo64(x0, x1) + set32(32);
	f1 = unpackhi64(x0, x1);
	f2 = unpacklo64(x2, x3);
	f3 = unpackhi64(x2, x3);
	
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
	__m128i r0 = packs32(h0 >> 6, h1 >> 6);
	__m128i r1 = packs32(h2 >> 6, h3 >> 6);
	
	// addition to values in place, clipping and storage
	size_t stride = ctx->stride[iYCbCr];
	if (ctx->clip[iYCbCr] == 255) {
		i16x8 p0 = cvt8zx16(((i32x4){*(int32_t*)(samples             ), *(int32_t*)(samples + stride    )}));
		i16x8 p1 = cvt8zx16(((i32x4){*(int32_t*)(samples + stride * 2), *(int32_t*)(samples + stride * 3)}));
		i32x4 u = packus16(adds16(p0, r0), adds16(p1, r1));
		*(int32_t *)(samples             ) = u[0];
		*(int32_t *)(samples + stride    ) = u[1];
		*(int32_t *)(samples + stride * 2) = u[2];
		*(int32_t *)(samples + stride * 3) = u[3];
	} else {
		i64x2 p0 = {*(int64_t *)(samples             ), *(int64_t *)(samples + stride    )};
		i64x2 p1 = {*(int64_t *)(samples + stride * 2), *(int64_t *)(samples + stride * 3)};
		i16x8 clip = set16(ctx->clip[iYCbCr]);
		i64x2 u0 = min16(max16(adds16(p0, r0), zero), clip);
		i64x2 u1 = min16(max16(adds16(p1, r1), zero), clip);
		*(int64_t *)(samples             ) = u0[0];
		*(int64_t *)(samples + stride    ) = u0[1];
		*(int64_t *)(samples + stride * 2) = u1[0];
		*(int64_t *)(samples + stride * 3) = u1[1];
	}
}

void FUNC(add_dc4x4, int iYCbCr, int DCidx, uint8_t *samples) {
	i32x4 x = (set32(ctx->c[16 + DCidx]) + set32(32)) >> 6;
	i32x4 r = packs32(x, x);
	i8x16 zero = {};
	size_t stride = ctx->stride[iYCbCr];
	if (ctx->clip[iYCbCr] == 255) {
		i16x8 p0 = cvt8zx16(((i32x4){*(int32_t*)(samples             ), *(int32_t*)(samples + stride    )}));
		i16x8 p1 = cvt8zx16(((i32x4){*(int32_t*)(samples + stride * 2), *(int32_t*)(samples + stride * 3)}));
		i32x4 u = packus16(adds16(p0, r), adds16(p1, r));
		*(int32_t *)(samples             ) = u[0];
		*(int32_t *)(samples + stride    ) = u[1];
		*(int32_t *)(samples + stride * 2) = u[2];
		*(int32_t *)(samples + stride * 3) = u[3];
	} else {
		i64x2 p0 = {*(int64_t *)(samples             ), *(int64_t *)(samples + stride    )};
		i64x2 p1 = {*(int64_t *)(samples + stride * 2), *(int64_t *)(samples + stride * 3)};
		i16x8 clip = set16(ctx->clip[iYCbCr]);
		i64x2 u0 = min16(max16(adds16(p0, r), zero), clip);
		i64x2 u1 = min16(max16(adds16(p1, r), zero), clip);
		*(int64_t *)(samples             ) = u0[0];
		*(int64_t *)(samples + stride    ) = u0[1];
		*(int64_t *)(samples + stride * 2) = u1[0];
		*(int64_t *)(samples + stride * 3) = u1[1];
	}
}



/**
 * Inverse 8x8 transform
 */
void FUNC(add_idct8x8, int iYCbCr, uint8_t *samples)
{
	int qP = ctx->QP[iYCbCr];
	size_t stride = ctx->stride[iYCbCr];
	if (ctx->clip[iYCbCr] == 255) {
		// loading and scaling
		int div = qP / 6;
		i8x16 zero = {};
		u8x16 *wS = ctx->ps.weightScale8x8_v[iYCbCr * 2 + mb->f.mbIsInterFlag];
		const i8x16 *nA = &normAdjust8x8[qP % 6 * 4];
		i16x8 LS0 = cvt8zx16(wS[0]) * cvt8zx16(nA[0]);
		i16x8 LS1 = (i16x8)unpackhi8(wS[0], zero) * (i16x8)unpackhi8(nA[0], zero);
		i16x8 LS2 = cvt8zx16(wS[1]) * cvt8zx16(nA[1]);
		i16x8 LS3 = (i16x8)unpackhi8(wS[1], zero) * (i16x8)unpackhi8(nA[1], zero);
		i16x8 LS4 = cvt8zx16(wS[2]) * cvt8zx16(nA[2]);
		i16x8 LS5 = (i16x8)unpackhi8(wS[2], zero) * (i16x8)unpackhi8(nA[2], zero);
		i16x8 LS6 = cvt8zx16(wS[3]) * cvt8zx16(nA[3]);
		i16x8 LS7 = (i16x8)unpackhi8(wS[3], zero) * (i16x8)unpackhi8(nA[3], zero);
		i32x4 *c = ctx->c_v;
		i16x8 d0, d1, d2, d3, d4, d5, d6, d7;
		if (div < 6) {
			i32x4 mul0 = madd16(cvt16zx32(LS0), c[0]);
			i32x4 mul1 = madd16(unpackhi16(LS0, zero), c[1]);
			i32x4 mul2 = madd16(cvt16zx32(LS1), c[2]);
			i32x4 mul3 = madd16(unpackhi16(LS1, zero), c[3]);
			i32x4 mul4 = madd16(cvt16zx32(LS2), c[4]);
			i32x4 mul5 = madd16(unpackhi16(LS2, zero), c[5]);
			i32x4 mul6 = madd16(cvt16zx32(LS3), c[6]);
			i32x4 mul7 = madd16(unpackhi16(LS3, zero), c[7]);
			i32x4 mul8 = madd16(cvt16zx32(LS4), c[8]);
			i32x4 mul9 = madd16(unpackhi16(LS4, zero), c[9]);
			i32x4 mulA = madd16(cvt16zx32(LS5), c[10]);
			i32x4 mulB = madd16(unpackhi16(LS5, zero), c[11]);
			i32x4 mulC = madd16(cvt16zx32(LS6), c[12]);
			i32x4 mulD = madd16(unpackhi16(LS6, zero), c[13]);
			i32x4 mulE = madd16(cvt16zx32(LS7), c[14]);
			i32x4 mulF = madd16(unpackhi16(LS7, zero), c[15]);
			i32x4 off = set32(1 << (5 - div));
			i32x4 sh = {6 - div};
			d0 = packs32(shr32((mul0 + off), sh), shr32((mul1 + off), sh));
			d1 = packs32(shr32((mul2 + off), sh), shr32((mul3 + off), sh));
			d2 = packs32(shr32((mul4 + off), sh), shr32((mul5 + off), sh));
			d3 = packs32(shr32((mul6 + off), sh), shr32((mul7 + off), sh));
			d4 = packs32(shr32((mul8 + off), sh), shr32((mul9 + off), sh));
			d5 = packs32(shr32((mulA + off), sh), shr32((mulB + off), sh));
			d6 = packs32(shr32((mulC + off), sh), shr32((mulD + off), sh));
			d7 = packs32(shr32((mulE + off), sh), shr32((mulF + off), sh));
		} else {
			i32x4 sh = {div - 6};
			d0 = shl16(packs32(c[0], c[1]) * LS0, sh);
			d1 = shl16(packs32(c[2], c[3]) * LS1, sh);
			d2 = shl16(packs32(c[4], c[5]) * LS2, sh);
			d3 = shl16(packs32(c[6], c[7]) * LS3, sh);
			d4 = shl16(packs32(c[8], c[9]) * LS4, sh);
			d5 = shl16(packs32(c[10], c[11]) * LS5, sh);
			d6 = shl16(packs32(c[12], c[13]) * LS6, sh);
			d7 = shl16(packs32(c[14], c[15]) * LS7, sh);
		}
		
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
			i16x8 x0 = unpacklo16(d0, d1);
			i16x8 x1 = unpacklo16(d2, d3);
			i16x8 x2 = unpacklo16(d4, d5);
			i16x8 x3 = unpacklo16(d6, d7);
			i16x8 x4 = unpackhi16(d0, d1);
			i16x8 x5 = unpackhi16(d2, d3);
			i16x8 x6 = unpackhi16(d4, d5);
			i16x8 x7 = unpackhi16(d6, d7);
			i16x8 x8 = unpacklo32(x0, x1);
			i16x8 x9 = unpacklo32(x2, x3);
			i16x8 xA = unpacklo32(x4, x5);
			i16x8 xB = unpacklo32(x6, x7);
			i16x8 xC = unpackhi32(x0, x1);
			i16x8 xD = unpackhi32(x2, x3);
			i16x8 xE = unpackhi32(x4, x5);
			i16x8 xF = unpackhi32(x6, x7);
			d0 = (i16x8)unpacklo64(x8, x9) + set16(32);
			d1 = unpackhi64(x8, x9);
			d2 = unpacklo64(xC, xD);
			d3 = unpackhi64(xC, xD);
			d4 = unpacklo64(xA, xB);
			d5 = unpackhi64(xA, xB);
			d6 = unpacklo64(xE, xF);
			d7 = unpackhi64(xE, xF);
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
		i16x8 p0 = load8zx16(samples             );
		i16x8 p1 = load8zx16(samples + stride    );
		i16x8 p2 = load8zx16(samples + stride * 2);
		i16x8 p3 = load8zx16(samples + stride * 3);
		i16x8 p4 = load8zx16(samples + stride * 4);
		i16x8 p5 = load8zx16(samples + stride * 5);
		i16x8 p6 = load8zx16(samples + stride * 6);
		i16x8 p7 = load8zx16(samples + stride * 7);
		i64x2 u0 = packus16(adds16(p0, r0), adds16(p1, r1));
		i64x2 u1 = packus16(adds16(p2, r2), adds16(p3, r3));
		i64x2 u2 = packus16(adds16(p4, r4), adds16(p5, r5));
		i64x2 u3 = packus16(adds16(p6, r6), adds16(p7, r7));
		*(int64_t *)(samples             ) = u0[0];
		*(int64_t *)(samples + stride    ) = u0[1];
		*(int64_t *)(samples + stride * 2) = u1[0];
		*(int64_t *)(samples + stride * 3) = u1[1];
		*(int64_t *)(samples + stride * 4) = u2[0];
		*(int64_t *)(samples + stride * 5) = u2[1];
		*(int64_t *)(samples + stride * 6) = u3[0];
		*(int64_t *)(samples + stride * 7) = u3[1];
	} // FIXME else 16bit
}



/**
 * DC transforms
 * 
 * These functions do not gain enough from 8bit to justify distinct versions.
 */
void FUNC(transform_dc4x4, int iYCbCr)
{
	// load matrix in column order and multiply right
	i32x4 x0 = ctx->c_v[0] + ctx->c_v[1];
	i32x4 x1 = ctx->c_v[2] + ctx->c_v[3];
	i32x4 x2 = ctx->c_v[0] - ctx->c_v[1];
	i32x4 x3 = ctx->c_v[2] - ctx->c_v[3];
	i32x4 x4 = x0 + x1;
	i32x4 x5 = x0 - x1;
	i32x4 x6 = x2 - x3;
	i32x4 x7 = x2 + x3;
	
	// transpose
	i32x4 x8 = unpacklo32(x4, x5);
	i32x4 x9 = unpacklo32(x6, x7);
	i32x4 xA = unpackhi32(x4, x5);
	i32x4 xB = unpackhi32(x6, x7);
	i32x4 xC = unpacklo64(x8, x9);
	i32x4 xD = unpackhi64(x8, x9);
	i32x4 xE = unpacklo64(xA, xB);
	i32x4 xF = unpackhi64(xA, xB);
	
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
	unsigned qP = ctx->QP[0]; // FIXME 4:4:4
	i32x4 s32 = set32(32);
	i32x4 LS = set32((ctx->ps.weightScale4x4[iYCbCr][0] * normAdjust4x4[qP % 6][0]) << (qP / 6));
	i32x4 dc0 = (f0 * LS + s32) >> 6;
	i32x4 dc1 = (f1 * LS + s32) >> 6;
	i32x4 dc2 = (f2 * LS + s32) >> 6;
	i32x4 dc3 = (f3 * LS + s32) >> 6;
	
	// store in zigzag order if needed later ...
	if (mb->bits[0] & 1 << 5) {
		ctx->c_v[4] = unpacklo64(dc0, dc1);
		ctx->c_v[5] = unpackhi64(dc0, dc1);
		ctx->c_v[6] = unpacklo64(dc2, dc3);
		ctx->c_v[7] = unpackhi64(dc2, dc3);
		
	// ... or prepare for storage in place
	} else {
		size_t stride = ctx->stride[iYCbCr];
		size_t stride3 = stride * 3;
		uint8_t *p = ctx->samples_mb[iYCbCr];
		uint8_t *q = p + stride * 4;
		uint8_t *r = p + stride * 8;
		uint8_t *s = q + stride * 8;
		i32x4 r0 = (dc0 + s32) >> 6;
		i32x4 r1 = (dc1 + s32) >> 6;
		i32x4 r2 = (dc2 + s32) >> 6;
		i32x4 r3 = (dc3 + s32) >> 6;
		i8x16 shuflo = {0, 1, 0, 1, 0, 1, 0, 1, 4, 5, 4, 5, 4, 5, 4, 5};
		i8x16 shufhi = {8, 9, 8, 9, 8, 9, 8, 9, 12, 13, 12, 13, 12, 13, 12, 13};
		i16x8 lo0 = shuffle8(r0, shuflo);
		i16x8 lo1 = shuffle8(r1, shuflo);
		i16x8 lo2 = shuffle8(r2, shuflo);
		i16x8 lo3 = shuffle8(r3, shuflo);
		i16x8 hi0 = shuffle8(r0, shufhi);
		i16x8 hi1 = shuffle8(r1, shufhi);
		i16x8 hi2 = shuffle8(r2, shufhi);
		i16x8 hi3 = shuffle8(r3, shufhi);
		
		// add to predicted samples
		if (ctx->clip[iYCbCr] == 255) {
			i8x16 zero = {};
			i8x16 p0 = *(i8x16 *)(p             );
			i8x16 p1 = *(i8x16 *)(p + stride    );
			i8x16 p2 = *(i8x16 *)(p + stride * 2);
			i8x16 p3 = *(i8x16 *)(p + stride3   );
			*(i8x16 *)(p             ) = packus16(adds16(cvt8zx16(p0), lo0), adds16(unpackhi8(p0, zero), hi0));
			*(i8x16 *)(p + stride    ) = packus16(adds16(cvt8zx16(p1), lo0), adds16(unpackhi8(p1, zero), hi0));
			*(i8x16 *)(p + stride * 2) = packus16(adds16(cvt8zx16(p2), lo0), adds16(unpackhi8(p2, zero), hi0));
			*(i8x16 *)(p + stride3   ) = packus16(adds16(cvt8zx16(p3), lo0), adds16(unpackhi8(p3, zero), hi0));
			i8x16 p4 = *(i8x16 *)(q             );
			i8x16 p5 = *(i8x16 *)(q + stride    );
			i8x16 p6 = *(i8x16 *)(q + stride * 2);
			i8x16 p7 = *(i8x16 *)(q + stride3   );
			*(i8x16 *)(q             ) = packus16(adds16(cvt8zx16(p4), lo1), adds16(unpackhi8(p4, zero), hi1));
			*(i8x16 *)(q + stride    ) = packus16(adds16(cvt8zx16(p5), lo1), adds16(unpackhi8(p5, zero), hi1));
			*(i8x16 *)(q + stride * 2) = packus16(adds16(cvt8zx16(p6), lo1), adds16(unpackhi8(p6, zero), hi1));
			*(i8x16 *)(q + stride3   ) = packus16(adds16(cvt8zx16(p7), lo1), adds16(unpackhi8(p7, zero), hi1));
			i8x16 p8 = *(i8x16 *)(r             );
			i8x16 p9 = *(i8x16 *)(r + stride    );
			i8x16 pA = *(i8x16 *)(r + stride * 2);
			i8x16 pB = *(i8x16 *)(r + stride3   );
			*(i8x16 *)(r             ) = packus16(adds16(cvt8zx16(p8), lo2), adds16(unpackhi8(p8, zero), hi2));
			*(i8x16 *)(r + stride    ) = packus16(adds16(cvt8zx16(p9), lo2), adds16(unpackhi8(p9, zero), hi2));
			*(i8x16 *)(r + stride * 2) = packus16(adds16(cvt8zx16(pA), lo2), adds16(unpackhi8(pA, zero), hi2));
			*(i8x16 *)(r + stride3   ) = packus16(adds16(cvt8zx16(pB), lo2), adds16(unpackhi8(pB, zero), hi2));
			i8x16 pC = *(i8x16 *)(s             );
			i8x16 pD = *(i8x16 *)(s + stride    );
			i8x16 pE = *(i8x16 *)(s + stride * 2);
			i8x16 pF = *(i8x16 *)(s + stride3   );
			*(i8x16 *)(s             ) = packus16(adds16(cvt8zx16(pC), lo3), adds16(unpackhi8(pC, zero), hi3));
			*(i8x16 *)(s + stride    ) = packus16(adds16(cvt8zx16(pD), lo3), adds16(unpackhi8(pD, zero), hi3));
			*(i8x16 *)(s + stride * 2) = packus16(adds16(cvt8zx16(pE), lo3), adds16(unpackhi8(pE, zero), hi3));
			*(i8x16 *)(s + stride3   ) = packus16(adds16(cvt8zx16(pF), lo3), adds16(unpackhi8(pF, zero), hi3));
		}
	}
}

void FUNC(transform_dc2x2)
{
	// load both matrices interlaced+transposed and multiply right
	i32x4 d0 = ctx->c_v[0] + ctx->c_v[1];
	i32x4 d1 = ctx->c_v[0] - ctx->c_v[1];
	
	// transpose and multiply left
	i32x4 e0 = unpacklo64(d0, d1);
	i32x4 e1 = unpackhi64(d0, d1);
	i32x4 f0 = e0 + e1;
	i32x4 f1 = e0 - e1;
	
	// deinterlace and scale
	unsigned qPb = ctx->QP[1];
	unsigned qPr = ctx->QP[2];
	i32x4 LSb = set32((ctx->ps.weightScale4x4[1 + mb->f.mbIsInterFlag * 3][0] * normAdjust4x4[qPb % 6][0]) << (qPb / 6));
	i32x4 LSr = set32((ctx->ps.weightScale4x4[2 + mb->f.mbIsInterFlag * 3][0] * normAdjust4x4[qPr % 6][0]) << (qPr / 6));
	i32x4 fb = shuffleps(f0, f1, 0, 2, 0, 2);
	i32x4 fr = shuffleps(f0, f1, 1, 3, 1, 3);
	i32x4 dcCb = (fb * LSb) >> 5;
	i32x4 dcCr = (fr * LSr) >> 5;
	
	// store if needed later ...
	if (mb->f.CodedBlockPatternChromaAC) {
		ctx->c_v[4] = dcCb;
		ctx->c_v[5] = dcCr;
		
	// ... or prepare for storage in place
	} else {
		size_t stride = ctx->stride[1];
		size_t stride3 = stride * 3;
		uint8_t *pb = ctx->samples_mb[1];
		uint8_t *pr = ctx->samples_mb[2];
		uint8_t *qb = pb + stride * 4;
		uint8_t *qr = pr + stride * 4;
		i32x4 s32 = set32(32);
		i32x4 rb = (dcCb + s32) >> 6;
		i32x4 rr = (dcCr + s32) >> 6;
		i8x16 shuflo = {0, 1, 0, 1, 0, 1, 0, 1, 4, 5, 4, 5, 4, 5, 4, 5};
		i8x16 shufhi = {8, 9, 8, 9, 8, 9, 8, 9, 12, 13, 12, 13, 12, 13, 12, 13};
		i16x8 lob = shuffle8(rb, shuflo);
		i16x8 hib = shuffle8(rb, shufhi);
		i16x8 lor = shuffle8(rr, shuflo);
		i16x8 hir = shuffle8(rr, shufhi);
		i8x16 zero = {};
		
		// add to predicted samples
		if (ctx->clip[1] == 255) {
			i16x8 b0 = adds16(load8zx16(pb             ), lob);
			i16x8 b1 = adds16(load8zx16(pb + stride    ), lob);
			i16x8 b2 = adds16(load8zx16(pb + stride * 2), lob);
			i16x8 b3 = adds16(load8zx16(pb + stride3   ), lob);
			i16x8 b4 = adds16(load8zx16(qb             ), hib);
			i16x8 b5 = adds16(load8zx16(qb + stride    ), hib);
			i16x8 b6 = adds16(load8zx16(qb + stride * 2), hib);
			i16x8 b7 = adds16(load8zx16(qb + stride3   ), hib);
			i64x2 b8 = packus16(b0, b1);
			i64x2 b9 = packus16(b2, b3);
			i64x2 bA = packus16(b4, b5);
			i64x2 bB = packus16(b6, b7);
			*(int64_t *)(pb             ) = b8[0];
			*(int64_t *)(pb + stride    ) = b8[1];
			*(int64_t *)(pb + stride * 2) = b9[0];
			*(int64_t *)(pb + stride3   ) = b9[1];
			*(int64_t *)(qb             ) = bA[0];
			*(int64_t *)(qb + stride    ) = bA[1];
			*(int64_t *)(qb + stride * 2) = bB[0];
			*(int64_t *)(qb + stride3   ) = bB[1];
			i16x8 r0 = adds16(load8zx16(pr             ), lor);
			i16x8 r1 = adds16(load8zx16(pr + stride    ), lor);
			i16x8 r2 = adds16(load8zx16(pr + stride * 2), lor);
			i16x8 r3 = adds16(load8zx16(pr + stride3   ), lor);
			i16x8 r4 = adds16(load8zx16(qr             ), hir);
			i16x8 r5 = adds16(load8zx16(qr + stride    ), hir);
			i16x8 r6 = adds16(load8zx16(qr + stride * 2), hir);
			i16x8 r7 = adds16(load8zx16(qr + stride3   ), hir);
			i64x2 r8 = packus16(r0, r1);
			i64x2 r9 = packus16(r2, r3);
			i64x2 rA = packus16(r4, r5);
			i64x2 rB = packus16(r6, r7);
			*(int64_t *)(pr             ) = r8[0];
			*(int64_t *)(pr + stride    ) = r8[1];
			*(int64_t *)(pr + stride * 2) = r9[0];
			*(int64_t *)(pr + stride3   ) = r9[1];
			*(int64_t *)(qr             ) = rA[0];
			*(int64_t *)(qr + stride    ) = rA[1];
			*(int64_t *)(qr + stride * 2) = rB[0];
			*(int64_t *)(qr + stride3   ) = rB[1];
		}
	}
}



// legacy code for 16-bit 8x8 transform
#if 0
void FUNC(transform_dc2x4)
{
	int iYCbCr = (0/*BlkIdx*/ - 8) >> 3; // BlkIdx is 16 or 24
	unsigned qP_DC = 0; //mb->QP[iYCbCr] + 3;
	int w = ctx->ps.weightScale4x4[iYCbCr + mb->f.mbIsInterFlag * 3][0];
	int nA = normAdjust4x4[qP_DC % 6][0];
	__m128i x0 = (__m128i)ctx->c_v[0]; // {c00, c01, c10, c11} as per 8.5.11.1
	__m128i x1 = (__m128i)ctx->c_v[1]; // {c20, c21, c30, c31}
	__m128i x2 = _mm_add_epi32(x0, x1); // {c00+c20, c01+c21, c10+c30, c11+c31}
	__m128i x3 = _mm_sub_epi32(x0, x1); // {c00-c20, c01-c21, c10-c30, c11-c31}
	__m128i x4 = unpacklo64(x2, x3); // {c00+c20, c01+c21, c00-c20, c01-c21}
	__m128i x5 = unpackhi64(x2, x3); // {c10+c30, c11+c31, c10-c30, c11-c31}
	__m128i x6 = _mm_add_epi32(x4, x5); // {d00, d01, d10, d11}
	__m128i x7 = _mm_sub_epi32(x4, x5); // {d30, d31, d20, d21}
	__m128i x8 = _mm_hadd_epi32(x6, x7); // {f00, f10, f30, f20}
	__m128i x9 = _mm_hsub_epi32(x6, x7); // {f01, f11, f31, f21}
	__m128i s = set32((w * nA) << (qP_DC / 6 + ctx->ps.BitDepth_C - 8));
	__m128i s32 = set32(32);
	__m128i dc0 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(x8, s), s32), 6);
	__m128i dc1 = _mm_srai_epi32(_mm_add_epi32(_mm_mullo_epi32(x9, s), s32), 6);
	__m128i *c = (__m128i *)ctx->c_v + 2 + iYCbCr * 2;
	c[0] = unpacklo32(dc0, dc1);
	c[1] = _mm_shuffle_epi32(unpackhi64(dc0, dc1), _MM_SHUFFLE(2, 0, 3, 1));
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
		__m256i r0 = _mm256_permute4x64_epi64(yG, _MM_SHUFFLE(3, 1, 2, 0));
		__m256i r1 = _mm256_permute4x64_epi64(yH, _MM_SHUFFLE(3, 1, 2, 0));
		__m256i r2 = _mm256_permute4x64_epi64(yI, _MM_SHUFFLE(3, 1, 2, 0));
		__m256i r3 = _mm256_permute4x64_epi64(yJ, _MM_SHUFFLE(3, 1, 2, 0));
		
		// addition to values in place, clipping and storage
		size_t stride = ctx->stride[0]; // FIXME 4:4:4
		size_t stride3 = stride * 3;
		uint8_t *p = NULL; //ctx->frame + ctx->frame_offsets_x[0/*BlkIdx2i4x4[BlkIdx]*/] + ctx->frame_offsets_y[0/*BlkIdx2i4x4[BlkIdx]*/];
		uint8_t *q = p + stride * 4;
		__m256i zero = _mm256_setzero_si256();
		__m256i clip = _mm256_set1_epi16(ctx->clip[0]); // FIXME 4:4:4
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
			__m128i x0 = unpacklo32((__m128i)ctx->c_v[8], (__m128i)ctx->c_v[10]);
			__m128i x1 = unpacklo32((__m128i)ctx->c_v[12], (__m128i)ctx->c_v[14]);
			__m128i x2 = unpackhi32((__m128i)ctx->c_v[8], (__m128i)ctx->c_v[10]);
			__m128i x3 = unpackhi32((__m128i)ctx->c_v[12], (__m128i)ctx->c_v[14]);
			__m128i x4 = unpacklo32(d4, d5);
			__m128i x5 = unpacklo32(d6, d7);
			__m128i x6 = unpackhi32(d4, d5);
			__m128i x7 = unpackhi32(d6, d7);
			ctx->c_v[1] = (v4si)_mm_add_epi32(unpacklo64(x0, x1), set32(32));
			ctx->c_v[3] = (v4si)unpackhi64(x0, x1);
			ctx->c_v[5] = (v4si)unpacklo64(x2, x3);
			ctx->c_v[7] = (v4si)unpackhi64(x2, x3);
			ctx->c_v[9] = (v4si)unpacklo64(x4, x5);
			ctx->c_v[11] = (v4si)unpackhi64(x4, x5);
			ctx->c_v[13] = (v4si)unpacklo64(x6, x7);
			ctx->c_v[15] = (v4si)unpackhi64(x6, x7);
			
			// transpose the half matrix staying in registers
			__m128i x8 = unpacklo32((__m128i)ctx->c_v[0], (__m128i)ctx->c_v[2]);
			__m128i x9 = unpacklo32((__m128i)ctx->c_v[4], (__m128i)ctx->c_v[6]);
			__m128i xA = unpackhi32((__m128i)ctx->c_v[0], (__m128i)ctx->c_v[2]);
			__m128i xB = unpackhi32((__m128i)ctx->c_v[4], (__m128i)ctx->c_v[6]);
			__m128i xC = unpacklo32(d0, d1);
			__m128i xD = unpacklo32(d2, d3);
			__m128i xE = unpackhi32(d0, d1);
			__m128i xF = unpackhi32(d2, d3);
			d0 = _mm_add_epi32(unpacklo64(x8, x9), set32(32));
			d1 = unpackhi64(x8, x9);
			d2 = unpacklo64(xA, xB);
			d3 = unpackhi64(xA, xB);
			d4 = unpacklo64(xC, xD);
			d5 = unpackhi64(xC, xD);
			d6 = unpacklo64(xE, xF);
			d7 = unpackhi64(xE, xF);
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
		size_t stride = ctx->stride[0]; // FIXME 4:4:4
		size_t stride3 = stride * 3;
		uint8_t *p = NULL; //ctx->frame + ctx->frame_offsets_x[0/*BlkIdx2i4x4[BlkIdx]*/] + ctx->frame_offsets_y[0/*BlkIdx2i4x4[BlkIdx]*/];
		uint8_t *q = p + stride * 4;
		__m128i zero = _mm_setzero_si128();
		__m128i clip = set16(ctx->clip[0]); // FIXME 4:4:4
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
