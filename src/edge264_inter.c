#include "edge264_internal.h"

#if defined(__SSE2__)
	#define shrrpus16(a, b, i) packus16(((i16x8)(a) + (1 << (i - 1))) >> i, ((i16x8)(b) + (1 << (i - 1))) >> i)
	#define zipmd64(a, b) (i64x2)_mm_shuffle_ps((__m128)(a), (__m128)(b), _MM_SHUFFLE(2, 1, 2, 1))
	static const i8x16 mul15 = {1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5};
	static const i8x16 mul20 = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20};
	static const i8x16 mul51 = {-5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1};
	static always_inline i16x8 cvthi8s16(i8x16 a) {return (i16x8)ziphi8(a, a) >> 8;}
	static always_inline u8x16 maddshrL(u8x16 q, u8x16 p, i8x16 w, i16x8 o, i64x2 wd) {
		i16x8 x0 = shr16(adds16(maddubs(ziplo8(q, p), w), o), wd);
		i16x8 x1 = shr16(adds16(maddubs(ziphi8(q, p), w), o), wd);
		return packus16(x0, x1);
	}
	static always_inline u8x16 maddshrC16(u8x16 q, u8x16 p, i8x16 wCb, i8x16 wCr, i16x8 oCb, i16x8 oCr, i64x2 wd) {
		i16x8 x0 = shr16(adds16(maddubs(ziplo8(q, p), wCb), oCb), wd);
		i16x8 x1 = shr16(adds16(maddubs(ziphi8(q, p), wCr), oCr), wd);
		return packus16(x0, x1);
	}
	static always_inline u8x16 maddshrC8(u8x16 q, u8x16 p, i8x16 w, i8x16 _, i16x8 o, i64x2 wd) {
		i16x8 x0 = shr16(adds16(maddubs(ziplo8(q, p), w), o), wd);
		i16x8 x1 = shr16(adds16(maddubs(ziphi8(q, p), w), o), wd);
		return packus16(x0, x1);
	}
	static always_inline i8x16 maddshrC4(u8x16 q, u8x16 p, i8x16 w, i8x16 _, i16x8 o, i64x2 wd) {
		i16x8 a = shr16(adds16(maddubs(ziplo8(q, p), w), o), wd);
		return packus16(a, a);
	}
	static always_inline i16x8 maddABCD(i8x16 ab, i8x16 cd, i8x16 shuf, i8x16 AB, i8x16 CD) {
		return maddubs(shuffle(ab, shuf), AB) + maddubs(shuffle(cd, shuf), CD);
	}
	static always_inline i16x8 sixtapVlo(i8x16 a, i8x16 b, i8x16 c, i8x16 d, i8x16 e, i8x16 f) {
		i8x16 ab = ziplo8(a, b);
		i8x16 cd = ziplo8(c, d);
		i8x16 ef = ziplo8(e, f);
		return maddubs(ab, mul15) + maddubs(cd, mul20) + maddubs(ef, mul51);
	}
	static always_inline i16x8 sixtapVhi(i8x16 a, i8x16 b, i8x16 c, i8x16 d, i8x16 e, i8x16 f) {
		i8x16 ab = ziphi8(a, b);
		i8x16 cd = ziphi8(c, d);
		i8x16 ef = ziphi8(e, f);
		return maddubs(ab, mul15) + maddubs(cd, mul20) + maddubs(ef, mul51);
	}
	static always_inline i16x8 sixtapH4(i8x16 l0, i8x16 l1) {
		i8x16 a = ziplo8(l0, shr128(l0, 1));
		i8x16 b = ziplo8(l1, shr128(l1, 1));
		i8x16 c = ziplo64(a, b);
		i8x16 d = shuffleps(a, b, 1, 2, 1, 2);
		i8x16 e = ziphi64(a, b);
		return maddubs(c, mul15) + maddubs(d, mul20) + maddubs(e, mul51);
	}
	static always_inline i16x8 sixtapH8(i8x16 a) {
		i8x16 a1 = shr128(a, 1);
		i8x16 ab = ziplo8(a, a1);
		i8x16 ij = ziphi8(a, a1);
		i8x16 cd = shrd128(ab, ij, 4);
		i8x16 ef = shrd128(ab, ij, 8);
		return maddubs(ab, mul15) + maddubs(cd, mul20) + maddubs(ef, mul51);
	}
	#define SIXTAPH16(v0, v1, a, b)\
		i8x16 _##b##1 = shr128(b, 1);\
		i8x16 _##a##0 = ziplo8(a, shr128(a, 1));\
		i8x16 _##b##0 = ziplo8(b, _##b##1);\
		i8x16 _##b##8 = ziphi8(b, _##b##1);\
		i16x8 v0 = maddubs(_##a##0, mul15) + maddubs(shrd128(_##a##0, _##b##0, 4), mul20) + maddubs(shrd128(_##a##0, _##b##0, 8), mul51);\
		i16x8 v1 = maddubs(_##b##0, mul15) + maddubs(shrd128(_##b##0, _##b##8, 4), mul20) + maddubs(shrd128(_##b##0, _##b##8, 8), mul51)
#elif defined(__ARM_NEON)
	static const i16x8 mul205 = {20, -5};
	static const i8x16 shuf_zipmd64 = {4, 5, 6, 7, 8, 9, 10, 11, 20, 21, 22, 23, 24, 25, 26, 27};
	#define zipmd64(a, b) (i64x2)vqtbl2q_s8((int8x16x2_t){a, b}, shuf_zipmd64)
	#ifdef __aarch64__
		#define cvthi8s16(a) (u16x8)vmovl_high_s8(a)
		#define shrrpus16(a, b, i) (u8x16)vqrshrun_high_n_s16(vqrshrun_n_s16(a, i), b, i)
		static always_inline i16x8 sixtapH4(i8x16 l0, i8x16 l1) {
			int8x16x2_t x = {l0, l1};
			i8x16 af = vqtbl2q_s8(x, (i8x16){0, 5, 1, 6, 2, 7, 3, 8, 16, 21, 17, 22, 18, 23, 19, 24});
			i8x16 be = vqtbl2q_s8(x, (i8x16){1, 4, 2, 5, 3, 6, 4, 7, 17, 20, 18, 21, 19, 22, 20, 23});
			i8x16 cd = vqtbl2q_s8(x, (i8x16){2, 3, 3, 4, 4, 5, 5, 6, 18, 19, 19, 20, 20, 21, 21, 22});
			return vmlaq_laneq_s16(vmlaq_laneq_s16(vpaddlq_u8(af), vpaddlq_u8(cd), mul205, 0), vpaddlq_u8(be), mul205, 1);
		}
	#else
		#define cvthi8s16(a) (u16x8)vmovl_s8(vget_high_s8(a))
		#define shrrpus16(a, b, i) (u8x16)vcombine_u8(vqrshrun_n_s16(a, i), vqrshrun_n_s16(b, i))
		#define vmulq_laneq_s16(a, v, i) vmulq_lane_s16(a, __builtin_choose_expr((i) < 4, vget_low_s16(v), vget_high_s16(v)), (i) & 3)
		#define vmlaq_laneq_s16(a, b, v, i) vmlaq_lane_s16(a, b, __builtin_choose_expr((i) < 4, vget_low_s16(v), vget_high_s16(v)), (i) & 3)
		static always_inline i16x8 sixtapH4(i8x16 l0, i8x16 l1) {
			i8x8 saf = {0, 5, 1, 6, 2, 7, 3, 8};
			i8x8 sbe = {1, 4, 2, 5, 3, 6, 4, 7};
			i8x8 scd = {2, 3, 3, 4, 4, 5, 5, 6};
			int8x8x2_t x0 = {vget_low_s8(l0), vget_high_s8(l0)};
			int8x8x2_t x1 = {vget_low_s8(l1), vget_high_s8(l1)};
			i8x16 af = vcombine_s8(vtbl2_s8(x0, saf), vtbl2_s8(x1, saf));
			i8x16 be = vcombine_s8(vtbl2_s8(x0, sbe), vtbl2_s8(x1, sbe));
			i8x16 cd = vcombine_s8(vtbl2_s8(x0, scd), vtbl2_s8(x1, scd));
			return vmlaq_laneq_s16(vmlaq_laneq_s16(vpaddlq_u8(af), vpaddlq_u8(cd), mul205, 0), vpaddlq_u8(be), mul205, 1);
		}
	#endif
	static always_inline u8x16 maddshrL(u8x16 q, u8x16 p, i16x8 w, i16x8 o, i16x8 wd) {
		i16x8 a = vmulq_laneq_s16(cvtlo8u16(q), w, 0);
		i16x8 b = vmulq_laneq_s16(cvthi8u16(q), w, 0);
		// accumulating before offset cannot overflow (see formula 8-298 in spec)
		i16x8 c = vqaddq_s16(vmlaq_laneq_s16(a, cvtlo8u16(p), w, 1), o);
		i16x8 d = vqaddq_s16(vmlaq_laneq_s16(b, cvthi8u16(p), w, 1), o);
		return packus16(vshlq_s16(c, wd), vshlq_s16(d, wd));
	}
	static always_inline i8x16 maddshrC16(u8x16 q, u8x16 p, i16x8 w, i16x8 _, i16x8 oCb, i16x8 oCr, i16x8 wd) {
		i16x8 a = vmulq_laneq_s16(cvtlo8u16(q), w, 0);
		i16x8 b = vmulq_laneq_s16(cvthi8u16(q), w, 2);
		i16x8 c = vqaddq_s16(vmlaq_laneq_s16(a, cvtlo8u16(p), w, 1), oCb);
		i16x8 d = vqaddq_s16(vmlaq_laneq_s16(b, cvthi8u16(p), w, 3), oCr);
		return packus16(vshlq_s16(c, wd), vshlq_s16(d, wd));
	}
	static always_inline i8x16 maddshrC8(u8x16 q, u8x16 p, i16x8 wq, i16x8 wp, i16x8 o, i16x8 wd) {
		i16x8 a = vmulq_s16(cvtlo8u16(q), wq);
		i16x8 b = vmulq_s16(cvthi8u16(q), wq);
		i16x8 c = vshlq_s16(vqaddq_s16(vmlaq_s16(a, cvtlo8u16(p), wp), o), wd);
		i16x8 d = vshlq_s16(vqaddq_s16(vmlaq_s16(b, cvthi8u16(p), wp), o), wd);
		return packus16(c, d);
	}
	static always_inline i8x16 maddshrC4(u8x16 q, u8x16 p, i16x8 wq, i16x8 wp, i16x8 o, i16x8 wd) {
		i16x8 a = vmulq_s16(cvtlo8u16(q), wq);
		i16x8 b = vshlq_s16(vqaddq_s16(vmlaq_s16(a, cvtlo8u16(p), wp), o), wd);
		return vcombine_u8(vqmovun_s16(b), (i8x8){});
	}
	static always_inline i16x8 maddABCD(i8x16 ab, i8x16 cd, i8x16 shuf, i8x16 AB, i8x16 CD) {
		i8x16 x0 = shuffle(ab, shuf);
		i8x16 x1 = shuffle(cd, shuf);
		i16x8 x2 = vmlal_high_u8(vmull_u8(vget_low_u8(x0), vget_low_u8(AB)), x0, AB);
		i16x8 x3 = vmlal_high_u8(vmull_u8(vget_low_u8(x1), vget_low_u8(CD)), x1, CD);
		return x2 + x3;
	}
	static always_inline i16x8 sixtapVlo(i8x16 a, i8x16 b, i8x16 c, i8x16 d, i8x16 e, i8x16 f) {
		i16x8 af = vaddl_u8(vget_low_u8(a), vget_low_u8(f));
		i16x8 be = vaddl_u8(vget_low_u8(b), vget_low_u8(e));
		i16x8 cd = vaddl_u8(vget_low_u8(c), vget_low_u8(d));
		return vmlaq_laneq_s16(vmlaq_laneq_s16(af, cd, mul205, 0), be, mul205, 1);
	}
	static always_inline i16x8 sixtapVhi(i8x16 a, i8x16 b, i8x16 c, i8x16 d, i8x16 e, i8x16 f) {
		i16x8 af = vaddl_high_u8(a, f);
		i16x8 be = vaddl_high_u8(b, e);
		i16x8 cd = vaddl_high_u8(c, d);
		return vmlaq_laneq_s16(vmlaq_laneq_s16(af, cd, mul205, 0), be, mul205, 1);
	}
	static always_inline i16x8 sixtapH8(i8x16 a) {
		return sixtapVlo(a, shr128(a, 1), shr128(a, 2), shr128(a, 3), shr128(a, 4), shr128(a, 5));
	}
	#define SIXTAPH16(v0, v1, a, b)\
		i16x8 v0 = sixtapVlo(a, shr128(a, 1), shr128(a, 2), shr128(a, 3), shr128(a, 4), shr128(a, 5));\
		i16x8 v1 = sixtapVlo(b, shr128(b, 1), shr128(b, 2), shr128(b, 3), shr128(b, 4), shr128(b, 5))
#endif

static always_inline i16x8 sixtapHV(i16x8 a, i16x8 b, i16x8 c, i16x8 d, i16x8 e, i16x8 f) {
	i16x8 af = a + f;
	i16x8 be = b + e;
	i16x8 cd = c + d;
	return ((((af - be) >> 2) + (cd - be)) >> 2) + cd;
}

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	#define pack_w(w0, w1) ((w1) << 8 | (w0) & 255)
#else
	#define pack_w(w0, w1) ((w0) << 8 | (w1) & 255)
#endif

// variables only used in this file so far, no need to care about undef until seeing errors
#if defined(__SSE2__)
	#define sstride2 (sstride * 2)
	#define sstride4 (sstride * 4)
	#define nstride2 (nstride * 2)
	#define dstride2 (dstride * 2)
	#define dstride4 (dstride * 4)
	#define premul_strides(sstride, nstride, dstride)
#elif defined(__ARM_NEON)
	#define premul_strides(sstride, nstride, dstride) ssize_t sstride2 = sstride * 2, sstride4 = sstride * 4, nstride2 = nstride * 2, dstride2 = dstride * 2, dstride4 = dstride * 4
#endif

enum {
	INTER_4xH_QPEL_00,
	INTER_4xH_QPEL_10,
	INTER_4xH_QPEL_20,
	INTER_4xH_QPEL_30,
	INTER_4xH_QPEL_01,
	INTER_4xH_QPEL_11,
	INTER_4xH_QPEL_21,
	INTER_4xH_QPEL_31,
	INTER_4xH_QPEL_02,
	INTER_4xH_QPEL_12,
	INTER_4xH_QPEL_22,
	INTER_4xH_QPEL_32,
	INTER_4xH_QPEL_03,
	INTER_4xH_QPEL_13,
	INTER_4xH_QPEL_23,
	INTER_4xH_QPEL_33,
	
	INTER_8xH_QPEL_00,
	INTER_8xH_QPEL_10,
	INTER_8xH_QPEL_20,
	INTER_8xH_QPEL_30,
	INTER_8xH_QPEL_01,
	INTER_8xH_QPEL_11,
	INTER_8xH_QPEL_21,
	INTER_8xH_QPEL_31,
	INTER_8xH_QPEL_02,
	INTER_8xH_QPEL_12,
	INTER_8xH_QPEL_22,
	INTER_8xH_QPEL_32,
	INTER_8xH_QPEL_03,
	INTER_8xH_QPEL_13,
	INTER_8xH_QPEL_23,
	INTER_8xH_QPEL_33,
	
	INTER_16xH_QPEL_00,
	INTER_16xH_QPEL_10,
	INTER_16xH_QPEL_20,
	INTER_16xH_QPEL_30,
	INTER_16xH_QPEL_01,
	INTER_16xH_QPEL_11,
	INTER_16xH_QPEL_21,
	INTER_16xH_QPEL_31,
	INTER_16xH_QPEL_02,
	INTER_16xH_QPEL_12,
	INTER_16xH_QPEL_22,
	INTER_16xH_QPEL_32,
	INTER_16xH_QPEL_03,
	INTER_16xH_QPEL_13,
	INTER_16xH_QPEL_23,
	INTER_16xH_QPEL_33,
};



/**
 * Inter 4x{4/8} prediction takes a 9x{9/13} matrix of 8/16bit luma samples as
 * input, and outputs a 4x{4/8} matrix in memory.
 * Loads are generally done by 8x1 matrices denoted as lRC in the code (R=row,
 * C=left column), or 4x2 matrices denoted as mRC. We may read 7 bytes past the
 * end of the buffer, which is fine since macroblock data follows pixel planes
 * in memory. Also functions follow ffmpeg's naming convention with qpelXY
 * (instead of qpelRC).
 *
 * Inter 8x{4/8/16} prediction takes a 13x{9/13/21} matrix and outputs a
 * 8x{4/8/16} matrix in memory.
 * This is actually simpler than 4xH since we always work on 8x1 lines, so we
 * need less shuffling tricks. The entire input matrix being too big to fit in
 * registers, we compute values from top to bottom and keep intermediate
 * results between iterations. We compute 2 lines at the same time to fill up
 * dual issue pipelines, but it could be reduced to 1 to reduce binary size.
 * 
 * Inter 16x{8/16} takes a 21x{13/21} matrix and outputs a 16x{8/16} matrix in
 * memory.
 * There the biggest challenge is register pressure, so we count on compilers
 * to spill/reload on stack. All functions were designed with 16 available
 * registers in mind, for older chips there will just be more spills.
 * 
 * The following approaches were tried for implementing the filters:
 * _ pmadd four rows with [1,-5,20,20,-5,1,0,0,0], [0,1,-5,20,20,-5,1,0,0],
 *   [0,0,1,-5,20,20,-5,1,0] and [0,0,0,1,-5,20,20,-5,1], then phadd and shift
 *   them to get a horizontally-filtered 4x4 matrix. While this is short and
 *   easy to read, both pmadd and phadd are slow even on later architectures.
 * _ doing the 2D filter by accumulating 4x4 matrices by coefficient (out of
 *   1,-5,20,25,-100,400), then summing and shifting them down to a 4x4 result.
 *   Although fun to code, this was not very clever and needed a lot of live
 *   registers at any time (thus many stack spills).
 * _ computing half-sample interpolations first and averaging them in qpel
 *   code. While it used very little code, it incurred a lot of reads/writes
 *   of temporary data and wasted many redundant operations.
 * _ making 4x4 filters jump to residual with their values in registers, to
 *   spare some packing/unpacking and writes/reads. However it was incompatible
 *   with variable-height filters, and the latter was deemed more advantageous
 *   for architectural simplicity.
 * _ reading each 9-byte row with two pmovzxbw, then obtaining every mRC with
 *   shufps (still used in QPEL_12_32). However it often resulted in two reads
 *   per row, which may be prohibitive if outside of cache.
 * _ splitting horizontal filters in 3 to maddubs then add (instead of hadd),
 *   when the equivalent code using shifts used at least 4 more shuffles
 *   (roughly the latency penalty from using maddubs).
 * _ merging the 16 qpel positions into 6 cases and refining the results inside
 *   each case with precomputed select masks, to reduce the binary size from
 *   24k to 10k.
 *
 * While it is impossible for functions to return multiple values in multiple
 * registers (stupid ABI), we cannot put redundant loads in functions and have
 * to duplicate a lot of code. The same goes for sixtap functions, which would
 * force all live vector registers on stack if not inlined.
 */
static void decode_inter_luma(int mode, int h, size_t sstride, const uint8_t * restrict src2, size_t dstride, uint8_t * restrict dst, i16x8 wod) {
	#if defined(__SSE2__)
		i8x16 w = broadcast16(wod, 0);
		u64x2 wd = (u64x2)wod << 16 >> 48;
	#elif defined(__ARM_NEON)
		i8x16 w = cvtlo8s16(wod);
		i16x8 wd = -broadcast16(wod, 2);
	#endif
	i16x8 o = broadcast16(wod, 1);
	i8x16 m0 = set8(-(0xd888 >> (mode & 15) & 1));
	i8x16 m1 = set8(-(0xa504 >> (mode & 15) & 1));
	i8x16 shufx = (i8x16){2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17} - m0;
	const uint8_t * restrict src0 = src2 - 2;
	ssize_t nstride = -sstride, dstride3 = dstride * 3;
	premul_strides(sstride, nstride, dstride);
	#define sstride3 (sstride * 3)
	
	switch (mode) {
	default: __builtin_unreachable();
	
	case INTER_4xH_QPEL_00:
		do {
			i32x4 p = loadu32x4(src2, src2 + sstride, src2 + sstride2, src2 + sstride3);
			i32x4 q = loada32x4(dst, dst + dstride, dst + dstride2, dst + dstride3);
			i32x4 r = maddshrL(q, p, w, o, wd);
			*(int32_t *)(dst           ) = r[0];
			*(int32_t *)(dst + dstride ) = r[1];
			*(int32_t *)(dst + dstride2) = r[2];
			*(int32_t *)(dst + dstride3) = r[3];
			dst += dstride4;
			src2 += sstride4;
		} while (h -= 4);
		return;
	
	case INTER_4xH_QPEL_10:
	case INTER_4xH_QPEL_20:
	case INTER_4xH_QPEL_30:
		do {
			i8x16 l2 = loadu128(src0           );
			i8x16 l3 = loadu128(src0 + sstride );
			i8x16 l4 = loadu128(src0 + sstride2);
			i8x16 l5 = loadu128(src0 + sstride3); /* overreads 7 bytes */
			i16x8 h01 = shrrpus16(sixtapH4(l2, l3), sixtapH4(l4, l5), 5);
			i8x16 s0 = shuffle(ziplo64(l2, l3), shufx);
			i8x16 s1 = shuffle(ziplo64(l4, l5), shufx);
			i8x16 s = unziplo32(s0, s1);
			i32x4 q = loada32x4(dst, dst + dstride, dst + dstride2, dst + dstride3);
			i32x4 r = maddshrL(q, avgu8(ifelse_mask(m1, h01, s), h01), w, o, wd);
			*(int32_t *)(dst           ) = r[0];
			*(int32_t *)(dst + dstride ) = r[1];
			*(int32_t *)(dst + dstride2) = r[2];
			*(int32_t *)(dst + dstride3) = r[3];
			dst += dstride4;
			src0 += sstride4;
		} while (h -= 4);
		return;
	
	case INTER_4xH_QPEL_01:
	case INTER_4xH_QPEL_02:
	case INTER_4xH_QPEL_03: {
		i8x16 m02 = loadu32x4(src2 + nstride2, src2 + nstride , src2           , src2 + sstride );
		i8x16 m12 = shrd128(m02, loadu32(src2 + sstride2), 4);
		do {
			src2 += sstride4;
			i8x16 m52 = loadu32x4(src2 + nstride , src2           , src2 + sstride , src2 + sstride2);
			i8x16 m22 = shrd128(m12, m52, 4);
			i8x16 m32 = shrd128(m12, m52, 8);
			i8x16 m42 = shrd128(m12, m52, 12);
			i16x8 v0 = sixtapVlo(m02, m12, m22, m32, m42, m52);
			i16x8 v1 = sixtapVhi(m02, m12, m22, m32, m42, m52);
			i8x16 v01 = shrrpus16(v0, v1, 5);
			i8x16 s = ifelse_mask(m1, v01, ifelse_mask(m0, m32, m22));
			i32x4 q = loada32x4(dst, dst + dstride, dst + dstride2, dst + dstride3);
			i32x4 r = maddshrL(q, avgu8(s, v01), w, o, wd);
			*(int32_t *)(dst           ) = r[0];
			*(int32_t *)(dst + dstride ) = r[1];
			*(int32_t *)(dst + dstride2) = r[2];
			*(int32_t *)(dst + dstride3) = r[3];
			m02 = m42, m12 = m52;
			dst += dstride4;
		} while (h -= 4);
		} return;
	
	case INTER_4xH_QPEL_11:
	case INTER_4xH_QPEL_31:
	case INTER_4xH_QPEL_13:
	case INTER_4xH_QPEL_33: {
		i8x16 l0 = loadu128(src0 + nstride2);
		i8x16 l1 = loadu128(src0 + nstride );
		i8x16 l2 = loadu128(src0           );
		i8x16 l3 = loadu128(src0 + sstride );
		i8x16 l4 = loadu128(src0 + sstride2);
		do {
			src0 += sstride4;
			i8x16 l5 = loadu128(src0 + nstride );
			i8x16 l6 = loadu128(src0           );
			i8x16 l7 = loadu128(src0 + sstride );
			i8x16 l8 = loadu128(src0 + sstride2);
			i8x16 l02 = shuffle(l0, shufx);
			i8x16 l12 = shuffle(l1, shufx);
			i8x16 l22 = shuffle(l2, shufx);
			i8x16 l32 = shuffle(l3, shufx);
			i8x16 l42 = shuffle(l4, shufx);
			i8x16 l52 = shuffle(l5, shufx);
			i8x16 l62 = shuffle(l6, shufx);
			i8x16 l72 = shuffle(l7, shufx);
			i8x16 l82 = shuffle(l8, shufx);
			i8x16 m02 = ziplo32(l02, l12);
			i8x16 m12 = ziplo32(l12, l22);
			i8x16 m22 = ziplo32(l22, l32);
			i8x16 m32 = ziplo32(l32, l42);
			i8x16 m42 = ziplo32(l42, l52);
			i8x16 m52 = ziplo32(l52, l62);
			i8x16 m62 = ziplo32(l62, l72);
			i8x16 m72 = ziplo32(l72, l82);
			i16x8 v0 = sixtapVlo(m02, m12, m22, m32, m42, m52);
			i16x8 v1 = sixtapVlo(m22, m32, m42, m52, m62, m72);
			i8x16 v01 = shrrpus16(v0, v1, 5);
			i16x8 h0 = sixtapH4(ifelse_mask(m1, l3, l2), ifelse_mask(m1, l4, l3));
			i16x8 h1 = sixtapH4(ifelse_mask(m1, l5, l4), ifelse_mask(m1, l6, l5));
			i8x16 s = avgu8(v01, shrrpus16(h0, h1, 5));
			i32x4 q = loada32x4(dst, dst + dstride, dst + dstride2, dst + dstride3);
			i32x4 r = maddshrL(q, s, w, o, wd);
			*(int32_t *)(dst           ) = r[0];
			*(int32_t *)(dst + dstride ) = r[1];
			*(int32_t *)(dst + dstride2) = r[2];
			*(int32_t *)(dst + dstride3) = r[3];
			l0 = l4, l1 = l5, l2 = l6, l3 = l7, l4 = l8;
			dst += dstride4;
		} while (h -= 4);
		} return;
	
	case INTER_4xH_QPEL_12:
	case INTER_4xH_QPEL_32: {
		i8x16 l0 = loadu128(src0 + nstride2);
		i8x16 l1 = loadu128(src0 + nstride );
		i8x16 l2 = loadu128(src0           );
		i8x16 l3 = loadu128(src0 + sstride );
		i8x16 l4 = loadu128(src0 + sstride2);
		do {
			src0 += sstride4;
			i8x16 l5 = loadu128(src0 + nstride );
			i8x16 l6 = loadu128(src0           );
			i8x16 l7 = loadu128(src0 + sstride );
			i8x16 l8 = loadu128(src0 + sstride2);
			i8x16 r0 = ziplo16(ziphi8(l0, l1), ziphi8(l2, l3));
			i8x16 r1 = ziplo16(ziphi8(l4, l5), ziphi8(l6, l7));
			i64x2 r2 = {((i64x2)ziplo32(r0, r1))[0], ((i64x2)l8)[1]};
			i16x8 v08 = sixtapH8(r2);
			i16x8 v00 = sixtapVlo(l0, l1, l2, l3, l4, l5);
			i16x8 v10 = sixtapVlo(l1, l2, l3, l4, l5, l6);
			i16x8 v20 = sixtapVlo(l2, l3, l4, l5, l6, l7);
			i16x8 v30 = sixtapVlo(l3, l4, l5, l6, l7, l8);
			i16x8 v01 = shrd128(v00, v08, 2);
			i16x8 v11 = shrd128(v10, shr128(v08, 2), 2);
			i16x8 v21 = shrd128(v20, shr128(v08, 4), 2);
			i16x8 v31 = shrd128(v30, shr128(v08, 6), 2);
			i16x8 m00 = ziplo64(v00, v10);
			i16x8 m01 = ziplo64(v01, v11);
			i16x8 m02 = zipmd64(v00, v10);
			i16x8 m03 = zipmd64(v01, v11);
			i16x8 m04 = ziphi64(v00, v10);
			i16x8 m05 = ziphi64(v01, v11);
			i16x8 m20 = ziplo64(v20, v30);
			i16x8 m21 = ziplo64(v21, v31);
			i16x8 m22 = zipmd64(v20, v30);
			i16x8 m23 = zipmd64(v21, v31);
			i16x8 m24 = ziphi64(v20, v30);
			i16x8 m25 = ziphi64(v21, v31);
			i16x8 vh0 = sixtapHV(m00, m01, m02, m03, m04, m05);
			i16x8 vh1 = sixtapHV(m20, m21, m22, m23, m24, m25);
			i8x16 vh = shrrpus16(vh0, vh1, 6);
			i8x16 s = shrrpus16(ifelse_mask(m0, m03, m02), ifelse_mask(m0, m23, m22), 5);
			i32x4 q = loada32x4(dst, dst + dstride, dst + dstride2, dst + dstride3);
			i32x4 r = maddshrL(q, avgu8(s, vh), w, o, wd);
			*(int32_t *)(dst           ) = r[0];
			*(int32_t *)(dst + dstride ) = r[1];
			*(int32_t *)(dst + dstride2) = r[2];
			*(int32_t *)(dst + dstride3) = r[3];
			l0 = l4, l1 = l5, l2 = l6, l3 = l7, l4 = l8;
			dst += dstride4;
		} while (h -= 4);
		} return;
	
	case INTER_4xH_QPEL_21:
	case INTER_4xH_QPEL_22:
	case INTER_4xH_QPEL_23: {
		i8x16 l0 = loadu128(src0 + nstride2);
		i8x16 l1 = loadu128(src0 + nstride );
		i8x16 l2 = loadu128(src0           );
		i8x16 l3 = loadu128(src0 + sstride );
		i8x16 l4 = loadu128(src0 + sstride2);
		i16x8 h0 = sixtapH4(l0, l1);
		i16x8 h2 = sixtapH4(l2, l3);
		i16x8 h3 = sixtapH4(l3, l4);
		do {
			src0 += sstride4;
			i8x16 l5 = loadu128(src0 + nstride );
			i8x16 l6 = loadu128(src0           );
			i8x16 l7 = loadu128(src0 + sstride );
			i8x16 l8 = loadu128(src0 + sstride2);
			i16x8 h5 = sixtapH4(l5, l6);
			i16x8 h7 = sixtapH4(l7, l8);
			i16x8 h1 = shrd128(h0, h2, 8);
			i16x8 h4 = shrd128(h3, h5, 8);
			i16x8 h6 = shrd128(h5, h7, 8);
			i16x8 hv0 = sixtapHV(h0, h1, h2, h3, h4, h5);
			i16x8 hv1 = sixtapHV(h2, h3, h4, h5, h6, h7);
			i8x16 hv = shrrpus16(hv0, hv1, 6);
			i8x16 s = shrrpus16(ifelse_mask(m0, h3, h2), ifelse_mask(m0, h5, h4), 5);
			i32x4 q = loada32x4(dst, dst + dstride, dst + dstride2, dst + dstride3);
			i32x4 r = maddshrL(q, avgu8(ifelse_mask(m1, hv, s), hv), w, o, wd);
			*(int32_t *)(dst           ) = r[0];
			*(int32_t *)(dst + dstride ) = r[1];
			*(int32_t *)(dst + dstride2) = r[2];
			*(int32_t *)(dst + dstride3) = r[3];
			h0 = h4, h2 = h6, h3 = h7;
			dst += dstride4;
		} while (h -= 4);
		} return;
	
	case INTER_8xH_QPEL_00:
		do {
			i8x16 p0 = loadu64x2(src2           ,  src2 + sstride );
			i8x16 p1 = loadu64x2(src2 + sstride2,  src2 + sstride3);
			i8x16 q0 = loadu64x2(dst           ,  dst + dstride );
			i8x16 q1 = loadu64x2(dst + dstride2,  dst + dstride3);
			i64x2 r0 = maddshrL(q0, p0, w, o, wd);
			i64x2 r1 = maddshrL(q1, p1, w, o, wd);
			*(int64_t *)(dst           ) = r0[0];
			*(int64_t *)(dst + dstride ) = r0[1];
			*(int64_t *)(dst + dstride2) = r1[0];
			*(int64_t *)(dst + dstride3) = r1[1];
			src2 += sstride4;
			dst += dstride4;
		} while (h -= 4);
		return;
	
	case INTER_8xH_QPEL_10:
	case INTER_8xH_QPEL_20:
	case INTER_8xH_QPEL_30:
		do {
			i8x16 l0 = loadu128(src0           ); /* overreads 3 bytes */
			i8x16 l1 = loadu128(src0 + sstride );
			i8x16 h01 = shrrpus16(sixtapH8(l0), sixtapH8(l1), 5);
			i8x16 s = ziplo64(shuffle(l0, shufx), shuffle(l1, shufx));
			i8x16 q = loada64x2(dst           , dst + dstride );
			i64x2 r = maddshrL(q, avgu8(ifelse_mask(m1, h01, s), h01), w, o, wd);
			*(int64_t *)(dst           ) = r[0];
			*(int64_t *)(dst + dstride ) = r[1];
			src0 += sstride2;
			dst += dstride2;
		} while (h -= 2);
		return;
	
	case INTER_8xH_QPEL_01:
	case INTER_8xH_QPEL_02:
	case INTER_8xH_QPEL_03: {
		i16x8 l0 = loadu64(src2 + nstride2);
		i16x8 l1 = loadu64(src2 + nstride );
		i16x8 l2 = loadu64(src2           );
		i16x8 l3 = loadu64(src2 + sstride );
		i16x8 l4 = loadu64(src2 + sstride2);
		do {
			src2 += sstride2;
			i16x8 l5 = loadu64(src2 + sstride );
			i16x8 l6 = loadu64(src2 + sstride2);
			i16x8 v0 = sixtapVlo(l0, l1, l2, l3, l4, l5);
			i16x8 v1 = sixtapVlo(l1, l2, l3, l4, l5, l6);
			i8x16 v01 = shrrpus16(v0, v1, 5);
			i8x16 s = ifelse_mask(m0, ziplo64(l3, l4), ziplo64(l2, l3));
			i8x16 q = loada64x2(dst           , dst + dstride );
			i64x2 r = maddshrL(q, avgu8(ifelse_mask(m1, v01, s), v01), w, o, wd);
			*(int64_t *)(dst           ) = r[0];
			*(int64_t *)(dst + dstride ) = r[1];
			l0 = l2, l1 = l3, l2 = l4, l3 = l5, l4 = l6;
			dst += dstride2;
		} while (h -= 2);
		} return;
	
	case INTER_8xH_QPEL_11:
	case INTER_8xH_QPEL_31:
	case INTER_8xH_QPEL_13:
	case INTER_8xH_QPEL_33: {
		i16x8 l02 = shuffle(loadu128(src0 + nstride2), shufx);
		i16x8 l12 = shuffle(loadu128(src0 + nstride ), shufx);
		i8x16 l2 = loadu128(src0           );
		i8x16 l3 = loadu128(src0 + sstride );
		i8x16 l4 = loadu128(src0 + sstride2);
		do {
			src0 += sstride2;
			i8x16 l5 = loadu128(src0 + sstride );
			i8x16 l6 = loadu128(src0 + sstride2);
			i16x8 l22 = shuffle(l2, shufx);
			i16x8 l32 = shuffle(l3, shufx);
			i16x8 l42 = shuffle(l4, shufx);
			i16x8 l52 = shuffle(l5, shufx);
			i16x8 l62 = shuffle(l6, shufx);
			i16x8 v0 = sixtapVlo(l02, l12, l22, l32, l42, l52);
			i16x8 v1 = sixtapVlo(l12, l22, l32, l42, l52, l62);
			i8x16 v01 = shrrpus16(v0, v1, 5);
			i16x8 h0 = sixtapH8(ifelse_mask(m1, l3, l2));
			i16x8 h1 = sixtapH8(ifelse_mask(m1, l4, l3));
			i8x16 s = avgu8(v01, shrrpus16(h0, h1, 5));
			i8x16 q = loada64x2(dst           , dst + dstride );
			i64x2 r = maddshrL(q, s, w, o, wd);
			*(int64_t *)(dst           ) = r[0];
			*(int64_t *)(dst + dstride ) = r[1];
			l02 = l22, l12 = l32, l2 = l4, l3 = l5;
			l4 = l6;
			dst += dstride2;
		} while (h -= 2);
		} return;
	
	case INTER_8xH_QPEL_12:
	case INTER_8xH_QPEL_32: {
		i8x16 l0 = loadu128(src0 + nstride2);
		i8x16 l1 = loadu128(src0 + nstride );
		i8x16 l2 = loadu128(src0           );
		i8x16 l3 = loadu128(src0 + sstride );
		i8x16 l4 = loadu128(src0 + sstride2);
		do {
			src0 += sstride2;
			i8x16 l5 = loadu128(src0 + sstride );
			i8x16 l6 = loadu128(src0 + sstride2);
			i16x8 x00 = sixtapVlo(l0, l1, l2, l3, l4, l5);
			i16x8 x10 = sixtapVlo(l1, l2, l3, l4, l5, l6);
			i16x8 x08 = sixtapVhi(l0, l1, l2, l3, l4, l5);
			i16x8 x18 = sixtapVhi(l1, l2, l3, l4, l5, l6);
			i16x8 x01 = shrd128(x00, x08, 2);
			i16x8 x11 = shrd128(x10, x18, 2);
			i16x8 x02 = shrd128(x00, x08, 4);
			i16x8 x12 = shrd128(x10, x18, 4);
			i16x8 x03 = shrd128(x00, x08, 6);
			i16x8 x13 = shrd128(x10, x18, 6);
			i16x8 x04 = shrd128(x00, x08, 8);
			i16x8 x14 = shrd128(x10, x18, 8);
			i16x8 x05 = shrd128(x00, x08, 10);
			i16x8 x15 = shrd128(x10, x18, 10);
			i16x8 vh0 = sixtapHV(x00, x01, x02, x03, x04, x05);
			i16x8 vh1 = sixtapHV(x10, x11, x12, x13, x14, x15);
			i8x16 vh = shrrpus16(vh0, vh1, 6);
			i8x16 s = shrrpus16(ifelse_mask(m0, x03, x02), ifelse_mask(m0, x13, x12), 5);
			i8x16 q = loada64x2(dst           , dst + dstride );
			i64x2 r = maddshrL(q, avgu8(vh, s), w, o, wd);
			*(int64_t *)(dst           ) = r[0];
			*(int64_t *)(dst + dstride ) = r[1];
			l0 = l2, l1 = l3, l2 = l4, l3 = l5, l4 = l6;
			dst += dstride2;
		} while (h -= 2);
		} return;
	
	case INTER_8xH_QPEL_21:
	case INTER_8xH_QPEL_22:
	case INTER_8xH_QPEL_23: {
		i16x8 v0 = sixtapH8(loadu128(src0 + nstride2));
		i16x8 v1 = sixtapH8(loadu128(src0 + nstride ));
		i16x8 v2 = sixtapH8(loadu128(src0           ));
		i16x8 v3 = sixtapH8(loadu128(src0 + sstride ));
		i16x8 v4 = sixtapH8(loadu128(src0 + sstride2));
		do {
			src0 += sstride2;
			i8x16 v5 = sixtapH8(loadu128(src0 + sstride ));
			i8x16 v6 = sixtapH8(loadu128(src0 + sstride2));
			i16x8 hv0 = sixtapHV(v0, v1, v2, v3, v4, v5);
			i16x8 hv1 = sixtapHV(v1, v2, v3, v4, v5, v6);
			i8x16 hv = shrrpus16(hv0, hv1, 6);
			i8x16 s = shrrpus16(ifelse_mask(m0, v3, v2), ifelse_mask(m0, v4, v3), 5);
			i8x16 q = loada64x2(dst           , dst + dstride );
			i64x2 r = maddshrL(q, avgu8(ifelse_mask(m1, hv, s), hv), w, o, wd);
			*(int64_t *)(dst           ) = r[0];
			*(int64_t *)(dst + dstride ) = r[1];
			v0 = v2, v1 = v3, v2 = v4, v3 = v5, v4 = v6;
			dst += dstride2;
		} while (h -= 2);
		} return;
	
	case INTER_16xH_QPEL_00:
		do {
			*(i8x16 *)(dst           ) = maddshrL(*(i8x16 *)(dst           ), loadu128(src2           ), w, o, wd);
			*(i8x16 *)(dst + dstride ) = maddshrL(*(i8x16 *)(dst + dstride ), loadu128(src2 + sstride ), w, o, wd);
			*(i8x16 *)(dst + dstride2) = maddshrL(*(i8x16 *)(dst + dstride2), loadu128(src2 + sstride2), w, o, wd);
			*(i8x16 *)(dst + dstride3) = maddshrL(*(i8x16 *)(dst + dstride3), loadu128(src2 + sstride3), w, o, wd);
			src2 += sstride4;
			dst += dstride4;
		} while (h -= 4);
		return;
	
	case INTER_16xH_QPEL_10:
	case INTER_16xH_QPEL_20:
	case INTER_16xH_QPEL_30:
		do {
			i8x16 l0 = loadu128(src0    );
			i8x16 l8 = loadu128(src0 + 8); /* overreads 3 bytes */
			SIXTAPH16(h0, h8, l0, l8);
			i8x16 h01 = shrrpus16(h0, h8, 5);
			i8x16 s = ifelse_mask(m1, h01, ziplo64(shuffle(l0, shufx), shuffle(l8, shufx)));
			*(i8x16 *)dst = maddshrL(*(i8x16 *)dst, avgu8(s, h01), w, o, wd);
			src0 += sstride;
			dst += dstride;
		} while (h -= 1);
		return;
	
	case INTER_16xH_QPEL_01:
	case INTER_16xH_QPEL_02:
	case INTER_16xH_QPEL_03: {
		i8x16 l0 = loadu128(src2 + nstride2);
		i8x16 l1 = loadu128(src2 + nstride );
		i8x16 l2 = loadu128(src2           );
		i8x16 l3 = loadu128(src2 + sstride );
		i8x16 l4 = loadu128(src2 + sstride2);
		do {
			src2 += sstride;
			i8x16 l5 = loadu128(src2 + sstride2);
			i16x8 v0 = sixtapVlo(l0, l1, l2, l3, l4, l5);
			i16x8 v8 = sixtapVhi(l0, l1, l2, l3, l4, l5);
			i8x16 v01 = shrrpus16(v0, v8, 5);
			i8x16 s = ifelse_mask(m1, v01, ifelse_mask(m0, l3, l2));
			*(i8x16 *)dst = maddshrL(*(i8x16 *)dst, avgu8(s, v01), w, o, wd);
			l0 = l1, l1 = l2, l2 = l3, l3 = l4, l4 = l5;
			dst += dstride;
		} while (h -= 1);
		} return;
	
	case INTER_16xH_QPEL_11:
	case INTER_16xH_QPEL_31:
	case INTER_16xH_QPEL_13:
	case INTER_16xH_QPEL_33: {
		const uint8_t * restrict src8 = src0 + 8;
		i8x16 l02 = shuffle(loadu128(src0 + nstride2), shufx);
		i8x16 l0A = shuffle(loadu128(src8 + nstride2), shufx);
		i8x16 l12 = shuffle(loadu128(src0 + nstride ), shufx);
		i8x16 l1A = shuffle(loadu128(src8 + nstride ), shufx);
		i8x16 l20 = loadu128(src0           );
		i8x16 l28 = loadu128(src8           );
		i8x16 l30 = loadu128(src0 + sstride );
		i8x16 l38 = loadu128(src8 + sstride );
		i8x16 l40 = loadu128(src0 + sstride2);
		i8x16 l48 = loadu128(src8 + sstride2);
		do {
			src0 += sstride;
			src8 += sstride;
			i8x16 l50 = loadu128(src0 + sstride2);
			i8x16 l58 = loadu128(src8 + sstride2);
			i8x16 l22 = shuffle(l20, shufx);
			i8x16 l32 = shuffle(l30, shufx);
			i8x16 l42 = shuffle(l40, shufx);
			i8x16 l52 = shuffle(l50, shufx);
			i16x8 v0 = sixtapVlo(l02, l12, l22, l32, l42, l52);
			i8x16 l2A = shuffle(l28, shufx);
			i8x16 l3A = shuffle(l38, shufx);
			i8x16 l4A = shuffle(l48, shufx);
			i8x16 l5A = shuffle(l58, shufx);
			i16x8 v1 = sixtapVlo(l0A, l1A, l2A, l3A, l4A, l5A);
			i8x16 v01 = shrrpus16(v0, v1, 5);
			i8x16 s0 = ifelse_mask(m1, l30, l20);
			i8x16 s1 = ifelse_mask(m1, l38, l28);
			SIXTAPH16(h0, h8, s0, s1);
			i8x16 h01 = shrrpus16(h0, h8, 5);
			*(i8x16 *)dst = maddshrL(*(i8x16 *)dst, avgu8(v01, h01), w, o, wd);
			l02 = l12, l0A = l1A, l12 = l22, l1A = l2A;
			l20 = l30, l28 = l38, l30 = l40, l38 = l48, l40 = l50, l48 = l58;
			dst += dstride;
		} while (h -= 1);
		} return;
	
	case INTER_16xH_QPEL_12:
	case INTER_16xH_QPEL_32: {
		const uint8_t * restrict srcG = src0 + 16;
		i8x16 l00 = loadu128(src0 + nstride2);
		i16x8 l0G = loadu64(srcG + nstride2);
		i8x16 l10 = loadu128(src0 + nstride );
		i16x8 l1G = loadu64(srcG + nstride );
		i8x16 l20 = loadu128(src0           );
		i16x8 l2G = loadu64(srcG           );
		i8x16 l30 = loadu128(src0 + sstride );
		i16x8 l3G = loadu64(srcG + sstride );
		i8x16 l40 = loadu128(src0 + sstride2);
		i16x8 l4G = loadu64(srcG + sstride2);
		do {
			src0 += sstride;
			srcG += sstride;
			i8x16 l50 = loadu128(src0 + sstride2);
			i16x8 l5G = loadu64(srcG + sstride2);
			i16x8 v0 = sixtapVlo(l00, l10, l20, l30, l40, l50);
			i16x8 v8 = sixtapVhi(l00, l10, l20, l30, l40, l50);
			i16x8 vG = sixtapVlo(l0G, l1G, l2G, l3G, l4G, l5G);
			i16x8 v1 = shrd128(v0, v8, 2);
			i16x8 v2 = shrd128(v0, v8, 4);
			i16x8 v3 = shrd128(v0, v8, 6);
			i16x8 v4 = shrd128(v0, v8, 8);
			i16x8 v5 = shrd128(v0, v8, 10);
			i16x8 vh0 = sixtapHV(v0, v1, v2, v3, v4, v5);
			i16x8 v9 = shrd128(v8, vG, 2);
			i16x8 vA = shrd128(v8, vG, 4);
			i16x8 vB = shrd128(v8, vG, 6);
			i16x8 vC = shrd128(v8, vG, 8);
			i16x8 vD = shrd128(v8, vG, 10);
			i16x8 vh1 = sixtapHV(v8, v9, vA, vB, vC, vD);
			i8x16 vh = shrrpus16(vh0, vh1, 6);
			i8x16 s = shrrpus16(ifelse_mask(m0, v3, v2), ifelse_mask(m0, vB, vA), 5);
			*(i8x16 *)dst = maddshrL(*(i8x16 *)dst, avgu8(s, vh), w, o, wd);
			l00 = l10, l10 = l20, l20 = l30, l30 = l40, l40 = l50;
			l0G = l1G, l1G = l2G, l2G = l3G, l3G = l4G, l4G = l5G;
			dst += dstride;
		} while (h -= 1);
		} return;
	
	case INTER_16xH_QPEL_21:
	case INTER_16xH_QPEL_22:
	case INTER_16xH_QPEL_23: {
		const uint8_t * restrict src8 = src0 + 8;
		i8x16 l00 = loadu128(src0 + nstride2);
		i8x16 l08 = loadu128(src8 + nstride2);
		SIXTAPH16(h00, h08, l00, l08);
		i8x16 l10 = loadu128(src0 + nstride );
		i8x16 l18 = loadu128(src8 + nstride );
		SIXTAPH16(h10, h18, l10, l18);
		i8x16 l20 = loadu128(src0           );
		i8x16 l28 = loadu128(src8           );
		SIXTAPH16(h20, h28, l20, l28);
		i8x16 l30 = loadu128(src0 + sstride );
		i8x16 l38 = loadu128(src8 + sstride );
		SIXTAPH16(h30, h38, l30, l38);
		i8x16 l40 = loadu128(src0 + sstride2);
		i8x16 l48 = loadu128(src8 + sstride2);
		SIXTAPH16(h40, h48, l40, l48);
		do {
			src0 += sstride;
			src8 += sstride;
			i8x16 l50 = loadu128(src0 + sstride2);
			i8x16 l58 = loadu128(src8 + sstride2);
			SIXTAPH16(h50, h58, l50, l58);
			i16x8 hv0 = sixtapHV(h00, h10, h20, h30, h40, h50);
			i16x8 hv1 = sixtapHV(h08, h18, h28, h38, h48, h58);
			i8x16 hv = shrrpus16(hv0, hv1, 6);
			i8x16 s = shrrpus16(ifelse_mask(m0, h30, h20), ifelse_mask(m0, h38, h28), 5);
			*(i8x16 *)dst = maddshrL(*(i8x16 *)dst, avgu8(ifelse_mask(m1, hv, s), hv), w, o, wd);
			h00 = h10, h08 = h18;
			h10 = h20, h18 = h28;
			h20 = h30, h28 = h38;
			h30 = h40, h38 = h48;
			h40 = h50, h48 = h58;
			dst += dstride;
		} while (h -= 1);
		} return;
	}
	#undef sstride3
}



/**
 * Combined Cb & Cr inter chroma prediction
 * 
 * dstride and sstride are half the strides of src and dst chroma planes.
 */
static void decode_inter_chroma(int w, int h, size_t sstride, const uint8_t *src, size_t dstride, uint8_t *dst, i8x16 ABCD, i16x8 wod) {
	#if defined(__SSE2__)
		i8x16 AB = broadcast16(ABCD, 0);
		i8x16 CD = broadcast16(ABCD, 1);
		i8x16 wo8 = ziphi16(wod, wod);
		u64x2 wd = (u64x2)wod >> 48;
	#elif defined(__ARM_NEON)
		i8x16 AB = shrd128(broadcast8(ABCD, 0), broadcast8(ABCD, 1), 8);
		i8x16 CD = shrd128(broadcast8(ABCD, 2), broadcast8(ABCD, 3), 8);
		i16x8 wo16 = cvthi8s16(wod);
		i16x8 oCb = broadcast16(wod, 6);
		i16x8 oCr = broadcast16(wod, 7);
		i16x8 wd = -broadcast16(wod, 3);
	#endif
	premul_strides(sstride, 0, dstride);
	size_t sstride3 = sstride * 3, dstride3 = dstride * 3;
	
	if (w == 16) {
		#if defined(__SSE2__)
			i8x16 w0 = broadcast32(wo8, 0);
			i8x16 w1 = broadcast32(wo8, 1);
			i16x8 oCb = broadcast32(wo8, 2);
			i16x8 oCr = broadcast32(wo8, 3);
			i8x16 shuf = {0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8};
		#elif defined(__ARM_NEON)
			i16x8 w0 = wo16;
			i16x8 w1 = wo16;
			i8x16 shuf = {0, 1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 8};
		#endif
		i8x16 l0 = loadu128(src);
		i8x16 l1 = loadu128(src + sstride);
		do {
			src += sstride2;
			i8x16 l2 = loadu128(src);
			i8x16 l3 = loadu128(src + sstride);
			i16x8 x0 = maddABCD(l0, l2, shuf, AB, CD);
			i16x8 x1 = maddABCD(l1, l3, shuf, AB, CD);
			i8x16 p = shrrpu16(x0, x1, 6);
			i8x16 q = loada64x2(dst, dst + dstride);
			i64x2 v = maddshrC16(q, p, w0, w1, oCb, oCr, wd);
			*(int64_t *)dst = v[0];
			*(int64_t *)(dst + dstride) = v[1];
			dst += dstride2;
			l0 = l2, l1 = l3;
		} while (h -= 2);
		
	} else if (w == 8) {
		#if defined(__SSE2__)
			i8x16 w0 = ziplo32(wo8, wo8);
			i8x16 w1 = w0;
			i16x8 o = ziphi32(wo8, wo8);
			i8x16 shuf = {0, 1, 1, 2, 2, 3, 3, 4, 8, 9, 9, 10, 10, 11, 11, 12};
		#elif defined(__ARM_NEON)
			i16x8 v0 = ziplo16(wo16, wo16);
			i16x8 w0 = trnlo32(v0);
			i16x8 w1 = trnhi32(v0);
			i16x8 o = ziplo64(oCb, oCr);
			i8x16 shuf = {0, 1, 2, 3, 8, 9, 10, 11, 1, 2, 3, 4, 9, 10, 11, 12};
		#endif
		i8x16 l0 = loadu64x2(src, src + sstride);
		src += sstride2;
		do {
			i8x16 l1 = loadu64x2(src, src + sstride);
			i8x16 l2 = loadu64x2(src + sstride2, src + sstride3);
			i16x8 x0 = maddABCD(l0, l1, shuf, AB, CD);
			i16x8 x1 = maddABCD(l1, l2, shuf, AB, CD);
			i8x16 p = shrrpu16(x0, x1, 6);
			i8x16 q = loada32x4(dst, dst + dstride, dst + dstride2, dst + dstride3);
			i32x4 v = maddshrC8(q, p, w0, w1, o, wd);
			*(int32_t *)dst = v[0];
			*(int32_t *)(dst + dstride) = v[1];
			*(int32_t *)(dst + dstride2) = v[2];
			*(int32_t *)(dst + dstride3) = v[3];
			src += sstride4;
			dst += dstride4;
			l0 = l2;
		} while (h -= 4);
			
	} else {
		#if defined(__SSE2__)
			i8x16 w0 = ziplo64(wo8, wo8);
			i8x16 w1 = w0;
			i16x8 o = ziphi64(wo8, wo8);
			i8x16 shuf = {0, 1, 1, 2, 4, 5, 5, 6, 8, 9, 9, 10, 12, 13, 13, 14};
		#elif defined(__ARM_NEON)
			i16x8 v0 = ziplo16(wo16, wo16);
			i16x8 w0 = unziplo32(v0, v0);
			i16x8 w1 = unziphi32(v0, v0);
			i16x8 o = ziplo32(oCb, oCr);
			i8x16 shuf = {0, 1, 4, 5, 8, 9, 12, 13, 1, 2, 5, 6, 9, 10, 13, 14};
		#endif
		i32x4 l0 = {*(int32_t *)src, *(int32_t *)(src + sstride)};
		src += sstride2;
		do {
			i8x16 l1 = loadu32x4(src, src + sstride, src + sstride2, src + sstride3);
			i16x8 x0 = maddABCD(ziplo64(l0, l1), l1, shuf, AB, CD);
			i8x16 p = shrrpu16(x0, (i16x8){}, 6);
			i16x8 q = {*(int16_t *)dst, *(int16_t *)(dst + dstride), *(int16_t *)(dst + dstride2), *(int16_t *)(dst + dstride3)};
			i16x8 v = maddshrC4(q, p, w0, w1, o, wd);
			*(int16_t *)dst = v[0];
			*(int16_t *)(dst + dstride) = v[1];
			*(int16_t *)(dst + dstride2) = v[2];
			*(int16_t *)(dst + dstride3) = v[3];
			src += sstride4;
			dst += dstride4;
			l0 = shr128(l1, 8);
		} while (h -= 4);
	}
	#ifdef __SSE2__
		#undef sstride2
		#undef sstride4
		#undef dstride2
		#undef dstride4
	#endif
}



/**
 * Decode a single Inter block, fetching refIdx and mv at the given index in
 * memory, then computing the samples for the three color planes.
 * 
 * There are 5 weighting schemes, which we select according to this table:
 *            +------------+--------------+------------+--------------+
 *            | ref0 alone | ref0 of pair | ref1 alone | ref1 of pair |
 * +----------+------------+--------------+------------+--------------+
 * | bipred=0 | no_weight  | no_weight    | no_weight  | default2     |
 * | bipred=1 | explicit1  | no_weight    | explicit1  | explicit2    |
 * | bipred=2 | no_weight  | no_weight    | no_weight  | implicit2    |
 * +----------+------------+--------------+------------+--------------+
 */
static void noinline decode_inter(Edge264Context *ctx, int i, int w, int h) {
	static int8_t shift_Y_8bit[46] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
	static int8_t shift_C_8bit[22] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 7, 7, 7, 7};
	
	// load motion vector and source pointers
	int x = mb->mvs[i * 2];
	int y = mb->mvs[i * 2 + 1];
	int i8x8 = i >> 2;
	int i4x4 = i & 15;
	const uint8_t *ref = ctx->t.samples_buffers[mb->refPic[i8x8]];
	int xInt_Y = ctx->mbx * 16 + x444[i4x4] + (x >> 2);
	int xInt_C = ctx->mbx * 8 + (x444[i4x4] >> 1) + (x >> 3);
	int yInt_Y = ctx->mby * 16 + y444[i4x4] + (y >> 2);
	int yInt_C = ctx->mby * 8 + (y444[i4x4] >> 1) + (y >> 3);
	const uint8_t *src_Y = ref + xInt_Y + yInt_Y * ctx->t.stride[0];
	const uint8_t *src_C = ref + xInt_C + yInt_C * ctx->t.stride[1] + ctx->t.plane_size_Y;
	size_t sstride_Y = ctx->t.stride[0];
	size_t sstride_C = ctx->t.stride[1] >> 1;
	// print_header(ctx->d, "<k></k><v>CurrMbAddr=%d, i=%d, w=%d, h=%d, x=%d, y=%d, idx=%d, pic=%d</v>\n", ctx->CurrMbAddr, i, w, h, x, y, mb->refIdx[i8x8], mb->refPic[i8x8]);
	
	// prefetch source data into L3 cache
	const uint8_t *pref_C = src_C;
	for (int y = h + 1; y-- > 0; pref_C += sstride_C) {
		__builtin_prefetch(pref_C, 0, 1);
		__builtin_prefetch(pref_C + 16, 0, 1);
	}
	const uint8_t *pref_Y = src_Y - sstride_Y * 2 - 2;
	for (int y = h + 5; y-- > 0; pref_Y += sstride_Y) {
		__builtin_prefetch(pref_Y, 0, 1);
		__builtin_prefetch(pref_Y + 20, 0, 1);
	}
	
	// prediction coeffs {wY, oY, logWD_Y, logWD_C, wCb, wCr, oCb, oCr}
	i16x8 wod = {pack_w(0, 1), 0, 0, 0, pack_w(0, 1), pack_w(0, 1), 0, 0}; // no_weight
	int refIdx = mb->refIdx[i8x8];
	int refIdxX = mb->refIdx[i8x8 ^ 4];
	if (ctx->t.pps.weighted_bipred_idc != 1) {
		if (((i8x8 - 4) | refIdxX) >= 0) {
			if (ctx->t.pps.weighted_bipred_idc == 0) { // default2
				wod = (i16x8){257, 1, 1, 1, 257, 257, 1, 1};
			} else { // implicit2
				int w1 = ctx->implicit_weights[refIdxX][refIdx] - 64;
				int p = pack_w(64 - w1, w1);
				wod = (u16x8){p, 32, 6, 6, p, p, 32, 32};
				// w0 or w1 will overflow if w1 is 128 or -64 (WARNING untested in conformance bitstreams)
				if (__builtin_expect((unsigned)(w1 + 63) >= 191, 0)) {
					p = pack_w(2 - (w1 >> 5), w1 >> 5);
					wod = (u16x8){p, 1, 1, 1, p, p, 1, 1};
				}
			}
		}
	} else if (refIdxX < 0) { // explicit1
		refIdx += (i8x8 & 4) * 8;
		if (__builtin_expect(ctx->t.explicit_weights[0][refIdx] < 128, 1)) {
			wod[0] = pack_w(0, ctx->t.explicit_weights[0][refIdx]);
			wod[1] = (ctx->t.explicit_offsets[0][refIdx] * 2 + 1) << ctx->t.luma_log2_weight_denom >> 1;
			wod[2] = ctx->t.luma_log2_weight_denom;
		}
		if (__builtin_expect(ctx->t.explicit_weights[1][refIdx] < 128, 1)) {
			wod[4] = pack_w(0, ctx->t.explicit_weights[1][refIdx]);
			wod[5] = pack_w(0, ctx->t.explicit_weights[2][refIdx]);
			wod[6] = (ctx->t.explicit_offsets[1][refIdx] * 2 + 1) << ctx->t.chroma_log2_weight_denom >> 1;
			wod[7] = (ctx->t.explicit_offsets[2][refIdx] * 2 + 1) << ctx->t.chroma_log2_weight_denom >> 1;
			wod[3] = ctx->t.chroma_log2_weight_denom;
		}
	} else if (i8x8 >= 4) { // explicit2
		refIdx += 32;
		if (__builtin_expect((ctx->t.explicit_weights[0][refIdxX] & ctx->t.explicit_weights[0][refIdx]) != 128, 1)) {
			wod[0] = pack_w(ctx->t.explicit_weights[0][refIdxX], ctx->t.explicit_weights[0][refIdx]);
			wod[1] = ((ctx->t.explicit_offsets[0][refIdxX] + ctx->t.explicit_offsets[0][refIdx] + 1) | 1) << ctx->t.luma_log2_weight_denom;
			wod[2] = ctx->t.luma_log2_weight_denom + 1;
		} else {
			wod[0] = pack_w(ctx->t.explicit_weights[0][refIdxX] >> 1, ctx->t.explicit_weights[0][refIdx] >> 1);
			wod[1] = ((ctx->t.explicit_offsets[0][refIdxX] + ctx->t.explicit_offsets[0][refIdx] + 1) | 1) << ctx->t.luma_log2_weight_denom >> 1;
			wod[2] = ctx->t.luma_log2_weight_denom;
		}
		if (__builtin_expect((ctx->t.explicit_weights[1][refIdxX] & ctx->t.explicit_weights[1][refIdx]) != 128, 1)) {
			wod[4] = pack_w(ctx->t.explicit_weights[1][refIdxX], ctx->t.explicit_weights[1][refIdx]);
			wod[5] = pack_w(ctx->t.explicit_weights[2][refIdxX], ctx->t.explicit_weights[2][refIdx]);
			wod[6] = ((ctx->t.explicit_offsets[1][refIdxX] + ctx->t.explicit_offsets[1][refIdx] + 1) | 1) << ctx->t.chroma_log2_weight_denom;
			wod[7] = ((ctx->t.explicit_offsets[2][refIdxX] + ctx->t.explicit_offsets[2][refIdx] + 1) | 1) << ctx->t.chroma_log2_weight_denom;
			wod[3] = ctx->t.chroma_log2_weight_denom + 1;
		} else {
			wod[4] = pack_w(ctx->t.explicit_weights[1][refIdxX] >> 1, ctx->t.explicit_weights[1][refIdx] >> 1);
			wod[5] = pack_w(ctx->t.explicit_weights[2][refIdxX] >> 1, ctx->t.explicit_weights[2][refIdx] >> 1);
			wod[6] = ((ctx->t.explicit_offsets[1][refIdxX] + ctx->t.explicit_offsets[1][refIdx] + 1) | 1) << ctx->t.chroma_log2_weight_denom >> 1;
			wod[7] = ((ctx->t.explicit_offsets[2][refIdxX] + ctx->t.explicit_offsets[2][refIdx] + 1) | 1) << ctx->t.chroma_log2_weight_denom >> 1;
			wod[3] = ctx->t.chroma_log2_weight_denom;
		}
	}
	
	// edge propagation is an annoying but beautiful piece of code
	int xWide = (x & 7) != 0;
	int yWide = (y & 7) != 0;
	int width_Y = ctx->t.pic_width_in_mbs * 16;
	if (__builtin_expect((unsigned)xInt_Y - xWide * 2 >= width_Y - w + 1 - xWide * 5 ||
		(unsigned)yInt_Y - yWide * 2 >= ctx->t.pic_height_in_mbs * 16 - h + 1 - yWide * 5, 0))
	{
		i8x16 shuf0 = loadu128(shift_Y_8bit + 15 + clip3(-15, 0, xInt_Y - 2) + clip3(0, 15, xInt_Y + 14 - width_Y));
		i8x16 shuf1 = loadu128(shift_Y_8bit + 15 + clip3(-15, 0, xInt_Y + 14) + clip3(0, 15, xInt_Y + 30 - width_Y));
		const uint8_t *src0 = ref + clip3(0, width_Y - 16, xInt_Y - 2);
		const uint8_t *src1 = ref + clip3(0, width_Y - 16, xInt_Y + 14);
		yInt_Y -= 2;
		for (i8x16 *buf = ctx->edge_buf_v; buf < ctx->edge_buf_v + 10 + h * 2; buf += 2, yInt_Y++) {
			int c = clip3(0, ctx->t.plane_size_Y - sstride_Y, yInt_Y * sstride_Y);
			buf[0] = shuffle(loadu128(src0 + c), shuf0);
			buf[1] = shuffle(loadu128(src1 + c), shuf1);
		}
		src_Y = ctx->edge_buf + 66;
		sstride_Y = 32;
		
		// chroma may read (and ignore) 1 bottom row and 1 right col out of bounds
		int width_C = width_Y >> 1;
		i8x16 shuf = loadu64(shift_C_8bit + 7 + clip3(-7, 0, xInt_C) + clip3(0, 7, xInt_C + 8 - width_C));
		src0 = ref + clip3(0, width_C - 8, xInt_C);
		src1 = ref + clip3(0, width_C - 1, xInt_C + 8);
		for (int j = 0; j <= h >> 1; j++, yInt_C++) {
			int cb = ctx->t.plane_size_Y + clip3(0, ctx->t.plane_size_C - sstride_C * 2, yInt_C * sstride_C * 2);
			int cr = sstride_C + cb;
			// reads are split in 2 to support 8px-wide frames
			ctx->edge_buf_l[j * 4 + 84] = ((i64x2)shuffle(loadu64(src0 + cb), shuf))[0];
			ctx->edge_buf[j * 32 + 680] = *(src1 + cb);
			ctx->edge_buf_l[j * 4 + 86] = ((i64x2)shuffle(loadu64(src0 + cr), shuf))[0];
			ctx->edge_buf[j * 32 + 696] = *(src1 + cr);
		}
		sstride_C = 16;
		src_C = ctx->edge_buf + 672;
	}
	
	// chroma prediction comes first since it can be inlined
	uint8_t *dst_C = ctx->samples_mb[1] + (y444[i4x4] >> 1) * ctx->t.stride[1] + (x444[i4x4] >> 1);
	size_t dstride_C = ctx->t.stride[1] >> 1;
	int xFrac_C = x & 7;
	int yFrac_C = y & 7;
	i32x4 ABCD = {little_endian32(((8 - xFrac_C) | xFrac_C << 8) * ((8 - yFrac_C) | yFrac_C << 16))};
	decode_inter_chroma(w, h, sstride_C, src_C, dstride_C, dst_C, ABCD, wod);
	
	// tail jump to luma prediction
	int xFrac_Y = x & 3;
	int yFrac_Y = y & 3;
	size_t dstride_Y = ctx->t.stride[0];
	uint8_t *dst_Y = ctx->samples_mb[0] + y444[i4x4] * dstride_Y + x444[i4x4];
	decode_inter_luma((w << 1 & 48) + yFrac_Y * 4 + xFrac_Y, h, sstride_Y, src_Y, dstride_Y, dst_Y, wod);
}
