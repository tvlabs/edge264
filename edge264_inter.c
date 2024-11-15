#include "edge264_internal.h"



static const i8x16 pairs0 = {0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8};
static const i8x16 pairs8 = {8, 9, 9, 10, 10, 11, 11, 12, -1, -1, -1, -1, -1, -1, -1, -1};
static const i8x16 shuf2 = {2, -1, 3, -1, 4, -1, 5, -1, 6, -1, 7, -1, 8, -1, 9, -1};
static const i8x16 shuf3 = {3, -1, 4, -1, 5, -1, 6, -1, 7, -1, 8, -1, 9, -1, 10, -1};

/**
 * All sixtap functions compute (a - 5b + 20c + 20d - 5e + f + 16) >> 5.
 * The 8-bit version expects samples to be packed as 8-bit pairs.
 * The Up/Down versions are used when layering two series of additions/shifts.
 * The last D16 function is used for qpel12/21/23/32, to compute 6-tap values
 * from intermediate sums, then averaging them with the original qpel22 values.
 */
#define sixtap8(ab, cd, ef) ((sixtapU8(ab, cd, ef) + 16) >> 5)
static always_inline i16x8 sixtap16(i16x8 a, i16x8 b, i16x8 c, i16x8 d, i16x8 e, i16x8 f) {
	i16x8 af = a + f;
	i16x8 be = b + e;
	i16x8 cd = c + d;
	return (((((af - be) >> 2) - (be - cd)) >> 2) + (cd - -1)) >> 1;
}
static always_inline i16x8 sixtapU16(i16x8 a, i16x8 b, i16x8 c, i16x8 d, i16x8 e, i16x8 f) {
	i16x8 af = a + f;
	i16x8 be = b + e;
	i16x8 cd = c + d;
	i16x8 x = (cd << 2) - be; // cd*4-be
	return (af + x) + (x << 2); // af+(cd*4-be)*5
}
static always_inline i16x8 sixtapUD16(i16x8 a, i16x8 b, i16x8 c, i16x8 d, i16x8 e, i16x8 f) {
	i16x8 af = a + f;
	i16x8 be = b + e;
	i16x8 cd = c + d;
	return (((((af - be) >> 2) - (be - cd)) >> 2) + (cd + 32)) >> 6;
}
static always_inline i8x16 sixtapD16(i16x8 sum0, i16x8 sum1, i8x16 qpel22) {
	return avg8(packus16((sum0 + 16) >> 5, (sum1 + 16) >> 5), qpel22);
}

/**
 * Functions for in-place weighting and storage.
 */
static always_inline void store4x4_8bit(size_t stride, uint8_t * restrict dst, i8x16 p, i8x16 w, i16x8 o, i64x2 logWD) {
	i32x4 q = {*(int32_t *)(dst             ), *(int32_t *)(dst + stride    ),
	           *(int32_t *)(dst + stride * 2), *(int32_t *)(dst + stride * 3)};
	i32x4 r = weight_add8(q, p, w, o, logWD);
	*(int32_t *)(dst             ) = r[0];
	*(int32_t *)(dst + stride    ) = r[1];
	*(int32_t *)(dst + stride * 2) = r[2];
	*(int32_t *)(dst + stride * 3) = r[3];
}
static always_inline void store8x2_8bit(size_t stride, uint8_t * restrict dst, i8x16 p, i8x16 w, i16x8 o, i64x2 logWD) {
	i64x2 q = {*(int64_t *)(dst         ), *(int64_t *)(dst + stride)};
	i64x2 r = weight_add8(q, p, w, o, logWD);
	*(int64_t *)(dst         ) = r[0];
	*(int64_t *)(dst + stride) = r[1];
}


#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	#define pack_w(w0, w1) ((w1) << 8 | (w0) & 255)
#else
	#define pack_w(w0, w1) ((w0) << 8 | (w1) & 255)
#endif



/**
 * Inter 4x{4/8} prediction takes a 9x{9/13} matrix of 8/16bit luma samples as
 * input, and outputs a 4x{4/8} matrix in memory.
 * Loads are generally done by 8x1 matrices denoted as lRC in the code (R=row,
 * C=left column), or 4x2 matrices denoted as mRC. We may read 7 bytes past the
 * end of the buffer, which is fine with the memory layout. Also functions
 * follow ffmpeg's naming convention with qpelXY (instead of qpelRC).
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
 *   shufps (still used in QPEL_12_32). However it resulted in two reads per
 *   row, which may be prohibitive if outside of cache.
 * _ splitting horizontal filters in 3 to pmadd then padd (instead of phadd),
 *   when the equivalent code using filter16 uses at least 4 more shuffles
 *   (roughly the latency penalty from using pmadd).
 *
 * While it is impossible for functions to return multiple values in multiple
 * registers (stupid ABI), we cannot put redundant loads in functions and have
 * to duplicate a lot of code. The same goes for filter16, which would force
 * all live vector registers on stack if not inlined.
 * Also, although I_HATE_MACROS they are very useful here to reduce 16 (big)
 * functions down to 6. This is a lot of code, but all qpel interpolations are
 * done each in one pass without intermediate storage!
 */
static void inter4xH_qpel00_8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {
	do {
		i32x4 p = ziplo64(ziplo32(load32(src), load32(src + sstride)), ziplo32(load32(src + sstride * 2), load32(src + sstride * 3)));
		store4x4_8bit(dstride, dst, p, w, o, logWD);
		dst += dstride * 4;
		src += sstride * 4;
	} while (h -= 4);
}

#define INTER4xH_QPEL_10_20_30(QPEL, P)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		do {\
			i8x16 l2 = shuffle(load128(src               - 2), pairs0);\
			i8x16 l3 = shuffle(load128(src + sstride     - 2), pairs0);\
			i8x16 l4 = shuffle(load128(src + sstride * 2 - 2), pairs0);\
			i8x16 l5 = shuffle(load128(src + sstride * 3 - 2), pairs0); /* overreads 7 bytes */\
			u16x8 m22 = shuffleps(l2, l3, 1, 2, 1, 2);\
			u16x8 m42 = shuffleps(l4, l5, 1, 2, 1, 2);\
			i16x8 h0 = sixtap8(ziplo64(l2, l3), m22, ziphi64(l2, l3));\
			i16x8 h1 = sixtap8(ziplo64(l4, l5), m42, ziphi64(l4, l5));\
			i8x16 h01 = packus16(h0, h1);\
			store4x4_8bit(dstride, dst, P, w, o, logWD);\
			dst += dstride * 4;\
			src += sstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_10_20_30(qpel10, avg8(h01, packus16(m22 << 8 >> 8, m42 << 8 >> 8)))
INTER4xH_QPEL_10_20_30(qpel20, h01)
INTER4xH_QPEL_10_20_30(qpel30, avg8(h01, packus16(m22 >> 8, m42 >> 8)))

#define INTER4xH_QPEL_01_02_03(QPEL, P)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride = -sstride;\
		i8x16 l0 = load32(src + nstride * 2);\
		i8x16 l1 = load32(src + nstride    );\
		i8x16 l2 = load32(src              );\
		i8x16 l3 = load32(src + sstride    );\
		i8x16 l4 = load32(src + sstride * 2);\
		i8x16 m02 = ziplo32(l0, l1);\
		i8x16 m12 = ziplo32(l1, l2);\
		i8x16 m22 = ziplo32(l2, l3);\
		i8x16 m32 = ziplo32(l3, l4);\
		do {\
			src += sstride * 4;\
			i8x16 l5 = load32(src + nstride    );\
			i8x16 l6 = load32(src              );\
			i8x16 l7 = load32(src + sstride    );\
			i8x16 l8 = load32(src + sstride * 2);\
			i8x16 m42 = ziplo32(l4, l5);\
			i8x16 m52 = ziplo32(l5, l6);\
			i8x16 m62 = ziplo32(l6, l7);\
			i8x16 m72 = ziplo32(l7, l8);\
			i8x16 x0 = ziplo8(m02, m12);\
			i8x16 x1 = ziplo8(m22, m32);\
			i8x16 x2 = ziplo8(m42, m52);\
			i8x16 x3 = ziplo8(m62, m72);\
			i8x16 v01 = packus16(sixtap8(x0, x1, x2), sixtap8(x1, x2, x3));\
			store4x4_8bit(dstride, dst, P, w, o, logWD);\
			l4 = l8, m02 = m42, m12 = m52, m22 = m62, m32 = m72;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_01_02_03(qpel01, avg8(v01, ziplo64(m22, m42)))
INTER4xH_QPEL_01_02_03(qpel02, v01)
INTER4xH_QPEL_01_02_03(qpel03, avg8(v01, ziplo64(m32, m52)))

#define INTER4xH_QPEL_11_31_13_33(QPEL, R0, R1, R2, R3, C, SL)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride = -sstride;\
		i8x16 l0 = load32(src + nstride * 2 - 2 + C);\
		i8x16 l1 = load32(src + nstride     - 2 + C);\
		i8x16 l2 = shuffle(load128(src               - 2), pairs0);\
		i8x16 l3 = shuffle(load128(src + sstride     - 2), pairs0);\
		i8x16 l4 = shuffle(load128(src + sstride * 2 - 2), pairs0);\
		i16x8 m02 = cvt8zx16(ziplo32(l0, l1));\
		do {\
			src += sstride * 4;\
			i8x16 l5 = shuffle(load128(src + nstride     - 2), pairs0);\
			i8x16 l6 = shuffle(load128(src               - 2), pairs0);\
			i8x16 x0 = shuffleps(R0, R1, 1, 2, 1, 2);\
			i8x16 x1 = shuffleps(R2, R3, 1, 2, 1, 2);\
			i16x8 h0 = sixtap8(ziplo64(R0, R1), x0, ziphi64(R0, R1));\
			i16x8 h1 = sixtap8(ziplo64(R2, R3), x1, ziphi64(R2, R3));\
			i8x16 h01 = packus16(h0, h1);\
			i8x16 l7 = shuffle(load128(src + sstride     - 2), pairs0);\
			i8x16 l8 = shuffle(load128(src + sstride * 2 - 2), pairs0);\
			i16x8 m22 = (u16x8)shuffleps(l2, l3, 1, 2, 1, 2) << SL >> 8;\
			i16x8 m32 = (u16x8)shuffleps(l3, l4, 1, 2, 1, 2) << SL >> 8;\
			i16x8 m42 = (u16x8)shuffleps(l4, l5, 1, 2, 1, 2) << SL >> 8;\
			i16x8 m52 = (u16x8)shuffleps(l5, l6, 1, 2, 1, 2) << SL >> 8;\
			i16x8 m72 = (u16x8)shuffleps(l7, l8, 1, 2, 1, 2) << SL >> 8;\
			i16x8 m12 = shrd128(m02, m22, 8);\
			i16x8 m62 = shrd128(m52, m72, 8);\
			i16x8 v0 = sixtap16(m02, m12, m22, m32, m42, m52);\
			i16x8 v1 = sixtap16(m22, m32, m42, m52, m62, m72);\
			i8x16 v01 = packus16(v0, v1);\
			store4x4_8bit(dstride, dst, avg8(v01, h01), w, o, logWD);\
			l2 = l6, l3 = l7, l4 = l8, m02 = m42;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_11_31_13_33(qpel11, l2, l3, l4, l5, 2, 8)
INTER4xH_QPEL_11_31_13_33(qpel31, l2, l3, l4, l5, 3, 0)
INTER4xH_QPEL_11_31_13_33(qpel13, l3, l4, l5, l6, 2, 8)
INTER4xH_QPEL_11_31_13_33(qpel33, l3, l4, l5, l6, 3, 0)

#define INTER4xH_QPEL_12_32(QPEL, P)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride = -sstride;\
		i8x16 l0 = load128(src + nstride * 2 - 2);\
		i8x16 l1 = load128(src + nstride     - 2);\
		i8x16 l2 = load128(src               - 2);\
		i8x16 l3 = load128(src + sstride     - 2);\
		i8x16 l4 = load128(src + sstride * 2 - 2);\
		i8x16 r0 = ziplo16(ziphi8(l0, l1), ziphi8(l2, l3));\
		i16x8 l00 = cvt8zx16(l0);\
		i16x8 l10 = cvt8zx16(l1);\
		i16x8 l20 = cvt8zx16(l2);\
		i16x8 l30 = cvt8zx16(l3);\
		do {\
			src += sstride * 4;\
			i8x16 l5 = load128(src + nstride     - 2);\
			i16x8 l40 = cvt8zx16(l4);\
			i16x8 l50 = cvt8zx16(l5);\
			i16x8 x00 = sixtapU16(l00, l10, l20, l30, l40, l50);\
			i8x16 l6 = load128(src               - 2);\
			i16x8 l60 = cvt8zx16(l6);\
			i16x8 x10 = sixtapU16(l10, l20, l30, l40, l50, l60);\
			i8x16 l7 = load128(src + sstride     - 2);\
			i8x16 r1 = ziplo32(r0, ziplo16(ziphi8(l4, l5), ziphi8(l6, l7)));\
			i16x8 l70 = cvt8zx16(l7);\
			i16x8 x20 = sixtapU16(l20, l30, l40, l50, l60, l70);\
			i8x16 l8 = load128(src + sstride * 2 - 2);\
			i8x16 r2 = shuffle(shuffleps(r1, l8, 0, 1, 2, 3), pairs0);\
			i16x8 l80 = cvt8zx16(l8);\
			i16x8 x30 = sixtapU16(l30, l40, l50, l60, l70, l80);\
			i16x8 x08 = sixtapU8(r2, shuffle32(r2, 1, 2, 3, 3), shuffle32(r2, 2, 3, 3, 3));\
			i16x8 x01 = shrd128(x00, x08, 2);\
			i16x8 x11 = shrd128(x10, shufflelo(x08, 1, 1, 1, 1), 2);\
			i16x8 x21 = shrd128(x20, shufflelo(x08, 2, 2, 2, 2), 2);\
			i16x8 x31 = shrd128(x30, shufflelo(x08, 3, 3, 3, 3), 2);\
			i16x8 y00 = shuffleps(x00, x10, 0, 1, 0, 1);\
			i16x8 y01 = shuffleps(x01, x11, 0, 1, 0, 1);\
			i16x8 y02 = shuffleps(x00, x10, 1, 2, 1, 2);\
			i16x8 y03 = shuffleps(x01, x11, 1, 2, 1, 2);\
			i16x8 y04 = shuffleps(x00, x10, 2, 3, 2, 3);\
			i16x8 y05 = shuffleps(x01, x11, 2, 3, 2, 3);\
			i16x8 y20 = shuffleps(x20, x30, 0, 1, 0, 1);\
			i16x8 y21 = shuffleps(x21, x31, 0, 1, 0, 1);\
			i16x8 y22 = shuffleps(x20, x30, 1, 2, 1, 2);\
			i16x8 y23 = shuffleps(x21, x31, 1, 2, 1, 2);\
			i16x8 y24 = shuffleps(x20, x30, 2, 3, 2, 3);\
			i16x8 y25 = shuffleps(x21, x31, 2, 3, 2, 3);\
			i16x8 vh0 = sixtapUD16(y00, y01, y02, y03, y04, y05);\
			i16x8 vh1 = sixtapUD16(y20, y21, y22, y23, y24, y25);\
			i8x16 vh = packus16(vh0, vh1);\
			store4x4_8bit(dstride, dst, P, w, o, logWD);\
			l00 = l40, l10 = l50, l20 = l60, l30 = l70;\
			l4 = l8, r0 = shuffle32(r1, 1, 1, 1, 1);\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_12_32(qpel12, sixtapD16(y02, y22, vh))
INTER4xH_QPEL_12_32(qpel32, sixtapD16(y03, y23, vh))

#define INTER4xH_QPEL_21_22_23(QPEL, P)\
	static void inter4xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride = -sstride;\
		i8x16 l0 = shuffle(load128(src + nstride * 2 - 2), pairs0);\
		i8x16 l1 = shuffle(load128(src + nstride     - 2), pairs0);\
		i8x16 l2 = shuffle(load128(src               - 2), pairs0);\
		i8x16 l3 = shuffle(load128(src + sstride     - 2), pairs0);\
		i8x16 l4 = shuffle(load128(src + sstride * 2 - 2), pairs0);\
		i16x8 x0 = shuffleps(l0, l1, 1, 2, 1, 2);\
		i16x8 x1 = shuffleps(l2, l3, 1, 2, 1, 2);\
		i16x8 x2 = shuffleps(l3, l4, 1, 2, 1, 2);\
		i16x8 x00 = sixtapU8(ziplo64(l0, l1), x0, ziphi64(l0, l1));\
		i16x8 x20 = sixtapU8(ziplo64(l2, l3), x1, ziphi64(l2, l3));\
		i16x8 x30 = sixtapU8(ziplo64(l3, l4), x2, ziphi64(l3, l4));\
		do {\
			src += sstride * 4;\
			i8x16 l5 = shuffle(load128(src + nstride     - 2), pairs0);\
			i8x16 l6 = shuffle(load128(src               - 2), pairs0);\
			i8x16 l7 = shuffle(load128(src + sstride     - 2), pairs0);\
			i8x16 l8 = shuffle(load128(src + sstride * 2 - 2), pairs0);\
			i16x8 x3 = shuffleps(l5, l6, 1, 2, 1, 2);\
			i16x8 x4 = shuffleps(l7, l8, 1, 2, 1, 2);\
			i16x8 x50 = sixtapU8(ziplo64(l5, l6), x3, ziphi64(l5, l6));\
			i16x8 x70 = sixtapU8(ziplo64(l7, l8), x4, ziphi64(l7, l8));\
			i16x8 x10 = shrd128(x00, x20, 8);\
			i16x8 x40 = shrd128(x30, x50, 8);\
			i16x8 x60 = shrd128(x50, x70, 8);\
			i16x8 hv0 = sixtapUD16(x00, x10, x20, x30, x40, x50);\
			i16x8 hv1 = sixtapUD16(x20, x30, x40, x50, x60, x70);\
			i16x8 hv = packus16(hv0, hv1);\
			store4x4_8bit(dstride, dst, P, w, o, logWD);\
			x00 = x40, x20 = x60, x30 = x70;\
			dst += dstride * 4;\
		} while (h -= 4);\
	}\

INTER4xH_QPEL_21_22_23(qpel21, sixtapD16(x20, x40, hv))
INTER4xH_QPEL_21_22_23(qpel22, hv)
INTER4xH_QPEL_21_22_23(qpel23, sixtapD16(x30, x50, hv))



/**
 * Inter 8x{4/8/16} prediction takes a 13x{9/13/21} matrix and outputs a
 * 8x{4/8/16} matrix in memory.
 * This is actually simpler than 4xH since we always work on 8x1 lines, so we
 * need less shuffling tricks. The entire input matrix being too big to fit in
 * registers, we compute values from top to bottom and keep intermediate
 * results between iterations. The code is not manually unrolled since it would
 * require a bit too much copy/paste (personal taste). However there is no
 * simple way to signal that h is multiple of 4, so compilers won't be able to
 * unroll sub-loops for greater performance.
 */
static void inter8xH_qpel00_8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {
	do {
		i64x2 p0 = loadh64(load64(src              ), src + sstride    );
		i64x2 p1 = loadh64(load64(src + sstride * 2), src + sstride * 3);
		store8x2_8bit(dstride, dst              , p0, w, o, logWD);
		store8x2_8bit(dstride, dst + dstride * 2, p1, w, o, logWD);
		src += sstride * 4;
		dst += dstride * 4;
	} while (h -= 4);
}

#define INTER8xH_QPEL_10_20_30(QPEL, P)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		do {\
			i8x16 l0 = load128(src           - 2); /* overreads 3 bytes */\
			i8x16 l1 = load128(src + sstride - 2);\
			i8x16 x0 = shuffle(l0, pairs0);\
			i8x16 x1 = shuffle(l0, pairs8);\
			i8x16 x2 = shuffle(l1, pairs0);\
			i8x16 x3 = shuffle(l1, pairs8);\
			i16x8 h0 = sixtap8(x0, shrd128(x0, x1, 4), shrd128(x0, x1, 8));\
			i16x8 h1 = sixtap8(x2, shrd128(x2, x3, 4), shrd128(x2, x3, 8));\
			i8x16 h01 = packus16(h0, h1);\
			store8x2_8bit(dstride, dst, P, w, o, logWD);\
			src += sstride * 2;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_10_20_30(qpel10, avg8(h01, ziplo64(shr128(l0, 2), shr128(l1, 2))))
INTER8xH_QPEL_10_20_30(qpel20, h01)
INTER8xH_QPEL_10_20_30(qpel30, avg8(h01, ziplo64(shr128(l0, 3), shr128(l1, 3))))

#define INTER8xH_QPEL_01_02_03(QPEL, P)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride = -sstride;\
		i16x8 l00 = load8zx16(src + nstride * 2);\
		i16x8 l10 = load8zx16(src + nstride    );\
		i16x8 l20 = load8zx16(src              );\
		i16x8 l30 = load8zx16(src + sstride    );\
		i16x8 l40 = load8zx16(src + sstride * 2);\
		do {\
			src += sstride * 2;\
			i16x8 l50 = load8zx16(src + sstride    );\
			i16x8 l60 = load8zx16(src + sstride * 2);\
			i16x8 v0 = sixtap16(l00, l10, l20, l30, l40, l50);\
			i16x8 v1 = sixtap16(l10, l20, l30, l40, l50, l60);\
			i8x16 v01 = packus16(v0, v1);\
			store8x2_8bit(dstride, dst, P, w, o, logWD);\
			l00 = l20, l10 = l30, l20 = l40, l30 = l50, l40 = l60;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_01_02_03(qpel01, avg8(v01, packus16(l20, l30)))
INTER8xH_QPEL_01_02_03(qpel02, v01)
INTER8xH_QPEL_01_02_03(qpel03, avg8(v01, packus16(l30, l40)))

#define INTER8xH_QPEL_11_31_13_33(QPEL, R0, R1, C)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride = -sstride;\
		i16x8 l02 = load8zx16(src + nstride * 2 - 2 + C);\
		i16x8 l12 = load8zx16(src + nstride     - 2 + C);\
		i8x16 l2 = load128(src               - 2);\
		i8x16 l3 = load128(src + sstride     - 2);\
		i8x16 l4 = load128(src + sstride * 2 - 2);\
		do {\
			src += sstride * 2;\
			i8x16 x0 = shuffle(R0, pairs0);\
			i8x16 x1 = shuffle(R0, pairs8);\
			i8x16 x2 = shuffle(R1, pairs0);\
			i8x16 x3 = shuffle(R1, pairs8);\
			i16x8 h0 = sixtap8(x0, shrd128(x0, x1, 4), shrd128(x0, x1, 8));\
			i16x8 h1 = sixtap8(x2, shrd128(x2, x3, 4), shrd128(x2, x3, 8));\
			i8x16 h01 = packus16(h0, h1);\
			i8x16 l5 = load128(src + sstride     - 2);\
			i8x16 l6 = load128(src + sstride * 2 - 2);\
			i16x8 l22 = shufflez(l2, shuf##C);\
			i16x8 l32 = shufflez(l3, shuf##C);\
			i16x8 l42 = shufflez(l4, shuf##C);\
			i16x8 l52 = shufflez(l5, shuf##C);\
			i16x8 l62 = shufflez(l6, shuf##C);\
			i16x8 v0 = sixtap16(l02, l12, l22, l32, l42, l52);\
			i16x8 v1 = sixtap16(l12, l22, l32, l42, l52, l62);\
			i8x16 v01 = packus16(v0, v1);\
			store8x2_8bit(dstride, dst, avg8(v01, h01), w, o, logWD);\
			l02 = l22, l12 = l32, l2 = l4, l3 = l5;\
			l4 = l6;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_11_31_13_33(qpel11, l2, l3, 2)
INTER8xH_QPEL_11_31_13_33(qpel31, l2, l3, 3)
INTER8xH_QPEL_11_31_13_33(qpel13, l3, l4, 2)
INTER8xH_QPEL_11_31_13_33(qpel33, l3, l4, 3)

#define INTER8xH_QPEL_12_32(QPEL, P)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride = -sstride;\
		i8x16 l0 = load128(src + nstride * 2 - 2);\
		i8x16 l1 = load128(src + nstride     - 2);\
		i8x16 l2 = load128(src               - 2);\
		i8x16 l3 = load128(src + sstride     - 2);\
		i8x16 l4 = load128(src + sstride * 2 - 2);\
		do {\
			src += sstride * 2;\
			i8x16 l5 = load128(src + sstride     - 2);\
			i8x16 l6 = load128(src + sstride * 2 - 2);\
			i8x16 zero = {};\
			i16x8 l08 = ziphi8(l0, zero);\
			i16x8 l18 = ziphi8(l1, zero);\
			i16x8 l28 = ziphi8(l2, zero);\
			i16x8 l38 = ziphi8(l3, zero);\
			i16x8 l48 = ziphi8(l4, zero);\
			i16x8 l58 = ziphi8(l5, zero);\
			i16x8 l68 = ziphi8(l6, zero);\
			i16x8 x08 = sixtapU16(l08, l18, l28, l38, l48, l58);\
			i16x8 x18 = sixtapU16(l18, l28, l38, l48, l58, l68);\
			i16x8 l00 = cvt8zx16(l0);\
			i16x8 l10 = cvt8zx16(l1);\
			i16x8 l20 = cvt8zx16(l2);\
			i16x8 l30 = cvt8zx16(l3);\
			i16x8 l40 = cvt8zx16(l4);\
			i16x8 l50 = cvt8zx16(l5);\
			i16x8 l60 = cvt8zx16(l6);\
			i16x8 x00 = sixtapU16(l00, l10, l20, l30, l40, l50);\
			i16x8 x10 = sixtapU16(l10, l20, l30, l40, l50, l60);\
			i16x8 x01 = shrd128(x00, x08, 2);\
			i16x8 x11 = shrd128(x10, x18, 2);\
			i16x8 x02 = shrd128(x00, x08, 4);\
			i16x8 x12 = shrd128(x10, x18, 4);\
			i16x8 x03 = shrd128(x00, x08, 6);\
			i16x8 x13 = shrd128(x10, x18, 6);\
			i16x8 x04 = shrd128(x00, x08, 8);\
			i16x8 x14 = shrd128(x10, x18, 8);\
			i16x8 x05 = shrd128(x00, x08, 10);\
			i16x8 x15 = shrd128(x10, x18, 10);\
			i16x8 vh0 = sixtapUD16(x00, x01, x02, x03, x04, x05);\
			i16x8 vh1 = sixtapUD16(x10, x11, x12, x13, x14, x15);\
			i8x16 vh = packus16(vh0, vh1);\
			store8x2_8bit(dstride, dst, P, w, o, logWD);\
			l0 = l2, l1 = l3, l2 = l4, l3 = l5, l4 = l6;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_12_32(qpel12, sixtapD16(x02, x12, vh))
INTER8xH_QPEL_12_32(qpel32, sixtapD16(x03, x13, vh))

#define INTER8xH_QPEL_21_22_23(QPEL, P)\
	static void inter8xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride = -sstride;\
		i8x16 l0 = load128(src + nstride * 2 - 2);\
		i8x16 l00 = shuffle(l0, pairs0);\
		i8x16 l08 = shuffle(l0, pairs8);\
		i16x8 x00 = sixtapU8(l00, shrd128(l00, l08, 4), shrd128(l00, l08, 8));\
		i8x16 l1 = load128(src + nstride     - 2);\
		i8x16 l10 = shuffle(l1, pairs0);\
		i8x16 l18 = shuffle(l1, pairs8);\
		i16x8 x10 = sixtapU8(l10, shrd128(l10, l18, 4), shrd128(l10, l18, 8));\
		i8x16 l2 = load128(src               - 2);\
		i8x16 l20 = shuffle(l2, pairs0);\
		i8x16 l28 = shuffle(l2, pairs8);\
		i16x8 x20 = sixtapU8(l20, shrd128(l20, l28, 4), shrd128(l20, l28, 8));\
		i8x16 l3 = load128(src + sstride     - 2);\
		i8x16 l30 = shuffle(l3, pairs0);\
		i8x16 l38 = shuffle(l3, pairs8);\
		i16x8 x30 = sixtapU8(l30, shrd128(l30, l38, 4), shrd128(l30, l38, 8));\
		i8x16 l4 = load128(src + sstride * 2 - 2);\
		i8x16 l40 = shuffle(l4, pairs0);\
		i8x16 l48 = shuffle(l4, pairs8);\
		i16x8 x40 = sixtapU8(l40, shrd128(l40, l48, 4), shrd128(l40, l48, 8));\
		do {\
			src += sstride * 2;\
			i8x16 l5 = load128(src + sstride     - 2);\
			i8x16 l50 = shuffle(l5, pairs0);\
			i8x16 l58 = shuffle(l5, pairs8);\
			i16x8 x50 = sixtapU8(l50, shrd128(l50, l58, 4), shrd128(l50, l58, 8));\
			i8x16 l6 = load128(src + sstride * 2 - 2);\
			i8x16 l60 = shuffle(l6, pairs0);\
			i8x16 l68 = shuffle(l6, pairs8);\
			i16x8 x60 = sixtapU8(l60, shrd128(l60, l68, 4), shrd128(l60, l68, 8));\
			i16x8 hv0 = sixtapUD16(x00, x10, x20, x30, x40, x50);\
			i16x8 hv1 = sixtapUD16(x10, x20, x30, x40, x50, x60);\
			i8x16 hv = packus16(hv0, hv1);\
			store8x2_8bit(dstride, dst, P, w, o, logWD);\
			x00 = x20, x10 = x30, x20 = x40, x30 = x50, x40 = x60;\
			dst += dstride * 2;\
		} while (h -= 2);\
	}\

INTER8xH_QPEL_21_22_23(qpel21, sixtapD16(x20, x30, hv))
INTER8xH_QPEL_21_22_23(qpel22, hv)
INTER8xH_QPEL_21_22_23(qpel23, sixtapD16(x30, x40, hv))



/**
 * Inter 16x{8/16} takes a 21x{13/21} matrix and outputs a 16x{8/16} matrix in
 * memory.
 * Here the biggest challenge is register pressure, so we count on compilers
 * to spill/reload on stack. All functions were designed with 16 available
 * registers in mind, for older chips there will just be more spills.
 */
static void inter16xH_qpel00_8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {
	do {
		*(i8x16 *)(dst              ) = weight_add8(*(i8x16 *)(dst              ), load128(src              ), w, o, logWD);
		*(i8x16 *)(dst + dstride    ) = weight_add8(*(i8x16 *)(dst + dstride    ), load128(src + sstride    ), w, o, logWD);
		*(i8x16 *)(dst + dstride * 2) = weight_add8(*(i8x16 *)(dst + dstride * 2), load128(src + sstride * 2), w, o, logWD);
		*(i8x16 *)(dst + dstride * 3) = weight_add8(*(i8x16 *)(dst + dstride * 3), load128(src + sstride * 3), w, o, logWD);
		src += sstride * 4;
		dst += dstride * 4;
	} while (h -= 4);
}

#define INTER16xH_QPEL_10_20_30(QPEL, P)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		do {\
			i8x16 l0 = load128(src - 2);\
			i8x16 l8 = load128(src + 6); /* overreads 3 bytes */\
			i8x16 x0 = shuffle(l0, pairs0);\
			i8x16 x8 = shuffle(l8, pairs0);\
			i8x16 xG = shuffle(l8, pairs8);\
			i16x8 h0 = sixtap8(x0, shrd128(x0, x8, 4), shrd128(x0, x8, 8));\
			i16x8 h1 = sixtap8(x8, shrd128(x8, xG, 4), shrd128(x8, xG, 8));\
			i8x16 h01 = packus16(h0, h1);\
			*(i8x16 *)dst = weight_add8(*(i8x16 *)dst, P, w, o, logWD);\
			src += sstride;\
			dst += dstride;\
		} while (h -= 1);\
	}\

INTER16xH_QPEL_10_20_30(qpel10, avg8(h01, shrd128(l0, shr128(l8, 8), 2)))
INTER16xH_QPEL_10_20_30(qpel20, h01)
INTER16xH_QPEL_10_20_30(qpel30, avg8(h01, shrd128(l0, shr128(l8, 8), 3)))

#define INTER16xH_QPEL_01_02_03(QPEL, P)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride= -sstride;\
		i8x16 l0 = load128(src + nstride * 2);\
		i8x16 l1 = load128(src + nstride    );\
		i8x16 l2 = load128(src              );\
		i8x16 l3 = load128(src + sstride    );\
		i8x16 l4 = load128(src + sstride * 2);\
		do {\
			src += sstride;\
			i8x16 l5 = load128(src + sstride * 2);\
			i16x8 l00 = cvt8zx16(l0);\
			i16x8 l10 = cvt8zx16(l1);\
			i16x8 l20 = cvt8zx16(l2);\
			i16x8 l30 = cvt8zx16(l3);\
			i16x8 l40 = cvt8zx16(l4);\
			i16x8 l50 = cvt8zx16(l5);\
			i16x8 v0 = sixtap16(l00, l10, l20, l30, l40, l50);\
			i8x16 zero = {};\
			i16x8 l08 = ziphi8(l0, zero);\
			i16x8 l18 = ziphi8(l1, zero);\
			i16x8 l28 = ziphi8(l2, zero);\
			i16x8 l38 = ziphi8(l3, zero);\
			i16x8 l48 = ziphi8(l4, zero);\
			i16x8 l58 = ziphi8(l5, zero);\
			i16x8 v1 = sixtap16(l08, l18, l28, l38, l48, l58);\
			i8x16 v01 = packus16(v0, v1);\
			*(i8x16 *)dst = weight_add8(*(i8x16 *)dst, P, w, o, logWD);\
			l0 = l1, l1 = l2, l2 = l3, l3 = l4, l4 = l5;\
			dst += dstride;\
		} while (h -= 1);\
	}\

INTER16xH_QPEL_01_02_03(qpel01, avg8(v01, packus16(l20, l28)))
INTER16xH_QPEL_01_02_03(qpel02, v01)
INTER16xH_QPEL_01_02_03(qpel03, avg8(v01, packus16(l30, l38)))

#define INTER16xH_QPEL_11_31_13_33(QPEL, R, C)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride= -sstride;\
		i8x16 l02 = load128(src + nstride * 2 - 2 + C);\
		i8x16 l12 = load128(src + nstride     - 2 + C);\
		i8x16 l20 = load128(src               - 2);\
		i8x16 l28 = load128(src               + 6);\
		i8x16 l30 = load128(src + sstride     - 2);\
		i8x16 l38 = load128(src + sstride     + 6);\
		i8x16 l40 = load128(src + sstride * 2 - 2);\
		i8x16 l48 = load128(src + sstride * 2 + 6);\
		do {\
			src += sstride;\
			i8x16 x0 = shuffle(l##R##0, pairs0);\
			i8x16 x8 = shuffle(l##R##8, pairs0);\
			i8x16 xG = shuffle(l##R##8, pairs8);\
			i16x8 h0 = sixtap8(x0, shrd128(x0, x8, 4), shrd128(x0, x8, 8));\
			i16x8 h1 = sixtap8(x8, shrd128(x8, xG, 4), shrd128(x8, xG, 8));\
			i8x16 h01 = packus16(h0, h1);\
			i8x16 l50 = load128(src + sstride * 2 - 2);\
			i8x16 l58 = load128(src + sstride * 2 + 6);\
			i16x8 L02 = cvt8zx16(l02);\
			i16x8 L12 = cvt8zx16(l12);\
			i16x8 L22 = shufflez(l20, shuf##C);\
			i16x8 L32 = shufflez(l30, shuf##C);\
			i16x8 L42 = shufflez(l40, shuf##C);\
			i16x8 L52 = shufflez(l50, shuf##C);\
			i16x8 v0 = sixtap16(L02, L12, L22, L32, L42, L52);\
			i8x16 zero = {};\
			i16x8 L0A = ziphi8(l02, zero);\
			i16x8 L1A = ziphi8(l12, zero);\
			i16x8 L2A = shufflez(l28, shuf##C);\
			i16x8 L3A = shufflez(l38, shuf##C);\
			i16x8 L4A = shufflez(l48, shuf##C);\
			i16x8 L5A = shufflez(l58, shuf##C);\
			i16x8 v1 = sixtap16(L0A, L1A, L2A, L3A, L4A, L5A);\
			i8x16 v01 = packus16(v0, v1);\
			*(i8x16 *)dst = weight_add8(*(i8x16 *)dst, avg8(v01, h01), w, o, logWD);\
			l02 = l12, l12 = packus16(L22, L2A);\
			l20 = l30, l28 = l38, l30 = l40, l38 = l48, l40 = l50, l48 = l58;\
			dst += dstride;\
		} while (h -= 1);\
	}\

INTER16xH_QPEL_11_31_13_33(qpel11, 2, 2)
INTER16xH_QPEL_11_31_13_33(qpel31, 2, 3)
INTER16xH_QPEL_11_31_13_33(qpel13, 3, 2)
INTER16xH_QPEL_11_31_13_33(qpel33, 3, 3)

#define INTER16xH_QPEL_12_32(QPEL, P)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride= -sstride;\
		i8x16 l0 = load128(src + nstride * 2 - 2);\
		i16x8 l0G = load8zx16(src + nstride * 2 + 14);\
		i8x16 l1 = load128(src + nstride     - 2);\
		i16x8 l1G = load8zx16(src + nstride     + 14);\
		i8x16 l2 = load128(src               - 2);\
		i16x8 l2G = load8zx16(src               + 14);\
		i8x16 l3 = load128(src + sstride     - 2);\
		i16x8 l3G = load8zx16(src + sstride     + 14);\
		i8x16 l4 = load128(src + sstride * 2 - 2);\
		i16x8 l4G = load8zx16(src + sstride * 2 + 14);\
		do {\
			src += sstride;\
			i8x16 l5 = load128(src + sstride * 2 - 2);\
			i16x8 l00 = cvt8zx16(l0);\
			i16x8 l10 = cvt8zx16(l1);\
			i16x8 l20 = cvt8zx16(l2);\
			i16x8 l30 = cvt8zx16(l3);\
			i16x8 l40 = cvt8zx16(l4);\
			i16x8 l50 = cvt8zx16(l5);\
			i16x8 v00 = sixtapU16(l00, l10, l20, l30, l40, l50);\
			i8x16 zero = {};\
			i16x8 l08 = ziphi8(l0, zero);\
			i16x8 l18 = ziphi8(l1, zero);\
			i16x8 l28 = ziphi8(l2, zero);\
			i16x8 l38 = ziphi8(l3, zero);\
			i16x8 l48 = ziphi8(l4, zero);\
			i16x8 l58 = ziphi8(l5, zero);\
			i16x8 v08 = sixtapU16(l08, l18, l28, l38, l48, l58);\
			i16x8 l5G = load8zx16(src + sstride * 2 + 14);\
			i16x8 v0G = sixtapU16(l0G, l1G, l2G, l3G, l4G, l5G);\
			i16x8 v01 = shrd128(v00, v08, 2);\
			i16x8 v02 = shrd128(v00, v08, 4);\
			i16x8 v03 = shrd128(v00, v08, 6);\
			i16x8 v04 = shrd128(v00, v08, 8);\
			i16x8 v05 = shrd128(v00, v08, 10);\
			i16x8 v09 = shrd128(v08, v0G, 2);\
			i16x8 v0A = shrd128(v08, v0G, 4);\
			i16x8 v0B = shrd128(v08, v0G, 6);\
			i16x8 v0C = shrd128(v08, v0G, 8);\
			i16x8 v0D = shrd128(v08, v0G, 10);\
			i16x8 vh0 = sixtapUD16(v00, v01, v02, v03, v04, v05);\
			i16x8 vh1 = sixtapUD16(v08, v09, v0A, v0B, v0C, v0D);\
			i8x16 vh = packus16(vh0, vh1);\
			*(i8x16 *)dst = weight_add8(*(i8x16 *)dst, P, w, o, logWD);\
			l0 = l1, l1 = l2, l2 = l3, l3 = l4, l4 = l5;\
			l0G = l1G, l1G = l2G, l2G = l3G, l3G = l4G, l4G = l5G;\
			dst += dstride;\
		} while (h -= 1);\
	}\

INTER16xH_QPEL_12_32(qpel12, sixtapD16(v02, v0A, vh))
INTER16xH_QPEL_12_32(qpel32, sixtapD16(v03, v0B, vh))

// with an array on stack or a preliminary loop, both compilers would get crazy
#define INTER16xH_QPEL_21_22_23(QPEL, P)\
	static void inter16xH_ ## QPEL ## _8bit(int h, size_t dstride, uint8_t * restrict dst, size_t sstride, const uint8_t *src, i8x16 w, i16x8 o, i64x2 logWD) {\
		ssize_t nstride= -sstride;\
		i8x16 l00 = load128(src + nstride * 2 - 2);\
		i8x16 l08 = load128(src + nstride * 2 + 6);\
		i8x16 x00 = shuffle(l00, pairs0);\
		i8x16 x08 = shuffle(l08, pairs0);\
		i8x16 x0G = shuffle(l08, pairs8);\
		i16x8 h00 = sixtapU8(x00, shrd128(x00, x08, 4), shrd128(x00, x08, 8));\
		i16x8 h08 = sixtapU8(x08, shrd128(x08, x0G, 4), shrd128(x08, x0G, 8));\
		i8x16 l10 = load128(src + nstride     - 2);\
		i8x16 l18 = load128(src + nstride     + 6);\
		i8x16 x10 = shuffle(l10, pairs0);\
		i8x16 x18 = shuffle(l18, pairs0);\
		i8x16 x1G = shuffle(l18, pairs8);\
		i16x8 h10 = sixtapU8(x10, shrd128(x10, x18, 4), shrd128(x10, x18, 8));\
		i16x8 h18 = sixtapU8(x18, shrd128(x18, x1G, 4), shrd128(x18, x1G, 8));\
		i8x16 l20 = load128(src               - 2);\
		i8x16 l28 = load128(src               + 6);\
		i8x16 x20 = shuffle(l20, pairs0);\
		i8x16 x28 = shuffle(l28, pairs0);\
		i8x16 x2G = shuffle(l28, pairs8);\
		i16x8 h20 = sixtapU8(x20, shrd128(x20, x28, 4), shrd128(x20, x28, 8));\
		i16x8 h28 = sixtapU8(x28, shrd128(x28, x2G, 4), shrd128(x28, x2G, 8));\
		i8x16 l30 = load128(src + sstride     - 2);\
		i8x16 l38 = load128(src + sstride     + 6);\
		i8x16 x30 = shuffle(l30, pairs0);\
		i8x16 x38 = shuffle(l38, pairs0);\
		i8x16 x3G = shuffle(l38, pairs8);\
		i16x8 h30 = sixtapU8(x30, shrd128(x30, x38, 4), shrd128(x30, x38, 8));\
		i16x8 h38 = sixtapU8(x38, shrd128(x38, x3G, 4), shrd128(x38, x3G, 8));\
		i8x16 l40 = load128(src + sstride * 2 - 2);\
		i8x16 l48 = load128(src + sstride * 2 + 6);\
		i8x16 x40 = shuffle(l40, pairs0);\
		i8x16 x48 = shuffle(l48, pairs0);\
		i8x16 x4G = shuffle(l48, pairs8);\
		i16x8 h40 = sixtapU8(x40, shrd128(x40, x48, 4), shrd128(x40, x48, 8));\
		i16x8 h48 = sixtapU8(x48, shrd128(x48, x4G, 4), shrd128(x48, x4G, 8));\
		do {\
			src += sstride;\
			i8x16 l50 = load128(src + sstride * 2 - 2);\
			i8x16 l58 = load128(src + sstride * 2 + 6);\
			i8x16 x50 = shuffle(l50, pairs0);\
			i8x16 x58 = shuffle(l58, pairs0);\
			i8x16 x5G = shuffle(l58, pairs8);\
			i16x8 h50 = sixtapU8(x50, shrd128(x50, x58, 4), shrd128(x50, x58, 8));\
			i16x8 h58 = sixtapU8(x58, shrd128(x58, x5G, 4), shrd128(x58, x5G, 8));\
			i16x8 hv0 = sixtapUD16(h00, h10, h20, h30, h40, h50);\
			i16x8 hv1 = sixtapUD16(h08, h18, h28, h38, h48, h58);\
			i8x16 hv = packus16(hv0, hv1);\
			*(i8x16 *)dst = weight_add8(*(i8x16 *)dst, P, w, o, logWD);\
			h00 = h10, h08 = h18;\
			h10 = h20, h18 = h28;\
			h20 = h30, h28 = h38;\
			h30 = h40, h38 = h48;\
			h40 = h50, h48 = h58;\
			dst += dstride;\
		} while (h -= 1);\
	}\

INTER16xH_QPEL_21_22_23(qpel21, sixtapD16(h20, h28, hv))
INTER16xH_QPEL_21_22_23(qpel22, hv)
INTER16xH_QPEL_21_22_23(qpel23, sixtapD16(h30, h38, hv))



/**
 * Combined Cb & Cr inter chroma prediction
 * 
 * dstride and sstride are half the strides of src and dst chroma planes.
 * A/B/C/D coefficients are passed in vector registers since their computation
 * is shared by all sizes. These functions should be inlined, otherwise AB/CD
 * would be spilled on stack.
 */
static always_inline void inter8xH_chroma_8bit(int h, size_t dstride, uint8_t *dst, size_t sstride, const uint8_t *src, i8x16 AB, i8x16 CD, i8x16 Wb, i8x16 Wr, i16x8 Ob, i16x8 Or, i64x2 logWD) {
	i8x16 zero = {};
	i8x16 r0 = shuffle(load128(src), pairs0);
	i8x16 r1 = shuffle(load128(src + sstride), pairs0);
	do {
		i8x16 r2 = shuffle(load128(src + sstride * 2), pairs0);
		i8x16 r3 = shuffle(load128(src + sstride * 3), pairs0);
		i16x8 x0 = maddubs(r0, AB) + maddubs(r2, CD);
		i16x8 x1 = maddubs(r1, AB) + maddubs(r3, CD);
		i8x16 p = packus16(avg16(x0 >> 5, zero), avg16(x1 >> 5, zero));
		i64x2 q = {*(int64_t *)dst, *(int64_t *)(dst + dstride)};
		i16x8 x2 = shr16(adds16(maddubs(ziplo8(q, p), Wb), Ob), logWD);
		i16x8 x3 = shr16(adds16(maddubs(ziphi8(q, p), Wr), Or), logWD);
		i64x2 v = packus16(x2, x3);
		*(int64_t *)dst = v[0];
		*(int64_t *)(dst + dstride) = v[1];
		src += sstride * 2;
		dst += dstride * 2;
		r0 = r2, r1 = r3;
	} while (h -= 1);
}

static always_inline void inter4xH_chroma_8bit(int h, size_t dstride, uint8_t *dst, size_t sstride, const uint8_t *src, i8x16 AB, i8x16 CD, i8x16 W, i16x8 O, i64x2 logWD) {
	static const i8x16 shuf = {0, 1, 1, 2, 2, 3, 3, 4, 8, 9, 9, 10, 10, 11, 11, 12};
	i8x16 zero = {};
	i8x16 r0 = shuffle(ziplo64(load64(src), load64(src + sstride)), shuf);
	do {
		i8x16 r1 = shuffle(ziplo64(load64(src + sstride * 2), load64(src + sstride * 3)), shuf);
		i8x16 r2 = shuffle(ziplo64(load64(src + sstride * 4), load64(src + sstride * 5)), shuf);
		i16x8 x0 = maddubs(r0, AB) + maddubs(r1, CD);
		i16x8 x1 = maddubs(r1, AB) + maddubs(r2, CD);
		i8x16 p = packus16(avg16(x0 >> 5, zero), avg16(x1 >> 5, zero));
		i32x4 q = {*(int32_t *)dst, *(int32_t *)(dst + dstride), *(int32_t *)(dst + dstride * 2), *(int32_t *)(dst + dstride * 3)};
		i32x4 v = weight_add8(q, p, W, O, logWD);
		*(int32_t *)dst = v[0];
		*(int32_t *)(dst + dstride) = v[1];
		*(int32_t *)(dst + dstride * 2) = v[2];
		*(int32_t *)(dst + dstride * 3) = v[3];
		src += sstride * 4;
		dst += dstride * 4;
		r0 = r2;
	} while (h -= 2);
}

static always_inline void inter2xH_chroma_8bit(int h, size_t dstride, uint8_t *dst, size_t sstride, const uint8_t *src, i8x16 AB, i8x16 CD, i8x16 W, i16x8 O, i64x2 logWD) {
	static const i8x16 shuf = {0, 1, 1, 2, 4, 5, 5, 6, 8, 9, 9, 10, 12, 13, 13, 14};
	i8x16 zero = {};
	i8x16 r0 = shuffle(ziplo32(load32(src), load32(src + sstride)), shuf);
	do {
		i8x16 r1 = ziplo32(load32(src + sstride * 2), load32(src + sstride * 3));
		i8x16 r2 = ziplo32(load32(src + sstride * 4), load32(src + sstride * 5));
		i8x16 x0 = shuffle(ziplo64(r1, r2), shuf);
		i16x8 x1 = maddubs(ziplo64(r0, x0), AB) + maddubs(x0, CD);
		i8x16 p = packus16(avg16(x1 >> 5, zero), zero);
		i16x8 q = {*(int16_t *)dst, *(int16_t *)(dst + dstride), *(int16_t *)(dst + dstride * 2), *(int16_t *)(dst + dstride * 3)};
		i16x8 v = packus16(shr16(adds16(maddubs(ziplo8(q, p), W), O), logWD), zero);
		*(int16_t *)dst = v[0];
		*(int16_t *)(dst + dstride) = v[1];
		*(int16_t *)(dst + dstride * 2) = v[2];
		*(int16_t *)(dst + dstride * 3) = v[3];
		src += sstride * 4;
		dst += dstride * 4;
		r0 = shr128(x0, 8);
	} while (h -= 2);
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
	static void (*luma_fcts[48])(int, size_t, uint8_t*, size_t, const uint8_t*, i8x16, i16x8, i64x2) = {
		inter4xH_qpel00_8bit, inter4xH_qpel10_8bit, inter4xH_qpel20_8bit, inter4xH_qpel30_8bit,
		inter4xH_qpel01_8bit, inter4xH_qpel11_8bit, inter4xH_qpel21_8bit, inter4xH_qpel31_8bit,
		inter4xH_qpel02_8bit, inter4xH_qpel12_8bit, inter4xH_qpel22_8bit, inter4xH_qpel32_8bit,
		inter4xH_qpel03_8bit, inter4xH_qpel13_8bit, inter4xH_qpel23_8bit, inter4xH_qpel33_8bit,
		inter8xH_qpel00_8bit, inter8xH_qpel10_8bit, inter8xH_qpel20_8bit, inter8xH_qpel30_8bit,
		inter8xH_qpel01_8bit, inter8xH_qpel11_8bit, inter8xH_qpel21_8bit, inter8xH_qpel31_8bit,
		inter8xH_qpel02_8bit, inter8xH_qpel12_8bit, inter8xH_qpel22_8bit, inter8xH_qpel32_8bit,
		inter8xH_qpel03_8bit, inter8xH_qpel13_8bit, inter8xH_qpel23_8bit, inter8xH_qpel33_8bit,
		inter16xH_qpel00_8bit, inter16xH_qpel10_8bit, inter16xH_qpel20_8bit, inter16xH_qpel30_8bit,
		inter16xH_qpel01_8bit, inter16xH_qpel11_8bit, inter16xH_qpel21_8bit, inter16xH_qpel31_8bit,
		inter16xH_qpel02_8bit, inter16xH_qpel12_8bit, inter16xH_qpel22_8bit, inter16xH_qpel32_8bit,
		inter16xH_qpel03_8bit, inter16xH_qpel13_8bit, inter16xH_qpel23_8bit, inter16xH_qpel33_8bit
	};
	
	// load motion vector and reference picture
	int x = mb->mvs[i * 2];
	int y = mb->mvs[i * 2 + 1];
	int i8x8 = i >> 2;
	int i4x4 = i & 15;
	const uint8_t *ref = ctx->t.frame_buffers[mb->refPic[i8x8]];
	//printf("<tr><td colspan=2>CurrMbAddr=%d, i=%d, w=%d, h=%d, x=%d, y=%d, idx=%d, pic=%d</td></tr>\n", ctx->CurrMbAddr, i, w, h, x, y, mb->refIdx[i8x8], mb->refPic[i8x8]);
	
	// prediction weights {wY, wCb, wCr, oY, oCb, oCr, logWD_Y, logWD_C}
	i16x8 wod = {pack_w(0, 1), pack_w(0, 1), pack_w(0, 1), 0, 0, 0, 0, 0}; // no_weight
	int refIdx = mb->refIdx[i8x8];
	int refIdxX = mb->refIdx[i8x8 ^ 4];
	if (ctx->t.pps.weighted_bipred_idc != 1) {
		if (((i8x8 - 4) | refIdxX) >= 0) {
			if (ctx->t.pps.weighted_bipred_idc == 0) { // default2
				wod = (i16x8){257, 257, 257, 1, 1, 1, 1, 1};
			} else { // implicit2
				int w1 = ctx->implicit_weights[refIdxX][refIdx] - 64;
				int p = pack_w(64 - w1, w1);
				wod = (u16x8){p, p, p, 32, 32, 32, 6, 6};
				// w0 or w1 will overflow if w1 is 128 or -64 (WARNING untested in conformance bitstreams)
				if (__builtin_expect((unsigned)(w1 + 63) >= 191, 0)) {
					p = pack_w(2 - (w1 >> 5), w1 >> 5);
					wod = (u16x8){p, p, p, 1, 1, 1, 1, 1};
				}
			}
		}
	} else if (refIdxX < 0) { // explicit1
		refIdx += (i8x8 & 4) * 8;
		if (__builtin_expect(ctx->t.explicit_weights[0][refIdx] < 128, 1)) {
			wod[0] = pack_w(0, ctx->t.explicit_weights[0][refIdx]);
			wod[3] = (ctx->t.explicit_offsets[0][refIdx] * 2 + 1) << ctx->t.luma_log2_weight_denom >> 1;
			wod[6] = ctx->t.luma_log2_weight_denom;
		}
		if (__builtin_expect(ctx->t.explicit_weights[1][refIdx] < 128, 1)) {
			wod[1] = pack_w(0, ctx->t.explicit_weights[1][refIdx]);
			wod[2] = pack_w(0, ctx->t.explicit_weights[2][refIdx]);
			wod[4] = (ctx->t.explicit_offsets[1][refIdx] * 2 + 1) << ctx->t.chroma_log2_weight_denom >> 1;
			wod[5] = (ctx->t.explicit_offsets[2][refIdx] * 2 + 1) << ctx->t.chroma_log2_weight_denom >> 1;
			wod[7] = ctx->t.chroma_log2_weight_denom;
		}
	} else if (i8x8 >= 4) { // explicit2
		refIdx += 32;
		if (__builtin_expect((ctx->t.explicit_weights[0][refIdxX] & ctx->t.explicit_weights[0][refIdx]) != 128, 1)) {
			wod[0] = pack_w(ctx->t.explicit_weights[0][refIdxX], ctx->t.explicit_weights[0][refIdx]);
			wod[3] = ((ctx->t.explicit_offsets[0][refIdxX] + ctx->t.explicit_offsets[0][refIdx] + 1) | 1) << ctx->t.luma_log2_weight_denom;
			wod[6] = ctx->t.luma_log2_weight_denom + 1;
		} else {
			wod[0] = pack_w(ctx->t.explicit_weights[0][refIdxX] >> 1, ctx->t.explicit_weights[0][refIdx] >> 1);
			wod[3] = ((ctx->t.explicit_offsets[0][refIdxX] + ctx->t.explicit_offsets[0][refIdx] + 1) | 1) << ctx->t.luma_log2_weight_denom >> 1;
			wod[6] = ctx->t.luma_log2_weight_denom;
		}
		if (__builtin_expect((ctx->t.explicit_weights[1][refIdxX] & ctx->t.explicit_weights[1][refIdx]) != 128, 1)) {
			wod[1] = pack_w(ctx->t.explicit_weights[1][refIdxX], ctx->t.explicit_weights[1][refIdx]);
			wod[2] = pack_w(ctx->t.explicit_weights[2][refIdxX], ctx->t.explicit_weights[2][refIdx]);
			wod[4] = ((ctx->t.explicit_offsets[1][refIdxX] + ctx->t.explicit_offsets[1][refIdx] + 1) | 1) << ctx->t.chroma_log2_weight_denom;
			wod[5] = ((ctx->t.explicit_offsets[2][refIdxX] + ctx->t.explicit_offsets[2][refIdx] + 1) | 1) << ctx->t.chroma_log2_weight_denom;
			wod[7] = ctx->t.chroma_log2_weight_denom + 1;
		} else {
			wod[1] = pack_w(ctx->t.explicit_weights[1][refIdxX] >> 1, ctx->t.explicit_weights[1][refIdx] >> 1);
			wod[2] = pack_w(ctx->t.explicit_weights[2][refIdxX] >> 1, ctx->t.explicit_weights[2][refIdx] >> 1);
			wod[4] = ((ctx->t.explicit_offsets[1][refIdxX] + ctx->t.explicit_offsets[1][refIdx] + 1) | 1) << ctx->t.chroma_log2_weight_denom >> 1;
			wod[5] = ((ctx->t.explicit_offsets[2][refIdxX] + ctx->t.explicit_offsets[2][refIdx] + 1) | 1) << ctx->t.chroma_log2_weight_denom >> 1;
			wod[7] = ctx->t.chroma_log2_weight_denom;
		}
	}
	
	// compute source pointers
	int xInt_Y = ctx->mbx * 16 + x444[i4x4] + (x >> 2);
	int xInt_C = ctx->mbx * 8 + (x444[i4x4] >> 1) + (x >> 3);
	int yInt_Y = ctx->mby * 16 + y444[i4x4] + (y >> 2);
	int yInt_C = ctx->mby * 8 + (y444[i4x4] >> 1) + (y >> 3);
	const uint8_t *src_Y = ref + xInt_Y + yInt_Y * ctx->t.stride[0];
	const uint8_t *src_C = ref + xInt_C + yInt_C * ctx->t.stride[1] + ctx->t.plane_size_Y;
	size_t sstride_Y = ctx->t.stride[0];
	size_t sstride_C = ctx->t.stride[1] >> 1;
	
	// edge propagation is an annoying but beautiful piece of code
	int xWide = (x & 7) != 0;
	int yWide = (y & 7) != 0;
	int width_Y = ctx->t.pic_width_in_mbs * 16;
	if (__builtin_expect((unsigned)xInt_Y - xWide * 2 >= width_Y - w + 1 - xWide * 5 ||
		(unsigned)yInt_Y - yWide * 2 >= ctx->t.pic_height_in_mbs * 16 - h + 1 - yWide * 5, 0))
	{
		i8x16 shuf0 = load128(shift_Y_8bit + 15 + clip3(-15, 0, xInt_Y - 2) + clip3(0, 15, xInt_Y + 14 - width_Y));
		i8x16 shuf1 = load128(shift_Y_8bit + 15 + clip3(-15, 0, xInt_Y + 14) + clip3(0, 15, xInt_Y + 30 - width_Y));
		const uint8_t *src0 = ref + clip3(0, width_Y - 16, xInt_Y - 2);
		const uint8_t *src1 = ref + clip3(0, width_Y - 16, xInt_Y + 14);
		yInt_Y -= 2;
		for (i8x16 *buf = ctx->edge_buf_v; buf < ctx->edge_buf_v + 10 + h * 2; buf += 2, yInt_Y++) {
			int c = clip3(0, ctx->t.plane_size_Y - sstride_Y, yInt_Y * sstride_Y);
			buf[0] = shuffle(load128(src0 + c), shuf0);
			buf[1] = shuffle(load128(src1 + c), shuf1);
		}
		src_Y = ctx->edge_buf + 66;
		sstride_Y = 32;
		
		// chroma may read (and ignore) 1 bottom row and 1 right col out of bounds
		int width_C = width_Y >> 1;
		i8x16 shuf = load64(shift_C_8bit + 7 + clip3(-7, 0, xInt_C) + clip3(0, 7, xInt_C + 8 - width_C));
		src0 = ref + clip3(0, width_C - 8, xInt_C);
		src1 = ref + clip3(0, width_C - 1, xInt_C + 8);
		for (int j = 0; j <= h >> 1; j++, yInt_C++) {
			int cb = ctx->t.plane_size_Y + clip3(0, ctx->t.plane_size_C - sstride_C * 2, yInt_C * sstride_C * 2);
			int cr = sstride_C + cb;
			// reads are split in 2 to support 8px-wide frames
			ctx->edge_buf_l[j * 4 + 84] = ((i64x2)shuffle(load64(src0 + cb), shuf))[0];
			ctx->edge_buf[j * 32 + 680] = *(src1 + cb);
			ctx->edge_buf_l[j * 4 + 86] = ((i64x2)shuffle(load64(src0 + cr), shuf))[0];
			ctx->edge_buf[j * 32 + 696] = *(src1 + cr);
		}
		sstride_C = 16;
		src_C = ctx->edge_buf + 672;
	}
	
	// chroma prediction comes first since it is inlined
	uint8_t *dst_C = ctx->samples_mb[1] + (y444[i4x4] >> 1) * ctx->t.stride[1] + (x444[i4x4] >> 1);
	size_t dstride_C = ctx->t.stride[1] >> 1;
	int xFrac_C = x & 7;
	int yFrac_C = y & 7;
	int mul = pack_w(8 - xFrac_C, xFrac_C);
	i8x16 AB = set16(mul * (8 - yFrac_C));
	i8x16 CD = set16(mul * yFrac_C);
	if (w == 16) {
		inter8xH_chroma_8bit(h >> 1, dstride_C, dst_C, sstride_C, src_C, AB, CD, broadcast16(wod, 1), broadcast16(wod, 2), broadcast16(wod, 4), broadcast16(wod, 5), shr128(wod, 14));
	} else if (w == 8) {
		inter4xH_chroma_8bit(h >> 1, dstride_C, dst_C, sstride_C, src_C, AB, CD, shuffle32(shufflelo(wod, 1, 1, 2, 2), 0, 0, 1, 1), shuffle32(shufflehi(wod, 0, 0, 1, 1), 2, 2, 3, 3), shr128(wod, 14));
	} else {
		inter2xH_chroma_8bit(h >> 1, dstride_C, dst_C, sstride_C, src_C, AB, CD, shuffle32(shufflelo(wod, 1, 1, 2, 2), 0, 1, 0, 1), shuffle32(shufflehi(wod, 0, 0, 1, 1), 2, 3, 2, 3), shr128(wod, 14));
	}
	
	// tail jump to luma prediction
	int xFrac_Y = x & 3;
	int yFrac_Y = y & 3;
	size_t dstride_Y = ctx->t.stride[0];
	uint8_t *dst_Y = ctx->samples_mb[0] + y444[i4x4] * dstride_Y + x444[i4x4];
	luma_fcts[(w == 4 ? 0 : w == 8 ? 16 : 32) + yFrac_Y * 4 + xFrac_Y]
		(h, dstride_Y, dst_Y, sstride_Y, src_Y, broadcast16(wod, 0), broadcast16(wod, 3), shr128(shl128(wod, 2), 14));
}
