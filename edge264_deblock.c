#include "edge264_internal.h"

#if defined(__SSE2__)
	#define packabd16(a, b, c, d) abs8(packs16(subs16(a, b), subs16(c, d)))
	#define trnlo32(a) shuffle32(a, 0, 0, 2, 2)
#elif defined(__ARM_NEON)
	#define packabd16(a, b, c, d) vqmovn_high_u16(vqmovn_u16(vabdq_s16(a, b)), vabdq_s16(c, d))
	static always_inline i32x4 trnlo32(i32x4 a) {return vtrn1q_s32(a, a);}
#endif
static always_inline i8x16 expand4(int32_t a) {
	i32x4 x0 = {a};
	i8x16 x1 = ziplo8(x0, x0);
	return ziplo8(x1, x1);
}
static always_inline i8x16 expand2(int64_t a) {
	i64x2 x0 = {a};
	return ziplo8(x0, x0);
}



/**
 * Filter a single edge in place for bS in [0..3].
 * tC0 should equal -1 for each position where bS=0 or beta=0.
 * 
 * I must acknowledge that I looked a LOT at ffmpeg's filters while developing
 * these filters. They taught me many tricks to handle 9~10 bit operations on
 * 8 bit hardware, so I owe them a lot of time saved. Thanks!
 * 
 * For 8-pixel chroma edges we filter both Cb and Cr in lower and upper halves
 * of registers. In these cases note that alpha and beta may contain zero and
 * non-zero values, so we cannot use alpha-1 or beta-1.
 */
#if defined(__SSE2__)
	#define DEBLOCK_LUMA_SOFT(p2, p1, p0, q0, q1, q2, ialpha, ibeta, itC0) {\
		/* compute the opposite of filterSamplesFlags as a mask, and transfer the mask bS=0 from tC0 */\
		i8x16 pq0 = subu8(p0, q0);\
		i8x16 qp0 = subu8(q0, p0);\
		i8x16 sub0 = subu8(addu8(qp0, set8(-128)), pq0); /* save 128+q0-p0 for later */\
		i8x16 abs0 = pq0 | qp0;\
		i8x16 abs1 = subu8(p1, p0) | subu8(p0, p1);\
		i8x16 abs2 = subu8(q1, q0) | subu8(q0, q1);\
		i8x16 beta = set8(ibeta);\
		i8x16 and = minu8(subu8(set8(ialpha), abs0), subu8(beta, maxu8(abs1, abs2)));\
		i8x16 tC0 = expand4(itC0);\
		i8x16 ignoreSamplesFlags = (and == 0) | (tC0 < 0);\
		i8x16 ftC0 = tC0 & ~ignoreSamplesFlags;\
		/* filter p1 and q1 (same as ffmpeg, I couldn't find better) */\
		i8x16 c1 = set8(1);\
		i8x16 x0 = avg8(p0, q0); /* (p0+q0+1)>>1 */\
		i8x16 x1 = avg8(p2, x0) - ((p2 ^ x0) & c1); /* (p2+((p0+q0+1)>>1))>>1 */\
		i8x16 x2 = avg8(q2, x0) - ((q2 ^ x0) & c1); /* (q2+((p0+q0+1)>>1))>>1 */\
		i8x16 pp1 = minu8(maxu8(x1, subu8(p1, ftC0)), addu8(p1, ftC0));\
		i8x16 qp1 = minu8(maxu8(x2, subu8(q1, ftC0)), addu8(q1, ftC0));\
		i8x16 cm1 = set8(-1);\
		i8x16 bm1 = (i8x16)beta + cm1;\
		i8x16 sub1 = avg8(p1, q1 ^ cm1); /* save 128+((p1-q1)>>1) for later */\
		i8x16 apltb = subu8(subu8(p2, p0), bm1) == subu8(subu8(p0, p2), bm1);\
		i8x16 aqltb = subu8(subu8(q2, q0), bm1) == subu8(subu8(q0, q2), bm1);\
		p1 = ifelse_mask(apltb, pp1, p1);\
		q1 = ifelse_mask(aqltb, qp1, q1);\
		/* filter p0 and q0 (by offsetting signed to unsigned to apply pavg) */\
		i8x16 ftC = (ftC0 - apltb - aqltb) & ~ignoreSamplesFlags;\
		i8x16 x3 = avg8(sub0, avg8(sub1, set8(127))); /* 128+((q0-p0+((p1-q1)>>2)+1)>>1) */\
		i8x16 c128 = set8(-128);\
		i8x16 delta = minu8(subu8(x3, c128), ftC); /* delta if delta>0 */\
		i8x16 ndelta = minu8(subu8(c128, x3), ftC); /* -delta if delta<0 */\
		p0 = subu8(addu8(p0, delta), ndelta);\
		q0 = subu8(addu8(q0, ndelta), delta);}
	#define DEBLOCK_CHROMA_SOFT(p1, p0, q0, q1, ialpha, ibeta, itC0) {\
		/* compute the opposite of filterSamplesFlags and apply if to tC */\
		i8x16 c128 = set8(-128);\
		i8x16 pq0 = subu8(p0, q0);\
		i8x16 qp0 = subu8(q0, p0);\
		i8x16 sub0 = subu8(addu8(qp0, c128), pq0); /* save 128+q0-p0 for later */\
		i8x16 abs0 = pq0 | qp0;\
		i8x16 abs1 = subu8(p1, p0) | subu8(p0, p1);\
		i8x16 abs2 = subu8(q1, q0) | subu8(q0, q1);\
		i8x16 shufab = {1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2};\
		i8x16 alpha = shuffle((i32x4){ialpha}, shufab);\
		i8x16 beta = shuffle((i32x4){ibeta}, shufab);\
		i8x16 and = minu8(subu8(alpha, abs0), subu8(beta, maxu8(abs1, abs2)));\
		i8x16 ignoreSamplesFlags = and == 0;\
		i8x16 cm1 = set8(-1);\
		i8x16 ftC = (expand2(itC0) - cm1) & ~ignoreSamplesFlags;\
		/* filter p0 and q0 (by offsetting signed to unsigned to apply pavg) */\
		i8x16 sub1 = avg8(p1, q1 ^ cm1); /* 128+((p1-q1)>>1) */\
		i8x16 x3 = avg8(sub0, avg8(sub1, set8(127))); /* 128+((q0-p0+((p1-q1)>>2)+1)>>1) */\
		i8x16 delta = minu8(subu8(x3, c128), ftC); /* delta if delta>0 */\
		i8x16 ndelta = minu8(subu8(c128, x3), ftC); /* -delta if delta<0 */\
		p0 = subu8(addu8(p0, delta), ndelta);\
		q0 = subu8(addu8(q0, ndelta), delta);}
#elif defined(__ARM_NEON)
	#define DEBLOCK_LUMA_SOFT(p2, p1, p0, q0, q1, q2, ialpha, ibeta, itC0) {\
		/* compute all common masks */\
		u8x16 alpha = set8(ialpha);\
		u8x16 beta = set8(ibeta);\
		i8x16 tC0 = expand4(itC0);\
		u8x16 abs0 = vabdq_u8(p0, q0);\
		u8x16 abs1 = vmaxq_u8(vabdq_u8(p1, p0), vabdq_u8(q1, q0));\
		i8x16 filterSamplesFlag = (tC0 >= 0) & (abs0 < alpha) & (abs1 < beta);\
		i8x16 ftC0 = tC0 & filterSamplesFlag;\
		i8x16 apltb = (u8x16)vabdq_u8(p2, p0) < beta;\
		i8x16 aqltb = (u8x16)vabdq_u8(q2, q0) < beta;\
		/* filter p1 and q1 (same as ffmpeg, I couldn't find better) */\
		u8x16 pq0 = vrhaddq_u8(p0, q0);\
		i8x16 pq1 = vhsubq_u8(p1, q1); /* save (p1+q1)>>1 for later */\
		u8x16 lop1 = vqsubq_u8(p1, ftC0);\
		u8x16 hip1 = vqaddq_u8(p1, ftC0);\
		u8x16 loq1 = vqsubq_u8(q1, ftC0);\
		u8x16 hiq1 = vqaddq_u8(q1, ftC0);\
		u8x16 pp1 = vminq_u8(vmaxq_u8(vhaddq_u8(p2, pq0), lop1), hip1);\
		u8x16 qp1 = vminq_u8(vmaxq_u8(vhaddq_u8(q2, pq0), loq1), hiq1);\
		p1 = ifelse_mask(apltb, pp1, p1);\
		q1 = ifelse_mask(aqltb, qp1, q1);\
		/* filter p0 and q0 (by offsetting unsigned to signed to properly saturate q0-p0) */\
		i8x16 tC = (ftC0 - apltb - aqltb) & filterSamplesFlag;\
		i8x16 v0 = vrhaddq_s8(vqsubq_s8(q0 + -128, p0 + -128), pq1 >> 1);\
		i8x16 delta = vmaxq_s8(vminq_s8(v0, tC), -tC);\
		p0 = vsqaddq_u8(p0, delta);\
		q0 = vsqaddq_u8(q0, -delta);}
	#define DEBLOCK_CHROMA_SOFT(p1, p0, q0, q1, ialpha, ibeta, itC0) {\
		/* compute all common masks */\
		i8x16 shufab = {1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2};\
		i8x16 alpha = shuffle((i32x4){ialpha}, shufab);\
		i8x16 beta = shuffle((i32x4){ibeta}, shufab);\
		i8x16 tC0 = expand2(itC0);\
		u8x16 abs0 = vabdq_u8(p0, q0);\
		u8x16 abs1 = vmaxq_u8(vabdq_u8(p1, p0), vabdq_u8(q1, q0));\
		i8x16 filterSamplesFlag = (tC0 >= 0) & (abs0 < alpha) & (abs1 < beta);\
		/* filter p0 and q0 (by offsetting unsigned to signed to properly saturate q0-p0) */\
		i8x16 c128 = set8(-128);\
		i8x16 pq1 = vhsubq_u8(p1, q1);\
		i8x16 tC = (tC0 - -1) & filterSamplesFlag;\
		i8x16 v0 = vrhaddq_s8(vqsubq_s8(q0 + c128, p0 + c128), pq1 >> 1);\
		i8x16 delta = vmaxq_s8(vminq_s8(v0, tC), -tC);\
		p0 = vsqaddq_u8(p0, delta);\
		q0 = vsqaddq_u8(q0, -delta);}
#endif



/**
 * Filter a single edge in place for bS=4.
 * 
 * The luma filter uses beta-1 thus should not be used with beta=0.
 */
#if defined(__SSE2__)
	#define DEBLOCK_LUMA_HARD(p3, p2, p1, p0, q0, q1, q2, q3, ialpha, ibeta) {\
		/* compute the opposite of filterSamplesFlags, and condition masks for filtering modes */\
		i8x16 alpha = set8(ialpha);\
		i8x16 beta = set8(ibeta);\
		i8x16 abs0 = subu8(p0, q0) | subu8(q0, p0);\
		i8x16 abs1 = subu8(p1, p0) | subu8(p0, p1);\
		i8x16 abs2 = subu8(q1, q0) | subu8(q0, q1);\
		i8x16 ignoreSamplesFlags = minu8(subu8(alpha, abs0), subu8(beta, maxu8(abs1, abs2))) == 0;\
		i8x16 c1 = set8(1);\
		i8x16 zero = {};\
		i8x16 condpq = subu8(abs0, avg8(avg8(alpha, c1), zero)); /* abs0-((alpha>>2)+1) */\
		i8x16 bm1 = (i8x16)beta - c1;\
		i8x16 condp = (subu8(subu8(p2, p0) | subu8(p0, p2), bm1) | condpq) == zero;\
		i8x16 condq = (subu8(subu8(q2, q0) | subu8(q0, q2), bm1) | condpq) == zero;\
		/* compute p'0 and q'0 */\
		i8x16 fix0 = (p0 ^ q0) & c1;\
		i8x16 pq0 = avg8(p0, q0) - fix0;\
		i8x16 and0 = fix0 ^ c1;\
		i8x16 p2q1 = avg8(p2, q1) - ((p2 ^ q1) & c1); /* (p2+q1)/2 */\
		i8x16 q2p1 = avg8(q2, p1) - ((q2 ^ p1) & c1); /* (q2+p1)/2 */\
		i8x16 p21q1 = avg8(p2q1, p1) - ((p2q1 ^ p1) & and0); /* p21q1+pq0 == (p2q1+p1+p0+q0)/2 */\
		i8x16 q21p1 = avg8(q2p1, q1) - ((q2p1 ^ q1) & and0); /* q21p1+pq0 == (q2p1+q1+p0+q0)/2 */\
		i8x16 pp0a = avg8(p21q1, pq0); /* p'0 (first formula) */\
		i8x16 qp0a = avg8(q21p1, pq0); /* q'0 (first formula) */\
		i8x16 pp0b = avg8(p1, avg8(p0, q1) - ((p0 ^ q1) & c1)); /* p'0 (second formula) */\
		i8x16 qp0b = avg8(q1, avg8(q0, p1) - ((q0 ^ p1) & c1)); /* q'0 (second formula) */\
		p0 = ifelse_mask(ignoreSamplesFlags, p0, ifelse_mask(condp, pp0a, pp0b));\
		q0 = ifelse_mask(ignoreSamplesFlags, q0, ifelse_mask(condq, qp0a, qp0b));\
		/* compute p'1 and q'1 */\
		i8x16 fcondp = condp & ~ignoreSamplesFlags;\
		i8x16 fcondq = condq & ~ignoreSamplesFlags;\
		i8x16 p21 = avg8(p2, p1) - ((p2 ^ p1) & and0); /* p21+pq0 == (p2+p1+p0+q0)/2 */\
		i8x16 q21 = avg8(q2, q1) - ((q2 ^ q1) & and0); /* q21+pq0 == (q2+q1+q0+p1)/2 */\
		i8x16 pp1 = avg8(p21, pq0); /* p'1 */\
		i8x16 qp1 = avg8(q21, pq0); /* q'1 */\
		p1 = ifelse_mask(fcondp, pp1, p1);\
		q1 = ifelse_mask(fcondq, qp1, q1);\
		/* compute p'2 and q'2 */\
		i8x16 fix1 = ((p21 ^ pq0) & c1);\
		i8x16 fix2 = ((q21 ^ pq0) & c1);\
		i8x16 p210q0 = pp1 - fix1; /* (p2+p1+p0+q0)/4 */\
		i8x16 q210p0 = qp1 - fix2; /* (q2+q1+q0+p0)/4 */\
		i8x16 p3p2 = avg8(p3, p2) - ((p3 ^ p2) & (fix1 ^ c1)); /* p3p2+p210q0 == (p3+p2+(p2+p1+p0+q0)/2)/2 */\
		i8x16 q3q2 = avg8(q3, q2) - ((q3 ^ q2) & (fix2 ^ c1)); /* q3q2+q210p0 == (q3+q2+(q2+q1+p0+q0)/2)/2 */\
		i8x16 pp2 = avg8(p3p2, p210q0); /* p'2 */\
		i8x16 qp2 = avg8(q3q2, q210p0); /* q'2 */\
		p2 = ifelse_mask(fcondp, pp2, p2);\
		q2 = ifelse_mask(fcondq, qp2, q2);}
	#define DEBLOCK_CHROMA_HARD(p1, p0, q0, q1, ialpha, ibeta) {\
		/* compute the opposite of filterSamplesFlags */\
		i8x16 abs0 = subu8(p0, q0) | subu8(q0, p0);\
		i8x16 abs1 = subu8(p1, p0) | subu8(p0, p1);\
		i8x16 abs2 = subu8(q1, q0) | subu8(q0, q1);\
		i8x16 shufab = {1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2};\
		i8x16 alpha = shuffle((i32x4){ialpha}, shufab);\
		i8x16 beta = shuffle((i32x4){ibeta}, shufab);\
		i8x16 and = minu8(subu8(alpha, abs0), subu8(beta, maxu8(abs1, abs2)));\
		i8x16 ignoreSamplesFlags = and == 0;\
		/* compute p'0 and q'0 */\
		i8x16 c1 = set8(1);\
		i8x16 pp0b = avg8(p1, avg8(p0, q1) - ((p0 ^ q1) & c1)); /* p'0 (second formula) */\
		i8x16 qp0b = avg8(q1, avg8(q0, p1) - ((q0 ^ p1) & c1)); /* q'0 (second formula) */\
		p0 = ifelse_mask(ignoreSamplesFlags, p0, pp0b);\
		q0 = ifelse_mask(ignoreSamplesFlags, q0, qp0b);}
#elif defined(__ARM_NEON)
	#define DEBLOCK_LUMA_HARD(p3, p2, p1, p0, q0, q1, q2, q3, ialpha, ibeta) {\
		/* common masks */\
		u8x16 alpha = set8(ialpha);\
		u8x16 beta = set8(ibeta);\
		u8x16 abs0 = vabdq_u8(p0, q0);\
		u8x16 abs1 = vmaxq_u8(vabdq_u8(p1, p0), vabdq_u8(q1, q0));\
		i8x16 filterSamplesFlag = (abs0 < alpha) & (abs1 < beta);\
		i8x16 condpq = abs0 < (alpha >> 2) + 2;\
		i8x16 condp = ((u8x16)vabdq_u8(p2, p0) < beta) & condpq;\
		i8x16 condq = ((u8x16)vabdq_u8(q2, q0) < beta) & condpq;\
		/* common wide sums */\
		u16x8 p10l = vaddl_u8(vget_low_u8(p1), vget_low_u8(p0));\
		u16x8 p10h = vaddl_high_u8(p1, p0);\
		u16x8 q01l = vaddl_u8(vget_low_u8(q0), vget_low_u8(q1));\
		u16x8 q01h = vaddl_high_u8(q0, q1);\
		u16x8 p210q0l = p10l + (u16x8)vaddl_u8(vget_low_u8(p2), vget_low_u8(q0));\
		u16x8 p210q0h = p10h + (u16x8)vaddl_high_u8(p2, q0);\
		u16x8 p0q012l = q01l + (u16x8)vaddl_u8(vget_low_u8(p0), vget_low_u8(q2));\
		u16x8 p0q012h = q01h + (u16x8)vaddl_high_u8(p0, q2);\
		u16x8 p10q01l = p10l + q01l;\
		u16x8 p10q01h = p10h + q01h;\
		/* compute p'0 and q'0 */\
		u8x16 pp0a = shrrpu16(p210q0l + p10q01l, p210q0h + p10q01h, 3); /* p'0 (first formula) */\
		u8x16 qp0a = shrrpu16(p10q01l + p0q012l, p10q01h + p0q012h, 3); /* q'0 (first formula) */\
		u8x16 pp0b = vrhaddq_u8(p1, vhaddq_u8(p0, q1)); /* p'0 (second formula) */\
		u8x16 qp0b = vrhaddq_u8(q1, vhaddq_u8(q0, p1)); /* q'0 (second formula) */\
		p0 = ifelse_mask(filterSamplesFlag, ifelse_mask(condp, pp0a, pp0b), p0);\
		q0 = ifelse_mask(filterSamplesFlag, ifelse_mask(condq, qp0a, qp0b), q0);\
		/* compute p'1 and q'1 */\
		i8x16 fcondp = condp & filterSamplesFlag;\
		i8x16 fcondq = condq & filterSamplesFlag;\
		p1 = ifelse_mask(fcondp, shrrpu16(p210q0l, p210q0h, 2), p1);\
		q1 = ifelse_mask(fcondq, shrrpu16(p0q012l, p0q012h, 2), q1);\
		/* compute p'2 and q'2 */\
		u16x8 p32l = vaddl_u8(vget_low_u8(p3), vget_low_u8(p2));\
		u16x8 p32h = vaddl_high_u8(p3, p2);\
		u16x8 q23l = vaddl_u8(vget_low_u8(q2), vget_low_u8(q3));\
		u16x8 q23h = vaddl_high_u8(q2, q3);\
		p2 = ifelse_mask(fcondp, shrrpu16(p32l + p210q0l, p32h + p210q0h, 3), p2);\
		q2 = ifelse_mask(fcondq, shrrpu16(q23l + p0q012l, q23h + p0q012h, 3), q2);}
	#define DEBLOCK_CHROMA_HARD(p1, p0, q0, q1, ialpha, ibeta) {\
		i8x16 shufab = {1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2};\
		i8x16 alpha = shuffle((i32x4){ialpha}, shufab);\
		i8x16 beta = shuffle((i32x4){ibeta}, shufab);\
		u8x16 abs0 = vabdq_u8(p0, q0);\
		u8x16 abs1 = vmaxq_u8(vabdq_u8(p1, p0), vabdq_u8(q1, q0));\
		i8x16 filterSamplesFlag = (abs0 < alpha) & (abs1 < beta);\
		p0 = ifelse_mask(filterSamplesFlag, vrhaddq_u8(p1, vhaddq_u8(p0, q1)), p0);\
		q0 = ifelse_mask(filterSamplesFlag, vrhaddq_u8(q1, vhaddq_u8(q0, p1)), q0);}
#endif



/**
 * Efficient addressing for both Intel and ARM architectures.
 * Call INIT_PX(p, stride) once to compute anchor addresses, then PX(x,y) will
 * return p + x + y * stride.
 */
#if defined(__SSE2__)
	#define INIT_PX(p, stride) size_t _stride = (stride); ssize_t _nstride = -_stride; uint8_t * restrict _p0 = (p), * restrict _p7 = _p0 + _stride * 8 + _nstride, * restrict _pE = _p7 + _stride * 8 + _nstride
	#define PX(x, y) (__builtin_choose_expr(y <= 2 || y == 4, _p0, __builtin_choose_expr(y <= 9 || y == 11, _p7, _pE)) +\
		__builtin_choose_expr(y < 0, x - y * _nstride, __builtin_choose_expr(y == 0 || y == 7 || y == 14, x, __builtin_choose_expr(y == 1 || y == 8 || y == 15, x + _stride, __builtin_choose_expr(y == 2 || y == 9, x + _stride * 2, __builtin_choose_expr(y == 3 || y == 10, x + _nstride * 4, __builtin_choose_expr(y == 4 || y == 11, x + _stride * 4, __builtin_choose_expr(y == 5 || y == 12, x + _nstride * 2, x + _nstride))))))))
#elif defined(__ARM_NEON)
	#define INIT_PX(p, stride) size_t _stride = (stride), _stridem2 = _stride - 2, _stridem4 = _stride - 4, _stridem8 = _stride - 8; uint8_t * restrict _p0 = (p), * restrict _p2 = _p0 + _stride * 2, * restrict _p4 = _p0 + _stride * 4, * restrict _p6 = _p2 + _stride * 4, * restrict _p8 = _p0 + _stride * 8, * restrict _pA = _p2 + _stride * 8, * restrict _pC = _p4 + _stride * 8, * restrict _pE = _p6 + _stride * 8
	#define PX(x, y) (__builtin_choose_expr(y < 2, _p0, __builtin_choose_expr(y < 4, _p2, __builtin_choose_expr(y < 6, _p4, __builtin_choose_expr(y < 8, _p6, __builtin_choose_expr(y < 10, _p8, __builtin_choose_expr(y < 12, _pA, __builtin_choose_expr(y < 14, _pC, _pE))))))) +\
		__builtin_choose_expr(y < 0 || y > 15, x + y * _stride, __builtin_choose_expr((y & 1) == 0, x, __builtin_choose_expr(x == 0, _stride, __builtin_choose_expr(x == -2, _stridem2, __builtin_choose_expr(x == -4, _stridem4, __builtin_choose_expr(x == -8, _stridem8, x + _stride)))))))
#endif



/**
 * Deblock both chroma planes of the current macroblock in place.
 */
static void deblock_CbCr_8bit(Edge264Context *ctx) {
	INIT_PX(ctx->samples_mb[1], ctx->t.stride[1] >> 1);
	i8x16 v0, v1, v2, v3, v4, v5, v6, v7;
	if (mb->f.filter_edges & 1) {
		// load and transpose both 12x8 matrices (with left macroblock)
		i8x16 xa0 = load128(PX(-8, 0));
		i8x16 xa8 = load128(PX(-8, 1));
		i8x16 xa1 = load128(PX(-8, 2));
		i8x16 xa9 = load128(PX(-8, 3));
		i8x16 xa2 = load128(PX(-8, 4));
		i8x16 xaA = load128(PX(-8, 5));
		i8x16 xa3 = load128(PX(-8, 6));
		i8x16 xaB = load128(PX(-8, 7));
		i8x16 xb0 = ziplo8(xa0, xa1);
		i8x16 xb1 = ziphi8(xa0, xa1);
		i8x16 xb2 = ziplo8(xa2, xa3);
		i8x16 xb3 = ziphi8(xa2, xa3);
		i8x16 xb8 = ziplo8(xa8, xa9);
		i8x16 xb9 = ziphi8(xa8, xa9);
		i8x16 xbA = ziplo8(xaA, xaB);
		i8x16 xbB = ziphi8(xaA, xaB);
		i8x16 xc0 = ziphi16(xb0, xb2);
		i8x16 xc1 = ziplo16(xb1, xb3);
		i8x16 xc2 = ziphi16(xb1, xb3);
		i8x16 xc6 = ziphi16(xb8, xbA);
		i8x16 xc7 = ziplo16(xb9, xbB);
		i8x16 xc8 = ziphi16(xb9, xbB);
		i8x16 xa4 = load128(PX(-8, 8));
		i8x16 xaC = load128(PX(-8, 9));
		i8x16 xa5 = load128(PX(-8, 10));
		i8x16 xaD = load128(PX(-8, 11));
		i8x16 xa6 = load128(PX(-8, 12));
		i8x16 xaE = load128(PX(-8, 13));
		i8x16 xa7 = load128(PX(-8, 14));
		i8x16 xaF = load128(PX(-8, 15));
		i8x16 xb4 = ziplo8(xa4, xa5);
		i8x16 xb5 = ziphi8(xa4, xa5);
		i8x16 xb6 = ziplo8(xa6, xa7);
		i8x16 xb7 = ziphi8(xa6, xa7);
		i8x16 xc3 = ziphi16(xb4, xb6);
		i8x16 xc4 = ziplo16(xb5, xb7);
		i8x16 xc5 = ziphi16(xb5, xb7);
		i8x16 xbC = ziplo8(xaC, xaD);
		i8x16 xbD = ziphi8(xaC, xaD);
		i8x16 xbE = ziplo8(xaE, xaF);
		i8x16 xbF = ziphi8(xaE, xaF);
		i8x16 xc9 = ziphi16(xbC, xbE);
		i8x16 xcA = ziplo16(xbD, xbF);
		i8x16 xcB = ziphi16(xbD, xbF);
		i8x16 xd0 = ziphi32(xc0, xc3);
		i8x16 xd1 = ziplo32(xc1, xc4);
		i8x16 xd2 = ziphi32(xc1, xc4);
		i8x16 xd3 = ziplo32(xc2, xc5);
		i8x16 xd4 = ziphi32(xc2, xc5);
		i8x16 xd5 = ziphi32(xc6, xc9);
		i8x16 xd6 = ziplo32(xc7, xcA);
		i8x16 xd7 = ziphi32(xc7, xcA);
		i8x16 xd8 = ziplo32(xc8, xcB);
		i8x16 xd9 = ziphi32(xc8, xcB);
		i8x16 vY = ziplo64(xd0, xd5);
		i8x16 vZ = ziphi64(xd0, xd5);
		v0 = ziplo64(xd1, xd6);
		v1 = ziphi64(xd1, xd6);
		v2 = ziplo64(xd2, xd7);
		v3 = ziphi64(xd2, xd7);
		v4 = ziplo64(xd3, xd8);
		v5 = ziphi64(xd3, xd8);
		v6 = ziplo64(xd4, xd9);
		v7 = ziphi64(xd4, xd9);
		
		// first vertical edge
		if (mbA->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			int64_t tC0a = ctx->tC0_l[4];
			if (tC0a != -1)
				DEBLOCK_CHROMA_SOFT(vY, vZ, v0, v1, ctx->alpha_s[2], ctx->beta_s[2], tC0a);
		} else {
			DEBLOCK_CHROMA_HARD(vY, vZ, v0, v1, ctx->alpha_s[2], ctx->beta_s[2]);
		}
		
		// store vY/vZ into the left macroblock
		i16x8 xf0 = ziplo8(vY, vZ);
		i16x8 xf1 = ziphi8(vY, vZ);
		*(int16_t *)PX(-2, 0) = xf0[0];
		*(int16_t *)PX(-2, 1) = xf1[0];
		*(int16_t *)PX(-2, 2) = xf0[1];
		*(int16_t *)PX(-2, 3) = xf1[1];
		*(int16_t *)PX(-2, 4) = xf0[2];
		*(int16_t *)PX(-2, 5) = xf1[2];
		*(int16_t *)PX(-2, 6) = xf0[3];
		*(int16_t *)PX(-2, 7) = xf1[3];
		*(int16_t *)PX(-2, 8) = xf0[4];
		*(int16_t *)PX(-2, 9) = xf1[4];
		*(int16_t *)PX(-2, 10) = xf0[5];
		*(int16_t *)PX(-2, 11) = xf1[5];
		*(int16_t *)PX(-2, 12) = xf0[6];
		*(int16_t *)PX(-2, 13) = xf1[6];
		*(int16_t *)PX(-2, 14) = xf0[7];
		*(int16_t *)PX(-2, 15) = xf1[7];
	} else {
		// load and transpose both 8x8 matrices
		i8x16 xa0 = load64(PX(0, 0));
		i8x16 xa8 = load64(PX(0, 1));
		i8x16 xa1 = load64(PX(0, 2));
		i8x16 xa9 = load64(PX(0, 3));
		i8x16 xa2 = load64(PX(0, 4));
		i8x16 xaA = load64(PX(0, 5));
		i8x16 xa3 = load64(PX(0, 6));
		i8x16 xaB = load64(PX(0, 7));
		i8x16 xb0 = ziplo8(xa0, xa1);
		i8x16 xb1 = ziplo8(xa2, xa3);
		i8x16 xb4 = ziplo8(xa8, xa9);
		i8x16 xb5 = ziplo8(xaA, xaB);
		i8x16 xc0 = ziplo16(xb0, xb1);
		i8x16 xc1 = ziphi16(xb0, xb1);
		i8x16 xc4 = ziplo16(xb4, xb5);
		i8x16 xc5 = ziphi16(xb4, xb5);
		i8x16 xa4 = load64(PX(0, 8));
		i8x16 xaC = load64(PX(0, 9));
		i8x16 xa5 = load64(PX(0, 10));
		i8x16 xaD = load64(PX(0, 11));
		i8x16 xa6 = load64(PX(0, 12));
		i8x16 xaE = load64(PX(0, 13));
		i8x16 xa7 = load64(PX(0, 14));
		i8x16 xaF = load64(PX(0, 15));
		i8x16 xb2 = ziplo8(xa4, xa5);
		i8x16 xb3 = ziplo8(xa6, xa7);
		i8x16 xb6 = ziplo8(xaC, xaD);
		i8x16 xb7 = ziplo8(xaE, xaF);
		i8x16 xc2 = ziplo16(xb2, xb3);
		i8x16 xc3 = ziphi16(xb2, xb3);
		i8x16 xc6 = ziplo16(xb6, xb7);
		i8x16 xc7 = ziphi16(xb6, xb7);
		i8x16 xd0 = ziplo32(xc0, xc2);
		i8x16 xd1 = ziphi32(xc0, xc2);
		i8x16 xd2 = ziplo32(xc1, xc3);
		i8x16 xd3 = ziphi32(xc1, xc3);
		i8x16 xd4 = ziplo32(xc4, xc6);
		i8x16 xd5 = ziphi32(xc4, xc6);
		i8x16 xd6 = ziplo32(xc5, xc7);
		i8x16 xd7 = ziphi32(xc5, xc7);
		v0 = ziplo64(xd0, xd4);
		v1 = ziphi64(xd0, xd4);
		v2 = ziplo64(xd1, xd5);
		v3 = ziphi64(xd1, xd5);
		v4 = ziplo64(xd2, xd6);
		v5 = ziphi64(xd2, xd6);
		v6 = ziplo64(xd3, xd7);
		v7 = ziphi64(xd3, xd7);
	}
	
	// second vertical edge
	int64_t tC0c = ctx->tC0_l[5];
	if (tC0c != -1)
		DEBLOCK_CHROMA_SOFT(v2, v3, v4, v5, ctx->alpha_s[0], ctx->beta_s[0], tC0c);
	
	// transpose both 8x8 matrices
	i8x16 xa0 = ziplo8(v0, v1);
	i8x16 xa1 = ziphi8(v0, v1);
	i8x16 xa2 = ziplo8(v2, v3);
	i8x16 xa3 = ziphi8(v2, v3);
	i8x16 xa4 = ziplo8(v4, v5);
	i8x16 xa5 = ziphi8(v4, v5);
	i8x16 xa6 = ziplo8(v6, v7);
	i8x16 xa7 = ziphi8(v6, v7);
	i8x16 xb0 = ziplo16(xa0, xa2);
	i8x16 xb1 = ziphi16(xa0, xa2);
	i8x16 xb2 = ziplo16(xa1, xa3);
	i8x16 xb3 = ziphi16(xa1, xa3);
	i8x16 xb4 = ziplo16(xa4, xa6);
	i8x16 xb5 = ziphi16(xa4, xa6);
	i8x16 xb6 = ziplo16(xa5, xa7);
	i8x16 xb7 = ziphi16(xa5, xa7);
	i8x16 xc0 = ziplo32(xb0, xb4);
	i8x16 xc1 = ziphi32(xb0, xb4);
	i8x16 xc2 = ziplo32(xb1, xb5);
	i8x16 xc3 = ziphi32(xb1, xb5);
	i8x16 xc4 = ziplo32(xb2, xb6);
	i8x16 xc5 = ziphi32(xb2, xb6);
	i8x16 xc6 = ziplo32(xb3, xb7);
	i8x16 xc7 = ziphi32(xb3, xb7);
	i8x16 h0 = ziplo64(xc0, xc4);
	i8x16 h1 = ziphi64(xc0, xc4);
	i8x16 h2 = ziplo64(xc1, xc5);
	i8x16 h3 = ziphi64(xc1, xc5);
	i8x16 h4 = ziplo64(xc2, xc6);
	i8x16 h5 = ziphi64(xc2, xc6);
	i8x16 h6 = ziplo64(xc3, xc7);
	i8x16 h7 = ziphi64(xc3, xc7);
	
	// first horizontal edge
	if (mb->f.filter_edges & 2) {
		i8x16 hY = (i64x2){*(int64_t *)(PX(0, -4)), *(int64_t *)(PX(0, -3))};
		i8x16 hZ = (i64x2){*(int64_t *)(PX(0, -2)), *(int64_t *)(PX(0, -1))};
		if (mbB->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			int64_t tC0e = ctx->tC0_l[6];
			if (tC0e != -1)
				DEBLOCK_CHROMA_SOFT(hY, hZ, h0, h1, ctx->alpha_s[3], ctx->beta_s[3], tC0e);
		} else {
			DEBLOCK_CHROMA_HARD(hY, hZ, h0, h1, ctx->alpha_s[3], ctx->beta_s[3]);
		}
		*(int64_t *)PX(0, -2) = ((i64x2)hZ)[0];
		*(int64_t *)PX(0, -1) = ((i64x2)hZ)[1];
	}
	mb->f.filter_edges = 0; // prevent redundant deblocking with deblock_idc==2 and ASO
	*(int64_t *)PX(0, 0) = ((i64x2)h0)[0];
	*(int64_t *)PX(0, 1) = ((i64x2)h0)[1];
	*(int64_t *)PX(0, 2) = ((i64x2)h1)[0];
	*(int64_t *)PX(0, 3) = ((i64x2)h1)[1];
	
	// second horizontal edge
	int64_t tC0g = ctx->tC0_l[7];
	if (tC0g != -1)
		DEBLOCK_CHROMA_SOFT(h2, h3, h4, h5, ctx->alpha_s[0], ctx->beta_s[0], tC0g);
	*(int64_t *)PX(0, 4) = ((i64x2)h2)[0];
	*(int64_t *)PX(0, 5) = ((i64x2)h2)[1];
	*(int64_t *)PX(0, 6) = ((i64x2)h3)[0];
	*(int64_t *)PX(0, 7) = ((i64x2)h3)[1];
	*(int64_t *)PX(0, 8) = ((i64x2)h4)[0];
	*(int64_t *)PX(0, 9) = ((i64x2)h4)[1];
	*(int64_t *)PX(0, 10) = ((i64x2)h5)[0];
	*(int64_t *)PX(0, 11) = ((i64x2)h5)[1];
	*(int64_t *)PX(0, 12) = ((i64x2)h6)[0];
	*(int64_t *)PX(0, 13) = ((i64x2)h6)[1];
	*(int64_t *)PX(0, 14) = ((i64x2)h7)[0];
	*(int64_t *)PX(0, 15) = ((i64x2)h7)[1];
}



/**
 * Deblock the luma plane of the current macroblock in place, then tail call to
 * chroma deblocking.
 */
static void deblock_Y_8bit(Edge264Context *ctx) {
	INIT_PX(ctx->samples_mb[0], ctx->t.stride[0]);
	i8x16 v0, v1, v2, v3, v4, v5, v6, v7;
	if (mb->f.filter_edges & 1) {
		// load and transpose the left 12x16 matrix
		i8x16 xa0 = load128(PX(-8, 0));
		i8x16 xa1 = load128(PX(-8, 1));
		i8x16 xa2 = load128(PX(-8, 2));
		i8x16 xa3 = load128(PX(-8, 3));
		i8x16 xa4 = load128(PX(-8, 4));
		i8x16 xa5 = load128(PX(-8, 5));
		i8x16 xa6 = load128(PX(-8, 6));
		i8x16 xa7 = load128(PX(-8, 7));
		i8x16 xb0 = ziplo8(xa0, xa1);
		i8x16 xb1 = ziphi8(xa0, xa1);
		i8x16 xb2 = ziplo8(xa2, xa3);
		i8x16 xb3 = ziphi8(xa2, xa3);
		i8x16 xb4 = ziplo8(xa4, xa5);
		i8x16 xb5 = ziphi8(xa4, xa5);
		i8x16 xb6 = ziplo8(xa6, xa7);
		i8x16 xb7 = ziphi8(xa6, xa7);
		i8x16 xc0 = ziphi16(xb0, xb2);
		i8x16 xc1 = ziplo16(xb1, xb3);
		i8x16 xc2 = ziphi16(xb1, xb3);
		i8x16 xc3 = ziphi16(xb4, xb6);
		i8x16 xc4 = ziplo16(xb5, xb7);
		i8x16 xc5 = ziphi16(xb5, xb7);
		i8x16 xa8 = load128(PX(-8, 8));
		i8x16 xa9 = load128(PX(-8, 9));
		i8x16 xaA = load128(PX(-8, 10));
		i8x16 xaB = load128(PX(-8, 11));
		i8x16 xaC = load128(PX(-8, 12));
		i8x16 xaD = load128(PX(-8, 13));
		i8x16 xaE = load128(PX(-8, 14));
		i8x16 xaF = load128(PX(-8, 15));
		i8x16 xb8 = ziplo8(xa8, xa9);
		i8x16 xb9 = ziphi8(xa8, xa9);
		i8x16 xbA = ziplo8(xaA, xaB);
		i8x16 xbB = ziphi8(xaA, xaB);
		i8x16 xbC = ziplo8(xaC, xaD);
		i8x16 xbD = ziphi8(xaC, xaD);
		i8x16 xbE = ziplo8(xaE, xaF);
		i8x16 xbF = ziphi8(xaE, xaF);
		i8x16 xc6 = ziphi16(xb8, xbA);
		i8x16 xc7 = ziplo16(xb9, xbB);
		i8x16 xc8 = ziphi16(xb9, xbB);
		i8x16 xc9 = ziphi16(xbC, xbE);
		i8x16 xcA = ziplo16(xbD, xbF);
		i8x16 xcB = ziphi16(xbD, xbF);
		i8x16 xd0 = ziplo32(xc0, xc3);
		i8x16 xd1 = ziphi32(xc0, xc3);
		i8x16 xd2 = ziplo32(xc1, xc4);
		i8x16 xd3 = ziphi32(xc1, xc4);
		i8x16 xd4 = ziplo32(xc2, xc5);
		i8x16 xd5 = ziphi32(xc2, xc5);
		i8x16 xd6 = ziplo32(xc6, xc9);
		i8x16 xd7 = ziphi32(xc6, xc9);
		i8x16 xd8 = ziplo32(xc7, xcA);
		i8x16 xd9 = ziphi32(xc7, xcA);
		i8x16 xdA = ziplo32(xc8, xcB);
		i8x16 xdB = ziphi32(xc8, xcB);
		i8x16 vW = ziplo64(xd0, xd6);
		i8x16 vX = ziphi64(xd0, xd6);
		i8x16 vY = ziplo64(xd1, xd7);
		i8x16 vZ = ziphi64(xd1, xd7);
		v0 = ziplo64(xd2, xd8);
		v1 = ziphi64(xd2, xd8);
		v2 = ziplo64(xd3, xd9);
		v3 = ziphi64(xd3, xd9);
		v4 = ziplo64(xd4, xdA);
		v5 = ziphi64(xd4, xdA);
		v6 = ziplo64(xd5, xdB);
		v7 = ziphi64(xd5, xdB);
		
		// first vertical edge
		if (mbA->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			int tC0a = ctx->tC0_s[0];
			if (tC0a != -1)
				DEBLOCK_LUMA_SOFT(vX, vY, vZ, v0, v1, v2, ctx->alpha[8], ctx->beta[8], tC0a);
		} else if (ctx->alpha[8] != 0) {
			DEBLOCK_LUMA_HARD(vW, vX, vY, vZ, v0, v1, v2, v3, ctx->alpha[8], ctx->beta[8]);
		}
		
		// store vW/vX/vY/vZ into the left macroblock
		i8x16 xe0 = ziplo8(vW, vX);
		i8x16 xe1 = ziphi8(vW, vX);
		i8x16 xe2 = ziplo8(vY, vZ);
		i8x16 xe3 = ziphi8(vY, vZ);
		i32x4 xe4 = ziplo16(xe0, xe2);
		i32x4 xe5 = ziphi16(xe0, xe2);
		i32x4 xe6 = ziplo16(xe1, xe3);
		i32x4 xe7 = ziphi16(xe1, xe3);
		*(int32_t *)PX(-4, 0) = xe4[0];
		*(int32_t *)PX(-4, 1) = xe4[1];
		*(int32_t *)PX(-4, 2) = xe4[2];
		*(int32_t *)PX(-4, 3) = xe4[3];
		*(int32_t *)PX(-4, 4) = xe5[0];
		*(int32_t *)PX(-4, 5) = xe5[1];
		*(int32_t *)PX(-4, 6) = xe5[2];
		*(int32_t *)PX(-4, 7) = xe5[3];
		*(int32_t *)PX(-4, 8) = xe6[0];
		*(int32_t *)PX(-4, 9) = xe6[1];
		*(int32_t *)PX(-4, 10) = xe6[2];
		*(int32_t *)PX(-4, 11) = xe6[3];
		*(int32_t *)PX(-4, 12) = xe7[0];
		*(int32_t *)PX(-4, 13) = xe7[1];
		*(int32_t *)PX(-4, 14) = xe7[2];
		*(int32_t *)PX(-4, 15) = xe7[3];
	} else {
		// load and transpose the left 8x16 matrix
		i8x16 xa0 = load64(PX(0, 0));
		i8x16 xa1 = load64(PX(0, 1));
		i8x16 xa2 = load64(PX(0, 2));
		i8x16 xa3 = load64(PX(0, 3));
		i8x16 xa4 = load64(PX(0, 4));
		i8x16 xa5 = load64(PX(0, 5));
		i8x16 xa6 = load64(PX(0, 6));
		i8x16 xa7 = load64(PX(0, 7));
		i8x16 xb0 = ziplo8(xa0, xa1);
		i8x16 xb1 = ziplo8(xa2, xa3);
		i8x16 xb2 = ziplo8(xa4, xa5);
		i8x16 xb3 = ziplo8(xa6, xa7);
		i8x16 xc0 = ziplo16(xb0, xb1);
		i8x16 xc1 = ziphi16(xb0, xb1);
		i8x16 xc2 = ziplo16(xb2, xb3);
		i8x16 xc3 = ziphi16(xb2, xb3);
		i8x16 xa8 = load64(PX(0, 8));
		i8x16 xa9 = load64(PX(0, 9));
		i8x16 xaA = load64(PX(0, 10));
		i8x16 xaB = load64(PX(0, 11));
		i8x16 xaC = load64(PX(0, 12));
		i8x16 xaD = load64(PX(0, 13));
		i8x16 xaE = load64(PX(0, 14));
		i8x16 xaF = load64(PX(0, 15));
		i8x16 xb4 = ziplo8(xa8, xa9);
		i8x16 xb5 = ziplo8(xaA, xaB);
		i8x16 xb6 = ziplo8(xaC, xaD);
		i8x16 xb7 = ziplo8(xaE, xaF);
		i8x16 xc4 = ziplo16(xb4, xb5);
		i8x16 xc5 = ziphi16(xb4, xb5);
		i8x16 xc6 = ziplo16(xb6, xb7);
		i8x16 xc7 = ziphi16(xb6, xb7);
		i8x16 xd0 = ziplo32(xc0, xc2);
		i8x16 xd1 = ziphi32(xc0, xc2);
		i8x16 xd2 = ziplo32(xc1, xc3);
		i8x16 xd3 = ziphi32(xc1, xc3);
		i8x16 xd4 = ziplo32(xc4, xc6);
		i8x16 xd5 = ziphi32(xc4, xc6);
		i8x16 xd6 = ziplo32(xc5, xc7);
		i8x16 xd7 = ziphi32(xc5, xc7);
		v0 = ziplo64(xd0, xd4);
		v1 = ziphi64(xd0, xd4);
		v2 = ziplo64(xd1, xd5);
		v3 = ziphi64(xd1, xd5);
		v4 = ziplo64(xd2, xd6);
		v5 = ziphi64(xd2, xd6);
		v6 = ziplo64(xd3, xd7);
		v7 = ziphi64(xd3, xd7);
	}
	
	// second vertical edge
	if (!mb->f.transform_size_8x8_flag) {
		int tC0b = ctx->tC0_s[1];
		if (tC0b != -1)
			DEBLOCK_LUMA_SOFT(v1, v2, v3, v4, v5, v6, ctx->alpha[0], ctx->beta[0], tC0b);
	}
	
	// load and transpose the right 8x16 matrix
	i8x16 xa0 = *(i8x16 *)PX(0, 0);
	i8x16 xa1 = *(i8x16 *)PX(0, 1);
	i8x16 xa2 = *(i8x16 *)PX(0, 2);
	i8x16 xa3 = *(i8x16 *)PX(0, 3);
	i8x16 xa4 = *(i8x16 *)PX(0, 4);
	i8x16 xa5 = *(i8x16 *)PX(0, 5);
	i8x16 xa6 = *(i8x16 *)PX(0, 6);
	i8x16 xa7 = *(i8x16 *)PX(0, 7);
	i8x16 xb0 = ziphi8(xa0, xa1);
	i8x16 xb1 = ziphi8(xa2, xa3);
	i8x16 xb2 = ziphi8(xa4, xa5);
	i8x16 xb3 = ziphi8(xa6, xa7);
	i8x16 xc0 = ziplo16(xb0, xb1);
	i8x16 xc1 = ziphi16(xb0, xb1);
	i8x16 xc2 = ziplo16(xb2, xb3);
	i8x16 xc3 = ziphi16(xb2, xb3);
	i8x16 xa8 = *(i8x16 *)PX(0, 8);
	i8x16 xa9 = *(i8x16 *)PX(0, 9);
	i8x16 xaA = *(i8x16 *)PX(0, 10);
	i8x16 xaB = *(i8x16 *)PX(0, 11);
	i8x16 xaC = *(i8x16 *)PX(0, 12);
	i8x16 xaD = *(i8x16 *)PX(0, 13);
	i8x16 xaE = *(i8x16 *)PX(0, 14);
	i8x16 xaF = *(i8x16 *)PX(0, 15);
	i8x16 xb4 = ziphi8(xa8, xa9);
	i8x16 xb5 = ziphi8(xaA, xaB);
	i8x16 xb6 = ziphi8(xaC, xaD);
	i8x16 xb7 = ziphi8(xaE, xaF);
	i8x16 xc4 = ziplo16(xb4, xb5);
	i8x16 xc5 = ziphi16(xb4, xb5);
	i8x16 xc6 = ziplo16(xb6, xb7);
	i8x16 xc7 = ziphi16(xb6, xb7);
	i8x16 xd0 = ziplo32(xc0, xc2);
	i8x16 xd1 = ziphi32(xc0, xc2);
	i8x16 xd2 = ziplo32(xc1, xc3);
	i8x16 xd3 = ziphi32(xc1, xc3);
	i8x16 xd4 = ziplo32(xc4, xc6);
	i8x16 xd5 = ziphi32(xc4, xc6);
	i8x16 xd6 = ziplo32(xc5, xc7);
	i8x16 xd7 = ziphi32(xc5, xc7);
	i8x16 v8 = ziplo64(xd0, xd4);
	i8x16 v9 = ziphi64(xd0, xd4);
	i8x16 vA = ziplo64(xd1, xd5);
	i8x16 vB = ziphi64(xd1, xd5);
	i8x16 vC = ziplo64(xd2, xd6);
	i8x16 vD = ziphi64(xd2, xd6);
	i8x16 vE = ziplo64(xd3, xd7);
	i8x16 vF = ziphi64(xd3, xd7);
	
	// third vertical edge
	int tC0c = ctx->tC0_s[2];
	if (tC0c != -1)
		DEBLOCK_LUMA_SOFT(v5, v6, v7, v8, v9, vA, ctx->alpha[0], ctx->beta[0], tC0c);
	
	// fourth vertical edge
	if (!mb->f.transform_size_8x8_flag) {
		int tC0d = ctx->tC0_s[3];
		if (tC0d != -1)
			DEBLOCK_LUMA_SOFT(v9, vA, vB, vC, vD, vE, ctx->alpha[0], ctx->beta[0], tC0d);
	}
	
	// transpose the top 16x8 matrix
	i8x16 xe0 = ziplo8(v0, v1);
	i8x16 xe1 = ziplo8(v2, v3);
	i8x16 xe2 = ziplo8(v4, v5);
	i8x16 xe3 = ziplo8(v6, v7);
	i8x16 xe4 = ziplo8(v8, v9);
	i8x16 xe5 = ziplo8(vA, vB);
	i8x16 xe6 = ziplo8(vC, vD);
	i8x16 xe7 = ziplo8(vE, vF);
	i8x16 xf0 = ziplo16(xe0, xe1);
	i8x16 xf1 = ziphi16(xe0, xe1);
	i8x16 xf2 = ziplo16(xe2, xe3);
	i8x16 xf3 = ziphi16(xe2, xe3);
	i8x16 xf4 = ziplo16(xe4, xe5);
	i8x16 xf5 = ziphi16(xe4, xe5);
	i8x16 xf6 = ziplo16(xe6, xe7);
	i8x16 xf7 = ziphi16(xe6, xe7);
	i8x16 xg0 = ziplo32(xf0, xf2);
	i8x16 xg1 = ziphi32(xf0, xf2);
	i8x16 xg2 = ziplo32(xf1, xf3);
	i8x16 xg3 = ziphi32(xf1, xf3);
	i8x16 xg4 = ziplo32(xf4, xf6);
	i8x16 xg5 = ziphi32(xf4, xf6);
	i8x16 xg6 = ziplo32(xf5, xf7);
	i8x16 xg7 = ziphi32(xf5, xf7);
	i8x16 h0 = ziplo64(xg0, xg4);
	i8x16 h1 = ziphi64(xg0, xg4);
	i8x16 h2 = ziplo64(xg1, xg5);
	i8x16 h3 = ziphi64(xg1, xg5);
	i8x16 h4 = ziplo64(xg2, xg6);
	i8x16 h5 = ziphi64(xg2, xg6);
	i8x16 h6 = ziplo64(xg3, xg7);
	i8x16 h7 = ziphi64(xg3, xg7);
	
	// first horizontal edge
	if (mb->f.filter_edges & 2) {
		i8x16 hx = *(i8x16 *)PX(0, -3);
		i8x16 hy = *(i8x16 *)PX(0, -2);
		i8x16 hz = *(i8x16 *)PX(0, -1);
		if (mbB->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			int tC0e = ctx->tC0_s[4];
			if (tC0e != -1)
				DEBLOCK_LUMA_SOFT(hx, hy, hz, h0, h1, h2, ctx->alpha[12], ctx->beta[12], tC0e);
		} else if (ctx->alpha[12] != 0) {
			i8x16 hw = *(i8x16 *)PX(0, -4);
			DEBLOCK_LUMA_HARD(hw, hx, hy, hz, h0, h1, h2, h3, ctx->alpha[12], ctx->beta[12]);
			*(i8x16 *)PX(0, -3) = hx;
		}
		*(i8x16 *)PX(0, -2) = hy;
		*(i8x16 *)PX(0, -1) = hz;
	}
	*(i8x16 *)PX(0, 0) = h0;
	*(i8x16 *)PX(0, 1) = h1;
	
	// second horizontal edge
	if (!mb->f.transform_size_8x8_flag) {
		int tC0f = ctx->tC0_s[5];
		if (tC0f != -1)
			DEBLOCK_LUMA_SOFT(h1, h2, h3, h4, h5, h6, ctx->alpha[0], ctx->beta[0], tC0f);
	}
	*(i8x16 *)PX(0, 2) = h2;
	*(i8x16 *)PX(0, 3) = h3;
	*(i8x16 *)PX(0, 4) = h4;
	*(i8x16 *)PX(0, 5) = h5;
	
	// transpose the bottom 16x8 matrix
	i8x16 xh0 = ziphi8(v0, v1);
	i8x16 xh1 = ziphi8(v2, v3);
	i8x16 xh2 = ziphi8(v4, v5);
	i8x16 xh3 = ziphi8(v6, v7);
	i8x16 xh4 = ziphi8(v8, v9);
	i8x16 xh5 = ziphi8(vA, vB);
	i8x16 xh6 = ziphi8(vC, vD);
	i8x16 xh7 = ziphi8(vE, vF);
	i8x16 xi0 = ziplo16(xh0, xh1);
	i8x16 xi1 = ziphi16(xh0, xh1);
	i8x16 xi2 = ziplo16(xh2, xh3);
	i8x16 xi3 = ziphi16(xh2, xh3);
	i8x16 xi4 = ziplo16(xh4, xh5);
	i8x16 xi5 = ziphi16(xh4, xh5);
	i8x16 xi6 = ziplo16(xh6, xh7);
	i8x16 xi7 = ziphi16(xh6, xh7);
	i8x16 xj0 = ziplo32(xi0, xi2);
	i8x16 xj1 = ziphi32(xi0, xi2);
	i8x16 xj2 = ziplo32(xi1, xi3);
	i8x16 xj3 = ziphi32(xi1, xi3);
	i8x16 xj4 = ziplo32(xi4, xi6);
	i8x16 xj5 = ziphi32(xi4, xi6);
	i8x16 xj6 = ziplo32(xi5, xi7);
	i8x16 xj7 = ziphi32(xi5, xi7);
	i8x16 h8 = ziplo64(xj0, xj4);
	i8x16 h9 = ziphi64(xj0, xj4);
	i8x16 hA = ziplo64(xj1, xj5);
	i8x16 hB = ziphi64(xj1, xj5);
	i8x16 hC = ziplo64(xj2, xj6);
	i8x16 hD = ziphi64(xj2, xj6);
	i8x16 hE = ziplo64(xj3, xj7);
	i8x16 hF = ziphi64(xj3, xj7);
	
	// third horizontal edge
	int tC0g = ctx->tC0_s[6];
	if (tC0g != -1)
		DEBLOCK_LUMA_SOFT(h5, h6, h7, h8, h9, hA, ctx->alpha[0], ctx->beta[0], tC0g);
	*(i8x16 *)PX(0, 6) = h6;
	*(i8x16 *)PX(0, 7) = h7;
	*(i8x16 *)PX(0, 8) = h8;
	*(i8x16 *)PX(0, 9) = h9;
	
	// fourth horizontal edge
	if (!mb->f.transform_size_8x8_flag) {
		int tC0h = ctx->tC0_s[7];
		if (tC0h != -1)
			DEBLOCK_LUMA_SOFT(h9, hA, hB, hC, hD, hE, ctx->alpha[0], ctx->beta[0], tC0h);
	}
	*(i8x16 *)PX(0, 10) = hA;
	*(i8x16 *)PX(0, 11) = hB;
	*(i8x16 *)PX(0, 12) = hC;
	*(i8x16 *)PX(0, 13) = hD;
	*(i8x16 *)PX(0, 14) = hE;
	*(i8x16 *)PX(0, 15) = hF;
	
	// jump to chroma deblocking filter
	deblock_CbCr_8bit(ctx);
}



/**
 * Compute alpha, beta and tC0 for all planes and all edges (labeled a to h in
 * deblocking order) of the current macroblock, then tail-call to luma filter.
 * 
 * One of the hard parts here is computing a mask for bS=1. Edges a/c/e/g and
 * b/d/f/h are handled separately, and we calculate 4 values per edge:
 * _ differences of references in parallel (l0 vs l0, l1 vs l1) -> refs_p
 *   (zero on b/d/f/h edges)
 * _ differences of references in cross (l0 vs l1, l1 vs l0) -> refs_c
 *   (zero on P macroblocks)
 * _ differences of motion vectors in parallel -> mvs_p
 * _ differences of motion vectors in cross -> mvs_c
 *   (zero on P macroblocks)
 * 
 * Then bS is inferred with the help of a Karnaugh map, where the AND operation
 * is achieved using umin8:
 *                 |      refs_p=0       |      refs_p>0       |
 *                 | refs_c=0 | refs_c>0 | refs_c>0 | refs_c=0 |
 * ----------------+----------+----------+----------+----------+
 *         mvs_c=0 |    0     |    0     |    1     |    0     |
 * mvs_p=0 --------+----------+----------+----------+----------+
 *         mvs_c>0 |    0     |    0     |    1     |    1     |
 * ----------------+----------+----------+----------+----------+
 *         mvs_c>0 |    1     |    1     |    1     |    1     |
 * mvs_p>0 --------+----------+----------+----------+----------+
 *         mvs_c=0 |    0     |    1     |    1     |    0     |
 * ----------------+----------+----------+----------+----------+
 */
static noinline void deblock_mb(Edge264Context *ctx)
{
	static const u8x16 idx2alpha[3] =
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 5, 6, 7, 8, 9, 10, 12, 13, 15, 17, 20, 22, 25, 28, 32, 36, 40, 45, 50, 56, 63, 71, 80, 90, 101, 113, 127, 144, 162, 182, 203, 226, 255, 255};
	static const i8x16 idx2beta[3] =
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18};
	static const i8x16 idx2tC0[3][3] = { // modified to disable deblocking when alpha or beta is zero
		{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 7, 8, 9, 10, 11, 13},
		{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 10, 11, 12, 13, 15, 17},
		{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 7, 8, 9, 10, 11, 13, 14, 16, 18, 20, 23, 25},
	};
	if (!mb->f.filter_edges)
		return;
	
	// compute all values of alpha and beta for each of the color planes first
	mbA = mb - 1;
	mbB = mbA - ctx->t.pic_width_in_mbs;
	i8x16 zero = {};
	i8x16 qP = set32((int32_t)mb->QP_s);
	i32x4 qPAB = {(int32_t)mbA->QP_s, (int32_t)mbB->QP_s};
	i8x16 qPav = avg8(qP, ziplo64(qP, qPAB)); // mid/mid/A/B
	i8x16 c51 = set8(51);
	#if defined(__SSE2__)
		i8x16 indexA = minu8(max8(qPav + set8(ctx->t.FilterOffsetA), zero), c51);
		i8x16 indexB = minu8(max8(qPav + set8(ctx->t.FilterOffsetB), zero), c51);
	#elif defined(__ARM_NEON)
		i8x16 indexA = minu8(vsqaddq_u8(qPav, set8(ctx->t.FilterOffsetA)), c51);
		i8x16 indexB = minu8(vsqaddq_u8(qPav, set8(ctx->t.FilterOffsetB)), c51);
	#endif
	i8x16 c4 = set8(4);
	i8x16 Am4 = subu8(indexA, c4);
	ctx->alpha_v = shuffle3((const i8x16 *)idx2alpha, Am4);
	ctx->beta_v = shuffle3(idx2beta, subu8(indexB, c4));
	
	// initialize tC0 for bS=1/2/3
	if (!mb->f.mbIsInterFlag) {
		i8x16 tC03 = shuffle3(idx2tC0[2], Am4);
		ctx->tC0_v[0] = ctx->tC0_v[1] = broadcast8(tC03, 0);
		ctx->tC0_v[2] = ctx->tC0_v[3] = shuffle(tC03, ((i8x16){-1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 2, 2, 2, 2}));
	} else {
		i8x16 tC01 = shuffle3(idx2tC0[0], Am4);
		i8x16 tC02 = shuffle3(idx2tC0[1], Am4);
		
		// compute masks for bS!=1 based on equality of references and motion vectors
		i8x16 bS0aceg, bS0bdfh;
		i8x16 c3 = set8(3);
		static const i8x16 shufVHAB = {0, 2, 1, 3, 0, 1, 2, 3, 9, 11, 0, 2, 14, 15, 0, 1};
		if ((mb->refIdx_s[1] & mbA->refIdx_s[1] & mbB->refIdx_s[1]) == -1) { // P macroblocks
			i16x8 mvsv0 = unziphi32(mbA->mvs_v[1], mbA->mvs_v[3]);
			i16x8 mvsv1 = unziplo32(mb->mvs_v[0], mb->mvs_v[2]);
			i16x8 mvsv2 = unziphi32(mb->mvs_v[0], mb->mvs_v[2]);
			i16x8 mvsv3 = unziplo32(mb->mvs_v[1], mb->mvs_v[3]);
			i16x8 mvsv4 = unziphi32(mb->mvs_v[1], mb->mvs_v[3]);
			i16x8 mvsh0 = ziphi64(mbB->mvs_v[2], mbB->mvs_v[3]);
			i16x8 mvsh1 = ziplo64(mb->mvs_v[0], mb->mvs_v[1]);
			i16x8 mvsh2 = ziphi64(mb->mvs_v[0], mb->mvs_v[1]);
			i16x8 mvsh3 = ziplo64(mb->mvs_v[2], mb->mvs_v[3]);
			i16x8 mvsh4 = ziphi64(mb->mvs_v[2], mb->mvs_v[3]);
			i8x16 mvsac = packabd16(mvsv0, mvsv1, mvsv2, mvsv3);
			i8x16 mvsbd = packabd16(mvsv1, mvsv2, mvsv3, mvsv4);
			i8x16 mvseg = packabd16(mvsh0, mvsh1, mvsh2, mvsh3);
			i8x16 mvsfh = packabd16(mvsh1, mvsh2, mvsh3, mvsh4);
			i8x16 mvsaceg = packs16(subu8(mvsac, c3), subu8(mvseg, c3));
			i8x16 mvsbdfh = packs16(subu8(mvsbd, c3), subu8(mvsfh, c3));
			i8x16 refs = shuffle(((i32x4){mb->refPic_s[0], 0, mbA->refPic_s[0], mbB->refPic_s[0]}), shufVHAB); // (v0,h0,A0,B0)
			i8x16 neq = refs ^ shr128(refs, 8); // (v0^A0,h0^B0,0,0)
			i8x16 refsaceg = ziplo8(neq, neq);
			bS0aceg = (refsaceg | mvsaceg) == zero;
			bS0bdfh = mvsbdfh == zero;
		} else if (mb->inter_eqs_s == little_endian32(0x1b5fbbff)) { // 16x16 B macroblock
			i16x8 mvsv0l0 = unziphi32(mbA->mvs_v[1], mbA->mvs_v[3]);
			i16x8 mvsv1l0 = unziplo32(mb->mvs_v[0], mb->mvs_v[2]);
			i16x8 mvsv0l1 = unziphi32(mbA->mvs_v[5], mbA->mvs_v[7]);
			i16x8 mvsv1l1 = unziplo32(mb->mvs_v[4], mb->mvs_v[6]);
			i16x8 mvsh0l0 = ziphi64(mbB->mvs_v[2], mbB->mvs_v[3]);
			i16x8 mvsh1l0 = ziplo64(mb->mvs_v[0], mb->mvs_v[1]);
			i16x8 mvsh0l1 = ziphi64(mbB->mvs_v[6], mbB->mvs_v[7]);
			i16x8 mvsh1l1 = ziplo64(mb->mvs_v[4], mb->mvs_v[5]);
			i8x16 mvsael00 = packabd16(mvsv0l0, mvsv1l0, mvsh0l0, mvsh1l0);
			i8x16 mvsael01 = packabd16(mvsv0l0, mvsv1l1, mvsh0l0, mvsh1l1);
			i8x16 mvsael10 = packabd16(mvsv0l1, mvsv1l0, mvsh0l1, mvsh1l0);
			i8x16 mvsael11 = packabd16(mvsv0l1, mvsv1l1, mvsh0l1, mvsh1l1);
			i8x16 mvsaep = subu8(maxu8(mvsael00, mvsael11), c3);
			i8x16 mvsaec = subu8(maxu8(mvsael01, mvsael10), c3);
			i8x16 mvsacegp = ziplo32(packs16(mvsaep, zero), zero);
			i8x16 mvsacegc = ziplo32(packs16(mvsaec, zero), zero);
			i64x2 refPic = {mb->refPic_l};
			i64x2 refPicAB = {mbA->refPic_l, mbB->refPic_l};
			i8x16 refs0 = shuffle(unziplo32(refPic, refPicAB), shufVHAB); // (v0,h0,A0,B0)
			i8x16 refs1 = shuffle(unziphi32(refPic, refPicAB), shufVHAB); // (v1,h1,A1,B1)
			i8x16 neq0 = refs0 ^ (i8x16)shrd128(refs1, refs0, 8); // (v0^A1,h0^B1,A0^v0,B0^h0)
			i8x16 neq1 = refs1 ^ (i8x16)shrd128(refs0, refs1, 8); // (v1^A0,h1^B0,A1^v1,B1^h1)
			i8x16 refsaceg = neq0 | neq1; // low=cross, high=parallel
			i8x16 refsacegc = ziplo8(refsaceg, refsaceg);
			i8x16 refsacegp = ziphi8(refsaceg, refsaceg);
			i8x16 neq3 = minu8(refsacegp, refsacegc) | minu8(mvsacegp, mvsacegc);
			i8x16 neq4 = minu8(refsacegp, mvsacegc) | minu8(mvsacegp, refsacegc);
			bS0aceg = (neq3 | neq4) == zero;
			bS0bdfh = set8(-1);
		} else { // B macroblocks
			i16x8 mvsv0l0 = unziphi32(mbA->mvs_v[1], mbA->mvs_v[3]);
			i16x8 mvsv1l0 = unziplo32(mb->mvs_v[0], mb->mvs_v[2]);
			i16x8 mvsv2l0 = unziphi32(mb->mvs_v[0], mb->mvs_v[2]);
			i16x8 mvsv3l0 = unziplo32(mb->mvs_v[1], mb->mvs_v[3]);
			i16x8 mvsv4l0 = unziphi32(mb->mvs_v[1], mb->mvs_v[3]);
			i16x8 mvsv0l1 = unziphi32(mbA->mvs_v[5], mbA->mvs_v[7]);
			i16x8 mvsv1l1 = unziplo32(mb->mvs_v[4], mb->mvs_v[6]);
			i16x8 mvsv2l1 = unziphi32(mb->mvs_v[4], mb->mvs_v[6]);
			i16x8 mvsv3l1 = unziplo32(mb->mvs_v[5], mb->mvs_v[7]);
			i16x8 mvsv4l1 = unziphi32(mb->mvs_v[5], mb->mvs_v[7]);
			i8x16 mvsacl00 = packabd16(mvsv0l0, mvsv1l0, mvsv2l0, mvsv3l0);
			i8x16 mvsbdl00 = packabd16(mvsv1l0, mvsv2l0, mvsv3l0, mvsv4l0);
			i8x16 mvsacl01 = packabd16(mvsv0l0, mvsv1l1, mvsv2l0, mvsv3l1);
			i8x16 mvsbdl01 = packabd16(mvsv1l0, mvsv2l1, mvsv3l0, mvsv4l1);
			i8x16 mvsacl10 = packabd16(mvsv0l1, mvsv1l0, mvsv2l1, mvsv3l0);
			i8x16 mvsbdl10 = packabd16(mvsv1l1, mvsv2l0, mvsv3l1, mvsv4l0);
			i8x16 mvsacl11 = packabd16(mvsv0l1, mvsv1l1, mvsv2l1, mvsv3l1);
			i8x16 mvsbdl11 = packabd16(mvsv1l1, mvsv2l1, mvsv3l1, mvsv4l1);
			i8x16 mvsacp = subu8(maxu8(mvsacl00, mvsacl11), c3);
			i8x16 mvsbdp = subu8(maxu8(mvsbdl00, mvsbdl11), c3);
			i8x16 mvsacc = subu8(maxu8(mvsacl01, mvsacl10), c3);
			i8x16 mvsbdc = subu8(maxu8(mvsbdl01, mvsbdl10), c3);
			i16x8 mvsh0l0 = ziphi64(mbB->mvs_v[2], mbB->mvs_v[3]);
			i16x8 mvsh1l0 = ziplo64(mb->mvs_v[0], mb->mvs_v[1]);
			i16x8 mvsh2l0 = ziphi64(mb->mvs_v[0], mb->mvs_v[1]);
			i16x8 mvsh3l0 = ziplo64(mb->mvs_v[2], mb->mvs_v[3]);
			i16x8 mvsh4l0 = ziphi64(mb->mvs_v[2], mb->mvs_v[3]);
			i16x8 mvsh0l1 = ziphi64(mbB->mvs_v[6], mbB->mvs_v[7]);
			i16x8 mvsh1l1 = ziplo64(mb->mvs_v[4], mb->mvs_v[5]);
			i16x8 mvsh2l1 = ziphi64(mb->mvs_v[4], mb->mvs_v[5]);
			i16x8 mvsh3l1 = ziplo64(mb->mvs_v[6], mb->mvs_v[7]);
			i16x8 mvsh4l1 = ziphi64(mb->mvs_v[6], mb->mvs_v[7]);
			i8x16 mvsegl00 = packabd16(mvsh0l0, mvsh1l0, mvsh2l0, mvsh3l0);
			i8x16 mvsfhl00 = packabd16(mvsh1l0, mvsh2l0, mvsh3l0, mvsh4l0);
			i8x16 mvsegl01 = packabd16(mvsh0l0, mvsh1l1, mvsh2l0, mvsh3l1);
			i8x16 mvsfhl01 = packabd16(mvsh1l0, mvsh2l1, mvsh3l0, mvsh4l1);
			i8x16 mvsegl10 = packabd16(mvsh0l1, mvsh1l0, mvsh2l1, mvsh3l0);
			i8x16 mvsfhl10 = packabd16(mvsh1l1, mvsh2l0, mvsh3l1, mvsh4l0);
			i8x16 mvsegl11 = packabd16(mvsh0l1, mvsh1l1, mvsh2l1, mvsh3l1);
			i8x16 mvsfhl11 = packabd16(mvsh1l1, mvsh2l1, mvsh3l1, mvsh4l1);
			i8x16 mvsegp = subu8(maxu8(mvsegl00, mvsegl11), c3);
			i8x16 mvsfhp = subu8(maxu8(mvsfhl00, mvsfhl11), c3);
			i8x16 mvsegc = subu8(maxu8(mvsegl01, mvsegl10), c3);
			i8x16 mvsfhc = subu8(maxu8(mvsfhl01, mvsfhl10), c3);
			i8x16 mvsacegp = packs16(mvsacp, mvsegp);
			i8x16 mvsbdfhp = packs16(mvsbdp, mvsfhp);
			i8x16 mvsacegc = packs16(mvsacc, mvsegc);
			i8x16 mvsbdfhc = packs16(mvsbdc, mvsfhc);
			i64x2 refPic = {mb->refPic_l};
			i64x2 refPicAB = {mbA->refPic_l, mbB->refPic_l};
			i8x16 refs0 = shuffle(unziplo32(refPic, refPicAB), shufVHAB); // (v0,h0,A0,B0)
			i8x16 refs1 = shuffle(unziphi32(refPic, refPicAB), shufVHAB); // (v1,h1,A1,B1)
			i8x16 neq0 = refs0 ^ shrd128(refs1, refs0, 8); // (v0^A1,h0^B1,A0^v0,B0^h0)
			i8x16 neq1 = refs1 ^ shrd128(refs0, refs1, 8); // (v1^A0,h1^B0,A1^v1,B1^h1)
			i8x16 neq2 = refs0 ^ refs1;
			i8x16 refsaceg = neq0 | neq1; // low=cross, high=parallel
			i8x16 refsacegc = ziplo8(refsaceg, refsaceg);
			i8x16 refsacegp = ziphi8(refsaceg, refsaceg);
			i8x16 refsbdfhc = ziplo8(neq2, neq2);
			i8x16 neq3 = minu8(refsacegp, refsacegc) | minu8(mvsacegp, mvsacegc);
			i8x16 neq4 = minu8(refsacegp, mvsacegc) | minu8(mvsacegp, refsacegc);
			bS0aceg = (neq3 | neq4) == zero;
			bS0bdfh = minu8(mvsbdfhp, refsbdfhc | mvsbdfhc) == zero;
		}
		i8x16 bS0abcd = ziplo32(bS0aceg, bS0bdfh);
		i8x16 bS0efgh = ziphi32(bS0aceg, bS0bdfh);
		i8x16 bS0aacc = ziplo32(bS0aceg, bS0aceg);
		i8x16 bS0eegg = ziphi32(bS0aceg, bS0aceg);
		
		// for 8x8 blocks with CAVLC, broadcast transform tokens beforehand
		i8x16 nC = mb->nC_v[0];
		if (!ctx->t.pps.entropy_coding_mode_flag && mb->f.transform_size_8x8_flag) {
			mb->nC_v[0] = nC = (i8x16)((i32x4)nC == 0) - -1;
		}
		
		// compute masks for edges with bS=2
		static const i8x16 shufV = {0, 2, 8, 10, 1, 3, 9, 11, 4, 6, 12, 14, 5, 7, 13, 15};
		static const i8x16 shufH = {0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15};
		i8x16 nnzv = shuffle(nC, shufV);
		i8x16 nnzl = shuffle(mbA->nC_v[0], shufV);
		i8x16 nnzh = shuffle(nC, shufH);
		i8x16 nnzt = shuffle(mbB->nC_v[0], shufH);
		i8x16 bS2abcd = (nnzv | shrd128(nnzl, nnzv, 12)) > zero;
		i8x16 bS2efgh = (nnzh | shrd128(nnzt, nnzh, 12)) > zero;
		i8x16 bS2aacc = trnlo32(bS2abcd);
		i8x16 bS2eegg = trnlo32(bS2efgh);
		
		// shuffle, blend and store tC0 values (tC00 is -1)
		static const i8x16 shuf0 = {8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		static const i8x16 shuf1 = {12, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		static const i8x16 shuf2 = {9, 9, 9, 9, 10, 10, 10, 10, 1, 1, 1, 1, 2, 2, 2, 2};
		static const i8x16 shuf3 = {13, 13, 13, 13, 14, 14, 14, 14, 1, 1, 1, 1, 2, 2, 2, 2};
		ctx->tC0_v[0] = ifelse_mask(bS2abcd, shuffle(tC02, shuf0), bS0abcd | shuffle(tC01, shuf0));
		ctx->tC0_v[1] = ifelse_mask(bS2efgh, shuffle(tC02, shuf1), bS0efgh | shuffle(tC01, shuf1));
		ctx->tC0_v[2] = ifelse_mask(bS2aacc, shuffle(tC02, shuf2), bS0aacc | shuffle(tC01, shuf2));
		ctx->tC0_v[3] = ifelse_mask(bS2eegg, shuffle(tC02, shuf3), bS0eegg | shuffle(tC01, shuf3));
	}
	
	// jump to luma deblocking filter
	deblock_Y_8bit(ctx);
}
