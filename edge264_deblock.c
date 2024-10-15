#include "edge264_internal.h"


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
#define DEBLOCK_LUMA_SOFT(p2, p1, p0, q0, q1, q2, palpha, pbeta, itC0) {\
	/* compute the opposite of filterSamplesFlags as a mask, and transfer the mask bS=0 from tC0 */\
	i8x16 pq0 = subus8(p0, q0);\
	i8x16 qp0 = subus8(q0, p0);\
	i8x16 sub0 = subus8(addus8(qp0, set8(-128)), pq0); /* save 128+q0-p0 for later */\
	i8x16 abs0 = pq0 | qp0;\
	i8x16 abs1 = subus8(p1, p0) | subus8(p0, p1);\
	i8x16 abs2 = subus8(q1, q0) | subus8(q0, q1);\
	i8x16 beta = set8(*(pbeta));\
	i8x16 and = umin8(subus8(set8(*(palpha)), abs0), subus8(beta, umax8(abs1, abs2)));\
	i8x16 tC0 = expand4(itC0);\
	i8x16 ignoreSamplesFlags = ifelse_msb(tC0, tC0, and == 0);\
	i8x16 ftC0 = tC0 & ~ignoreSamplesFlags;\
	/* filter p1 and q1 (same as ffmpeg, I couldn't find better) */\
	i8x16 c1 = set8(1);\
	i8x16 x0 = avg8(p0, q0); /* (p0+q0+1)>>1 */\
	i8x16 x1 = avg8(p2, x0) - ((p2 ^ x0) & c1); /* (p2+((p0+q0+1)>>1))>>1 */\
	i8x16 x2 = avg8(q2, x0) - ((q2 ^ x0) & c1); /* (q2+((p0+q0+1)>>1))>>1 */\
	i8x16 pp1 = umin8(umax8(x1, subus8(p1, ftC0)), addus8(p1, ftC0));\
	i8x16 qp1 = umin8(umax8(x2, subus8(q1, ftC0)), addus8(q1, ftC0));\
	i8x16 cm1 = set8(-1);\
	i8x16 bm1 = (i8x16)beta + cm1;\
	i8x16 sub1 = avg8(p1, q1 ^ cm1); /* save 128+((p1-q1)>>1) for later */\
	i8x16 apltb = subus8(subus8(p2, p0), bm1) == subus8(subus8(p0, p2), bm1);\
	i8x16 aqltb = subus8(subus8(q2, q0), bm1) == subus8(subus8(q0, q2), bm1);\
	p1 = ifelse_mask(apltb, pp1, p1);\
	q1 = ifelse_mask(aqltb, qp1, q1);\
	/* filter p0 and q0 (by offsetting signed to unsigned to apply pavg) */\
	i8x16 ftC = (ftC0 - apltb - aqltb) & ~ignoreSamplesFlags;\
	i8x16 x3 = avg8(sub0, avg8(sub1, set8(127))); /* 128+((q0-p0+((p1-q1)>>2)+1)>>1) */\
	i8x16 c128 = set8(-128);\
	i8x16 delta = umin8(subus8(x3, c128), ftC); /* delta if delta>0 */\
	i8x16 ndelta = umin8(subus8(c128, x3), ftC); /* -delta if delta<0 */\
	p0 = subus8(addus8(p0, delta), ndelta);\
	q0 = subus8(addus8(q0, ndelta), delta);}

#define DEBLOCK_CHROMA_SOFT(p1, p0, q0, q1, shufab, itC0) {\
	/* compute the opposite of filterSamplesFlags and apply if to tC */\
	i8x16 c128 = set8(-128);\
	i8x16 pq0 = subus8(p0, q0);\
	i8x16 qp0 = subus8(q0, p0);\
	i8x16 sub0 = subus8(addus8(qp0, c128), pq0); /* save 128+q0-p0 for later */\
	i8x16 abs0 = pq0 | qp0;\
	i8x16 abs1 = subus8(p1, p0) | subus8(p0, p1);\
	i8x16 abs2 = subus8(q1, q0) | subus8(q0, q1);\
	i8x16 alpha = shuffle8(ctx->t.alpha_v, shufab);\
	i8x16 beta = shuffle8(ctx->t.beta_v, shufab);\
	i8x16 and = umin8(subus8(alpha, abs0), subus8(beta, umax8(abs1, abs2)));\
	i8x16 ignoreSamplesFlags = and == 0;\
	i8x16 cm1 = set8(-1);\
	i8x16 ftC = (expand2(itC0) - cm1) & ~ignoreSamplesFlags;\
	/* filter p0 and q0 (by offsetting signed to unsigned to apply pavg) */\
	i8x16 sub1 = avg8(p1, q1 ^ cm1); /* 128+((p1-q1)>>1) */\
	i8x16 x3 = avg8(sub0, avg8(sub1, set8(127))); /* 128+((q0-p0+((p1-q1)>>2)+1)>>1) */\
	i8x16 delta = umin8(subus8(x3, c128), ftC); /* delta if delta>0 */\
	i8x16 ndelta = umin8(subus8(c128, x3), ftC); /* -delta if delta<0 */\
	p0 = subus8(addus8(p0, delta), ndelta);\
	q0 = subus8(addus8(q0, ndelta), delta);}



/**
 * Filter a single edge in place for bS=4.
 * 
 * The luma filter uses beta-1 thus should not be used with beta=0.
 */
#define DEBLOCK_LUMA_HARD(p3, p2, p1, p0, q0, q1, q2, q3, palpha, pbeta) {\
	/* compute the opposite of filterSamplesFlags, and condition masks for filtering modes */\
	i8x16 alpha = set8(*(palpha));\
	i8x16 beta = set8(*(pbeta));\
	i8x16 abs0 = subus8(p0, q0) | subus8(q0, p0);\
	i8x16 abs1 = subus8(p1, p0) | subus8(p0, p1);\
	i8x16 abs2 = subus8(q1, q0) | subus8(q0, q1);\
	i8x16 ignoreSamplesFlags = umin8(subus8(alpha, abs0), subus8(beta, umax8(abs1, abs2))) == 0;\
	i8x16 c1 = set8(1);\
	i8x16 condpq = subus8(abs0, avg8(avg8(alpha, c1), (i8x16){})); /* abs0-((alpha>>2)+1) */\
	i8x16 bm1 = (i8x16)beta - c1;\
	i8x16 condp = (subus8(subus8(p2, p0) | subus8(p0, p2), bm1) | condpq) == 0;\
	i8x16 condq = (subus8(subus8(q2, q0) | subus8(q0, q2), bm1) | condpq) == 0;\
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

#define DEBLOCK_CHROMA_HARD(p1, p0, q0, q1, shufab) {\
	/* compute the opposite of filterSamplesFlags */\
	i8x16 abs0 = subus8(p0, q0) | subus8(q0, p0);\
	i8x16 abs1 = subus8(p1, p0) | subus8(p0, p1);\
	i8x16 abs2 = subus8(q1, q0) | subus8(q0, q1);\
	i8x16 alpha = shuffle8(ctx->t.alpha_v, shufab);\
	i8x16 beta = shuffle8(ctx->t.beta_v, shufab);\
	i8x16 and = umin8(subus8(alpha, abs0), subus8(beta, umax8(abs1, abs2)));\
	i8x16 ignoreSamplesFlags = and == 0;\
	/* compute p'0 and q'0 */\
	i8x16 c1 = set8(1);\
	i8x16 pp0b = avg8(p1, avg8(p0, q1) - ((p0 ^ q1) & c1)); /* p'0 (second formula) */\
	i8x16 qp0b = avg8(q1, avg8(q0, p1) - ((q0 ^ p1) & c1)); /* q'0 (second formula) */\
	p0 = ifelse_mask(ignoreSamplesFlags, p0, pp0b);\
	q0 = ifelse_mask(ignoreSamplesFlags, q0, qp0b);}



/**
 * Helper functions
 */
static always_inline i8x16 expand4(int32_t a) {
	i32x4 x0 = {a};
	i8x16 x1 = unpacklo8(x0, x0);
	return unpacklo8(x1, x1);
}
static always_inline i8x16 expand2(int64_t a) {
	i64x2 x0 = {a};
	return unpacklo8(x0, x0);
}



/**
 * Deblock both chroma planes of the current macroblock in place.
 */
static noinline void FUNC_CTX(deblock_CbCr_8bit, size_t stride, ssize_t nstride, size_t stride7)
{
	static const i8x16 shuf_a = {9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10};
	static const i8x16 shuf_cg = {1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2};
	static const i8x16 shuf_e = {13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14};
	i8x16 v0, v1, v2, v3, v4, v5, v6, v7;
	if (mb->f.filter_edges & 1) {
		// load and transpose both 12x8 matrices (with left macroblock)
		uint8_t * restrict Cb0 = ctx->samples_mb[1] - 8;
		i8x16 xa0 = load128(Cb0              );
		i8x16 xa1 = load128(Cb0 +  stride    );
		i8x16 xa2 = load128(Cb0 +  stride * 2);
		uint8_t * restrict Cb7 = Cb0 + stride7;
		i8x16 xa3 = load128(Cb7 + nstride * 4);
		i8x16 xa4 = load128(Cb0 +  stride * 4);
		i8x16 xa5 = load128(Cb7 + nstride * 2);
		i8x16 xa6 = load128(Cb7 + nstride    );
		i8x16 xa7 = load128(Cb7              );
		i8x16 xb0 = unpacklo8(xa0, xa1);
		i8x16 xb1 = unpackhi8(xa0, xa1);
		i8x16 xb2 = unpacklo8(xa2, xa3);
		i8x16 xb3 = unpackhi8(xa2, xa3);
		i8x16 xb4 = unpacklo8(xa4, xa5);
		i8x16 xb5 = unpackhi8(xa4, xa5);
		i8x16 xb6 = unpacklo8(xa6, xa7);
		i8x16 xb7 = unpackhi8(xa6, xa7);
		i8x16 xc0 = unpackhi16(xb0, xb2);
		i8x16 xc1 = unpacklo16(xb1, xb3);
		i8x16 xc2 = unpackhi16(xb1, xb3);
		i8x16 xc3 = unpackhi16(xb4, xb6);
		i8x16 xc4 = unpacklo16(xb5, xb7);
		i8x16 xc5 = unpackhi16(xb5, xb7);
		uint8_t * restrict Cr0 = ctx->samples_mb[2] - 8;
		i8x16 xa8 = load128(Cr0              );
		i8x16 xa9 = load128(Cr0 +  stride    );
		i8x16 xaA = load128(Cr0 +  stride * 2);
		uint8_t * restrict Cr7 = Cr0 + stride7;
		i8x16 xaB = load128(Cr7 + nstride * 4);
		i8x16 xaC = load128(Cr0 +  stride * 4);
		i8x16 xaD = load128(Cr7 + nstride * 2);
		i8x16 xaE = load128(Cr7 + nstride    );
		i8x16 xaF = load128(Cr7              );
		i8x16 xb8 = unpacklo8(xa8, xa9);
		i8x16 xb9 = unpackhi8(xa8, xa9);
		i8x16 xbA = unpacklo8(xaA, xaB);
		i8x16 xbB = unpackhi8(xaA, xaB);
		i8x16 xbC = unpacklo8(xaC, xaD);
		i8x16 xbD = unpackhi8(xaC, xaD);
		i8x16 xbE = unpacklo8(xaE, xaF);
		i8x16 xbF = unpackhi8(xaE, xaF);
		i8x16 xc6 = unpackhi16(xb8, xbA);
		i8x16 xc7 = unpacklo16(xb9, xbB);
		i8x16 xc8 = unpackhi16(xb9, xbB);
		i8x16 xc9 = unpackhi16(xbC, xbE);
		i8x16 xcA = unpacklo16(xbD, xbF);
		i8x16 xcB = unpackhi16(xbD, xbF);
		i8x16 xd0 = unpackhi32(xc0, xc3);
		i8x16 xd1 = unpacklo32(xc1, xc4);
		i8x16 xd2 = unpackhi32(xc1, xc4);
		i8x16 xd3 = unpacklo32(xc2, xc5);
		i8x16 xd4 = unpackhi32(xc2, xc5);
		i8x16 xd5 = unpackhi32(xc6, xc9);
		i8x16 xd6 = unpacklo32(xc7, xcA);
		i8x16 xd7 = unpackhi32(xc7, xcA);
		i8x16 xd8 = unpacklo32(xc8, xcB);
		i8x16 xd9 = unpackhi32(xc8, xcB);
		i8x16 vY = unpacklo64(xd0, xd5);
		i8x16 vZ = unpackhi64(xd0, xd5);
		v0 = unpacklo64(xd1, xd6);
		v1 = unpackhi64(xd1, xd6);
		v2 = unpacklo64(xd2, xd7);
		v3 = unpackhi64(xd2, xd7);
		v4 = unpacklo64(xd3, xd8);
		v5 = unpackhi64(xd3, xd8);
		v6 = unpacklo64(xd4, xd9);
		v7 = unpackhi64(xd4, xd9);
		
		// first vertical edge
		if (mbA->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			int64_t tC0a = ctx->t.tC0_l[4];
			if (tC0a != -1)
				DEBLOCK_CHROMA_SOFT(vY, vZ, v0, v1, shuf_a, tC0a);
		} else {
			DEBLOCK_CHROMA_HARD(vY, vZ, v0, v1, shuf_a);
		}
		
		// store vY/vZ into the left macroblock
		i16x8 xf0 = unpacklo8(vY, vZ);
		i16x8 xf1 = unpackhi8(vY, vZ);
		Cb0 = ctx->samples_mb[1] - 2;
		*(int16_t *)(Cb0              ) = xf0[0];
		*(int16_t *)(Cb0 +  stride    ) = xf0[1];
		*(int16_t *)(Cb0 +  stride * 2) = xf0[2];
		Cb7 = Cb0 + stride7;
		*(int16_t *)(Cb7 + nstride * 4) = xf0[3];
		*(int16_t *)(Cb0 +  stride * 4) = xf0[4];
		*(int16_t *)(Cb7 + nstride * 2) = xf0[5];
		*(int16_t *)(Cb7 + nstride    ) = xf0[6];
		*(int16_t *)(Cb7              ) = xf0[7];
		Cr0 = ctx->samples_mb[2] - 2;
		*(int16_t *)(Cr0              ) = xf1[0];
		*(int16_t *)(Cr0 +  stride    ) = xf1[1];
		*(int16_t *)(Cr0 +  stride * 2) = xf1[2];
		Cr7 = Cr0 + stride7;
		*(int16_t *)(Cr7 + nstride * 4) = xf1[3];
		*(int16_t *)(Cr0 +  stride * 4) = xf1[4];
		*(int16_t *)(Cr7 + nstride * 2) = xf1[5];
		*(int16_t *)(Cr7 + nstride    ) = xf1[6];
		*(int16_t *)(Cr7              ) = xf1[7];
	} else {
		// load and transpose both 8x8 matrices
		uint8_t * restrict Cb0 = ctx->samples_mb[1];
		i8x16 xa0 = load64(Cb0              );
		i8x16 xa1 = load64(Cb0 +  stride    );
		i8x16 xa2 = load64(Cb0 +  stride * 2);
		uint8_t * restrict Cb7 = Cb0 + stride7;
		i8x16 xa3 = load64(Cb7 + nstride * 4);
		i8x16 xa4 = load64(Cb0 +  stride * 4);
		i8x16 xa5 = load64(Cb7 + nstride * 2);
		i8x16 xa6 = load64(Cb7 + nstride    );
		i8x16 xa7 = load64(Cb7              );
		i8x16 xb0 = unpacklo8(xa0, xa1);
		i8x16 xb1 = unpacklo8(xa2, xa3);
		i8x16 xb2 = unpacklo8(xa4, xa5);
		i8x16 xb3 = unpacklo8(xa6, xa7);
		uint8_t * restrict Cr0 = ctx->samples_mb[2];
		i8x16 xa8 = load64(Cr0              );
		i8x16 xa9 = load64(Cr0 +  stride    );
		i8x16 xaA = load64(Cr0 +  stride * 2);
		uint8_t * restrict Cr7 = Cr0 + stride7;
		i8x16 xaB = load64(Cr7 + nstride * 4);
		i8x16 xaC = load64(Cr0 +  stride * 4);
		i8x16 xaD = load64(Cr7 + nstride * 2);
		i8x16 xaE = load64(Cr7 + nstride    );
		i8x16 xaF = load64(Cr7              );
		i8x16 xb4 = unpacklo8(xa8, xa9);
		i8x16 xb5 = unpacklo8(xaA, xaB);
		i8x16 xb6 = unpacklo8(xaC, xaD);
		i8x16 xb7 = unpacklo8(xaE, xaF);
		i8x16 xc0 = unpacklo16(xb0, xb1);
		i8x16 xc1 = unpackhi16(xb0, xb1);
		i8x16 xc2 = unpacklo16(xb2, xb3);
		i8x16 xc3 = unpackhi16(xb2, xb3);
		i8x16 xc4 = unpacklo16(xb4, xb5);
		i8x16 xc5 = unpackhi16(xb4, xb5);
		i8x16 xc6 = unpacklo16(xb6, xb7);
		i8x16 xc7 = unpackhi16(xb6, xb7);
		i8x16 xd0 = unpacklo32(xc0, xc2);
		i8x16 xd1 = unpackhi32(xc0, xc2);
		i8x16 xd2 = unpacklo32(xc1, xc3);
		i8x16 xd3 = unpackhi32(xc1, xc3);
		i8x16 xd4 = unpacklo32(xc4, xc6);
		i8x16 xd5 = unpackhi32(xc4, xc6);
		i8x16 xd6 = unpacklo32(xc5, xc7);
		i8x16 xd7 = unpackhi32(xc5, xc7);
		v0 = unpacklo64(xd0, xd4);
		v1 = unpackhi64(xd0, xd4);
		v2 = unpacklo64(xd1, xd5);
		v3 = unpackhi64(xd1, xd5);
		v4 = unpacklo64(xd2, xd6);
		v5 = unpackhi64(xd2, xd6);
		v6 = unpacklo64(xd3, xd7);
		v7 = unpackhi64(xd3, xd7);
	}
	
	// second vertical edge
	int64_t tC0c = ctx->t.tC0_l[5];
	if (tC0c != -1)
		DEBLOCK_CHROMA_SOFT(v2, v3, v4, v5, shuf_cg, tC0c);
	
	// transpose both 8x8 matrices
	i8x16 xa0 = unpacklo8(v0, v1);
	i8x16 xa1 = unpackhi8(v0, v1);
	i8x16 xa2 = unpacklo8(v2, v3);
	i8x16 xa3 = unpackhi8(v2, v3);
	i8x16 xa4 = unpacklo8(v4, v5);
	i8x16 xa5 = unpackhi8(v4, v5);
	i8x16 xa6 = unpacklo8(v6, v7);
	i8x16 xa7 = unpackhi8(v6, v7);
	i8x16 xb0 = unpacklo16(xa0, xa2);
	i8x16 xb1 = unpackhi16(xa0, xa2);
	i8x16 xb2 = unpacklo16(xa1, xa3);
	i8x16 xb3 = unpackhi16(xa1, xa3);
	i8x16 xb4 = unpacklo16(xa4, xa6);
	i8x16 xb5 = unpackhi16(xa4, xa6);
	i8x16 xb6 = unpacklo16(xa5, xa7);
	i8x16 xb7 = unpackhi16(xa5, xa7);
	i8x16 xc0 = unpacklo32(xb0, xb4);
	i8x16 xc1 = unpackhi32(xb0, xb4);
	i8x16 xc2 = unpacklo32(xb1, xb5);
	i8x16 xc3 = unpackhi32(xb1, xb5);
	i8x16 xc4 = unpacklo32(xb2, xb6);
	i8x16 xc5 = unpackhi32(xb2, xb6);
	i8x16 xc6 = unpacklo32(xb3, xb7);
	i8x16 xc7 = unpackhi32(xb3, xb7);
	i8x16 h0 = unpacklo64(xc0, xc4);
	i8x16 h1 = unpackhi64(xc0, xc4);
	i8x16 h2 = unpacklo64(xc1, xc5);
	i8x16 h3 = unpackhi64(xc1, xc5);
	i8x16 h4 = unpacklo64(xc2, xc6);
	i8x16 h5 = unpackhi64(xc2, xc6);
	i8x16 h6 = unpacklo64(xc3, xc7);
	i8x16 h7 = unpackhi64(xc3, xc7);
	
	// first horizontal edge
	uint8_t * restrict Cb0 = ctx->samples_mb[1];
	uint8_t * restrict Cr0 = ctx->samples_mb[2];
	if (mb->f.filter_edges & 2) {
		i8x16 hY = (i64x2){*(int64_t *)(Cb0 + nstride * 2), *(int64_t *)(Cr0 + nstride * 2)};
		i8x16 hZ = (i64x2){*(int64_t *)(Cb0 + nstride    ), *(int64_t *)(Cr0 + nstride    )};
		if (mbB->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			int64_t tC0e = ctx->t.tC0_l[6];
			if (tC0e != -1)
				DEBLOCK_CHROMA_SOFT(hY, hZ, h0, h1, shuf_e, tC0e);
		} else {
			DEBLOCK_CHROMA_HARD(hY, hZ, h0, h1, shuf_e);
		}
		*(int64_t *)(Cb0 + nstride * 2) = ((i64x2)hY)[0];
		*(int64_t *)(Cr0 + nstride * 2) = ((i64x2)hY)[1];
		*(int64_t *)(Cb0 + nstride    ) = ((i64x2)hZ)[0];
		*(int64_t *)(Cr0 + nstride    ) = ((i64x2)hZ)[1];
	}
	*(int64_t *)(Cb0              ) = ((i64x2)h0)[0];
	*(int64_t *)(Cr0              ) = ((i64x2)h0)[1];
	*(int64_t *)(Cb0 +  stride    ) = ((i64x2)h1)[0];
	*(int64_t *)(Cr0 +  stride    ) = ((i64x2)h1)[1];
	
	// second horizontal edge
	int64_t tC0g = ctx->t.tC0_l[7];
	if (tC0g != -1)
		DEBLOCK_CHROMA_SOFT(h2, h3, h4, h5, shuf_cg, tC0g);
	*(int64_t *)(Cb0 +  stride * 2) = ((i64x2)h2)[0];
	*(int64_t *)(Cr0 +  stride * 2) = ((i64x2)h2)[1];
	uint8_t * restrict Cb7 = Cb0 + stride7;
	uint8_t * restrict Cr7 = Cr0 + stride7;
	*(int64_t *)(Cb7 + nstride * 4) = ((i64x2)h3)[0];
	*(int64_t *)(Cr7 + nstride * 4) = ((i64x2)h3)[1];
	*(int64_t *)(Cb0 +  stride * 4) = ((i64x2)h4)[0];
	*(int64_t *)(Cr0 +  stride * 4) = ((i64x2)h4)[1];
	*(int64_t *)(Cb7 + nstride * 2) = ((i64x2)h5)[0];
	*(int64_t *)(Cr7 + nstride * 2) = ((i64x2)h5)[1];
	*(int64_t *)(Cb7 + nstride    ) = ((i64x2)h6)[0];
	*(int64_t *)(Cr7 + nstride    ) = ((i64x2)h6)[1];
	*(int64_t *)(Cb7              ) = ((i64x2)h7)[0];
	*(int64_t *)(Cr7              ) = ((i64x2)h7)[1];
}



/**
 * Deblock the luma plane of the current macroblock in place, then tail call to
 * chroma deblocking.
 */
static noinline void FUNC_CTX(deblock_Y_8bit, size_t stride, ssize_t nstride, size_t stride7)
{
	i8x16 zero = {};
	i8x16 v0, v1, v2, v3, v4, v5, v6, v7;
	if (mb->f.filter_edges & 1) {
		// load and transpose the left 12x16 matrix
		uint8_t * restrict px0 = ctx->samples_mb[0] - 8;
		i8x16 xa0 = load128(px0              );
		i8x16 xa1 = load128(px0 +  stride    );
		i8x16 xa2 = load128(px0 +  stride * 2);
		uint8_t * restrict px7 = px0 + stride7;
		i8x16 xa3 = load128(px7 + nstride * 4);
		i8x16 xa4 = load128(px0 +  stride * 4);
		i8x16 xa5 = load128(px7 + nstride * 2);
		i8x16 xa6 = load128(px7 + nstride    );
		i8x16 xa7 = load128(px7              );
		i8x16 xb0 = unpacklo8(xa0, xa1);
		i8x16 xb1 = unpackhi8(xa0, xa1);
		i8x16 xb2 = unpacklo8(xa2, xa3);
		i8x16 xb3 = unpackhi8(xa2, xa3);
		i8x16 xb4 = unpacklo8(xa4, xa5);
		i8x16 xb5 = unpackhi8(xa4, xa5);
		i8x16 xb6 = unpacklo8(xa6, xa7);
		i8x16 xb7 = unpackhi8(xa6, xa7);
		i8x16 xc0 = unpackhi16(xb0, xb2);
		i8x16 xc1 = unpacklo16(xb1, xb3);
		i8x16 xc2 = unpackhi16(xb1, xb3);
		i8x16 xc3 = unpackhi16(xb4, xb6);
		i8x16 xc4 = unpacklo16(xb5, xb7);
		i8x16 xc5 = unpackhi16(xb5, xb7);
		i8x16 xa8 = load128(px7 +  stride    );
		i8x16 xa9 = load128(px7 +  stride * 2);
		uint8_t * restrict pxE = px7 + stride7;
		i8x16 xaA = load128(pxE + nstride * 4);
		i8x16 xaB = load128(px7 +  stride * 4);
		i8x16 xaC = load128(pxE + nstride * 2);
		i8x16 xaD = load128(pxE + nstride    );
		i8x16 xaE = load128(pxE              );
		i8x16 xaF = load128(pxE +  stride    );
		i8x16 xb8 = unpacklo8(xa8, xa9);
		i8x16 xb9 = unpackhi8(xa8, xa9);
		i8x16 xbA = unpacklo8(xaA, xaB);
		i8x16 xbB = unpackhi8(xaA, xaB);
		i8x16 xbC = unpacklo8(xaC, xaD);
		i8x16 xbD = unpackhi8(xaC, xaD);
		i8x16 xbE = unpacklo8(xaE, xaF);
		i8x16 xbF = unpackhi8(xaE, xaF);
		i8x16 xc6 = unpackhi16(xb8, xbA);
		i8x16 xc7 = unpacklo16(xb9, xbB);
		i8x16 xc8 = unpackhi16(xb9, xbB);
		i8x16 xc9 = unpackhi16(xbC, xbE);
		i8x16 xcA = unpacklo16(xbD, xbF);
		i8x16 xcB = unpackhi16(xbD, xbF);
		i8x16 xd0 = unpacklo32(xc0, xc3);
		i8x16 xd1 = unpackhi32(xc0, xc3);
		i8x16 xd2 = unpacklo32(xc1, xc4);
		i8x16 xd3 = unpackhi32(xc1, xc4);
		i8x16 xd4 = unpacklo32(xc2, xc5);
		i8x16 xd5 = unpackhi32(xc2, xc5);
		i8x16 xd6 = unpacklo32(xc6, xc9);
		i8x16 xd7 = unpackhi32(xc6, xc9);
		i8x16 xd8 = unpacklo32(xc7, xcA);
		i8x16 xd9 = unpackhi32(xc7, xcA);
		i8x16 xdA = unpacklo32(xc8, xcB);
		i8x16 xdB = unpackhi32(xc8, xcB);
		i8x16 vW = unpacklo64(xd0, xd6);
		i8x16 vX = unpackhi64(xd0, xd6);
		i8x16 vY = unpacklo64(xd1, xd7);
		i8x16 vZ = unpackhi64(xd1, xd7);
		v0 = unpacklo64(xd2, xd8);
		v1 = unpackhi64(xd2, xd8);
		v2 = unpacklo64(xd3, xd9);
		v3 = unpackhi64(xd3, xd9);
		v4 = unpacklo64(xd4, xdA);
		v5 = unpackhi64(xd4, xdA);
		v6 = unpacklo64(xd5, xdB);
		v7 = unpackhi64(xd5, xdB);
		
		// first vertical edge
		if (mbA->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			int tC0a = ctx->t.tC0_s[0];
			if (tC0a != -1)
				DEBLOCK_LUMA_SOFT(vX, vY, vZ, v0, v1, v2, ctx->t.alpha + 8, ctx->t.beta + 8, tC0a);
		} else if (ctx->t.alpha[8] != 0) {
			DEBLOCK_LUMA_HARD(vW, vX, vY, vZ, v0, v1, v2, v3, ctx->t.alpha + 8, ctx->t.beta + 8);
		}
		
		// store vW/vX/vY/vZ into the left macroblock
		i8x16 xe0 = unpacklo8(vW, vX);
		i8x16 xe1 = unpackhi8(vW, vX);
		i8x16 xe2 = unpacklo8(vY, vZ);
		i8x16 xe3 = unpackhi8(vY, vZ);
		i32x4 xe4 = unpacklo16(xe0, xe2);
		i32x4 xe5 = unpackhi16(xe0, xe2);
		i32x4 xe6 = unpacklo16(xe1, xe3);
		i32x4 xe7 = unpackhi16(xe1, xe3);
		px0 = ctx->samples_mb[0] - 4;
		*(int32_t *)(px0              ) = xe4[0];
		*(int32_t *)(px0 +  stride    ) = xe4[1];
		*(int32_t *)(px0 +  stride * 2) = xe4[2];
		px7 = px0 + stride7;
		*(int32_t *)(px7 + nstride * 4) = xe4[3];
		*(int32_t *)(px0 +  stride * 4) = xe5[0];
		*(int32_t *)(px7 + nstride * 2) = xe5[1];
		*(int32_t *)(px7 + nstride    ) = xe5[2];
		*(int32_t *)(px7              ) = xe5[3];
		*(int32_t *)(px7 +  stride    ) = xe6[0];
		*(int32_t *)(px7 +  stride * 2) = xe6[1];
		pxE = px7 + stride7;
		*(int32_t *)(pxE + nstride * 4) = xe6[2];
		*(int32_t *)(px7 +  stride * 4) = xe6[3];
		*(int32_t *)(pxE + nstride * 2) = xe7[0];
		*(int32_t *)(pxE + nstride    ) = xe7[1];
		*(int32_t *)(pxE              ) = xe7[2];
		*(int32_t *)(pxE +  stride    ) = xe7[3];
	} else {
		// load and transpose the left 8x16 matrix
		uint8_t * restrict px0 = ctx->samples_mb[0];
		i8x16 xa0 = load64(px0              );
		i8x16 xa1 = load64(px0 +  stride    );
		i8x16 xa2 = load64(px0 +  stride * 2);
		uint8_t * restrict px7 = px0 + stride7;
		i8x16 xa3 = load64(px7 + nstride * 4);
		i8x16 xa4 = load64(px0 +  stride * 4);
		i8x16 xa5 = load64(px7 + nstride * 2);
		i8x16 xa6 = load64(px7 + nstride    );
		i8x16 xa7 = load64(px7              );
		i8x16 xb0 = unpacklo8(xa0, xa1);
		i8x16 xb1 = unpacklo8(xa2, xa3);
		i8x16 xb2 = unpacklo8(xa4, xa5);
		i8x16 xb3 = unpacklo8(xa6, xa7);
		i8x16 xc0 = unpacklo16(xb0, xb1);
		i8x16 xc1 = unpackhi16(xb0, xb1);
		i8x16 xc2 = unpacklo16(xb2, xb3);
		i8x16 xc3 = unpackhi16(xb2, xb3);
		i8x16 xa8 = load64(px7 +  stride    );
		i8x16 xa9 = load64(px7 +  stride * 2);
		uint8_t * restrict pxE = px7 + stride7;
		i8x16 xaA = load64(pxE + nstride * 4);
		i8x16 xaB = load64(px7 +  stride * 4);
		i8x16 xaC = load64(pxE + nstride * 2);
		i8x16 xaD = load64(pxE + nstride    );
		i8x16 xaE = load64(pxE              );
		i8x16 xaF = load64(pxE +  stride    );
		i8x16 xb4 = unpacklo8(xa8, xa9);
		i8x16 xb5 = unpacklo8(xaA, xaB);
		i8x16 xb6 = unpacklo8(xaC, xaD);
		i8x16 xb7 = unpacklo8(xaE, xaF);
		i8x16 xc4 = unpacklo16(xb4, xb5);
		i8x16 xc5 = unpackhi16(xb4, xb5);
		i8x16 xc6 = unpacklo16(xb6, xb7);
		i8x16 xc7 = unpackhi16(xb6, xb7);
		i8x16 xd0 = unpacklo32(xc0, xc2);
		i8x16 xd1 = unpackhi32(xc0, xc2);
		i8x16 xd2 = unpacklo32(xc1, xc3);
		i8x16 xd3 = unpackhi32(xc1, xc3);
		i8x16 xd4 = unpacklo32(xc4, xc6);
		i8x16 xd5 = unpackhi32(xc4, xc6);
		i8x16 xd6 = unpacklo32(xc5, xc7);
		i8x16 xd7 = unpackhi32(xc5, xc7);
		v0 = unpacklo64(xd0, xd4);
		v1 = unpackhi64(xd0, xd4);
		v2 = unpacklo64(xd1, xd5);
		v3 = unpackhi64(xd1, xd5);
		v4 = unpacklo64(xd2, xd6);
		v5 = unpackhi64(xd2, xd6);
		v6 = unpacklo64(xd3, xd7);
		v7 = unpackhi64(xd3, xd7);
	}
	
	// second vertical edge
	if (!mb->f.transform_size_8x8_flag) {
		int tC0b = ctx->t.tC0_s[1];
		if (tC0b != -1)
			DEBLOCK_LUMA_SOFT(v1, v2, v3, v4, v5, v6, ctx->t.alpha + 0, ctx->t.beta + 0, tC0b);
	}
	
	// load and transpose the right 8x16 matrix
	uint8_t * restrict px0 = ctx->samples_mb[0];
	i8x16 xa0 = *(i8x16 *)(px0              );
	i8x16 xa1 = *(i8x16 *)(px0 +  stride    );
	i8x16 xa2 = *(i8x16 *)(px0 +  stride * 2);
	uint8_t * restrict px7 = px0 + stride7;
	i8x16 xa3 = *(i8x16 *)(px7 + nstride * 4);
	i8x16 xa4 = *(i8x16 *)(px0 +  stride * 4);
	i8x16 xa5 = *(i8x16 *)(px7 + nstride * 2);
	i8x16 xa6 = *(i8x16 *)(px7 + nstride    );
	i8x16 xa7 = *(i8x16 *)(px7              );
	i8x16 xb0 = unpackhi8(xa0, xa1);
	i8x16 xb1 = unpackhi8(xa2, xa3);
	i8x16 xb2 = unpackhi8(xa4, xa5);
	i8x16 xb3 = unpackhi8(xa6, xa7);
	i8x16 xc0 = unpacklo16(xb0, xb1);
	i8x16 xc1 = unpackhi16(xb0, xb1);
	i8x16 xc2 = unpacklo16(xb2, xb3);
	i8x16 xc3 = unpackhi16(xb2, xb3);
	i8x16 xa8 = *(i8x16 *)(px7 +  stride    );
	i8x16 xa9 = *(i8x16 *)(px7 +  stride * 2);
	uint8_t * restrict pxE = px7 + stride7;
	i8x16 xaA = *(i8x16 *)(pxE + nstride * 4);
	i8x16 xaB = *(i8x16 *)(px7 +  stride * 4);
	i8x16 xaC = *(i8x16 *)(pxE + nstride * 2);
	i8x16 xaD = *(i8x16 *)(pxE + nstride    );
	i8x16 xaE = *(i8x16 *)(pxE              );
	i8x16 xaF = *(i8x16 *)(pxE +  stride    );
	i8x16 xb4 = unpackhi8(xa8, xa9);
	i8x16 xb5 = unpackhi8(xaA, xaB);
	i8x16 xb6 = unpackhi8(xaC, xaD);
	i8x16 xb7 = unpackhi8(xaE, xaF);
	i8x16 xc4 = unpacklo16(xb4, xb5);
	i8x16 xc5 = unpackhi16(xb4, xb5);
	i8x16 xc6 = unpacklo16(xb6, xb7);
	i8x16 xc7 = unpackhi16(xb6, xb7);
	i8x16 xd0 = unpacklo32(xc0, xc2);
	i8x16 xd1 = unpackhi32(xc0, xc2);
	i8x16 xd2 = unpacklo32(xc1, xc3);
	i8x16 xd3 = unpackhi32(xc1, xc3);
	i8x16 xd4 = unpacklo32(xc4, xc6);
	i8x16 xd5 = unpackhi32(xc4, xc6);
	i8x16 xd6 = unpacklo32(xc5, xc7);
	i8x16 xd7 = unpackhi32(xc5, xc7);
	i8x16 v8 = unpacklo64(xd0, xd4);
	i8x16 v9 = unpackhi64(xd0, xd4);
	i8x16 vA = unpacklo64(xd1, xd5);
	i8x16 vB = unpackhi64(xd1, xd5);
	i8x16 vC = unpacklo64(xd2, xd6);
	i8x16 vD = unpackhi64(xd2, xd6);
	i8x16 vE = unpacklo64(xd3, xd7);
	i8x16 vF = unpackhi64(xd3, xd7);
	
	// third vertical edge
	int tC0c = ctx->t.tC0_s[2];
	if (tC0c != -1)
		DEBLOCK_LUMA_SOFT(v5, v6, v7, v8, v9, vA, ctx->t.alpha + 0, ctx->t.beta + 0, tC0c);
	
	// fourth vertical edge
	if (!mb->f.transform_size_8x8_flag) {
		int tC0d = ctx->t.tC0_s[3];
		if (tC0d != -1)
			DEBLOCK_LUMA_SOFT(v9, vA, vB, vC, vD, vE, ctx->t.alpha + 0, ctx->t.beta + 0, tC0d);
	}
	
	// transpose the top 16x8 matrix
	i8x16 xe0 = unpacklo8(v0, v1);
	i8x16 xe1 = unpacklo8(v2, v3);
	i8x16 xe2 = unpacklo8(v4, v5);
	i8x16 xe3 = unpacklo8(v6, v7);
	i8x16 xe4 = unpacklo8(v8, v9);
	i8x16 xe5 = unpacklo8(vA, vB);
	i8x16 xe6 = unpacklo8(vC, vD);
	i8x16 xe7 = unpacklo8(vE, vF);
	i8x16 xf0 = unpacklo16(xe0, xe1);
	i8x16 xf1 = unpackhi16(xe0, xe1);
	i8x16 xf2 = unpacklo16(xe2, xe3);
	i8x16 xf3 = unpackhi16(xe2, xe3);
	i8x16 xf4 = unpacklo16(xe4, xe5);
	i8x16 xf5 = unpackhi16(xe4, xe5);
	i8x16 xf6 = unpacklo16(xe6, xe7);
	i8x16 xf7 = unpackhi16(xe6, xe7);
	i8x16 xg0 = unpacklo32(xf0, xf2);
	i8x16 xg1 = unpackhi32(xf0, xf2);
	i8x16 xg2 = unpacklo32(xf1, xf3);
	i8x16 xg3 = unpackhi32(xf1, xf3);
	i8x16 xg4 = unpacklo32(xf4, xf6);
	i8x16 xg5 = unpackhi32(xf4, xf6);
	i8x16 xg6 = unpacklo32(xf5, xf7);
	i8x16 xg7 = unpackhi32(xf5, xf7);
	i8x16 h0 = unpacklo64(xg0, xg4);
	i8x16 h1 = unpackhi64(xg0, xg4);
	i8x16 h2 = unpacklo64(xg1, xg5);
	i8x16 h3 = unpackhi64(xg1, xg5);
	i8x16 h4 = unpacklo64(xg2, xg6);
	i8x16 h5 = unpackhi64(xg2, xg6);
	i8x16 h6 = unpacklo64(xg3, xg7);
	i8x16 h7 = unpackhi64(xg3, xg7);
	
	// first horizontal edge
	px0 = ctx->samples_mb[0];
	if (mb->f.filter_edges & 2) {
		if (mbB->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			int tC0e = ctx->t.tC0_s[4];
			if (tC0e != -1)
				DEBLOCK_LUMA_SOFT(*(i8x16 *)(px0 + nstride * 3), *(i8x16 *)(px0 + nstride * 2), *(i8x16 *)(px0 + nstride    ), h0, h1, h2, ctx->t.alpha + 12, ctx->t.beta + 12, tC0e);
		} else if (ctx->t.alpha[12] != 0) {
			DEBLOCK_LUMA_HARD(*(i8x16 *)(px0 + nstride * 4), *(i8x16 *)(px0 + nstride * 3), *(i8x16 *)(px0 + nstride * 2), *(i8x16 *)(px0 + nstride    ), h0, h1, h2, h3, ctx->t.alpha + 12, ctx->t.beta + 12);
		}
	}
	*(i8x16 *)(px0              ) = h0;
	*(i8x16 *)(px0 +  stride    ) = h1;
	
	// second horizontal edge
	if (!mb->f.transform_size_8x8_flag) {
		int tC0f = ctx->t.tC0_s[5];
		if (tC0f != -1)
			DEBLOCK_LUMA_SOFT(h1, h2, h3, h4, h5, h6, ctx->t.alpha + 0, ctx->t.beta + 0, tC0f);
	}
	px7 = px0 + stride7;
	*(i8x16 *)(px0 +  stride * 2) = h2;
	*(i8x16 *)(px7 + nstride * 4) = h3;
	*(i8x16 *)(px0 +  stride * 4) = h4;
	*(i8x16 *)(px7 + nstride * 2) = h5;
	
	// transpose the bottom 16x8 matrix
	i8x16 xh0 = unpackhi8(v0, v1);
	i8x16 xh1 = unpackhi8(v2, v3);
	i8x16 xh2 = unpackhi8(v4, v5);
	i8x16 xh3 = unpackhi8(v6, v7);
	i8x16 xh4 = unpackhi8(v8, v9);
	i8x16 xh5 = unpackhi8(vA, vB);
	i8x16 xh6 = unpackhi8(vC, vD);
	i8x16 xh7 = unpackhi8(vE, vF);
	i8x16 xi0 = unpacklo16(xh0, xh1);
	i8x16 xi1 = unpackhi16(xh0, xh1);
	i8x16 xi2 = unpacklo16(xh2, xh3);
	i8x16 xi3 = unpackhi16(xh2, xh3);
	i8x16 xi4 = unpacklo16(xh4, xh5);
	i8x16 xi5 = unpackhi16(xh4, xh5);
	i8x16 xi6 = unpacklo16(xh6, xh7);
	i8x16 xi7 = unpackhi16(xh6, xh7);
	i8x16 xj0 = unpacklo32(xi0, xi2);
	i8x16 xj1 = unpackhi32(xi0, xi2);
	i8x16 xj2 = unpacklo32(xi1, xi3);
	i8x16 xj3 = unpackhi32(xi1, xi3);
	i8x16 xj4 = unpacklo32(xi4, xi6);
	i8x16 xj5 = unpackhi32(xi4, xi6);
	i8x16 xj6 = unpacklo32(xi5, xi7);
	i8x16 xj7 = unpackhi32(xi5, xi7);
	i8x16 h8 = unpacklo64(xj0, xj4);
	i8x16 h9 = unpackhi64(xj0, xj4);
	i8x16 hA = unpacklo64(xj1, xj5);
	i8x16 hB = unpackhi64(xj1, xj5);
	i8x16 hC = unpacklo64(xj2, xj6);
	i8x16 hD = unpackhi64(xj2, xj6);
	i8x16 hE = unpacklo64(xj3, xj7);
	i8x16 hF = unpackhi64(xj3, xj7);
	
	// third horizontal edge
	int tC0g = ctx->t.tC0_s[6];
	if (tC0g != -1)
		DEBLOCK_LUMA_SOFT(h5, h6, h7, h8, h9, hA, ctx->t.alpha + 0, ctx->t.beta + 0, tC0g);
	*(i8x16 *)(px7 + nstride    ) = h6;
	*(i8x16 *)(px7              ) = h7;
	*(i8x16 *)(px7 +  stride    ) = h8;
	*(i8x16 *)(px7 +  stride * 2) = h9;
	
	// fourth horizontal edge
	if (!mb->f.transform_size_8x8_flag) {
		int tC0h = ctx->t.tC0_s[7];
		if (tC0h != -1)
			DEBLOCK_LUMA_SOFT(h9, hA, hB, hC, hD, hE, ctx->t.alpha + 0, ctx->t.beta + 0, tC0h);
	}
	pxE = px7 + stride7;
	*(i8x16 *)(pxE + nstride * 4) = hA;
	*(i8x16 *)(px7 +  stride * 4) = hB;
	*(i8x16 *)(pxE + nstride * 2) = hC;
	*(i8x16 *)(pxE + nstride    ) = hD;
	*(i8x16 *)(pxE              ) = hE;
	*(i8x16 *)(pxE +  stride    ) = hF;
	
	// jump to chroma deblocking filter
	size_t strideC = ctx->t.stride[1];
	JUMP_CTX(deblock_CbCr_8bit, strideC, -strideC, strideC * 7);
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
noinline void FUNC_CTX(deblock_mb)
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
	
	// compute all values of indexA and indexB for each of the color planes first
	mbA = mb - 1;
	mbB = mbA - ctx->t.pic_width_in_mbs;
	i8x16 zero = {};
	i8x16 qP = set32((int32_t)mb->QP_s);
	i32x4 qPAB = {(int32_t)mbA->QP_s, (int32_t)mbB->QP_s};
	i8x16 qPav = avg8(qP, unpacklo64(qP, qPAB)); // mid/mid/A/B
	i8x16 c51 = set8(51);
	i8x16 indexA = umin8(max8(qPav + set8(ctx->t.FilterOffsetA), zero), c51);
	i8x16 indexB = umin8(max8(qPav + set8(ctx->t.FilterOffsetB), zero), c51);
	
	// compute all values of alpha and beta using vectorized array accesses
	i8x16 c4 = set8(4);
	i8x16 c15 = set8(15);
	i8x16 c31 = set8(31);
	i8x16 Am4 = indexA - c4;
	i8x16 Bm4 = indexB - c4;
	i8x16 Agte20 = Am4 > c15;
	i8x16 Agte36 = Am4 > c31;
	i8x16 Bgte20 = Bm4 > c15;
	i8x16 Bgte36 = Bm4 > c31;
	i8x16 alphalo = shuffle8(idx2alpha[0], Am4);
	i8x16 alphamd = shuffle8(idx2alpha[1], Am4);
	i8x16 alphahi = shuffle8(idx2alpha[2], Am4);
	i8x16 betalo = shuffle8(idx2beta[0], Bm4);
	i8x16 betamd = shuffle8(idx2beta[1], Bm4);
	i8x16 betahi = shuffle8(idx2beta[2], Bm4);
	ctx->t.alpha_v = ifelse_mask(Agte36, alphahi, ifelse_mask(Agte20, alphamd, alphalo));
	ctx->t.beta_v = ifelse_mask(Bgte36, betahi, ifelse_mask(Bgte20, betamd, betalo));
	
	// initialize tC0 with bS=3 for internal edges of Intra macroblock
	i8x16 tC0neg = zero > Am4;
	if (!mb->f.mbIsInterFlag) {
		i8x16 tC03lo = shuffle8(idx2tC0[2][0], Am4);
		i8x16 tC03md = shuffle8(idx2tC0[2][1], Am4);
		i8x16 tC03hi = shuffle8(idx2tC0[2][2], Am4);
		i8x16 tC03 = ifelse_mask(Agte36, tC03hi, ifelse_mask(Agte20, tC03md, tC03lo)) | tC0neg;
		ctx->t.tC0_v[0] = ctx->t.tC0_v[1] = shuffle8(tC03, zero);
		ctx->t.tC0_v[2] = ctx->t.tC0_v[3] = shuffle8(tC03, ((i8x16){-1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 2, 2, 2, 2}));
	} else {
		// compute all values of tC0 for bS=1 and bS=2
		i8x16 tC01lo = shuffle8(idx2tC0[0][0], Am4);
		i8x16 tC01md = shuffle8(idx2tC0[0][1], Am4);
		i8x16 tC01hi = shuffle8(idx2tC0[0][2], Am4);
		i8x16 tC01 = ifelse_mask(Agte36, tC01hi, ifelse_mask(Agte20, tC01md, tC01lo)) | tC0neg;
		i8x16 tC02lo = shuffle8(idx2tC0[1][0], Am4);
		i8x16 tC02md = shuffle8(idx2tC0[1][1], Am4);
		i8x16 tC02hi = shuffle8(idx2tC0[1][2], Am4);
		i8x16 tC02 = ifelse_mask(Agte36, tC02hi, ifelse_mask(Agte20, tC02md, tC02lo)) | tC0neg;
		
		// compute masks for bS!=1 based on equality of references and motion vectors
		i8x16 bS0aceg, bS0bdfh;
		i8x16 c3 = set8(3);
		static const i8x16 shufVHAB = {0, 2, 1, 3, 0, 1, 2, 3, 9, 11, 0, 2, 14, 15, 0, 1};
		if (mb->inter_eqs_s == little_endian32(0x1b5fbbff)) { // 16x16 macroblock
			i16x8 mvsv0l0 = shuffleps(mbA->mvs_v[1], mbA->mvs_v[3], 1, 3, 1, 3);
			i16x8 mvsv1l0 = shuffleps(mb->mvs_v[0], mb->mvs_v[2], 0, 2, 0, 2);
			i16x8 mvsv0l1 = shuffleps(mbA->mvs_v[5], mbA->mvs_v[7], 1, 3, 1, 3);
			i16x8 mvsv1l1 = shuffleps(mb->mvs_v[4], mb->mvs_v[6], 0, 2, 0, 2);
			i16x8 mvsh0l0 = unpackhi64(mbB->mvs_v[2], mbB->mvs_v[3]);
			i16x8 mvsh1l0 = unpacklo64(mb->mvs_v[0], mb->mvs_v[1]);
			i16x8 mvsh0l1 = unpackhi64(mbB->mvs_v[6], mbB->mvs_v[7]);
			i16x8 mvsh1l1 = unpacklo64(mb->mvs_v[4], mb->mvs_v[5]);
			i8x16 mvsael00 = packs16(mvsv0l0 - mvsv1l0, mvsh0l0 - mvsh1l0);
			i8x16 mvsael01 = packs16(mvsv0l0 - mvsv1l1, mvsh0l0 - mvsh1l1);
			i8x16 mvsael10 = packs16(mvsv0l1 - mvsv1l0, mvsh0l1 - mvsh1l0);
			i8x16 mvsael11 = packs16(mvsv0l1 - mvsv1l1, mvsh0l1 - mvsh1l1);
			i8x16 mvsaep = subus8(umax8(abs8(mvsael00), abs8(mvsael11)), c3);
			i8x16 mvsaec = subus8(umax8(abs8(mvsael01), abs8(mvsael10)), c3);
			i8x16 mvsacegp = shuffle32(packs16(mvsaep, zero), 0, 2, 1, 3);
			i8x16 mvsacegc = shuffle32(packs16(mvsaec, zero), 0, 2, 1, 3);
			i64x2 refPic = {mb->refPic_l};
			i64x2 refPicAB = {mbA->refPic_l, mbB->refPic_l};
			i8x16 refs0 = shuffle8(shuffleps(refPic, refPicAB, 0, 0, 0, 2), shufVHAB); // (v0,h0,A0,B0)
			i8x16 refs1 = shuffle8(shuffleps(refPic, refPicAB, 1, 1, 1, 3), shufVHAB); // (v1,h1,A1,B1)
			i8x16 neq0 = refs0 ^ (i8x16)shuffleps(refs1, refs0, 2, 3, 0, 1); // (v0^A1,h0^B1,A0^v0,B0^h0)
			i8x16 neq1 = refs1 ^ (i8x16)shuffleps(refs0, refs1, 2, 3, 0, 1); // (v1^A0,h1^B0,A1^v1,B1^h1)
			i8x16 refsaceg = neq0 | neq1; // low=cross, high=parallel
			i8x16 refsacegc = unpacklo8(refsaceg, refsaceg);
			i8x16 refsacegp = unpackhi8(refsaceg, refsaceg);
			i8x16 neq3 = umin8(refsacegp, refsacegc) | umin8(mvsacegp, mvsacegc);
			i8x16 neq4 = umin8(refsacegp, mvsacegc) | umin8(mvsacegp, refsacegc);
			bS0aceg = (neq3 | neq4) == zero;
			bS0bdfh = set8(-1);
		} else if ((mb->refIdx_s[1] & mbA->refIdx_s[1] & mbB->refIdx_s[1]) == -1) { // P macroblocks
			i16x8 mvsv0 = shuffleps(mbA->mvs_v[1], mbA->mvs_v[3], 1, 3, 1, 3);
			i16x8 mvsv1 = shuffleps(mb->mvs_v[0], mb->mvs_v[2], 0, 2, 0, 2);
			i16x8 mvsv2 = shuffleps(mb->mvs_v[0], mb->mvs_v[2], 1, 3, 1, 3);
			i16x8 mvsv3 = shuffleps(mb->mvs_v[1], mb->mvs_v[3], 0, 2, 0, 2);
			i16x8 mvsv4 = shuffleps(mb->mvs_v[1], mb->mvs_v[3], 1, 3, 1, 3);
			i16x8 mvsh0 = unpackhi64(mbB->mvs_v[2], mbB->mvs_v[3]);
			i16x8 mvsh1 = unpacklo64(mb->mvs_v[0], mb->mvs_v[1]);
			i16x8 mvsh2 = unpackhi64(mb->mvs_v[0], mb->mvs_v[1]);
			i16x8 mvsh3 = unpacklo64(mb->mvs_v[2], mb->mvs_v[3]);
			i16x8 mvsh4 = unpackhi64(mb->mvs_v[2], mb->mvs_v[3]);
			i8x16 mvsac = packs16(mvsv0 - mvsv1, mvsv2 - mvsv3);
			i8x16 mvsbd = packs16(mvsv1 - mvsv2, mvsv3 - mvsv4);
			i8x16 mvseg = packs16(mvsh0 - mvsh1, mvsh2 - mvsh3);
			i8x16 mvsfh = packs16(mvsh1 - mvsh2, mvsh3 - mvsh4);
			i8x16 mvsaceg = packs16(subus8(abs8(mvsac), c3), subus8(abs8(mvseg), c3));
			i8x16 mvsbdfh = packs16(subus8(abs8(mvsbd), c3), subus8(abs8(mvsfh), c3));
			i8x16 refs = shuffle8(((i32x4){mb->refPic_s[0], 0, mbA->refPic_s[0], mbB->refPic_s[0]}), shufVHAB); // (v0,h0,A0,B0)
			i8x16 neq = refs ^ (i8x16)unpackhi64(refs, refs); // (v0^A0,h0^B0,0,0)
			i8x16 refsaceg = unpacklo8(neq, neq);
			bS0aceg = (refsaceg | mvsaceg) == zero;
			bS0bdfh = mvsbdfh == zero;
		} else { // B macroblocks
			i16x8 mvsv0l0 = shuffleps(mbA->mvs_v[1], mbA->mvs_v[3], 1, 3, 1, 3);
			i16x8 mvsv1l0 = shuffleps(mb->mvs_v[0], mb->mvs_v[2], 0, 2, 0, 2);
			i16x8 mvsv2l0 = shuffleps(mb->mvs_v[0], mb->mvs_v[2], 1, 3, 1, 3);
			i16x8 mvsv3l0 = shuffleps(mb->mvs_v[1], mb->mvs_v[3], 0, 2, 0, 2);
			i16x8 mvsv4l0 = shuffleps(mb->mvs_v[1], mb->mvs_v[3], 1, 3, 1, 3);
			i16x8 mvsv0l1 = shuffleps(mbA->mvs_v[5], mbA->mvs_v[7], 1, 3, 1, 3);
			i16x8 mvsv1l1 = shuffleps(mb->mvs_v[4], mb->mvs_v[6], 0, 2, 0, 2);
			i16x8 mvsv2l1 = shuffleps(mb->mvs_v[4], mb->mvs_v[6], 1, 3, 1, 3);
			i16x8 mvsv3l1 = shuffleps(mb->mvs_v[5], mb->mvs_v[7], 0, 2, 0, 2);
			i16x8 mvsv4l1 = shuffleps(mb->mvs_v[5], mb->mvs_v[7], 1, 3, 1, 3);
			i8x16 mvsacl00 = packs16(mvsv0l0 - mvsv1l0, mvsv2l0 - mvsv3l0);
			i8x16 mvsbdl00 = packs16(mvsv1l0 - mvsv2l0, mvsv3l0 - mvsv4l0);
			i8x16 mvsacl01 = packs16(mvsv0l0 - mvsv1l1, mvsv2l0 - mvsv3l1);
			i8x16 mvsbdl01 = packs16(mvsv1l0 - mvsv2l1, mvsv3l0 - mvsv4l1);
			i8x16 mvsacl10 = packs16(mvsv0l1 - mvsv1l0, mvsv2l1 - mvsv3l0);
			i8x16 mvsbdl10 = packs16(mvsv1l1 - mvsv2l0, mvsv3l1 - mvsv4l0);
			i8x16 mvsacl11 = packs16(mvsv0l1 - mvsv1l1, mvsv2l1 - mvsv3l1);
			i8x16 mvsbdl11 = packs16(mvsv1l1 - mvsv2l1, mvsv3l1 - mvsv4l1);
			i8x16 mvsacp = subus8(umax8(abs8(mvsacl00), abs8(mvsacl11)), c3);
			i8x16 mvsbdp = subus8(umax8(abs8(mvsbdl00), abs8(mvsbdl11)), c3);
			i8x16 mvsacc = subus8(umax8(abs8(mvsacl01), abs8(mvsacl10)), c3);
			i8x16 mvsbdc = subus8(umax8(abs8(mvsbdl01), abs8(mvsbdl10)), c3);
			i16x8 mvsh0l0 = unpackhi64(mbB->mvs_v[2], mbB->mvs_v[3]);
			i16x8 mvsh1l0 = unpacklo64(mb->mvs_v[0], mb->mvs_v[1]);
			i16x8 mvsh2l0 = unpackhi64(mb->mvs_v[0], mb->mvs_v[1]);
			i16x8 mvsh3l0 = unpacklo64(mb->mvs_v[2], mb->mvs_v[3]);
			i16x8 mvsh4l0 = unpackhi64(mb->mvs_v[2], mb->mvs_v[3]);
			i16x8 mvsh0l1 = unpackhi64(mbB->mvs_v[6], mbB->mvs_v[7]);
			i16x8 mvsh1l1 = unpacklo64(mb->mvs_v[4], mb->mvs_v[5]);
			i16x8 mvsh2l1 = unpackhi64(mb->mvs_v[4], mb->mvs_v[5]);
			i16x8 mvsh3l1 = unpacklo64(mb->mvs_v[6], mb->mvs_v[7]);
			i16x8 mvsh4l1 = unpackhi64(mb->mvs_v[6], mb->mvs_v[7]);
			i8x16 mvsegl00 = packs16(mvsh0l0 - mvsh1l0, mvsh2l0 - mvsh3l0);
			i8x16 mvsfhl00 = packs16(mvsh1l0 - mvsh2l0, mvsh3l0 - mvsh4l0);
			i8x16 mvsegl01 = packs16(mvsh0l0 - mvsh1l1, mvsh2l0 - mvsh3l1);
			i8x16 mvsfhl01 = packs16(mvsh1l0 - mvsh2l1, mvsh3l0 - mvsh4l1);
			i8x16 mvsegl10 = packs16(mvsh0l1 - mvsh1l0, mvsh2l1 - mvsh3l0);
			i8x16 mvsfhl10 = packs16(mvsh1l1 - mvsh2l0, mvsh3l1 - mvsh4l0);
			i8x16 mvsegl11 = packs16(mvsh0l1 - mvsh1l1, mvsh2l1 - mvsh3l1);
			i8x16 mvsfhl11 = packs16(mvsh1l1 - mvsh2l1, mvsh3l1 - mvsh4l1);
			i8x16 mvsegp = subus8(umax8(abs8(mvsegl00), abs8(mvsegl11)), c3);
			i8x16 mvsfhp = subus8(umax8(abs8(mvsfhl00), abs8(mvsfhl11)), c3);
			i8x16 mvsegc = subus8(umax8(abs8(mvsegl01), abs8(mvsegl10)), c3);
			i8x16 mvsfhc = subus8(umax8(abs8(mvsfhl01), abs8(mvsfhl10)), c3);
			i8x16 mvsacegp = packs16(mvsacp, mvsegp);
			i8x16 mvsbdfhp = packs16(mvsbdp, mvsfhp);
			i8x16 mvsacegc = packs16(mvsacc, mvsegc);
			i8x16 mvsbdfhc = packs16(mvsbdc, mvsfhc);
			i64x2 refPic = {mb->refPic_l};
			i64x2 refPicAB = {mbA->refPic_l, mbB->refPic_l};
			i8x16 refs0 = shuffle8(shuffleps(refPic, refPicAB, 0, 0, 0, 2), shufVHAB); // (v0,h0,A0,B0)
			i8x16 refs1 = shuffle8(shuffleps(refPic, refPicAB, 1, 1, 1, 3), shufVHAB); // (v1,h1,A1,B1)
			i8x16 neq0 = refs0 ^ (i8x16)shuffleps(refs1, refs0, 2, 3, 0, 1); // (v0^A1,h0^B1,A0^v0,B0^h0)
			i8x16 neq1 = refs1 ^ (i8x16)shuffleps(refs0, refs1, 2, 3, 0, 1); // (v1^A0,h1^B0,A1^v1,B1^h1)
			i8x16 neq2 = refs0 ^ refs1;
			i8x16 refsaceg = neq0 | neq1; // low=cross, high=parallel
			i8x16 refsacegc = unpacklo8(refsaceg, refsaceg);
			i8x16 refsacegp = unpackhi8(refsaceg, refsaceg);
			i8x16 refsbdfhc = unpacklo8(neq2, neq2);
			i8x16 neq3 = umin8(refsacegp, refsacegc) | umin8(mvsacegp, mvsacegc);
			i8x16 neq4 = umin8(refsacegp, mvsacegc) | umin8(mvsacegp, refsacegc);
			bS0aceg = (neq3 | neq4) == zero;
			bS0bdfh = umin8(mvsbdfhp, refsbdfhc | mvsbdfhc) == zero;
		}
		i8x16 bS0abcd = unpacklo32(bS0aceg, bS0bdfh);
		i8x16 bS0efgh = unpackhi32(bS0aceg, bS0bdfh);
		i8x16 bS0aacc = unpacklo32(bS0aceg, bS0aceg);
		i8x16 bS0eegg = unpackhi32(bS0aceg, bS0aceg);
		
		// for 8x8 blocks with CAVLC, broadcast transform tokens beforehand
		i8x16 nC = mb->nC_v[0];
		if (!ctx->t.pps.entropy_coding_mode_flag && mb->f.transform_size_8x8_flag) {
			i8x16 x = (i32x4)mb->nC_v[0] > 0;
			mb->nC_v[0] = nC = sign8(x, x);
		}
		
		// compute masks for edges with bS=2
		static const i8x16 shufV = {0, 2, 8, 10, 1, 3, 9, 11, 4, 6, 12, 14, 5, 7, 13, 15};
		static const i8x16 shufH = {0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15};
		i8x16 nnzv = shuffle8(nC, shufV);
		i8x16 nnzl = shuffle8(mbA->nC_v[0], shufV);
		i8x16 nnzh = shuffle8(nC, shufH);
		i8x16 nnzt = shuffle8(mbB->nC_v[0], shufH);
		i8x16 bS2abcd = (nnzv | alignr(nnzv, nnzl, 12)) > zero;
		i8x16 bS2efgh = (nnzh | alignr(nnzh, nnzt, 12)) > zero;
		i8x16 bS2aacc = shuffle32(bS2abcd, 0, 0, 2, 2);
		i8x16 bS2eegg = shuffle32(bS2efgh, 0, 0, 2, 2);
		
		// shuffle, blend and store tC0 values
		i8x16 tC00 = set8(-1);
		static const i8x16 shuf0 = {8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		static const i8x16 shuf1 = {12, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		static const i8x16 shuf2 = {9, 9, 9, 9, 10, 10, 10, 10, 1, 1, 1, 1, 2, 2, 2, 2};
		static const i8x16 shuf3 = {13, 13, 13, 13, 14, 14, 14, 14, 1, 1, 1, 1, 2, 2, 2, 2};
		ctx->t.tC0_v[0] = ifelse_mask(bS2abcd, shuffle8(tC02, shuf0), ifelse_mask(bS0abcd, tC00, shuffle8(tC01, shuf0)));
		ctx->t.tC0_v[1] = ifelse_mask(bS2efgh, shuffle8(tC02, shuf1), ifelse_mask(bS0efgh, tC00, shuffle8(tC01, shuf1)));
		ctx->t.tC0_v[2] = ifelse_mask(bS2aacc, shuffle8(tC02, shuf2), ifelse_mask(bS0aacc, tC00, shuffle8(tC01, shuf2)));
		ctx->t.tC0_v[3] = ifelse_mask(bS2eegg, shuffle8(tC02, shuf3), ifelse_mask(bS0eegg, tC00, shuffle8(tC01, shuf3)));
	}
	
	// jump to luma deblocking filter
	size_t strideY = ctx->t.stride[0];
	JUMP_CTX(deblock_Y_8bit, strideY, -strideY, strideY * 7);
}
