#include "edge264_common.h"



// [a-b]<=c
static always_inline __m128i diff_lte(__m128i a, __m128i b, __m128i c) {
	return _mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(a, b), c), _mm_subs_epu8(_mm_subs_epu8(b, a), c));
}



#define DEBLOCK_LUMA_SOFT(p2, p1, p0, q0, q1, q2) {\
	/* compute a mask for each lane that will be filtered, and apply it to tc0 */\
	__m128i c128 = _mm_set1_epi8(-128);\
	__m128i pq0 = _mm_subs_epu8(p0, q0);\
	__m128i qp0 = _mm_subs_epu8(q0, p0);\
	__m128i abs0 = _mm_or_si128(pq0, qp0);\
	__m128i sub0 = _mm_subs_epu8(_mm_adds_epu8(qp0, c128), pq0); /* save 128+q0-p0 for later */\
	__m128i abs1 = _mm_or_si128(_mm_subs_epu8(p1, p0), _mm_subs_epu8(p0, p1));\
	__m128i abs2 = _mm_or_si128(_mm_subs_epu8(q1, q0), _mm_subs_epu8(q0, q1));\
	__m128i am1 = _mm_set1_epi8(alpha - 1);\
	__m128i bm1 = _mm_set1_epi8(beta - 1);\
	__m128i filterSamplesFlags = _mm_cmpeq_epi8(_mm_or_si128(_mm_subs_epu8(abs0, am1), _mm_subs_epu8(_mm_max_epu8(abs1, abs2), bm1)), _mm_setzero_si128());\
	__m128i mtc0 = _mm_and_si128(tc0, filterSamplesFlags);\
	/* filter p1 and q1 (same as ffmpeg, I couldn't find better) */\
	__m128i c1 = _mm_set1_epi8(1);\
	__m128i x0 = _mm_avg_epu8(p0, q0); /* (p0+q0+1)>>1 */\
	__m128i x1 = _mm_sub_epi8(_mm_avg_epu8(p2, x0), _mm_and_si128(_mm_xor_si128(p2, x0), c1)); /* (p2+((p0+q0+1)>>1))>>1 */\
	__m128i x2 = _mm_sub_epi8(_mm_avg_epu8(q2, x0), _mm_and_si128(_mm_xor_si128(q2, x0), c1)); /* (p2+((p0+q0+1)>>1))>>1 */\
	__m128i pp1 = _mm_min_epu8(_mm_max_epu8(x1, _mm_subs_epu8(p1, mtc0)), _mm_adds_epu8(p1, mtc0));\
	__m128i qp1 = _mm_min_epu8(_mm_max_epu8(x2, _mm_subs_epu8(q1, mtc0)), _mm_adds_epu8(q1, mtc0));\
	__m128i apltb = diff_lte(p2, p0, bm1);\
	__m128i aqltb = diff_lte(q2, q0, bm1);\
	__m128i sub1 = _mm_avg_epu8(p1, _mm_xor_si128(q1, _mm_set1_epi8(-1))); /* save 128+((p1-q1)>>1) for later */\
	p1 = vector_select(apltb, pp1, p1);\
	q1 = vector_select(aqltb, qp1, q1);\
	/* filter p0 and q0 (by offsetting signed to unsigned to apply pavg) */\
	__m128i mtc = _mm_sub_epi8(_mm_sub_epi8(mtc0, apltb), aqltb);\
	__m128i x3 = _mm_avg_epu8(sub0, _mm_avg_epu8(sub1, _mm_set1_epi8(127))); /* 128+((q0-p0+((p1-q1)>>2)+1)>>1) */\
	__m128i delta = _mm_min_epu8(_mm_subs_epu8(x3, c128), mtc); /* delta if delta>0 */\
	__m128i ndelta = _mm_min_epu8(_mm_subs_epu8(c128, x3), mtc); /* -delta if delta<0 */\
	p0 = _mm_subs_epu8(_mm_adds_epu8(p0, delta), ndelta);\
	q0 = _mm_subs_epu8(_mm_adds_epu8(p0, delta), ndelta);}



#define DEBLOCK_LUMA_HARD(p3, p2, p1, p0, q0, q1, q2, q3) {\
	/* compute condition masks for filtering modes */\
	__m128i bm1 = _mm_set1_epi8(beta - 1);\
	__m128i condpq = diff_lte(p0, q0, _mm_set1_epi8((alpha >> 2) + 1));\
	__m128i condp = _mm_and_si128(diff_lte(p2, p0, bm1), condpq);\
	__m128i condq = _mm_and_si128(diff_lte(q2, q0, bm1), condpq);\
	__m128i c1 = _mm_set1_epi8(1);\
	/* compute p'0 and q'0 */\
	__m128i fix0 = _mm_and_si128(_mm_xor_si128(p0, q0), c1);\
	__m128i pq0 = _mm_sub_epi8(_mm_avg_epu8(p0, q0), fix0);\
	__m128i and0 = _mm_xor_si128(fix0, c1);\
	__m128i p2q1 = _mm_sub_epi8(_mm_avg_epu8(p2, q1), _mm_and_si128(_mm_xor_si128(p2, q1), c1)); /* (p2+q1)/2 */\
	__m128i q2p1 = _mm_sub_epi8(_mm_avg_epu8(q2, p1), _mm_and_si128(_mm_xor_si128(q2, p1), c1)); /* (q2+p1)/2 */\
	__m128i p21q1 = _mm_sub_epi8(_mm_avg_epu8(p2q1, p1), _mm_and_si128(_mm_xor_si128(p2q1, p1), and0)); /* p21q1+pq0 == (p2q1+p1+p0+q0)/2 */\
	__m128i q21p1 = _mm_sub_epi8(_mm_avg_epu8(q2p1, q1), _mm_and_si128(_mm_xor_si128(q2p1, q1), and0)); /* q21p1+pq0 == (q2p1+q1+p0+q0)/2 */\
	__m128i pp0a = _mm_avg_epu8(p21q1, pq0); /* p'0 (first formula) */\
	__m128i qp0a = _mm_avg_epu8(q21p1, pq0); /* q'0 (first formula) */\
	__m128i pp0b = _mm_avg_epu8(p1, _mm_sub_epi8(_mm_avg_epu8(p0, q1), _mm_and_si128(_mm_xor_si128(p0, q1), c1))); /* p'0 (second formula) */\
	__m128i qp0b = _mm_avg_epu8(q1, _mm_sub_epi8(_mm_avg_epu8(q0, p1), _mm_and_si128(_mm_xor_si128(q0, p1), c1))); /* q'0 (second formula) */\
	p0 = vector_select(condp, pp0a, pp0b);\
	q0 = vector_select(condq, qp0a, qp0b);\
	/* compute p'1 and q'1 */\
	__m128i p21 = _mm_sub_epi8(_mm_avg_epu8(p2, p1), _mm_and_si128(_mm_xor_si128(p2, p1), and0)); /* p21+pq0 == (p2+p1+p0+q0)/2 */\
	__m128i q21 = _mm_sub_epi8(_mm_avg_epu8(q2, q1), _mm_and_si128(_mm_xor_si128(q2, q1), and0)); /* q21+pq0 == (q2+q1+q0+p1)/2 */\
	__m128i pp1 = _mm_avg_epu8(p21, pq0); /* p'1 */\
	__m128i qp1 = _mm_avg_epu8(q21, pq0); /* q'1 */\
	p1 = vector_select(condp, pp1, p1);\
	q1 = vector_select(condq, qp1, q1);\
	/* compute p'2 and q'2 */\
	__m128i fix1 = _mm_and_si128(_mm_xor_si128(p21, pq0), c1);\
	__m128i fix2 = _mm_and_si128(_mm_xor_si128(q21, pq0), c1);\
	__m128i p210q0 = _mm_sub_epi8(pp1, fix1); /* (p2+p1+p0+q0)/4 */\
	__m128i q210p0 = _mm_sub_epi8(qp1, fix2); /* (q2+q1+q0+p0)/4 */\
	__m128i p3p2 = _mm_sub_epi8(_mm_avg_epu8(p3, p2), _mm_and_si128(_mm_xor_si128(p3, p2), _mm_xor_si128(fix1, c1))); /* p3p2+p210q0 == (p3+p2+(p2+p1+p0+q0)/2)/2 */\
	__m128i q3q2 = _mm_sub_epi8(_mm_avg_epu8(q3, q2), _mm_and_si128(_mm_xor_si128(q3, q2), _mm_xor_si128(fix2, c1))); /* q3q2+q210p0 == (q3+q2+(q2+q1+p0+q0)/2)/2 */\
	__m128i pp2 = _mm_avg_epu8(p3p2, p210q0); /* p'2 */\
	__m128i qp2 = _mm_avg_epu8(q3q2, q210p0); /* q'2 */\
	p2 = vector_select(condp, pp2, p2);\
	q2 = vector_select(condq, qp2, q2);}



#define TRANSPOSE_8x16(src, half, dst0, dst1, dst2, dst3, dst4, dst5, dst6, dst7) {\
	__m128i a0 = _mm_unpack##half##_epi8(src##0, src##1);\
	__m128i a1 = _mm_unpack##half##_epi8(src##2, src##3);\
	__m128i a2 = _mm_unpack##half##_epi8(src##4, src##5);\
	__m128i a3 = _mm_unpack##half##_epi8(src##6, src##7);\
	__m128i a4 = _mm_unpack##half##_epi8(src##8, src##9);\
	__m128i a5 = _mm_unpack##half##_epi8(src##A, src##B);\
	__m128i a6 = _mm_unpack##half##_epi8(src##C, src##D);\
	__m128i a7 = _mm_unpack##half##_epi8(src##E, src##F);\
	__m128i b0 = _mm_unpacklo_epi16(a0, a1);\
	__m128i b1 = _mm_unpackhi_epi16(a0, a1);\
	__m128i b2 = _mm_unpacklo_epi16(a2, a3);\
	__m128i b3 = _mm_unpackhi_epi16(a2, a3);\
	__m128i b4 = _mm_unpacklo_epi16(a4, a5);\
	__m128i b5 = _mm_unpackhi_epi16(a4, a5);\
	__m128i b6 = _mm_unpacklo_epi16(a6, a7);\
	__m128i b7 = _mm_unpackhi_epi16(a6, a7);\
	__m128i c0 = _mm_unpacklo_epi32(b0, b2);\
	__m128i c1 = _mm_unpackhi_epi32(b0, b2);\
	__m128i c2 = _mm_unpacklo_epi32(b4, b6);\
	__m128i c3 = _mm_unpackhi_epi32(b4, b6);\
	__m128i c4 = _mm_unpacklo_epi32(b1, b3);\
	__m128i c5 = _mm_unpackhi_epi32(b1, b3);\
	__m128i c6 = _mm_unpacklo_epi32(b3, b5);\
	__m128i c7 = _mm_unpackhi_epi32(b3, b5);\
	dst0 = _mm_unpacklo_epi64(c0, c2);\
	dst1 = _mm_unpackhi_epi64(c0, c2);\
	dst2 = _mm_unpacklo_epi64(c1, c3);\
	dst3 = _mm_unpackhi_epi64(c1, c3);\
	dst4 = _mm_unpacklo_epi64(c4, c6);\
	dst5 = _mm_unpackhi_epi64(c4, c6);\
	dst6 = _mm_unpacklo_epi64(c5, c7);\
	dst7 = _mm_unpackhi_epi64(c5, c7);}



void FUNC(deblock_luma_row, size_t stride, ssize_t nstride, uint8_t*restrict px0, uint8_t*restrict px7, uint8_t*restrict pxE, uint8_t*restrict pxX, __m128i tc0, int alpha, int beta)
{
	do {
		// first vertical edge
		__m128i v0, v1, v2, v3, v4, v5, v6, v7;
		if (1) { // FIXME with QP
			// load and transpose the left 12x16 matrix
			__m128i xa0 = _mm_loadu_si128((__m128i *)(px0               - 8));
			__m128i xa1 = _mm_loadu_si128((__m128i *)(px0 +  stride     - 8));
			__m128i xa2 = _mm_loadu_si128((__m128i *)(px0 +  stride * 2 - 8));
			__m128i xa3 = _mm_loadu_si128((__m128i *)(px7 + nstride * 4 - 8));
			__m128i xa4 = _mm_loadu_si128((__m128i *)(px0 +  stride * 4 - 8));
			__m128i xa5 = _mm_loadu_si128((__m128i *)(px7 + nstride * 2 - 8));
			__m128i xa6 = _mm_loadu_si128((__m128i *)(px7 + nstride     - 8));
			__m128i xa7 = _mm_loadu_si128((__m128i *)(px7               - 8));
			__m128i xa8 = _mm_loadu_si128((__m128i *)(px7 +  stride     - 8));
			__m128i xa9 = _mm_loadu_si128((__m128i *)(px7 +  stride * 2 - 8));
			__m128i xaA = _mm_loadu_si128((__m128i *)(pxE + nstride * 4 - 8));
			__m128i xaB = _mm_loadu_si128((__m128i *)(px7 +  stride * 4 - 8));
			__m128i xaC = _mm_loadu_si128((__m128i *)(pxE + nstride * 2 - 8));
			__m128i xaD = _mm_loadu_si128((__m128i *)(pxE + nstride     - 8));
			__m128i xaE = _mm_loadu_si128((__m128i *)(pxE               - 8));
			__m128i xaF = _mm_loadu_si128((__m128i *)(pxE +  stride     - 8));
			__m128i xb0 = _mm_unpackhi_epi16(_mm_unpacklo_epi8(xa0, xa1), _mm_unpacklo_epi8(xa2, xa3));
			__m128i xb1 = _mm_unpackhi_epi16(_mm_unpacklo_epi8(xa4, xa5), _mm_unpacklo_epi8(xa6, xa7));
			__m128i xb2 = _mm_unpackhi_epi16(_mm_unpacklo_epi8(xa8, xa9), _mm_unpacklo_epi8(xaA, xaB));
			__m128i xb3 = _mm_unpackhi_epi16(_mm_unpacklo_epi8(xaC, xaD), _mm_unpacklo_epi8(xaE, xaF));
			__m128i xb4 = _mm_unpacklo_epi32(xb0, xb1);
			__m128i xb5 = _mm_unpackhi_epi32(xb0, xb1);
			__m128i xb6 = _mm_unpacklo_epi32(xb2, xb3);
			__m128i xb7 = _mm_unpackhi_epi32(xb2, xb3);
			__m128i vW = _mm_unpacklo_epi64(xb4, xb6);
			__m128i vX = _mm_unpackhi_epi64(xb4, xb6);
			__m128i vY = _mm_unpacklo_epi64(xb5, xb7);
			__m128i vZ = _mm_unpackhi_epi64(xb5, xb7);
			TRANSPOSE_8x16(xa, hi, v0, v1, v2, v3, v4, v5, v6, v7);
			
			if (1) { // FIXME is intra block
				DEBLOCK_LUMA_HARD(vW, vX, vY, vZ, v0, v1, v2, v3);
			} else {
				DEBLOCK_LUMA_SOFT(vX, vY, vZ, v0, v1, v2);
			}
			
			// store vW/vX/vY/vZ into the previous macroblock
			__m128i xc0 = _mm_unpacklo_epi8(vW, vX);
			__m128i xc1 = _mm_unpackhi_epi8(vW, vX);
			__m128i xc2 = _mm_unpacklo_epi8(vY, vZ);
			__m128i xc3 = _mm_unpackhi_epi8(vY, vZ);
			v4si xc4 = (v4si)_mm_unpacklo_epi16(xc0, xc1);
			v4si xc5 = (v4si)_mm_unpackhi_epi16(xc0, xc1);
			v4si xc6 = (v4si)_mm_unpacklo_epi16(xc2, xc3);
			v4si xc7 = (v4si)_mm_unpackhi_epi16(xc2, xc3);
			*(int32_t *)(px0               - 4) = xc4[0];
			*(int32_t *)(px0 +  stride     - 4) = xc4[1];
			*(int32_t *)(px0 +  stride * 2 - 4) = xc4[2];
			*(int32_t *)(px7 + nstride * 4 - 4) = xc4[3];
			*(int32_t *)(px0 +  stride * 4 - 4) = xc5[0];
			*(int32_t *)(px7 + nstride * 2 - 4) = xc5[1];
			*(int32_t *)(px7 + nstride     - 4) = xc5[2];
			*(int32_t *)(px7               - 4) = xc5[3];
			*(int32_t *)(px7 +  stride     - 4) = xc6[0];
			*(int32_t *)(px7 +  stride * 2 - 4) = xc6[1];
			*(int32_t *)(pxE + nstride * 4 - 4) = xc6[2];
			*(int32_t *)(px7 +  stride * 4 - 4) = xc6[3];
			*(int32_t *)(pxE + nstride * 2 - 4) = xc7[0];
			*(int32_t *)(pxE + nstride     - 4) = xc7[1];
			*(int32_t *)(pxE               - 4) = xc7[2];
			*(int32_t *)(pxE +  stride     - 4) = xc7[3];
		} else {
			// load and transpose the left 8x16 matrix
			__m128i xa0 = *(__m128i *)(px0              );
			__m128i xa1 = *(__m128i *)(px0 +  stride    );
			__m128i xa2 = *(__m128i *)(px0 +  stride * 2);
			__m128i xa3 = *(__m128i *)(px7 + nstride * 4);
			__m128i xa4 = *(__m128i *)(px0 +  stride * 4);
			__m128i xa5 = *(__m128i *)(px7 + nstride * 2);
			__m128i xa6 = *(__m128i *)(px7 + nstride    );
			__m128i xa7 = *(__m128i *)(px7              );
			__m128i xa8 = *(__m128i *)(px7 +  stride    );
			__m128i xa9 = *(__m128i *)(px7 +  stride * 2);
			__m128i xaA = *(__m128i *)(pxE + nstride * 4);
			__m128i xaB = *(__m128i *)(px7 +  stride * 4);
			__m128i xaC = *(__m128i *)(pxE + nstride * 2);
			__m128i xaD = *(__m128i *)(pxE + nstride    );
			__m128i xaE = *(__m128i *)(pxE              );
			__m128i xaF = *(__m128i *)(pxE +  stride    );
			TRANSPOSE_8x16(xa, lo, v0, v1, v2, v3, v4, v5, v6, v7);
		}
		
		// second vertical edge
		if (!mb->f.transform_size_8x8_flag) {
			DEBLOCK_LUMA_SOFT(v1, v2, v3, v4, v5, v6);
		}
		
		// load and transpose the right 8x16 matrix
		__m128i xa0 = *(__m128i *)(px0              );
		__m128i xa1 = *(__m128i *)(px0 +  stride    );
		__m128i xa2 = *(__m128i *)(px0 +  stride * 2);
		__m128i xa3 = *(__m128i *)(px7 + nstride * 4);
		__m128i xa4 = *(__m128i *)(px0 +  stride * 4);
		__m128i xa5 = *(__m128i *)(px7 + nstride * 2);
		__m128i xa6 = *(__m128i *)(px7 + nstride    );
		__m128i xa7 = *(__m128i *)(px7              );
		__m128i xa8 = *(__m128i *)(px7 +  stride    );
		__m128i xa9 = *(__m128i *)(px7 +  stride * 2);
		__m128i xaA = *(__m128i *)(pxE + nstride * 4);
		__m128i xaB = *(__m128i *)(px7 +  stride * 4);
		__m128i xaC = *(__m128i *)(pxE + nstride * 2);
		__m128i xaD = *(__m128i *)(pxE + nstride    );
		__m128i xaE = *(__m128i *)(pxE              );
		__m128i xaF = *(__m128i *)(pxE +  stride    );
		__m128i v8, v9, vA, vB, vC, vD, vE, vF;
		TRANSPOSE_8x16(xa, hi, v8, v9, vA, vB, vC, vD, vE, vF);
		
		// third vertical edge
		DEBLOCK_LUMA_SOFT(v5, v6, v7, v8, v9, vA);
		
		// fourth vertical edge
		if (!mb->f.transform_size_8x8_flag) {
			DEBLOCK_LUMA_SOFT(v9, vA, vB, vC, vD, vE);
		}
		
		// transpose the top 16x8 matrix
		__m128i h0, h1, h2, h3, h4, h5, h6, h7;
		TRANSPOSE_8x16(v, lo, h0, h1, h2, h3, h4, h5, h6, h7);
		
		// first horizontal edge
		if (!ctx->mbB->f.unavailable) { // FIXME with QP
			if (1) { // FIXME
				DEBLOCK_LUMA_HARD(*(__m128i *)(px0 + nstride * 4), *(__m128i *)(pxX              ), *(__m128i *)(px0 + nstride * 2), *(__m128i *)(px0 + nstride    ), h0, h1, h2, h3);
			} else {
				DEBLOCK_LUMA_SOFT(*(__m128i *)(pxX              ), *(__m128i *)(px0 + nstride * 2), *(__m128i *)(px0 + nstride    ), h0, h1, h2);
			}
		}
		*(__m128i *)(px0              ) = h0;
		*(__m128i *)(px0 +  stride    ) = h1;
		
		// second horizontal edge
		if (!mb->f.transform_size_8x8_flag) {
			DEBLOCK_LUMA_SOFT(h1, h2, h3, h4, h5, h6);
		}
		*(__m128i *)(px0 +  stride * 2) = h2;
		*(__m128i *)(px7 + nstride * 4) = h3;
		*(__m128i *)(px0 +  stride * 4) = h4;
		*(__m128i *)(px7 + nstride * 2) = h5;
		
		// transpose the bottom 16x8 matrix
		__m128i h8, h9, hA, hB, hC, hD, hE, hF;
		TRANSPOSE_8x16(v, hi, h8, h9, hA, hB, hC, hD, hE, hF);
		*(__m128i *)(pxE              ) = hE;
		*(__m128i *)(pxE +  stride    ) = hF;
		
		// third horizontal edge
		DEBLOCK_LUMA_SOFT(h5, h6, h7, h8, h9, hA);
		*(__m128i *)(px7 + nstride    ) = h6;
		*(__m128i *)(px7              ) = h7;
		*(__m128i *)(px7 +  stride    ) = h8;
		*(__m128i *)(px7 +  stride * 2) = h9;
		
		// fourth horizontal edge
		if (!mb->f.transform_size_8x8_flag) {
			DEBLOCK_LUMA_SOFT(h9, hA, hB, hC, hD, hE);
		}
		*(__m128i *)(pxE + nstride * 4) = hA;
		*(__m128i *)(px7 +  stride * 4) = hB;
		*(__m128i *)(pxE + nstride * 2) = hC;
		*(__m128i *)(pxE + nstride    ) = hD;
		
		// point to the next macroblock
		mb++;
		px0 += 16;
		px7 += 16;
		pxE += 16;
		pxX += 16;
	} while (1); // FIXME
}
