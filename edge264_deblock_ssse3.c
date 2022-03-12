#include "edge264_common.h"



// compute [a-b]<=c
static __m128i diff_lte(__m128i a, __m128i b, __m128i c) {
	return _mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(a, b), c), _mm_subs_epu8(_mm_subs_epu8(b, a), c));
}

void tail6(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
void tail8(__m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);

void filter123_luma(__m128i p2, __m128i p1, __m128i p0, __m128i q0, __m128i q1, __m128i q2, __m128i tc0, int alpha, int beta) {
	// compute a mask for each lane that will be filtered, and apply it to tc0
	__m128i c128 = _mm_set1_epi8(-128);
	__m128i pq0 = _mm_subs_epu8(p0, q0);
	__m128i qp0 = _mm_subs_epu8(q0, p0);
	__m128i abs0 = _mm_or_si128(pq0, qp0);
	__m128i sub0 = _mm_subs_epu8(_mm_adds_epu8(qp0, c128), pq0); // save 128+q0-p0 for later
	__m128i abs1 = _mm_or_si128(_mm_subs_epu8(p1, p0), _mm_subs_epu8(p0, p1));
	__m128i abs2 = _mm_or_si128(_mm_subs_epu8(q1, q0), _mm_subs_epu8(q0, q1));
	__m128i am1 = _mm_set1_epi8(alpha - 1);
	__m128i bm1 = _mm_set1_epi8(beta - 1);
	__m128i filterSamplesFlags = _mm_cmpeq_epi8(_mm_or_si128(_mm_subs_epu8(abs0, am1), _mm_subs_epu8(_mm_max_epu8(abs1, abs2), bm1)), _mm_setzero_si128());
	tc0 = _mm_and_si128(tc0, filterSamplesFlags);
	
	// filter p1 and q1 (same as ffmpeg, I couldn't find better)
	__m128i c1 = _mm_set1_epi8(1);
	__m128i x0 = _mm_avg_epu8(p0, q0); // (p0+q0+1)>>1
	__m128i x1 = _mm_sub_epi8(_mm_avg_epu8(p2, x0), _mm_and_si128(_mm_xor_si128(p2, x0), c1)); // (p2+((p0+q0+1)>>1))>>1
	__m128i x2 = _mm_sub_epi8(_mm_avg_epu8(q2, x0), _mm_and_si128(_mm_xor_si128(q2, x0), c1)); // (p2+((p0+q0+1)>>1))>>1
	__m128i pp1 = _mm_min_epu8(_mm_max_epu8(x1, _mm_subs_epu8(p1, tc0)), _mm_adds_epu8(p1, tc0));
	__m128i qp1 = _mm_min_epu8(_mm_max_epu8(x2, _mm_subs_epu8(q1, tc0)), _mm_adds_epu8(q1, tc0));
	__m128i apltb = diff_lte(p2, p0, bm1);
	__m128i aqltb = diff_lte(q2, q0, bm1);
	__m128i sub1 = _mm_avg_epu8(p1, _mm_xor_si128(q1, _mm_set1_epi8(-1))); // save 128+((p1-q1)>>1) for later
	p1 = vector_select(apltb, pp1, p1);
	q1 = vector_select(aqltb, qp1, q1);
	
	// filter p0 and q0 (by offsetting signed to unsigned to apply pavg)
	__m128i tc = _mm_sub_epi8(_mm_sub_epi8(tc0, apltb), aqltb);
	__m128i x3 = _mm_avg_epu8(sub0, _mm_avg_epu8(sub1, _mm_set1_epi8(127))); // 128+((q0-p0+((p1-q1)>>2)+1)>>1)
	__m128i delta = _mm_min_epu8(_mm_subs_epu8(x3, c128), tc); // delta if delta>0
	__m128i ndelta = _mm_min_epu8(_mm_subs_epu8(c128, x3), tc); // -delta if delta<0
	p0 = _mm_subs_epu8(_mm_adds_epu8(p0, delta), ndelta);
	q0 = _mm_subs_epu8(_mm_adds_epu8(p0, delta), ndelta);
	tail6(p2, p1, p0, q0, q1, q2);
}



void filter4_luma(__m128i p3, __m128i p2, __m128i p1, __m128i p0, __m128i q0, __m128i q1, __m128i q2, __m128i q3, int alpha, int beta) {
	// compute condition masks for filtering modes
	__m128i bm1 = _mm_set1_epi8(beta - 1);
	__m128i condpq = diff_lte(p0, q0, _mm_set1_epi8((alpha >> 2) + 1));
	__m128i condp = _mm_and_si128(diff_lte(p2, p0, bm1), condpq);
	__m128i condq = _mm_and_si128(diff_lte(q2, q0, bm1), condpq);
	__m128i c1 = _mm_set1_epi8(1);
	
	// compute p'0 and q'0
	__m128i fix0 = _mm_and_si128(_mm_xor_si128(p0, q0), c1);
	__m128i pq0 = _mm_sub_epi8(_mm_avg_epu8(p0, q0), fix0);
	__m128i and0 = _mm_xor_si128(fix0, c1);
	__m128i p2q1 = _mm_sub_epi8(_mm_avg_epu8(p2, q1), _mm_and_si128(_mm_xor_si128(p2, q1), c1)); // (p2+q1)/2
	__m128i q2p1 = _mm_sub_epi8(_mm_avg_epu8(q2, p1), _mm_and_si128(_mm_xor_si128(q2, p1), c1)); // (q2+p1)/2
	__m128i p21q1 = _mm_sub_epi8(_mm_avg_epu8(p2q1, p1), _mm_and_si128(_mm_xor_si128(p2q1, p1), and0)); // p21q1+pq0 == (p2q1+p1+p0+q0)/2
	__m128i q21p1 = _mm_sub_epi8(_mm_avg_epu8(q2p1, q1), _mm_and_si128(_mm_xor_si128(q2p1, q1), and0)); // q21p1+pq0 == (q2p1+q1+p0+q0)/2
	__m128i pp0a = _mm_avg_epu8(p21q1, pq0); // p'0 (first formula)
	__m128i qp0a = _mm_avg_epu8(q21p1, pq0); // q'0 (first formula)
	__m128i pp0b = _mm_avg_epu8(p1, _mm_sub_epi8(_mm_avg_epu8(p0, q1), _mm_and_si128(_mm_xor_si128(p0, q1), c1))); // p'0 (second formula)
	__m128i qp0b = _mm_avg_epu8(q1, _mm_sub_epi8(_mm_avg_epu8(q0, p1), _mm_and_si128(_mm_xor_si128(q0, p1), c1))); // q'0 (second formula)
	p0 = vector_select(condp, pp0a, pp0b);
	q0 = vector_select(condq, qp0a, qp0b);
	
	// compute p'1 and q'1
	__m128i p21 = _mm_sub_epi8(_mm_avg_epu8(p2, p1), _mm_and_si128(_mm_xor_si128(p2, p1), and0)); // p21+pq0 == (p2+p1+p0+q0)/2
	__m128i q21 = _mm_sub_epi8(_mm_avg_epu8(q2, q1), _mm_and_si128(_mm_xor_si128(q2, q1), and0)); // q21+pq0 == (q2+q1+q0+p1)/2
	__m128i pp1 = _mm_avg_epu8(p21, pq0); // p'1
	__m128i qp1 = _mm_avg_epu8(q21, pq0); // q'1
	p1 = vector_select(condp, pp1, p1);
	q1 = vector_select(condq, qp1, q1);
	
	// compute p'2 and q'2
	__m128i fix1 = _mm_and_si128(_mm_xor_si128(p21, pq0), c1);
	__m128i fix2 = _mm_and_si128(_mm_xor_si128(q21, pq0), c1);
	__m128i p210q0 = _mm_sub_epi8(pp1, fix1); // (p2+p1+p0+q0)/4
	__m128i q210p0 = _mm_sub_epi8(qp1, fix2); // (q2+q1+q0+p0)/4
	__m128i p3p2 = _mm_sub_epi8(_mm_avg_epu8(p3, p2), _mm_and_si128(_mm_xor_si128(p3, p2), _mm_xor_si128(fix1, c1))); // p3p2+p210q0 == (p3+p2+(p2+p1+p0+q0)/2)/2
	__m128i q3q2 = _mm_sub_epi8(_mm_avg_epu8(q3, q2), _mm_and_si128(_mm_xor_si128(q3, q2), _mm_xor_si128(fix2, c1))); // q3q2+q210p0 == (q3+q2+(q2+q1+p0+q0)/2)/2
	__m128i pp2 = _mm_avg_epu8(p3p2, p210q0); // p'2
	__m128i qp2 = _mm_avg_epu8(q3q2, q210p0); // q'2
	p2 = vector_select(condp, pp2, p2);
	q2 = vector_select(condq, qp2, q2);
	tail8(p3, p2, p1, p0, q0, q1, q2, q3);
}



void FUNC(deblock_luma_row, size_t stride, ssize_t nstride, uint8_t *px0, uint8_t *px7, uint8_t *pxE, uint8_t *pxX)
{
	// load and transpose the left 8x16 matrix (FIXME order reads top to bottom)
	__m128i xa0 = _mm_unpacklo_epi8(*(__m128i *)(px0              ), *(__m128i *)(px0 +  stride    ));
	__m128i xa1 = _mm_unpacklo_epi8(*(__m128i *)(px0 +  stride * 2), *(__m128i *)(px7 + nstride * 4));
	__m128i xa2 = _mm_unpacklo_epi8(*(__m128i *)(px0 +  stride * 4), *(__m128i *)(px7 + nstride * 2));
	__m128i xa3 = _mm_unpacklo_epi8(*(__m128i *)(px7 + nstride    ), *(__m128i *)(px7              ));
	__m128i xa4 = _mm_unpacklo_epi8(*(__m128i *)(px7 +  stride    ), *(__m128i *)(px7 +  stride * 2));
	__m128i xa5 = _mm_unpacklo_epi8(*(__m128i *)(pxE + nstride * 4), *(__m128i *)(px7 +  stride * 4));
	__m128i xa6 = _mm_unpacklo_epi8(*(__m128i *)(pxE + nstride * 2), *(__m128i *)(pxE + nstride    ));
	__m128i xa7 = _mm_unpacklo_epi8(*(__m128i *)(pxE              ), *(__m128i *)(pxE +  stride    ));
	__m128i xb0 = _mm_unpacklo_epi16(xa0, xa1);
	__m128i xb1 = _mm_unpackhi_epi16(xa0, xa1);
	__m128i xb2 = _mm_unpacklo_epi16(xa2, xa3);
	__m128i xb3 = _mm_unpackhi_epi16(xa2, xa3);
	__m128i xb4 = _mm_unpacklo_epi16(xa4, xa5);
	__m128i xb5 = _mm_unpackhi_epi16(xa4, xa5);
	__m128i xb6 = _mm_unpacklo_epi16(xa6, xa7);
	__m128i xb7 = _mm_unpackhi_epi16(xa6, xa7);
	__m128i xc0 = _mm_unpacklo_epi32(xb0, xb2);
	__m128i xc1 = _mm_unpackhi_epi32(xb0, xb2);
	__m128i xc2 = _mm_unpacklo_epi32(xb4, xb6);
	__m128i xc3 = _mm_unpackhi_epi32(xb4, xb6);
	__m128i xc4 = _mm_unpacklo_epi32(xb1, xb3);
	__m128i xc5 = _mm_unpackhi_epi32(xb1, xb3);
	__m128i xc6 = _mm_unpacklo_epi32(xb3, xb5);
	__m128i xc7 = _mm_unpackhi_epi32(xb3, xb5);
	__m128i v0 = _mm_unpacklo_epi64(xc0, xc2);
	__m128i v1 = _mm_unpackhi_epi64(xc0, xc2);
	__m128i v2 = _mm_unpacklo_epi64(xc1, xc3);
	__m128i v3 = _mm_unpackhi_epi64(xc1, xc3);
	__m128i v4 = _mm_unpacklo_epi64(xc4, xc6);
	__m128i v5 = _mm_unpackhi_epi64(xc4, xc6);
	__m128i v6 = _mm_unpacklo_epi64(xc5, xc7);
	__m128i v7 = _mm_unpackhi_epi64(xc5, xc7);
	
	goto start;
	do {
		// first vertical edge
		if (1) { // FIXME with QP
			if (1) { // FIXME is intra block
				FILTER_HARD(vW, vX, vY, vZ, v0, v1, v2, v3);
			} else {
				FILTER_SOFT(vX, vY, vZ, v0, v1, v2);
			}
			// FIXME merge vW/vX/vY/vZ into the previous macroblock and store it
		}
		
		// second vertical edge
		start:
		if (!mb->f.transform_size_8x8_flag) {
			FILTER_SOFT(v1, v2, v3, v4, v5, v6);
		}
		
		// load and transpose the right 8x16 matrix
		__m128i xd0 = _mm_unpackhi_epi8(*(__m128i *)(px0              ), *(__m128i *)(px0 +  stride    ));
		__m128i xd1 = _mm_unpackhi_epi8(*(__m128i *)(px0 +  stride * 2), *(__m128i *)(px7 + nstride * 4));
		__m128i xd2 = _mm_unpackhi_epi8(*(__m128i *)(px0 +  stride * 4), *(__m128i *)(px7 + nstride * 2));
		__m128i xd3 = _mm_unpackhi_epi8(*(__m128i *)(px7 + nstride    ), *(__m128i *)(px7              ));
		__m128i xd4 = _mm_unpackhi_epi8(*(__m128i *)(px7 +  stride    ), *(__m128i *)(px7 +  stride * 2));
		__m128i xd5 = _mm_unpackhi_epi8(*(__m128i *)(pxE + nstride * 4), *(__m128i *)(px7 +  stride * 4));
		__m128i xd6 = _mm_unpackhi_epi8(*(__m128i *)(pxE + nstride * 2), *(__m128i *)(pxE + nstride    ));
		__m128i xd7 = _mm_unpackhi_epi8(*(__m128i *)(pxE              ), *(__m128i *)(pxE +  stride    ));
		__m128i xe0 = _mm_unpacklo_epi16(xd0, xd1);
		__m128i xe1 = _mm_unpackhi_epi16(xd0, xd1);
		__m128i xe2 = _mm_unpacklo_epi16(xd2, xd3);
		__m128i xe3 = _mm_unpackhi_epi16(xd2, xd3);
		__m128i xe4 = _mm_unpacklo_epi16(xd4, xd5);
		__m128i xe5 = _mm_unpackhi_epi16(xd4, xd5);
		__m128i xe6 = _mm_unpacklo_epi16(xd6, xd7);
		__m128i xe7 = _mm_unpackhi_epi16(xd6, xd7);
		__m128i xf0 = _mm_unpacklo_epi32(xe0, xe2);
		__m128i xf1 = _mm_unpackhi_epi32(xe0, xe2);
		__m128i xf2 = _mm_unpacklo_epi32(xe4, xe6);
		__m128i xf3 = _mm_unpackhi_epi32(xe4, xe6);
		__m128i xf4 = _mm_unpacklo_epi32(xe1, xe3);
		__m128i xf5 = _mm_unpackhi_epi32(xe1, xe3);
		__m128i xf6 = _mm_unpacklo_epi32(xe5, xe7);
		__m128i xf7 = _mm_unpackhi_epi32(xe5, xe7);
		__m128i v8 = _mm_unpacklo_epi64(xf0, xf2);
		__m128i v9 = _mm_unpackhi_epi64(xf0, xf2);
		__m128i vA = _mm_unpacklo_epi64(xf1, xf3);
		__m128i vB = _mm_unpackhi_epi64(xf1, xf3);
		__m128i vC = _mm_unpacklo_epi64(xf4, xf6);
		__m128i vD = _mm_unpackhi_epi64(xf4, xf6);
		__m128i vE = _mm_unpacklo_epi64(xf5, xf7);
		__m128i vF = _mm_unpackhi_epi64(xf5, xf7);
		
		// third vertical edge
		FILTER_SOFT(v5, v6, v7, v8, v9, vA);
		
		// fourth vertical edge
		if (!mb->f.transform_size_8x8_flag) {
			FILTER_SOFT(v9, vA, vB, vC, vD, vE);
		}
		
		// transpose the top 16x8 matrix
		__m128i xg0 = _mm_unpacklo_epi8(v0, v1);
		__m128i xg1 = _mm_unpacklo_epi8(v2, v3);
		__m128i xg2 = _mm_unpacklo_epi8(v4, v5);
		__m128i xg3 = _mm_unpacklo_epi8(v6, v7);
		__m128i xg4 = _mm_unpacklo_epi8(v8, v9);
		__m128i xg5 = _mm_unpacklo_epi8(vA, vB);
		__m128i xg6 = _mm_unpacklo_epi8(vC, vD);
		__m128i xg7 = _mm_unpacklo_epi8(vE, vF);
		__m128i xh0 = _mm_unpacklo_epi16(xg0, xg1);
		__m128i xh1 = _mm_unpackhi_epi16(xg0, xg1);
		__m128i xh2 = _mm_unpacklo_epi16(xg2, xg3);
		__m128i xh3 = _mm_unpackhi_epi16(xg2, xg3);
		__m128i xh4 = _mm_unpacklo_epi16(xg4, xg5);
		__m128i xh5 = _mm_unpackhi_epi16(xg4, xg5);
		__m128i xh6 = _mm_unpacklo_epi16(xg6, xg7);
		__m128i xh7 = _mm_unpackhi_epi16(xg6, xg7);
		__m128i xi0 = _mm_unpacklo_epi32(xh0, xh2);
		__m128i xi1 = _mm_unpackhi_epi32(xh0, xh2);
		__m128i xi2 = _mm_unpacklo_epi32(xh4, xh6);
		__m128i xi3 = _mm_unpackhi_epi32(xh4, xh6);
		__m128i xi4 = _mm_unpacklo_epi32(xh1, xh3);
		__m128i xi5 = _mm_unpackhi_epi32(xh1, xh3);
		__m128i xi6 = _mm_unpacklo_epi32(xh5, xh7);
		__m128i xi7 = _mm_unpackhi_epi32(xh5, xh7);
		__m128i h0 = _mm_unpacklo_epi64(xi0, xi2);
		__m128i h1 = _mm_unpackhi_epi64(xi0, xi2);
		__m128i h2 = _mm_unpacklo_epi64(xi1, xi3);
		__m128i h3 = _mm_unpackhi_epi64(xi1, xi3);
		__m128i h4 = _mm_unpacklo_epi64(xi4, xi6);
		__m128i h5 = _mm_unpackhi_epi64(xi4, xi6);
		__m128i h6 = _mm_unpacklo_epi64(xi5, xi7);
		__m128i h7 = _mm_unpackhi_epi64(xi5, xi7);
		
		// first horizontal edge
		if (!ctx->mbB->f.unavailable) { // FIXME with QP
			if (1) { // FIXME
				FILTER_HARD(*(__m128i *)(px0 + nstride * 4), *(__m128i *)(pxX              ), *(__m128i *)(px0 + nstride * 2), *(__m128i *)(px0 + nstride    ), h0, h1, h2, h3);
			} else {
				FILTER_SOFT(*(__m128i *)(pxX              ), *(__m128i *)(px0 + nstride * 2), *(__m128i *)(px0 + nstride    ), h0, h1, h2);
			}
		}
		
		// second horizontal edge
		if (!mb->f.transform_size_8x8_flag) {
			FILTER_SOFT(h1, h2, h3, h4, h5, h6);
		}
		
		// transpose the bottom 16x8 matrix
		__m128i xj0 = _mm_unpacklo_epi8(v0, v1);
		__m128i xj1 = _mm_unpacklo_epi8(v2, v3);
		__m128i xj2 = _mm_unpacklo_epi8(v4, v5);
		__m128i xj3 = _mm_unpacklo_epi8(v6, v7);
		__m128i xj4 = _mm_unpacklo_epi8(v8, v9);
		__m128i xj5 = _mm_unpacklo_epi8(vA, vB);
		__m128i xj6 = _mm_unpacklo_epi8(vC, vD);
		__m128i xj7 = _mm_unpacklo_epi8(vE, vF);
		__m128i xk0 = _mm_unpacklo_epi16(xj0, xj1);
		__m128i xk1 = _mm_unpackhi_epi16(xj0, xj1);
		__m128i xk2 = _mm_unpacklo_epi16(xj2, xj3);
		__m128i xk3 = _mm_unpackhi_epi16(xj2, xj3);
		__m128i xk4 = _mm_unpacklo_epi16(xj4, xj5);
		__m128i xk5 = _mm_unpackhi_epi16(xj4, xj5);
		__m128i xk6 = _mm_unpacklo_epi16(xj6, xj7);
		__m128i xk7 = _mm_unpackhi_epi16(xj6, xj7);
		__m128i xl0 = _mm_unpacklo_epi32(xk0, xk2);
		__m128i xl1 = _mm_unpackhi_epi32(xk0, xk2);
		__m128i xl2 = _mm_unpacklo_epi32(xk4, xk6);
		__m128i xl3 = _mm_unpackhi_epi32(xk4, xk6);
		__m128i xl4 = _mm_unpacklo_epi32(xk1, xk3);
		__m128i xl5 = _mm_unpackhi_epi32(xk1, xk3);
		__m128i xl6 = _mm_unpacklo_epi32(xk5, xk7);
		__m128i xl7 = _mm_unpackhi_epi32(xk5, xk7);
		__m128i h8 = _mm_unpacklo_epi64(xl0, xl2);
		__m128i h9 = _mm_unpackhi_epi64(xl0, xl2);
		__m128i hA = _mm_unpacklo_epi64(xl1, xl3);
		__m128i hB = _mm_unpackhi_epi64(xl1, xl3);
		__m128i hC = _mm_unpacklo_epi64(xl4, xl6);
		__m128i hD = _mm_unpackhi_epi64(xl4, xl6);
		__m128i hE = _mm_unpacklo_epi64(xl5, xl7);
		__m128i hF = _mm_unpackhi_epi64(xl5, xl7);
		
		// third horizontal edge
		FILTER_SOFT(h5, h6, h7, h8, h9, hA);
		
		// fourth horizontal edge
		if (!mb->f.transform_size_8x8_flag) {
			FILTER_SOFT(h9, hA, hB, hC, hD, hE);
		}
	} while (1);
}
