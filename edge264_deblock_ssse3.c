#include "edge264_common.h"

#ifndef __clang__
static inline __m256i _mm256_loadu2_m128i (__m128i const* hiaddr, __m128i const* loaddr) {
	return _mm256_inserti128_si256(_mm256_castsi128_si256(*loaddr), *hiaddr, 1);
}
#endif

#ifdef __AVX2__
int deblock_Luma16x16_8bit(size_t stride, ssize_t nstride, uint8_t *p, uint8_t *q, uint8_t *r, uint8_t *s) {
	// load macroblock samples while moving them in low/high lanes
	__m256i a0 = _mm256_permute4x64_epi64(*(__m256i *)(p +  stride     - 16), _MM_SHUFFLE(3, 0, 2, 1)); // l08~l0F m00~m07 ... m08~m0F
	__m256i a1 = _mm256_permute4x64_epi64(*(__m256i *)(p +  stride * 2 - 16), _MM_SHUFFLE(3, 0, 2, 1)); // l18~l1F m10~m17 ... m18~m1F
	__m256i a2 = _mm256_permute4x64_epi64(*(__m256i *)(q + nstride * 2 - 16), _MM_SHUFFLE(3, 0, 2, 1)); // l28~l2F m20~m27 ... m28~m2F
	__m256i a3 = _mm256_permute4x64_epi64(*(__m256i *)(q + nstride     - 16), _MM_SHUFFLE(3, 0, 2, 1)); // l38~l3F m30~m37 ... m38~m3F
	__m256i a4 = _mm256_permute4x64_epi64(*(__m256i *)(q               - 16), _MM_SHUFFLE(3, 0, 2, 1)); // l48~l4F m40~m47 ... m48~m4F
	__m256i a5 = _mm256_permute4x64_epi64(*(__m256i *)(q +  stride     - 16), _MM_SHUFFLE(3, 0, 2, 1)); // l58~l5F m50~m57 ... m58~m5F
	__m256i a6 = _mm256_permute4x64_epi64(*(__m256i *)(q +  stride * 2 - 16), _MM_SHUFFLE(3, 0, 2, 1)); // l68~l6F m60~m67 ... m68~m6F
	__m256i a7 = _mm256_permute4x64_epi64(*(__m256i *)(r + nstride * 2 - 16), _MM_SHUFFLE(3, 0, 2, 1)); // l78~l7F m70~m77 ... m78~m7F
	__m256i a8 = _mm256_permute4x64_epi64(*(__m256i *)(r + nstride     - 16), _MM_SHUFFLE(3, 1, 2, 0)); // ... m80~m87 l88~l8F m88~m8F
	__m256i a9 = _mm256_permute4x64_epi64(*(__m256i *)(r               - 16), _MM_SHUFFLE(3, 1, 2, 0)); // ... m90~m97 l98~l9F m98~m9F
	__m256i aA = _mm256_permute4x64_epi64(*(__m256i *)(r +  stride     - 16), _MM_SHUFFLE(3, 1, 2, 0)); // ... mA0~mA7 lA8~lAF mA8~mAF
	__m256i aB = _mm256_permute4x64_epi64(*(__m256i *)(r +  stride * 2 - 16), _MM_SHUFFLE(3, 1, 2, 0)); // ... mB0~mB7 lB8~lBF mB8~mBF
	__m256i aC = _mm256_permute4x64_epi64(*(__m256i *)(s + nstride * 2 - 16), _MM_SHUFFLE(3, 1, 2, 0)); // ... mC0~mC7 lC8~lCF mC8~mCF
	__m256i aD = _mm256_permute4x64_epi64(*(__m256i *)(s + nstride     - 16), _MM_SHUFFLE(3, 1, 2, 0)); // ... mD0~mD7 lD8~lDF mD8~mDF
	__m256i aE = _mm256_permute4x64_epi64(*(__m256i *)(s               - 16), _MM_SHUFFLE(3, 1, 2, 0)); // ... mE0~mE7 lE8~lEF mE8~mEF
	__m256i aF = _mm256_permute4x64_epi64(*(__m256i *)(s +  stride     - 16), _MM_SHUFFLE(3, 1, 2, 0)); // ... mF0~mF7 lF8~lFF mF8~mFF
	
	// transpose three vertical lines from the left border
	__m256i b0 = _mm256_blend_epi32(a0, a8, 0x30); // l08~l0F ... l88~l8F ...
	__m256i b1 = _mm256_blend_epi32(a1, a8, 0x30); // l18~l1F ... l98~l9F ...
	__m256i b2 = _mm256_blend_epi32(a2, a8, 0x30); // l28~l2F ... lA8~lAF ...
	__m256i b3 = _mm256_blend_epi32(a3, a8, 0x30); // l38~l3F ... lB8~lBF ...
	__m256i b4 = _mm256_blend_epi32(a4, a8, 0x30); // l48~l4F ... lC8~lCF ...
	__m256i b5 = _mm256_blend_epi32(a5, a8, 0x30); // l58~l5F ... lD8~lDF ...
	__m256i b6 = _mm256_blend_epi32(a6, a8, 0x30); // l68~l6F ... lE8~lEF ...
	__m256i b7 = _mm256_blend_epi32(a7, a8, 0x30); // l78~l7F ... lF8~lFF ...
	__m256i b8 = _mm256_unpacklo_epi8(b0, b1); // ... l0D l1D l0E l1E l0F l1F ... l8D l9D l8E l9E l8F l9F
	__m256i b9 = _mm256_unpacklo_epi8(b2, b3); // ... l2D l3D l2E l3E l2F l3F ... lAD lBD lAE lBE lAF lBF
	__m256i bA = _mm256_unpacklo_epi8(b4, b5); // ... l4D l5D l4E l5E l4F l5F ... lCD lDD lCE lDE lCF lDF
	__m256i bB = _mm256_unpacklo_epi8(b6, b7); // ... l6D l7D l6E l7E l6F l7F ... lED lFD lEE lFE lEF lFF
	__m256i bC = _mm256_unpackhi_epi16(b8, b9); // ... l0D~l3D l0E~l3E l0F~l3F ... l8D~lBD l8E~lBE l8F~lBF
	__m256i bD = _mm256_unpackhi_epi16(bA, bB); // ... l4D~l7D l4E~l7E l4F~l7F ... lCD~lFD lCE~lFE lCF~lFF
	__m256i bE = _mm256_unpacklo_epi32(bC, bD); // ... l0D~l7D ... l8D~lFD
	__m256i bF = _mm256_unpackhi_epi32(bC, bD); // l0E~l7E l0F~l7F l8E~lFE l8F~lFF
	
	// pack the entire macroblock with left/right sides in low/high lanes
	__m256i m0 = _mm256_unpackhi_epi64(a0, a1); // m00~m07 m10~m17 m08~m0F m18~m1F
	__m256i m1 = _mm256_unpackhi_epi64(a2, a3); // m20~m27 m30~m37 m28~m2F m38~m3F
	__m256i m2 = _mm256_unpackhi_epi64(a4, a5); // m40~m47 m50~m57 m48~m4F m58~m5F
	__m256i m3 = _mm256_unpackhi_epi64(a6, a7); // m60~m67 m70~m77 m68~m6F m78~m7F
	__m256i m4 = _mm256_unpackhi_epi64(a8, a9); // m80~m87 m90~m97 m88~m8F m98~m9F
	__m256i m5 = _mm256_unpackhi_epi64(aA, aB); // mA0~mA7 mB0~mB7 mA8~mAF mB8~mBF
	__m256i m6 = _mm256_unpackhi_epi64(aC, aD); // mC0~mC7 mD0~mD7 mC8~mCF mD8~mDF
	__m256i m7 = _mm256_unpackhi_epi64(aE, aF); // mE0~mE7 mF0~mF7 mE8~mEF mF8~mFF
	
	__m256i zero = _mm256_setzero_si256();
	for (int i = 2;;) {
		// transpose 16x16 macroblock
		__m256i c0 = _mm256_permute4x64_epi64(_mm256_unpacklo_epi64(m0, m4), _MM_SHUFFLE(3, 1, 2, 0)); // m00~m0F m80~m8F
		__m256i c1 = _mm256_permute4x64_epi64(_mm256_unpackhi_epi64(m0, m4), _MM_SHUFFLE(3, 1, 2, 0)); // m10~m1F m90~m9F
		__m256i c2 = _mm256_permute4x64_epi64(_mm256_unpacklo_epi64(m1, m5), _MM_SHUFFLE(3, 1, 2, 0)); // m20~m2F mA0~mAF
		__m256i c3 = _mm256_permute4x64_epi64(_mm256_unpackhi_epi64(m1, m5), _MM_SHUFFLE(3, 1, 2, 0)); // m30~m3F mB0~mBF
		__m256i c4 = _mm256_permute4x64_epi64(_mm256_unpacklo_epi64(m2, m6), _MM_SHUFFLE(3, 1, 2, 0)); // m40~m4F mC0~mCF
		__m256i c5 = _mm256_permute4x64_epi64(_mm256_unpackhi_epi64(m2, m6), _MM_SHUFFLE(3, 1, 2, 0)); // m50~m5F mD0~mDF
		__m256i c6 = _mm256_permute4x64_epi64(_mm256_unpacklo_epi64(m3, m7), _MM_SHUFFLE(3, 1, 2, 0)); // m60~m6F mE0~mEF
		__m256i c7 = _mm256_permute4x64_epi64(_mm256_unpackhi_epi64(m3, m7), _MM_SHUFFLE(3, 1, 2, 0)); // m70~m7F mF0~mFF
		__m256i c8 = _mm256_unpacklo_epi8(c0, c1); // m0x m1x[0~7] m8x m9x[0~7]
		__m256i c9 = _mm256_unpacklo_epi8(c2, c3); // m2x m3x[0~7] mAx mBx[0~7]
		__m256i cA = _mm256_unpacklo_epi8(c4, c5); // m4x m5x[0~7] mCx mDx[0~7]
		__m256i cB = _mm256_unpacklo_epi8(c6, c7); // m6x m7x[0~7] mEx mFx[0~7]
		__m256i cC = _mm256_unpackhi_epi8(c0, c1); // m0x m1x[8~F] m8x m9x[8~F]
		__m256i cD = _mm256_unpackhi_epi8(c2, c3); // m2x m3x[8~F] mAx mBx[8~F]
		__m256i cE = _mm256_unpackhi_epi8(c4, c5); // m4x m5x[8~F] mCx mDx[8~F]
		__m256i cF = _mm256_unpackhi_epi8(c6, c7); // m6x m7x[8~F] mEx mFx[8~F]
		__m256i d0 = _mm256_unpacklo_epi16(c8, c9); // m0x~m3x[0~3] m8x~mBx[0~3]
		__m256i d1 = _mm256_unpacklo_epi16(cA, cB); // m4x~m7x[0~3] mCx~mFx[0~3]
		__m256i d2 = _mm256_unpackhi_epi16(c8, c9); // m0x~m3x[4~7] m8x~mBx[4~7]
		__m256i d3 = _mm256_unpackhi_epi16(cA, cB); // m4x~m7x[4~7] mCx~mFx[4~7]
		__m256i d4 = _mm256_unpacklo_epi16(cC, cD); // m0x~m3x[8~B] m8x~mBx[8~B]
		__m256i d5 = _mm256_unpacklo_epi16(cE, cF); // m4x~m7x[8~B] mCx~mFx[8~B]
		__m256i d6 = _mm256_unpackhi_epi16(cC, cD); // m0x~m3x[C~F] m8x~mBx[C~F]
		__m256i d7 = _mm256_unpackhi_epi16(cE, cF); // m4x~m7x[C~F] mCx~mFx[C~F]
		m0 = _mm256_unpacklo_epi32(d0, d1); // m00~m70 m01~m71 m80~mF0 m81~mF1
		m1 = _mm256_unpackhi_epi32(d0, d1); // m02~m72 m03~m73 m82~mF2 m83~mF3
		m2 = _mm256_unpacklo_epi32(d2, d3); // m04~m74 m05~m75 m84~mF4 m85~mF5
		m3 = _mm256_unpackhi_epi32(d2, d3); // m06~m76 m07~m77 m86~mF6 m87~mF7
		m4 = _mm256_unpacklo_epi32(d4, d5); // m08~m78 m09~m79 m88~mF8 m89~mF9
		m5 = _mm256_unpackhi_epi32(d4, d5); // m0A~m7A m0B~m7B m8A~mFA m8B~mFB
		m6 = _mm256_unpacklo_epi32(d6, d7); // m0C~m7C m0D~m7D m8C~mFC m8D~mFD
		m7 = _mm256_unpackhi_epi32(d6, d7); // m0E~m7E m0F~m7F m8E~mFE m8F~mFF
		
		// filter edges
		// deblock_edge_8bit(bE, &bF, &m0, m1);
		// deblock_edge_8bit(m0, &m1, &m2, m3);
		// deblock_edge_8bit(m2, &m3, &m4, m5);
		// deblock_edge_8bit(m4, &m5, &m6, m7);
		
		// load the three topmost horizontal lines in registers
		if (--i == 0)
			break;
		
	}
	
	// store the filtered samples in memory
	__m256i e0 = _mm256_permute4x64_epi64(m0, _MM_SHUFFLE(3, 1, 2, 0));
	__m256i e1 = _mm256_permute4x64_epi64(m1, _MM_SHUFFLE(3, 1, 2, 0));
	__m256i e2 = _mm256_permute4x64_epi64(m2, _MM_SHUFFLE(3, 1, 2, 0));
	__m256i e3 = _mm256_permute4x64_epi64(m3, _MM_SHUFFLE(3, 1, 2, 0));
	__m256i e4 = _mm256_permute4x64_epi64(m4, _MM_SHUFFLE(3, 1, 2, 0));
	__m256i e5 = _mm256_permute4x64_epi64(m5, _MM_SHUFFLE(3, 1, 2, 0));
	__m256i e6 = _mm256_permute4x64_epi64(m6, _MM_SHUFFLE(3, 1, 2, 0));
	__m256i e7 = _mm256_permute4x64_epi64(m7, _MM_SHUFFLE(3, 1, 2, 0));
	*(__m128i *)(p +  stride    ) = _mm256_extracti128_si256(e0, 0);
	*(__m128i *)(p +  stride * 2) = _mm256_extracti128_si256(e0, 1);
	*(__m128i *)(q + nstride * 2) = _mm256_extracti128_si256(e1, 0);
	*(__m128i *)(q + nstride    ) = _mm256_extracti128_si256(e1, 1);
	*(__m128i *)(q              ) = _mm256_extracti128_si256(e2, 0);
	*(__m128i *)(q +  stride    ) = _mm256_extracti128_si256(e2, 1);
	*(__m128i *)(q +  stride * 2) = _mm256_extracti128_si256(e3, 0);
	*(__m128i *)(r + nstride * 2) = _mm256_extracti128_si256(e3, 1);
	*(__m128i *)(r + nstride    ) = _mm256_extracti128_si256(e4, 0);
	*(__m128i *)(r              ) = _mm256_extracti128_si256(e4, 1);
	*(__m128i *)(r +  stride    ) = _mm256_extracti128_si256(e5, 0);
	*(__m128i *)(r +  stride * 2) = _mm256_extracti128_si256(e5, 1);
	*(__m128i *)(s + nstride * 2) = _mm256_extracti128_si256(e6, 0);
	*(__m128i *)(s + nstride    ) = _mm256_extracti128_si256(e6, 1);
	*(__m128i *)(s              ) = _mm256_extracti128_si256(e7, 0);
	*(__m128i *)(s +  stride    ) = _mm256_extracti128_si256(e7, 1);
}
#endif
