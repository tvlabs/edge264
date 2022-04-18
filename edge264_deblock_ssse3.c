// TODO disable deblocking of single edge if indexA<16
// TODO initialize QP to 0 for PCM macroblocks
// TODO put indexAB in union to memory since clang cannot fetch directly from memory
// TODO précalculer un tableau de correspondance qP -> alpha/beta/tC0
// TODO check that alpha=0 or beta=0 does not break the decoder
// TODO introduce an alternative to blendv expecting full mask for a simpler SSSE3 fallback
// TODO acknowledge influence of ffmpeg source
// TODO make deblock_mb iterate on entire frame to simplify future improvements (i.e. slices)

#include "edge264_common.h"



static const uint8_t idx2alpha[52] =
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 5, 6, 7, 8, 9, 10, 12, 13, 15, 17, 20, 22, 25, 28, 32, 36, 40, 45, 50, 56, 63, 71, 80, 90, 101, 113, 127, 144, 162, 182, 203, 226, 255, 255};
static const uint8_t idx2beta[52] =
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18};
static const int8_t idx2tC0[3][52] = { // modified to disable deblocking when alpha or beta is zero
	{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 7, 8, 9, 10, 11, 13},
	{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 10, 11, 12, 13, 15, 17},
	{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 6, 6, 7, 8, 9, 10, 11, 13, 14, 16, 18, 20, 23, 25},
};



/**
 * Compute alpha, beta and tC0 for all planes and all edges (labeled a to h in
 * deblocking order) of the current macroblock.
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
 * is achieved using _mm_min_epu8:
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
static inline void FUNC(init_alpha_beta_tC0)
{
	// compute all values of qPav for each of the color planes first
	__m128i qP = _mm_set1_epi32(mb->QP_s);
	__m128i qP_AB = _mm_unpacklo_epi64(qP, _mm_unpacklo_epi32(_mm_cvtsi32_si128(mb[-1].QP_s), _mm_cvtsi32_si128(ctx->mbB->QP_s)));
	__m128i qP_av = _mm_sub_epi8(_mm_avg_epu8(qP, qP_AB), _mm_set1_epi8(4)); // mid/mid/A/B
	
	// compute all values of alpha and beta using vectorized array accesses
	__m128i gte20 = _mm_cmpgt_epi8(qP_av, _mm_set1_epi8(19));
	__m128i gte36 = _mm_cmpgt_epi8(qP_av, _mm_set1_epi8(35));
	__m128i alphalo = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2alpha + 4)), qP_av);
	__m128i alphamd = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2alpha + 20)), qP_av);
	__m128i alphahi = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2alpha + 36)), qP_av);
	__m128i betalo = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2beta + 4)), qP_av);
	__m128i betamd = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2beta + 20)), qP_av);
	__m128i betahi = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2beta + 36)), qP_av);
	ctx->alpha_v = (v16qu)_mm_blendv_epi8(_mm_blendv_epi8(alphalo, alphamd, gte20), alphahi, gte36);
	ctx->beta_v = (v16qu)_mm_blendv_epi8(_mm_blendv_epi8(betalo, betamd, gte20), betahi, gte36);
	
	// initialize tC0 with bS=3 for internal edges of Intra macroblock
	__m128i tC0neg = _mm_cmpgt_epi8(_mm_setzero_si128(), qP_av);
	if (!mb->f.mbIsInterFlag) {
		__m128i tC03lo = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2tC0[2] + 4)), qP_av);
		__m128i tC03md = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2tC0[2] + 20)), qP_av);
		__m128i tC03hi = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2tC0[2] + 36)), qP_av);
		__m128i tC03 = _mm_or_si128(_mm_blendv_epi8(_mm_blendv_epi8(tC03lo, tC03md, gte20), tC03hi, gte36), tC0neg);
		ctx->tC0_v[0] = ctx->tC0_v[1] = (v16qi)_mm_shuffle_epi8(tC03, _mm_setzero_si128());
		ctx->tC0_v[2] = ctx->tC0_v[3] = (v16qi)_mm_shuffle_epi8(tC03, _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 2, 2, 2, 2));
	} else {
		// compute all values of tC0 for bS=1 and bS=2
		__m128i tC01lo = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2tC0[0] + 4)), qP_av);
		__m128i tC01md = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2tC0[0] + 20)), qP_av);
		__m128i tC01hi = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2tC0[0] + 36)), qP_av);
		__m128i tC01 = _mm_or_si128(_mm_blendv_epi8(_mm_blendv_epi8(tC01lo, tC01md, gte20), tC01hi, gte36), tC0neg);
		__m128i tC02lo = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2tC0[1] + 4)), qP_av);
		__m128i tC02md = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2tC0[1] + 20)), qP_av);
		__m128i tC02hi = _mm_shuffle_epi8(_mm_loadu_si128((__m128i *)(ctx->qP2tC0[1] + 36)), qP_av);
		__m128i tC02 = _mm_or_si128(_mm_blendv_epi8(_mm_blendv_epi8(tC02lo, tC02md, gte20), tC02hi, gte36), tC0neg);
		
		// compute masks for bS=1 based on equality of references and motion vectors
		__m128i bS1aceg, bS1bdfh, zero;
		__m128i c3 = _mm_set1_epi8(3);
		if (mb->inter_blocks == 0x01231111) { // 16x16 macroblock
			__m128i mvsv0l0 = (__m128i)_mm_shuffle_ps((__m128)mb[-1].mvs_v[1], (__m128)mb[-1].mvs_v[3], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsv1l0 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[0], (__m128)mb->mvs_v[2], _MM_SHUFFLE(2, 0, 2, 0));
			__m128i mvsv0l1 = (__m128i)_mm_shuffle_ps((__m128)mb[-1].mvs_v[5], (__m128)mb[-1].mvs_v[7], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsv1l1 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[4], (__m128)mb->mvs_v[6], _MM_SHUFFLE(2, 0, 2, 0));
			__m128i mvsh0l0 = _mm_unpackhi_epi64((__m128i)ctx->mbB->mvs_v[2], (__m128i)ctx->mbB->mvs_v[3]);
			__m128i mvsh1l0 = _mm_unpacklo_epi64((__m128i)mb->mvs_v[0], (__m128i)mb->mvs_v[1]);
			__m128i mvsh0l1 = _mm_unpackhi_epi64((__m128i)ctx->mbB->mvs_v[6], (__m128i)ctx->mbB->mvs_v[7]);
			__m128i mvsh1l1 = _mm_unpacklo_epi64((__m128i)mb->mvs_v[4], (__m128i)mb->mvs_v[5]);
			__m128i mvsael00 = _mm_packs_epi16(_mm_sub_epi16(mvsv0l0, mvsv1l0), _mm_sub_epi16(mvsh0l0, mvsh1l0));
			__m128i mvsael01 = _mm_packs_epi16(_mm_sub_epi16(mvsv0l0, mvsv1l1), _mm_sub_epi16(mvsh0l0, mvsh1l1));
			__m128i mvsael10 = _mm_packs_epi16(_mm_sub_epi16(mvsv0l1, mvsv1l0), _mm_sub_epi16(mvsh0l1, mvsh1l0));
			__m128i mvsael11 = _mm_packs_epi16(_mm_sub_epi16(mvsv0l1, mvsv1l1), _mm_sub_epi16(mvsh0l1, mvsh1l1));
			__m128i mvsaep = _mm_subs_epu8(_mm_max_epu8(_mm_abs_epi8(mvsael00), _mm_abs_epi8(mvsael11)), c3);
			__m128i mvsaec = _mm_subs_epu8(_mm_max_epu8(_mm_abs_epi8(mvsael01), _mm_abs_epi8(mvsael10)), c3);
			zero = _mm_setzero_si128();
			__m128i mvsacegp = _mm_shuffle_epi32(_mm_packs_epi16(mvsaep, zero), _MM_SHUFFLE(3, 1, 2, 0));
			__m128i mvsacegc = _mm_shuffle_epi32(_mm_packs_epi16(mvsaec, zero), _MM_SHUFFLE(3, 1, 2, 0));
			__m128i refIdx0 = _mm_setr_epi32(mb->refIdx_s[0], mb[-1].refIdx_s[0], ctx->mbB->refIdx_s[0], 0);
			__m128i refIdx1 = _mm_setr_epi32(mb->refIdx_s[1], mb[-1].refIdx_s[1], ctx->mbB->refIdx_s[1], 0);
			__m128i shufVHAB = _mm_setr_epi8(0, 2, 1, 3, 0, 1, 2, 3, 5, 7, 0, 2, 10, 11, 0, 1);
			__m128i refs0 = _mm_shuffle_epi8(_mm_shuffle_epi8((__m128i)ctx->RefPicList_v[0], refIdx0), shufVHAB); // (v0,h0,A0,B0)
			__m128i refs1 = _mm_shuffle_epi8(_mm_shuffle_epi8((__m128i)ctx->RefPicList_v[1], refIdx1), shufVHAB); // (v1,h1,A1,B1)
			__m128i neq0 = _mm_xor_si128(refs0, (__m128i)_mm_shuffle_ps((__m128)refs1, (__m128)refs0, _MM_SHUFFLE(1, 0, 3, 2))); // (v0^A1,h0^B1,A0^v0,B0^h0)
			__m128i neq1 = _mm_xor_si128(refs1, (__m128i)_mm_shuffle_ps((__m128)refs0, (__m128)refs1, _MM_SHUFFLE(1, 0, 3, 2))); // (v1^A0,h1^B0,A1^v1,B1^h1)
			__m128i refsaceg = _mm_or_si128(neq0, neq1); // low=cross, high=parallel
			__m128i refsacegc = _mm_unpacklo_epi8(refsaceg, refsaceg);
			__m128i refsacegp = _mm_unpackhi_epi8(refsaceg, refsaceg);
			__m128i neq3 = _mm_or_si128(_mm_min_epu8(refsacegp, refsacegc), _mm_min_epu8(mvsacegp, mvsacegc));
			__m128i neq4 = _mm_or_si128(_mm_min_epu8(refsacegp, mvsacegc), _mm_min_epu8(mvsacegp, refsacegc));
			bS1aceg = _mm_cmpgt_epi8(_mm_or_si128(neq3, neq4), zero);
			bS1bdfh = zero;
		} else if (mb->refIdx_s[1] & mb[-1].refIdx_s[1] & ctx->mbB->refIdx_s[1] == -1) { // P macroblocks
			__m128i mvsv0 = (__m128i)_mm_shuffle_ps((__m128)mb[-1].mvs_v[1], (__m128)mb[-1].mvs_v[3], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsv1 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[0], (__m128)mb->mvs_v[2], _MM_SHUFFLE(2, 0, 2, 0));
			__m128i mvsv2 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[0], (__m128)mb->mvs_v[2], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsv3 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[1], (__m128)mb->mvs_v[3], _MM_SHUFFLE(2, 0, 2, 0));
			__m128i mvsv4 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[1], (__m128)mb->mvs_v[3], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsh0 = _mm_unpackhi_epi64((__m128i)ctx->mbB->mvs_v[2], (__m128i)ctx->mbB->mvs_v[3]);
			__m128i mvsh1 = _mm_unpacklo_epi64((__m128i)mb->mvs_v[0], (__m128i)mb->mvs_v[1]);
			__m128i mvsh2 = _mm_unpackhi_epi64((__m128i)mb->mvs_v[0], (__m128i)mb->mvs_v[1]);
			__m128i mvsh3 = _mm_unpacklo_epi64((__m128i)mb->mvs_v[2], (__m128i)mb->mvs_v[3]);
			__m128i mvsh4 = _mm_unpackhi_epi64((__m128i)mb->mvs_v[2], (__m128i)mb->mvs_v[3]);
			__m128i mvsac = _mm_packs_epi16(_mm_sub_epi16(mvsv0, mvsv1), _mm_sub_epi16(mvsv2, mvsv3));
			__m128i mvsbd = _mm_packs_epi16(_mm_sub_epi16(mvsv1, mvsv2), _mm_sub_epi16(mvsv3, mvsv4));
			__m128i mvseg = _mm_packs_epi16(_mm_sub_epi16(mvsh0, mvsh1), _mm_sub_epi16(mvsh2, mvsh3));
			__m128i mvsfh = _mm_packs_epi16(_mm_sub_epi16(mvsh1, mvsh2), _mm_sub_epi16(mvsh3, mvsh4));
			__m128i mvsaceg = _mm_packs_epi16(_mm_subs_epu8(_mm_abs_epi8(mvsac), c3), _mm_subs_epu8(_mm_abs_epi8(mvseg), c3));
			__m128i mvsbdfh = _mm_packs_epi16(_mm_subs_epu8(_mm_abs_epi8(mvsbd), c3), _mm_subs_epu8(_mm_abs_epi8(mvsfh), c3));
			__m128i refIdx = _mm_setr_epi32(mb->refIdx_s[0], mb[-1].refIdx_s[0], ctx->mbB->refIdx_s[0], 0);
			__m128i shufVHAB = _mm_setr_epi8(0, 2, 1, 3, 0, 1, 2, 3, 5, 7, 0, 2, 10, 11, 0, 1);
			__m128i refs = _mm_shuffle_epi8(_mm_shuffle_epi8((__m128i)ctx->RefPicList_v[0], refIdx), shufVHAB); // (v0,h0,A0,B0)
			__m128i neq = _mm_xor_si128(refs, _mm_unpackhi_epi64(refs, refs)); // (v0^A0,h0^B0,0,0)
			__m128i refsaceg = _mm_unpacklo_epi8(neq, neq);
			zero = _mm_setzero_si128();
			bS1aceg = _mm_cmpgt_epi8(_mm_or_si128(refsaceg, mvsaceg), zero);
			bS1bdfh = _mm_cmpgt_epi8(mvsbdfh, zero);
		} else { // B macroblocks
			__m128i mvsv0l0 = (__m128i)_mm_shuffle_ps((__m128)mb[-1].mvs_v[1], (__m128)mb[-1].mvs_v[3], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsv1l0 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[0], (__m128)mb->mvs_v[2], _MM_SHUFFLE(2, 0, 2, 0));
			__m128i mvsv2l0 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[0], (__m128)mb->mvs_v[2], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsv3l0 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[1], (__m128)mb->mvs_v[3], _MM_SHUFFLE(2, 0, 2, 0));
			__m128i mvsv4l0 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[1], (__m128)mb->mvs_v[3], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsv0l1 = (__m128i)_mm_shuffle_ps((__m128)mb[-1].mvs_v[5], (__m128)mb[-1].mvs_v[7], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsv1l1 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[4], (__m128)mb->mvs_v[6], _MM_SHUFFLE(2, 0, 2, 0));
			__m128i mvsv2l1 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[4], (__m128)mb->mvs_v[6], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsv3l1 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[5], (__m128)mb->mvs_v[7], _MM_SHUFFLE(2, 0, 2, 0));
			__m128i mvsv4l1 = (__m128i)_mm_shuffle_ps((__m128)mb->mvs_v[5], (__m128)mb->mvs_v[7], _MM_SHUFFLE(3, 1, 3, 1));
			__m128i mvsacl00 = _mm_packs_epi16(_mm_sub_epi16(mvsv0l0, mvsv1l0), _mm_sub_epi16(mvsv2l0, mvsv3l0));
			__m128i mvsbdl00 = _mm_packs_epi16(_mm_sub_epi16(mvsv1l0, mvsv2l0), _mm_sub_epi16(mvsv3l0, mvsv4l0));
			__m128i mvsacl01 = _mm_packs_epi16(_mm_sub_epi16(mvsv0l0, mvsv1l1), _mm_sub_epi16(mvsv2l0, mvsv3l1));
			__m128i mvsbdl01 = _mm_packs_epi16(_mm_sub_epi16(mvsv1l0, mvsv2l1), _mm_sub_epi16(mvsv3l0, mvsv4l1));
			__m128i mvsacl10 = _mm_packs_epi16(_mm_sub_epi16(mvsv0l1, mvsv1l0), _mm_sub_epi16(mvsv2l1, mvsv3l0));
			__m128i mvsbdl10 = _mm_packs_epi16(_mm_sub_epi16(mvsv1l1, mvsv2l0), _mm_sub_epi16(mvsv3l1, mvsv4l0));
			__m128i mvsacl11 = _mm_packs_epi16(_mm_sub_epi16(mvsv0l1, mvsv1l1), _mm_sub_epi16(mvsv2l1, mvsv3l1));
			__m128i mvsbdl11 = _mm_packs_epi16(_mm_sub_epi16(mvsv1l1, mvsv2l1), _mm_sub_epi16(mvsv3l1, mvsv4l1));
			__m128i mvsacp = _mm_subs_epu8(_mm_max_epu8(_mm_abs_epi8(mvsacl00), _mm_abs_epi8(mvsacl11)), c3);
			__m128i mvsbdp = _mm_subs_epu8(_mm_max_epu8(_mm_abs_epi8(mvsbdl00), _mm_abs_epi8(mvsbdl11)), c3);
			__m128i mvsacc = _mm_subs_epu8(_mm_max_epu8(_mm_abs_epi8(mvsacl01), _mm_abs_epi8(mvsacl10)), c3);
			__m128i mvsbdc = _mm_subs_epu8(_mm_max_epu8(_mm_abs_epi8(mvsbdl01), _mm_abs_epi8(mvsbdl10)), c3);
			__m128i mvsh0l0 = _mm_unpackhi_epi64((__m128i)ctx->mbB->mvs_v[2], (__m128i)ctx->mbB->mvs_v[3]);
			__m128i mvsh1l0 = _mm_unpacklo_epi64((__m128i)mb->mvs_v[0], (__m128i)mb->mvs_v[1]);
			__m128i mvsh2l0 = _mm_unpackhi_epi64((__m128i)mb->mvs_v[0], (__m128i)mb->mvs_v[1]);
			__m128i mvsh3l0 = _mm_unpacklo_epi64((__m128i)mb->mvs_v[2], (__m128i)mb->mvs_v[3]);
			__m128i mvsh4l0 = _mm_unpackhi_epi64((__m128i)mb->mvs_v[2], (__m128i)mb->mvs_v[3]);
			__m128i mvsh0l1 = _mm_unpackhi_epi64((__m128i)ctx->mbB->mvs_v[6], (__m128i)ctx->mbB->mvs_v[7]);
			__m128i mvsh1l1 = _mm_unpacklo_epi64((__m128i)mb->mvs_v[4], (__m128i)mb->mvs_v[5]);
			__m128i mvsh2l1 = _mm_unpackhi_epi64((__m128i)mb->mvs_v[4], (__m128i)mb->mvs_v[5]);
			__m128i mvsh3l1 = _mm_unpacklo_epi64((__m128i)mb->mvs_v[6], (__m128i)mb->mvs_v[7]);
			__m128i mvsh4l1 = _mm_unpackhi_epi64((__m128i)mb->mvs_v[6], (__m128i)mb->mvs_v[7]);
			__m128i mvsegl00 = _mm_packs_epi16(_mm_sub_epi16(mvsh0l0, mvsh1l0), _mm_sub_epi16(mvsh2l0, mvsh3l0));
			__m128i mvsfhl00 = _mm_packs_epi16(_mm_sub_epi16(mvsh1l0, mvsh2l0), _mm_sub_epi16(mvsh3l0, mvsh4l0));
			__m128i mvsegl01 = _mm_packs_epi16(_mm_sub_epi16(mvsh0l0, mvsh1l1), _mm_sub_epi16(mvsh2l0, mvsh3l1));
			__m128i mvsfhl01 = _mm_packs_epi16(_mm_sub_epi16(mvsh1l0, mvsh2l1), _mm_sub_epi16(mvsh3l0, mvsh4l1));
			__m128i mvsegl10 = _mm_packs_epi16(_mm_sub_epi16(mvsh0l1, mvsh1l0), _mm_sub_epi16(mvsh2l1, mvsh3l0));
			__m128i mvsfhl10 = _mm_packs_epi16(_mm_sub_epi16(mvsh1l1, mvsh2l0), _mm_sub_epi16(mvsh3l1, mvsh4l0));
			__m128i mvsegl11 = _mm_packs_epi16(_mm_sub_epi16(mvsh0l1, mvsh1l1), _mm_sub_epi16(mvsh2l1, mvsh3l1));
			__m128i mvsfhl11 = _mm_packs_epi16(_mm_sub_epi16(mvsh1l1, mvsh2l1), _mm_sub_epi16(mvsh3l1, mvsh4l1));
			__m128i mvsegp = _mm_subs_epu8(_mm_max_epu8(_mm_abs_epi8(mvsegl00), _mm_abs_epi8(mvsegl11)), c3);
			__m128i mvsfhp = _mm_subs_epu8(_mm_max_epu8(_mm_abs_epi8(mvsfhl00), _mm_abs_epi8(mvsfhl11)), c3);
			__m128i mvsegc = _mm_subs_epu8(_mm_max_epu8(_mm_abs_epi8(mvsegl01), _mm_abs_epi8(mvsegl10)), c3);
			__m128i mvsfhc = _mm_subs_epu8(_mm_max_epu8(_mm_abs_epi8(mvsfhl01), _mm_abs_epi8(mvsfhl10)), c3);
			__m128i mvsacegp = _mm_packs_epi16(mvsacp, mvsegp);
			__m128i mvsbdfhp = _mm_packs_epi16(mvsbdp, mvsfhp);
			__m128i mvsacegc = _mm_packs_epi16(mvsacc, mvsegc);
			__m128i mvsbdfhc = _mm_packs_epi16(mvsbdc, mvsfhc);
			__m128i refIdx0 = _mm_setr_epi32(mb->refIdx_s[0], mb[-1].refIdx_s[0], ctx->mbB->refIdx_s[0], 0);
			__m128i refIdx1 = _mm_setr_epi32(mb->refIdx_s[1], mb[-1].refIdx_s[1], ctx->mbB->refIdx_s[1], 0);
			__m128i shufVHAB = _mm_setr_epi8(0, 2, 1, 3, 0, 1, 2, 3, 5, 7, 0, 2, 10, 11, 0, 1);
			__m128i refs0 = _mm_shuffle_epi8(_mm_shuffle_epi8((__m128i)ctx->RefPicList_v[0], refIdx0), shufVHAB); // (v0,h0,A0,B0)
			__m128i refs1 = _mm_shuffle_epi8(_mm_shuffle_epi8((__m128i)ctx->RefPicList_v[1], refIdx1), shufVHAB); // (v1,h1,A1,B1)
			__m128i neq0 = _mm_xor_si128(refs0, (__m128i)_mm_shuffle_ps((__m128)refs1, (__m128)refs0, _MM_SHUFFLE(1, 0, 3, 2))); // (v0^A1,h0^B1,A0^v0,B0^h0)
			__m128i neq1 = _mm_xor_si128(refs1, (__m128i)_mm_shuffle_ps((__m128)refs0, (__m128)refs1, _MM_SHUFFLE(1, 0, 3, 2))); // (v1^A0,h1^B0,A1^v1,B1^h1)
			__m128i neq2 = _mm_xor_si128(refs0, refs1);
			__m128i refsaceg = _mm_or_si128(neq0, neq1); // low=cross, high=parallel
			__m128i refsacegc = _mm_unpacklo_epi8(refsaceg, refsaceg);
			__m128i refsacegp = _mm_unpackhi_epi8(refsaceg, refsaceg);
			__m128i refsbdfhc = _mm_unpacklo_epi8(neq2, neq2);
			__m128i neq3 = _mm_or_si128(_mm_min_epu8(refsacegp, refsacegc), _mm_min_epu8(mvsacegp, mvsacegc));
			__m128i neq4 = _mm_or_si128(_mm_min_epu8(refsacegp, mvsacegc), _mm_min_epu8(mvsacegp, refsacegc));
			zero = _mm_setzero_si128();
			bS1aceg = _mm_cmpgt_epi8(_mm_or_si128(neq3, neq4), zero);
			bS1bdfh = _mm_cmpgt_epi8(_mm_min_epu8(mvsbdfhp, _mm_or_si128(refsbdfhc, mvsbdfhc)), zero);
		}
		__m128i bS1abcd = _mm_unpacklo_epi32(bS1aceg, bS1bdfh);
		__m128i bS1efgh = _mm_unpackhi_epi32(bS1aceg, bS1bdfh);
		__m128i bS1aacc = _mm_unpacklo_epi32(bS1aceg, bS1aceg);
		__m128i bS1eegg = _mm_unpackhi_epi32(bS1aceg, bS1aceg);
		
		// compute masks for edges with bS=2
		__m128i shufV = _mm_setr_epi8(0, 2, 8, 10, 1, 3, 9, 11, 4, 6, 12, 14, 5, 7, 13, 15);
		__m128i shufH = _mm_setr_epi8(0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15);
		__m128i nnzv = _mm_shuffle_epi8((__m128i)mb->nC_v[0], shufV);
		__m128i nnzl = _mm_shuffle_epi8((__m128i)mb[-1].nC_v[0], shufV);
		__m128i nnzh = _mm_shuffle_epi8((__m128i)mb->nC_v[0], shufH);
		__m128i nnzt = _mm_shuffle_epi8((__m128i)ctx->mbB->nC_v[0], shufH);
		__m128i bS2abcd = _mm_cmpgt_epi8(_mm_or_si128(nnzv, _mm_alignr_epi8(nnzv, nnzl, 12)), zero);
		__m128i bS2efgh = _mm_cmpgt_epi8(_mm_or_si128(nnzh, _mm_alignr_epi8(nnzh, nnzt, 12)), zero);
		__m128i bS2aacc = _mm_shuffle_epi32(bS2abcd, _MM_SHUFFLE(2, 2, 0, 0));
		__m128i bS2eegg = _mm_shuffle_epi32(bS2efgh, _MM_SHUFFLE(2, 2, 0, 0));
		
		// shuffle, blend and store tC0 values
		__m128i tC00 = _mm_set1_epi8(-1);
		__m128i shuf0 = _mm_setr_epi8(8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		__m128i shuf1 = _mm_setr_epi8(12, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		__m128i shuf2 = _mm_setr_epi8(9, 9, 9, 9, 10, 10, 10, 10, 1, 1, 1, 1, 2, 2, 2, 2);
		__m128i shuf3 = _mm_setr_epi8(13, 13, 13, 13, 14, 14, 14, 14, 1, 1, 1, 1, 2, 2, 2, 2);
		ctx->tC0_v[0] = (v16qi)_mm_blendv_epi8(_mm_blendv_epi8(tC00, _mm_shuffle_epi8(tC01, shuf0), bS1abcd), _mm_shuffle_epi8(tC02, shuf0), bS2abcd);
		ctx->tC0_v[1] = (v16qi)_mm_blendv_epi8(_mm_blendv_epi8(tC00, _mm_shuffle_epi8(tC01, shuf1), bS1efgh), _mm_shuffle_epi8(tC02, shuf1), bS2efgh);
		ctx->tC0_v[2] = (v16qi)_mm_blendv_epi8(_mm_blendv_epi8(tC00, _mm_shuffle_epi8(tC01, shuf2), bS1aacc), _mm_shuffle_epi8(tC02, shuf2), bS2aacc);
		ctx->tC0_v[3] = (v16qi)_mm_blendv_epi8(_mm_blendv_epi8(tC00, _mm_shuffle_epi8(tC01, shuf3), bS1eegg), _mm_shuffle_epi8(tC02, shuf3), bS2eegg);
	}
}



/**
 * Filter a single edge in place for bS in [0..3].
 * 
 * am1 is _mm_set1_epi8(alpha - 1)
 * bm1 is _mm_set1_epi8(beta - 1)
 * tC0 should equal -1 for each position where bS=0
 */
#define DEBLOCK_LUMA_SOFT(p2, p1, p0, q0, q1, q2, am1, bm1, tC0) {\
	/* compute filterSamplesFlags as a mask, and apply it to tC0 */\
	__m128i c128 = _mm_set1_epi8(-128);\
	__m128i pq0 = _mm_subs_epu8(p0, q0);\
	__m128i qp0 = _mm_subs_epu8(q0, p0);\
	__m128i abs0 = _mm_or_si128(pq0, qp0);\
	__m128i sub0 = _mm_subs_epu8(_mm_adds_epu8(qp0, c128), pq0); /* save 128+q0-p0 for later */\
	__m128i abs1 = _mm_or_si128(_mm_subs_epu8(p1, p0), _mm_subs_epu8(p0, p1));\
	__m128i abs2 = _mm_or_si128(_mm_subs_epu8(q1, q0), _mm_subs_epu8(q0, q1));\
	__m128i or = _mm_or_si128(_mm_subs_epu8(abs0, am1), _mm_subs_epu8(_mm_max_epu8(abs1, abs2), bm1));\
	__m128i zero = _mm_setzero_si128();\
	__m128i filterSamplesFlags = _mm_blendv_epi8(_mm_cmpeq_epi8(or, zero), zero, tC0);\
	__m128i ftC0 = _mm_and_si128(tC0, filterSamplesFlags);\
	/* filter p1 and q1 (same as ffmpeg, I couldn't find better) */\
	__m128i c1 = _mm_set1_epi8(1);\
	__m128i x0 = _mm_avg_epu8(p0, q0); /* (p0+q0+1)>>1 */\
	__m128i x1 = _mm_sub_epi8(_mm_avg_epu8(p2, x0), _mm_and_si128(_mm_xor_si128(p2, x0), c1)); /* (p2+((p0+q0+1)>>1))>>1 */\
	__m128i x2 = _mm_sub_epi8(_mm_avg_epu8(q2, x0), _mm_and_si128(_mm_xor_si128(q2, x0), c1)); /* (q2+((p0+q0+1)>>1))>>1 */\
	__m128i pp1 = _mm_min_epu8(_mm_max_epu8(x1, _mm_subs_epu8(p1, ftC0)), _mm_adds_epu8(p1, ftC0));\
	__m128i qp1 = _mm_min_epu8(_mm_max_epu8(x2, _mm_subs_epu8(q1, ftC0)), _mm_adds_epu8(q1, ftC0));\
	__m128i apltb = _mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(p2, p0), bm1), _mm_subs_epu8(_mm_subs_epu8(p0, p2), bm1));\
	__m128i aqltb = _mm_cmpeq_epi8(_mm_subs_epu8(_mm_subs_epu8(q2, q0), bm1), _mm_subs_epu8(_mm_subs_epu8(q0, q2), bm1));\
	__m128i sub1 = _mm_avg_epu8(p1, _mm_xor_si128(q1, _mm_set1_epi8(-1))); /* save 128+((p1-q1)>>1) for later */\
	p1 = _mm_blendv_epi8(p1, pp1, apltb);\
	q1 = _mm_blendv_epi8(q1, qp1, aqltb);\
	/* filter p0 and q0 (by offsetting signed to unsigned to apply pavg) */\
	__m128i ftC = _mm_and_si128(_mm_sub_epi8(_mm_sub_epi8(ftC0, apltb), aqltb), filterSamplesFlags);\
	__m128i x3 = _mm_avg_epu8(sub0, _mm_avg_epu8(sub1, _mm_set1_epi8(127))); /* 128+((q0-p0+((p1-q1)>>2)+1)>>1) */\
	__m128i delta = _mm_min_epu8(_mm_subs_epu8(x3, c128), ftC); /* delta if delta>0 */\
	__m128i ndelta = _mm_min_epu8(_mm_subs_epu8(c128, x3), ftC); /* -delta if delta<0 */\
	p0 = _mm_subs_epu8(_mm_adds_epu8(p0, delta), ndelta);\
	q0 = _mm_subs_epu8(_mm_adds_epu8(p0, delta), ndelta);}

#define DEBLOCK_CHROMA_SOFT(p1, p0, q0, q1, am1, bm1, tC0) {\
	/* compute filterSamplesFlags and apply if to tC */\
	__m128i c128 = _mm_set1_epi8(-128);\
	__m128i pq0 = _mm_subs_epu8(p0, q0);\
	__m128i qp0 = _mm_subs_epu8(q0, p0);\
	__m128i abs0 = _mm_or_si128(pq0, qp0);\
	__m128i sub0 = _mm_subs_epu8(_mm_adds_epu8(qp0, c128), pq0); /* save 128+q0-p0 for later */\
	__m128i abs1 = _mm_or_si128(_mm_subs_epu8(p1, p0), _mm_subs_epu8(p0, p1));\
	__m128i abs2 = _mm_or_si128(_mm_subs_epu8(q1, q0), _mm_subs_epu8(q0, q1));\
	__m128i or = _mm_or_si128(_mm_subs_epu8(abs0, am1), _mm_subs_epu8(_mm_max_epu8(abs1, abs2), bm1));\
	__m128i filterSamplesFlags = _mm_cmpeq_epi8(or, _mm_setzero_si128()); /* bS=0 is already handled in tC0 */\
	__m128i cm1 = _mm_set1_epi8(-1);\
	__m128i ftC = _mm_and_si128(_mm_sub_epi8(tC0, cm1), filterSamplesFlags);\
	/* filter p0 and q0 (by offsetting signed to unsigned to apply pavg) */\
	__m128i sub1 = _mm_avg_epu8(p1, _mm_xor_si128(q1, cm1)); /* 128+((p1-q1)>>1) */\
	__m128i x3 = _mm_avg_epu8(sub0, _mm_avg_epu8(sub1, _mm_set1_epi8(127))); /* 128+((q0-p0+((p1-q1)>>2)+1)>>1) */\
	__m128i delta = _mm_min_epu8(_mm_subs_epu8(x3, c128), ftC); /* delta if delta>0 */\
	__m128i ndelta = _mm_min_epu8(_mm_subs_epu8(c128, x3), ftC); /* -delta if delta<0 */\
	p0 = _mm_subs_epu8(_mm_adds_epu8(p0, delta), ndelta);\
	q0 = _mm_subs_epu8(_mm_adds_epu8(p0, delta), ndelta);}



/**
 * Filter a single edge in place for bS=4.
 */
#define DEBLOCK_LUMA_HARD(p3, p2, p1, p0, q0, q1, q2, q3, am1, bm1) {\
	/* compute filterSamplesFlags and condition masks for filtering modes */\
	__m128i abs0 = _mm_or_si128(_mm_subs_epu8(p0, q0), _mm_subs_epu8(q0, p0));\
	__m128i abs1 = _mm_or_si128(_mm_subs_epu8(p1, p0), _mm_subs_epu8(p0, p1));\
	__m128i abs2 = _mm_or_si128(_mm_subs_epu8(q1, q0), _mm_subs_epu8(q0, q1));\
	__m128i abs3 = _mm_or_si128(_mm_subs_epu8(p2, p0), _mm_subs_epu8(p0, p2));\
	__m128i abs4 = _mm_or_si128(_mm_subs_epu8(q2, q0), _mm_subs_epu8(q0, q2));\
	__m128i zero = _mm_setzero_si128();\
	__m128i c1 = _mm_set1_epi8(1);\
	__m128i filterSamplesFlags = _mm_cmpeq_epi8(_mm_or_si128(_mm_subs_epu8(abs0, am1), _mm_subs_epu8(_mm_max_epu8(abs1, abs2), bm1)), zero);\
	__m128i condpq = _mm_subs_epu8(abs0, _mm_add_epi8(_mm_avg_epu8(_mm_avg_epu8(am1, zero), zero), c1));\
	__m128i condp = _mm_cmpeq_epi8(_mm_or_si128(_mm_subs_epu8(abs3, bm1), condpq), zero);\
	__m128i condq = _mm_cmpeq_epi8(_mm_or_si128(_mm_subs_epu8(abs4, bm1), condpq), zero);\
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
	p0 = _mm_blendv_epi8(p0, _mm_blendv_epi8(pp0b, pp0a, condp), filterSamplesFlags);\
	q0 = _mm_blendv_epi8(q0, _mm_blendv_epi8(qp0b, qp0a, condq), filterSamplesFlags);\
	/* compute p'1 and q'1 */\
	__m128i fcondp = _mm_and_si128(condp, filterSamplesFlags);\
	__m128i fcondq = _mm_and_si128(condq, filterSamplesFlags);\
	__m128i p21 = _mm_sub_epi8(_mm_avg_epu8(p2, p1), _mm_and_si128(_mm_xor_si128(p2, p1), and0)); /* p21+pq0 == (p2+p1+p0+q0)/2 */\
	__m128i q21 = _mm_sub_epi8(_mm_avg_epu8(q2, q1), _mm_and_si128(_mm_xor_si128(q2, q1), and0)); /* q21+pq0 == (q2+q1+q0+p1)/2 */\
	__m128i pp1 = _mm_avg_epu8(p21, pq0); /* p'1 */\
	__m128i qp1 = _mm_avg_epu8(q21, pq0); /* q'1 */\
	p1 = _mm_blendv_epi8(p1, pp1, fcondp);\
	q1 = _mm_blendv_epi8(q1, qp1, fcondq);\
	/* compute p'2 and q'2 */\
	__m128i fix1 = _mm_and_si128(_mm_xor_si128(p21, pq0), c1);\
	__m128i fix2 = _mm_and_si128(_mm_xor_si128(q21, pq0), c1);\
	__m128i p210q0 = _mm_sub_epi8(pp1, fix1); /* (p2+p1+p0+q0)/4 */\
	__m128i q210p0 = _mm_sub_epi8(qp1, fix2); /* (q2+q1+q0+p0)/4 */\
	__m128i p3p2 = _mm_sub_epi8(_mm_avg_epu8(p3, p2), _mm_and_si128(_mm_xor_si128(p3, p2), _mm_xor_si128(fix1, c1))); /* p3p2+p210q0 == (p3+p2+(p2+p1+p0+q0)/2)/2 */\
	__m128i q3q2 = _mm_sub_epi8(_mm_avg_epu8(q3, q2), _mm_and_si128(_mm_xor_si128(q3, q2), _mm_xor_si128(fix2, c1))); /* q3q2+q210p0 == (q3+q2+(q2+q1+p0+q0)/2)/2 */\
	__m128i pp2 = _mm_avg_epu8(p3p2, p210q0); /* p'2 */\
	__m128i qp2 = _mm_avg_epu8(q3q2, q210p0); /* q'2 */\
	p2 = _mm_blendv_epi8(p2, pp2, fcondp);\
	q2 = _mm_blendv_epi8(q2, qp2, fcondq);}

#define DEBLOCK_CHROMA_HARD(p1, p0, q0, q1, am1, bm1) {\
	/* compute filterSamplesFlags */\
	__m128i abs0 = _mm_or_si128(_mm_subs_epu8(p0, q0), _mm_subs_epu8(q0, p0));\
	__m128i abs1 = _mm_or_si128(_mm_subs_epu8(p1, p0), _mm_subs_epu8(p0, p1));\
	__m128i abs2 = _mm_or_si128(_mm_subs_epu8(q1, q0), _mm_subs_epu8(q0, q1));\
	__m128i or = _mm_or_si128(_mm_subs_epu8(abs0, am1), _mm_subs_epu8(_mm_max_epu8(abs1, abs2), bm1));\
	__m128i filterSamplesFlags = _mm_cmpeq_epi8(or, _mm_setzero_si128());\
	/* compute p'0 and q'0 */\
	__m128i c1 = _mm_set1_epi8(1);\
	__m128i pp0b = _mm_avg_epu8(p1, _mm_sub_epi8(_mm_avg_epu8(p0, q1), _mm_and_si128(_mm_xor_si128(p0, q1), c1))); /* p'0 (second formula) */\
	__m128i qp0b = _mm_avg_epu8(q1, _mm_sub_epi8(_mm_avg_epu8(q0, p1), _mm_and_si128(_mm_xor_si128(q0, p1), c1))); /* q'0 (second formula) */\
	p0 = _mm_blendv_epi8(p0, pp0b, filterSamplesFlags);\
	q0 = _mm_blendv_epi8(q0, qp0b, filterSamplesFlags);}



/**
 * Transpose a 8x16 matrix into 16x8.
 * 
 * With 16 available registers it is expected that most input vectors are on
 * stack, so this macro is to be viewed as "reload and transpose".
 */
#define TRANSPOSE_8x16(src, lohi, dst0, dst1, dst2, dst3, dst4, dst5, dst6, dst7) {\
	__m128i a0 = _mm_unpack##lohi##_epi8(src##0, src##1);\
	__m128i a1 = _mm_unpack##lohi##_epi8(src##2, src##3);\
	__m128i a2 = _mm_unpack##lohi##_epi8(src##4, src##5);\
	__m128i a3 = _mm_unpack##lohi##_epi8(src##6, src##7);\
	__m128i a4 = _mm_unpack##lohi##_epi8(src##8, src##9);\
	__m128i a5 = _mm_unpack##lohi##_epi8(src##A, src##B);\
	__m128i a6 = _mm_unpack##lohi##_epi8(src##C, src##D);\
	__m128i a7 = _mm_unpack##lohi##_epi8(src##E, src##F);\
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



/**
 * Helper functions
 */
static always_inline __m128i expand4(int a) {
	__m128i x0 = _mm_cvtsi32_si128(a);
	__m128i x1 = _mm_unpacklo_epi8(x0, x0);
	return _mm_unpacklo_epi8(x1, x1);
}
static always_inline __m128i expand2(int64_t a) {
	__m128i x0 = _mm_cvtsi64_si128(a);
	return _mm_unpacklo_epi8(x0, x0);
}



/**
 * Deblock a single luma macroblock in place.
 */
static noinline void FUNC(deblock_Y_8bit, uint8_t * restrict px0, size_t stride, ssize_t nstride, size_t stride7)
{
	__m128i v0, v1, v2, v3, v4, v5, v6, v7;
	if (!mb[-1].f.unavailable) {
		// load and transpose the left 12x16 matrix
		__m128i xa0 = _mm_loadu_si128((__m128i *)(px0               - 8));
		__m128i xa1 = _mm_loadu_si128((__m128i *)(px0 +  stride     - 8));
		__m128i xa2 = _mm_loadu_si128((__m128i *)(px0 +  stride * 2 - 8));
		uint8_t * restrict px7 = px0 + stride7;
		__m128i xa3 = _mm_loadu_si128((__m128i *)(px7 + nstride * 4 - 8));
		__m128i xa4 = _mm_loadu_si128((__m128i *)(px0 +  stride * 4 - 8));
		__m128i xa5 = _mm_loadu_si128((__m128i *)(px7 + nstride * 2 - 8));
		__m128i xa6 = _mm_loadu_si128((__m128i *)(px7 + nstride     - 8));
		__m128i xa7 = _mm_loadu_si128((__m128i *)(px7               - 8));
		__m128i xa8 = _mm_loadu_si128((__m128i *)(px7 +  stride     - 8));
		__m128i xa9 = _mm_loadu_si128((__m128i *)(px7 +  stride * 2 - 8));
		uint8_t * restrict pxE = px7 + stride7;
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
		
		// first vertical edge
		__m128i cm1a = _mm_set1_epi8(-1);
		__m128i am1a = _mm_add_epi8(_mm_set1_epi8(ctx->alpha[8]), cm1a);
		__m128i bm1a = _mm_add_epi8(_mm_set1_epi8(ctx->beta[8]), cm1a);
		if (mb[-1].f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			__m128i tC0a = expand4(ctx->tC0_s[0]);
			if (ctx->tC0_s[0] != -1)
				DEBLOCK_LUMA_SOFT(vX, vY, vZ, v0, v1, v2, am1a, bm1a, tC0a);
		} else if (ctx->alpha[8] != 0) {
			DEBLOCK_LUMA_HARD(vW, vX, vY, vZ, v0, v1, v2, v3, am1a, bm1a);
		}
		
		// store vW/vX/vY/vZ into the left macroblock
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
		px7 = px0 + stride7;
		*(int32_t *)(px7 + nstride * 4 - 4) = xc4[3];
		*(int32_t *)(px0 +  stride * 4 - 4) = xc5[0];
		*(int32_t *)(px7 + nstride * 2 - 4) = xc5[1];
		*(int32_t *)(px7 + nstride     - 4) = xc5[2];
		*(int32_t *)(px7               - 4) = xc5[3];
		*(int32_t *)(px7 +  stride     - 4) = xc6[0];
		*(int32_t *)(px7 +  stride * 2 - 4) = xc6[1];
		pxE = px7 + stride7;
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
		uint8_t * restrict px7 = px0 + stride7;
		__m128i xa3 = *(__m128i *)(px7 + nstride * 4);
		__m128i xa4 = *(__m128i *)(px0 +  stride * 4);
		__m128i xa5 = *(__m128i *)(px7 + nstride * 2);
		__m128i xa6 = *(__m128i *)(px7 + nstride    );
		__m128i xa7 = *(__m128i *)(px7              );
		__m128i xa8 = *(__m128i *)(px7 +  stride    );
		__m128i xa9 = *(__m128i *)(px7 +  stride * 2);
		uint8_t * restrict pxE = px7 + stride7;
		__m128i xaA = *(__m128i *)(pxE + nstride * 4);
		__m128i xaB = *(__m128i *)(px7 +  stride * 4);
		__m128i xaC = *(__m128i *)(pxE + nstride * 2);
		__m128i xaD = *(__m128i *)(pxE + nstride    );
		__m128i xaE = *(__m128i *)(pxE              );
		__m128i xaF = *(__m128i *)(pxE +  stride    );
		TRANSPOSE_8x16(xa, lo, v0, v1, v2, v3, v4, v5, v6, v7);
	}
	
	// second vertical edge
	__m128i cm1bcdfgh = _mm_set1_epi8(-1);
	__m128i am1bcdfgh = _mm_add_epi8(_mm_set1_epi8(ctx->alpha[0]), cm1bcdfgh);
	__m128i bm1bcdfgh = _mm_add_epi8(_mm_set1_epi8(ctx->beta[0]), cm1bcdfgh);
	if (!mb->f.transform_size_8x8_flag) {
		__m128i tC0b = expand4(ctx->tC0_s[1]);
		if (ctx->tC0_s[1] != -1)
			DEBLOCK_LUMA_SOFT(v1, v2, v3, v4, v5, v6, am1bcdfgh, bm1bcdfgh, tC0b);
	}
	
	// load and transpose the right 8x16 matrix
	__m128i xa0 = *(__m128i *)(px0              );
	__m128i xa1 = *(__m128i *)(px0 +  stride    );
	__m128i xa2 = *(__m128i *)(px0 +  stride * 2);
	uint8_t * restrict px7 = px0 + stride7;
	__m128i xa3 = *(__m128i *)(px7 + nstride * 4);
	__m128i xa4 = *(__m128i *)(px0 +  stride * 4);
	__m128i xa5 = *(__m128i *)(px7 + nstride * 2);
	__m128i xa6 = *(__m128i *)(px7 + nstride    );
	__m128i xa7 = *(__m128i *)(px7              );
	__m128i xa8 = *(__m128i *)(px7 +  stride    );
	__m128i xa9 = *(__m128i *)(px7 +  stride * 2);
	uint8_t * restrict pxE = px7 + stride7;
	__m128i xaA = *(__m128i *)(pxE + nstride * 4);
	__m128i xaB = *(__m128i *)(px7 +  stride * 4);
	__m128i xaC = *(__m128i *)(pxE + nstride * 2);
	__m128i xaD = *(__m128i *)(pxE + nstride    );
	__m128i xaE = *(__m128i *)(pxE              );
	__m128i xaF = *(__m128i *)(pxE +  stride    );
	__m128i v8, v9, vA, vB, vC, vD, vE, vF;
	TRANSPOSE_8x16(xa, hi, v8, v9, vA, vB, vC, vD, vE, vF);
	
	// third vertical edge
	__m128i tC0c = expand4(ctx->tC0_s[2]);
	if (ctx->tC0_s[2] != -1)
		DEBLOCK_LUMA_SOFT(v5, v6, v7, v8, v9, vA, am1bcdfgh, bm1bcdfgh, tC0c);
	
	// fourth vertical edge
	if (!mb->f.transform_size_8x8_flag) {
		__m128i tC0d = expand4(ctx->tC0_s[3]);
		if (ctx->tC0_s[3] != -1)
			DEBLOCK_LUMA_SOFT(v9, vA, vB, vC, vD, vE, am1bcdfgh, bm1bcdfgh, tC0d);
	}
	
	// transpose the top 16x8 matrix
	__m128i h0, h1, h2, h3, h4, h5, h6, h7;
	TRANSPOSE_8x16(v, lo, h0, h1, h2, h3, h4, h5, h6, h7);
	
	// first horizontal edge
	if (!ctx->mbB->f.unavailable) {
		__m128i cm1e = _mm_set1_epi8(-1);
		__m128i am1e = _mm_add_epi8(_mm_set1_epi8(ctx->alpha[12]), cm1e);
		__m128i bm1e = _mm_add_epi8(_mm_set1_epi8(ctx->beta[12]), cm1e);
		if (ctx->mbB->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			__m128i tC0e = expand4(ctx->tC0_s[4]);
			if (ctx->tC0_s[4] != -1)
				DEBLOCK_LUMA_SOFT(*(__m128i *)(px0 + nstride * 3), *(__m128i *)(px0 + nstride * 2), *(__m128i *)(px0 + nstride    ), h0, h1, h2, am1e, bm1e, tC0e);
		} else if (ctx->alpha[12] != 0) {
			DEBLOCK_LUMA_HARD(*(__m128i *)(px0 + nstride * 4), *(__m128i *)(px0 + nstride * 3), *(__m128i *)(px0 + nstride * 2), *(__m128i *)(px0 + nstride    ), h0, h1, h2, h3, am1e, bm1e);
		}
	}
	*(__m128i *)(px0              ) = h0;
	*(__m128i *)(px0 +  stride    ) = h1;
	
	// second horizontal edge
	if (!mb->f.transform_size_8x8_flag) {
		__m128i tC0f = expand4(ctx->tC0_s[5]);
		if (ctx->tC0_s[5] != -1)
			DEBLOCK_LUMA_SOFT(h1, h2, h3, h4, h5, h6, am1bcdfgh, bm1bcdfgh, tC0f);
	}
	px7 = px0 + stride7;
	*(__m128i *)(px0 +  stride * 2) = h2;
	*(__m128i *)(px7 + nstride * 4) = h3;
	*(__m128i *)(px0 +  stride * 4) = h4;
	*(__m128i *)(px7 + nstride * 2) = h5;
	
	// transpose the bottom 16x8 matrix
	__m128i h8, h9, hA, hB, hC, hD, hE, hF;
	TRANSPOSE_8x16(v, hi, h8, h9, hA, hB, hC, hD, hE, hF);
	pxE = px0 + stride7;
	*(__m128i *)(pxE              ) = hE;
	*(__m128i *)(pxE +  stride    ) = hF;
	
	// third horizontal edge
	__m128i tC0g = expand4(ctx->tC0_s[6]);
	if (ctx->tC0_s[6] != -1)
		DEBLOCK_LUMA_SOFT(h5, h6, h7, h8, h9, hA, am1bcdfgh, bm1bcdfgh, tC0g);
	*(__m128i *)(px7 + nstride    ) = h6;
	*(__m128i *)(px7              ) = h7;
	*(__m128i *)(px7 +  stride    ) = h8;
	*(__m128i *)(px7 +  stride * 2) = h9;
	
	// fourth horizontal edge
	if (!mb->f.transform_size_8x8_flag) {
		__m128i tC0h = expand4(ctx->tC0_s[7]);
		if (ctx->tC0_s[7] != -1)
			DEBLOCK_LUMA_SOFT(h9, hA, hB, hC, hD, hE, am1bcdfgh, bm1bcdfgh, tC0h);
	}
	*(__m128i *)(pxE + nstride * 4) = hA;
	*(__m128i *)(px7 +  stride * 4) = hB;
	*(__m128i *)(pxE + nstride * 2) = hC;
	*(__m128i *)(pxE + nstride    ) = hD;
}



static noinline void FUNC(deblock_CbCr_8bit, uint8_t * restrict Cb0, uint8_t * restrict Cr0, size_t stride, ssize_t nstride, size_t stride7)
{
	__m128i v0, v1, v2, v3, v4, v5, v6, v7;
	if (!mb[-1].f.unavailable) {
		// load and transpose both 12x8 matrices (with left macroblock)
		__m128i xa0 = _mm_loadu_si128((__m128i *)(Cb0               - 8));
		__m128i xa1 = _mm_loadu_si128((__m128i *)(Cb0 +  stride     - 8));
		__m128i xa2 = _mm_loadu_si128((__m128i *)(Cb0 +  stride * 2 - 8));
		uint8_t *Cb7 = Cb0 + stride7;
		__m128i xa3 = _mm_loadu_si128((__m128i *)(Cb7 + nstride * 4 - 8));
		__m128i xa4 = _mm_loadu_si128((__m128i *)(Cb0 +  stride * 4 - 8));
		__m128i xa5 = _mm_loadu_si128((__m128i *)(Cb7 + nstride * 2 - 8));
		__m128i xa6 = _mm_loadu_si128((__m128i *)(Cb7 + nstride     - 8));
		__m128i xa7 = _mm_loadu_si128((__m128i *)(Cb7               - 8));
		__m128i xa8 = _mm_loadu_si128((__m128i *)(Cr0               - 8));
		__m128i xa9 = _mm_loadu_si128((__m128i *)(Cr0 +  stride     - 8));
		__m128i xaA = _mm_loadu_si128((__m128i *)(Cr0 +  stride * 2 - 8));
		uint8_t *Cr7 = Cr0 + stride7;
		__m128i xaB = _mm_loadu_si128((__m128i *)(Cr7 + nstride * 4 - 8));
		__m128i xaC = _mm_loadu_si128((__m128i *)(Cr0 +  stride * 4 - 8));
		__m128i xaD = _mm_loadu_si128((__m128i *)(Cr7 + nstride * 2 - 8));
		__m128i xaE = _mm_loadu_si128((__m128i *)(Cr7 + nstride     - 8));
		__m128i xaF = _mm_loadu_si128((__m128i *)(Cr7               - 8));
		__m128i xb0 = _mm_unpackhi_epi16(_mm_unpacklo_epi8(xa0, xa1), _mm_unpacklo_epi8(xa2, xa3));
		__m128i xb1 = _mm_unpackhi_epi16(_mm_unpacklo_epi8(xa4, xa5), _mm_unpacklo_epi8(xa6, xa7));
		__m128i xb2 = _mm_unpackhi_epi16(_mm_unpacklo_epi8(xa8, xa9), _mm_unpacklo_epi8(xaA, xaB));
		__m128i xb3 = _mm_unpackhi_epi16(_mm_unpacklo_epi8(xaC, xaD), _mm_unpacklo_epi8(xaE, xaF));
		__m128i xb4 = _mm_unpackhi_epi32(xb0, xb1);
		__m128i xb5 = _mm_unpackhi_epi32(xb2, xb3);
		__m128i vY = _mm_unpacklo_epi64(xb4, xb5);
		__m128i vZ = _mm_unpackhi_epi64(xb4, xb5);
		TRANSPOSE_8x16(xa, hi, v0, v1, v2, v3, v4, v5, v6, v7);
		
		// first vertical edge
		__m128i shuf_a = _mm_setr_epi8(9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10);
		__m128i cm1a = _mm_set1_epi8(-1);
		__m128i am1a = _mm_add_epi8(_mm_shuffle_epi8((__m128i)ctx->alpha_v, shuf_a), cm1a);
		__m128i bm1a = _mm_add_epi8(_mm_shuffle_epi8((__m128i)ctx->beta_v, shuf_a), cm1a);
		if (mb[-1].f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			__m128i tC0a = expand2(ctx->tC0_l[4]);
			if (ctx->tC0_l[4] != -1)
				DEBLOCK_CHROMA_SOFT(vY, vZ, v0, v1, am1a, bm1a, tC0a);
		} else { // FIXME alpha may be 0 and not 0 !
			DEBLOCK_CHROMA_HARD(vY, vZ, v0, v1, am1a, bm1a);
		}
		
		// store vY/vZ into the left macroblock
		v8hi xc0 = (v8hi)_mm_unpacklo_epi8(vY, vZ);
		v8hi xc1 = (v8hi)_mm_unpackhi_epi8(vY, vZ);
		*(int16_t *)(Cb0               - 2) = xc0[0];
		*(int16_t *)(Cb0 +  stride     - 2) = xc0[1];
		*(int16_t *)(Cb0 +  stride * 2 - 2) = xc0[2];
		Cb7 = Cb0 + stride7;
		*(int16_t *)(Cb7 + nstride * 4 - 2) = xc0[3];
		*(int16_t *)(Cb0 +  stride * 4 - 2) = xc0[4];
		*(int16_t *)(Cb7 + nstride * 2 - 2) = xc0[5];
		*(int16_t *)(Cb7 + nstride     - 2) = xc0[6];
		*(int16_t *)(Cb7               - 2) = xc0[7];
		*(int16_t *)(Cr0               - 2) = xc1[0];
		*(int16_t *)(Cr0 +  stride     - 2) = xc1[1];
		*(int16_t *)(Cr0 +  stride * 2 - 2) = xc1[2];
		Cr7 = Cr0 + stride7;
		*(int16_t *)(Cr7 + nstride * 4 - 2) = xc1[3];
		*(int16_t *)(Cr0 +  stride * 4 - 2) = xc1[4];
		*(int16_t *)(Cr7 + nstride * 2 - 2) = xc1[5];
		*(int16_t *)(Cr7 + nstride     - 2) = xc1[6];
		*(int16_t *)(Cr7               - 2) = xc1[7];
	} else {
		// load and transpose both 8x8 matrices
		__m128i xa0 = _mm_loadl_epi64((__m128i *)(Cb0              ));
		__m128i xa1 = _mm_loadl_epi64((__m128i *)(Cb0 +  stride    ));
		__m128i xa2 = _mm_loadl_epi64((__m128i *)(Cb0 +  stride * 2));
		uint8_t *Cb7 = Cb0 + stride7;
		__m128i xa3 = _mm_loadl_epi64((__m128i *)(Cb7 + nstride * 4));
		__m128i xa4 = _mm_loadl_epi64((__m128i *)(Cb0 +  stride * 4));
		__m128i xa5 = _mm_loadl_epi64((__m128i *)(Cb7 + nstride * 2));
		__m128i xa6 = _mm_loadl_epi64((__m128i *)(Cb7 + nstride    ));
		__m128i xa7 = _mm_loadl_epi64((__m128i *)(Cb7              ));
		__m128i xa8 = _mm_loadl_epi64((__m128i *)(Cr0              ));
		__m128i xa9 = _mm_loadl_epi64((__m128i *)(Cr0 +  stride    ));
		__m128i xaA = _mm_loadl_epi64((__m128i *)(Cr0 +  stride * 2));
		uint8_t *Cr7 = Cr0 + stride7;
		__m128i xaB = _mm_loadl_epi64((__m128i *)(Cr7 + nstride * 4));
		__m128i xaC = _mm_loadl_epi64((__m128i *)(Cr0 +  stride * 4));
		__m128i xaD = _mm_loadl_epi64((__m128i *)(Cr7 + nstride * 2));
		__m128i xaE = _mm_loadl_epi64((__m128i *)(Cr7 + nstride    ));
		__m128i xaF = _mm_loadl_epi64((__m128i *)(Cr7              ));
		TRANSPOSE_8x16(xa, lo, v0, v1, v2, v3, v4, v5, v6, v7);
	}
	
	// third vertical edge
	__m128i shuf_cg = _mm_setr_epi8(1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2);
	__m128i cm1cg = _mm_set1_epi8(-1);
	__m128i am1cg = _mm_add_epi8(_mm_shuffle_epi8((__m128i)ctx->alpha_v, shuf_cg), cm1cg);
	__m128i bm1cg = _mm_add_epi8(_mm_shuffle_epi8((__m128i)ctx->beta_v, shuf_cg), cm1cg);
	__m128i tC0c = expand2(ctx->tC0_l[5]);
	if (ctx->tC0_l[5] != -1)
		DEBLOCK_CHROMA_SOFT(v2, v3, v4, v5, am1cg, bm1cg, tC0c);
	
	// transpose both 8x8 matrices
	__m128i xd0 = _mm_unpacklo_epi8(v0, v1);
	__m128i xd1 = _mm_unpackhi_epi8(v0, v1);
	__m128i xd2 = _mm_unpacklo_epi8(v2, v3);
	__m128i xd3 = _mm_unpackhi_epi8(v2, v3);
	__m128i xd4 = _mm_unpacklo_epi8(v4, v5);
	__m128i xd5 = _mm_unpackhi_epi8(v4, v5);
	__m128i xd6 = _mm_unpacklo_epi8(v6, v7);
	__m128i xd7 = _mm_unpackhi_epi8(v6, v7);
	__m128i xe0 = _mm_unpacklo_epi16(xd0, xd2);
	__m128i xe1 = _mm_unpackhi_epi16(xd0, xd2);
	__m128i xe2 = _mm_unpacklo_epi16(xd1, xd3);
	__m128i xe3 = _mm_unpackhi_epi16(xd1, xd3);
	__m128i xe4 = _mm_unpacklo_epi16(xd4, xd6);
	__m128i xe5 = _mm_unpackhi_epi16(xd4, xd6);
	__m128i xe6 = _mm_unpacklo_epi16(xd5, xd7);
	__m128i xe7 = _mm_unpackhi_epi16(xd5, xd7);
	__m128i xf0 = _mm_unpacklo_epi32(xe0, xe4);
	__m128i xf1 = _mm_unpackhi_epi32(xe0, xe4);
	__m128i xf2 = _mm_unpacklo_epi32(xe1, xe5);
	__m128i xf3 = _mm_unpackhi_epi32(xe1, xe5);
	__m128i xf4 = _mm_unpacklo_epi32(xe2, xe6);
	__m128i xf5 = _mm_unpackhi_epi32(xe2, xe6);
	__m128i xf6 = _mm_unpacklo_epi32(xe3, xe7);
	__m128i xf7 = _mm_unpackhi_epi32(xe3, xe7);
	__m128i h0 = _mm_unpacklo_epi64(xf0, xf4);
	__m128i h1 = _mm_unpackhi_epi64(xf0, xf4);
	__m128i h2 = _mm_unpacklo_epi64(xf1, xf5);
	__m128i h3 = _mm_unpackhi_epi64(xf1, xf5);
	__m128i h4 = _mm_unpacklo_epi64(xf2, xf6);
	__m128i h5 = _mm_unpackhi_epi64(xf2, xf6);
	__m128i h6 = _mm_unpacklo_epi64(xf3, xf7);
	__m128i h7 = _mm_unpackhi_epi64(xf3, xf7);
	
	// first horizontal edge
	if (!ctx->mbB->f.unavailable) {
		__m128i shuf_e = _mm_setr_epi8(13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14);
		__m128i cm1e = _mm_set1_epi8(-1);
		__m128i am1e = _mm_add_epi8(_mm_shuffle_epi8((__m128i)ctx->alpha_v, shuf_e), cm1e);
		__m128i bm1e = _mm_add_epi8(_mm_shuffle_epi8((__m128i)ctx->beta_v, shuf_e), cm1e);
		__m128i hY = _mm_setr_epi64(*(__m64 *)(Cb0 + nstride * 2), *(__m64 *)(Cr0 + nstride * 2));
		__m128i hZ = _mm_setr_epi64(*(__m64 *)(Cb0 + nstride    ), *(__m64 *)(Cr0 + nstride    ));
		if (ctx->mbB->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			__m128i tC0e = expand2(ctx->tC0_l[6]);
			if (ctx->tC0_l[6] != -1)
				DEBLOCK_CHROMA_SOFT(hY, hZ, h0, h1, am1e, bm1e, tC0e);
		} else {
			DEBLOCK_CHROMA_HARD(hY, hZ, h0, h1, am1e, bm1e);
		}
		*(int64_t *)(Cb0 + nstride * 2) = ((v2li)hY)[0];
		*(int64_t *)(Cr0 + nstride * 2) = ((v2li)hY)[1];
		*(int64_t *)(Cb0 + nstride    ) = ((v2li)hZ)[0];
		*(int64_t *)(Cr0 + nstride    ) = ((v2li)hZ)[1];
	}
	*(int64_t *)(Cb0              ) = ((v2li)h0)[0];
	*(int64_t *)(Cr0              ) = ((v2li)h0)[1];
	*(int64_t *)(Cb0 +  stride    ) = ((v2li)h1)[0];
	*(int64_t *)(Cr0 +  stride    ) = ((v2li)h1)[1];
	
	// third horizontal edge
	__m128i tC0g = expand2(ctx->tC0_l[7]);
	if (ctx->tC0_l[7] != -1)
		DEBLOCK_CHROMA_SOFT(h2, h3, h4, h5, am1cg, bm1cg, tC0g);
	*(int64_t *)(Cb0 +  stride * 2) = ((v2li)h2)[0];
	*(int64_t *)(Cr0 +  stride * 2) = ((v2li)h2)[1];
	uint8_t *Cb7 = Cb0 + stride7;
	uint8_t *Cr7 = Cr0 + stride7;
	*(int64_t *)(Cb7 + nstride * 4) = ((v2li)h3)[0];
	*(int64_t *)(Cr7 + nstride * 4) = ((v2li)h3)[1];
	*(int64_t *)(Cb0 +  stride * 4) = ((v2li)h4)[0];
	*(int64_t *)(Cr0 +  stride * 4) = ((v2li)h4)[1];
	*(int64_t *)(Cb7 + nstride * 2) = ((v2li)h5)[0];
	*(int64_t *)(Cr7 + nstride * 2) = ((v2li)h5)[1];
	*(int64_t *)(Cb7 + nstride    ) = ((v2li)h6)[0];
	*(int64_t *)(Cr7 + nstride    ) = ((v2li)h6)[1];
	*(int64_t *)(Cb7              ) = ((v2li)h7)[0];
	*(int64_t *)(Cr7              ) = ((v2li)h7)[1];
}



noinline void FUNC(deblock_mb, uint8_t *samplesY, uint8_t *samplesCb, uint8_t *samplesCr, Edge264_macroblock *_mb) {
	size_t offsetB = (void *)mb - (void *)ctx->mbB;
	Edge264_macroblock *old = mb;
	mb = _mb;
	ctx->mbB = _mb - offsetB;
	CALL(init_alpha_beta_tC0);
	size_t strideY = ctx->stride[0];
	CALL(deblock_Y_8bit, samplesY, strideY, -strideY, strideY * 7);
	size_t strideC = ctx->stride[1];
	CALL(deblock_CbCr_8bit, samplesCb, samplesCr, strideC, -strideC, strideC * 7);
	mb = old;
	ctx->mbB = old - offsetB;
}
