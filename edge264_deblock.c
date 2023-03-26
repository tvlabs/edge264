#include "edge264_internal.h"


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
static inline void FUNC(init_alpha_beta_tC0)
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
	i8x16 zero = {};
	i8x16 qP = set32((int32_t)mb->QP_s);
	i32x4 qPAB = {(int32_t)mb[-1].QP_s, (int32_t)mbB->QP_s};
	i8x16 qPav = avg8(qP, unpacklo64(qP, qPAB)); // mid/mid/A/B
	i8x16 c51 = set8(51);
	i8x16 indexA = umin8(max8(qPav + set8(ctx->FilterOffsetA), zero), c51);
	i8x16 indexB = umin8(max8(qPav + set8(ctx->FilterOffsetB), zero), c51);
	
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
	ctx->alpha_v = ifelse_mask(Agte36, alphahi, ifelse_mask(Agte20, alphamd, alphalo));
	ctx->beta_v = ifelse_mask(Bgte36, betahi, ifelse_mask(Bgte20, betamd, betalo));
	
	// initialize tC0 with bS=3 for internal edges of Intra macroblock
	i8x16 tC0neg = zero > Am4;
	if (!mb->f.mbIsInterFlag) {
		i8x16 tC03lo = shuffle8(idx2tC0[2][0], Am4);
		i8x16 tC03md = shuffle8(idx2tC0[2][1], Am4);
		i8x16 tC03hi = shuffle8(idx2tC0[2][2], Am4);
		i8x16 tC03 = ifelse_mask(Agte36, tC03hi, ifelse_mask(Agte20, tC03md, tC03lo)) | tC0neg;
		ctx->tC0_v[0] = ctx->tC0_v[1] = shuffle8(tC03, zero);
		ctx->tC0_v[2] = ctx->tC0_v[3] = shuffle8(tC03, ((i8x16){-1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 2, 2, 2, 2}));
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
			i16x8 mvsv0l0 = shuffleps(mb[-1].mvs_v[1], mb[-1].mvs_v[3], 1, 3, 1, 3);
			i16x8 mvsv1l0 = shuffleps(mb->mvs_v[0], mb->mvs_v[2], 0, 2, 0, 2);
			i16x8 mvsv0l1 = shuffleps(mb[-1].mvs_v[5], mb[-1].mvs_v[7], 1, 3, 1, 3);
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
			i64x2 refPicAB = {mb[-1].refPic_l, mbB->refPic_l};
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
		} else if ((mb->refIdx_s[1] & mb[-1].refIdx_s[1] & mbB->refIdx_s[1]) == -1) { // P macroblocks
			i16x8 mvsv0 = shuffleps(mb[-1].mvs_v[1], mb[-1].mvs_v[3], 1, 3, 1, 3);
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
			i8x16 refs = shuffle8(((i32x4){mb->refPic_s[0], 0, mb[-1].refPic_s[0], mbB->refPic_s[0]}), shufVHAB); // (v0,h0,A0,B0)
			i8x16 neq = refs ^ (i8x16)unpackhi64(refs, refs); // (v0^A0,h0^B0,0,0)
			i8x16 refsaceg = unpacklo8(neq, neq);
			bS0aceg = (refsaceg | mvsaceg) == zero;
			bS0bdfh = mvsbdfh == zero;
		} else { // B macroblocks
			i16x8 mvsv0l0 = shuffleps(mb[-1].mvs_v[1], mb[-1].mvs_v[3], 1, 3, 1, 3);
			i16x8 mvsv1l0 = shuffleps(mb->mvs_v[0], mb->mvs_v[2], 0, 2, 0, 2);
			i16x8 mvsv2l0 = shuffleps(mb->mvs_v[0], mb->mvs_v[2], 1, 3, 1, 3);
			i16x8 mvsv3l0 = shuffleps(mb->mvs_v[1], mb->mvs_v[3], 0, 2, 0, 2);
			i16x8 mvsv4l0 = shuffleps(mb->mvs_v[1], mb->mvs_v[3], 1, 3, 1, 3);
			i16x8 mvsv0l1 = shuffleps(mb[-1].mvs_v[5], mb[-1].mvs_v[7], 1, 3, 1, 3);
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
			i64x2 refPicAB = {mb[-1].refPic_l, mbB->refPic_l};
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
		if (!ctx->pps.entropy_coding_mode_flag && mb->f.transform_size_8x8_flag) {
			i8x16 x = (i32x4)mb->nC_v[0] > 0;
			mb->nC_v[0] = nC = sign8(x, x);
		}
		
		// compute masks for edges with bS=2
		static const i8x16 shufV = {0, 2, 8, 10, 1, 3, 9, 11, 4, 6, 12, 14, 5, 7, 13, 15};
		static const i8x16 shufH = {0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15};
		i8x16 nnzv = shuffle8(nC, shufV);
		i8x16 nnzl = shuffle8(mb[-1].nC_v[0], shufV);
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
		ctx->tC0_v[0] = ifelse_mask(bS2abcd, shuffle8(tC02, shuf0), ifelse_mask(bS0abcd, tC00, shuffle8(tC01, shuf0)));
		ctx->tC0_v[1] = ifelse_mask(bS2efgh, shuffle8(tC02, shuf1), ifelse_mask(bS0efgh, tC00, shuffle8(tC01, shuf1)));
		ctx->tC0_v[2] = ifelse_mask(bS2aacc, shuffle8(tC02, shuf2), ifelse_mask(bS0aacc, tC00, shuffle8(tC01, shuf2)));
		ctx->tC0_v[3] = ifelse_mask(bS2eegg, shuffle8(tC02, shuf3), ifelse_mask(bS0eegg, tC00, shuffle8(tC01, shuf3)));
	}
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
#define DEBLOCK_LUMA_SOFT(p2, p1, p0, q0, q1, q2, alpha, beta, tC0) {\
	/* compute the opposite of filterSamplesFlags as a mask, and transfer the mask bS=0 from tC0 */\
	i8x16 c128 = set8(-128);\
	i8x16 pq0 = subus8(p0, q0);\
	i8x16 qp0 = subus8(q0, p0);\
	i8x16 sub0 = subus8(addus8(qp0, c128), pq0); /* save 128+q0-p0 for later */\
	i8x16 abs0 = pq0 | qp0;\
	i8x16 abs1 = subus8(p1, p0) | subus8(p0, p1);\
	i8x16 abs2 = subus8(q1, q0) | subus8(q0, q1);\
	i8x16 and = umin8(subus8(alpha, abs0), subus8(beta, umax8(abs1, abs2)));\
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
	i8x16 apltb = subus8(subus8(p2, p0), bm1) == subus8(subus8(p0, p2), bm1);\
	i8x16 aqltb = subus8(subus8(q2, q0), bm1) == subus8(subus8(q0, q2), bm1);\
	i8x16 sub1 = avg8(p1, q1 ^ cm1); /* save 128+((p1-q1)>>1) for later */\
	p1 = ifelse_mask(apltb, pp1, p1);\
	q1 = ifelse_mask(aqltb, qp1, q1);\
	/* filter p0 and q0 (by offsetting signed to unsigned to apply pavg) */\
	i8x16 ftC = (ftC0 - apltb - aqltb) & ~ignoreSamplesFlags;\
	i8x16 x3 = avg8(sub0, avg8(sub1, set8(127))); /* 128+((q0-p0+((p1-q1)>>2)+1)>>1) */\
	i8x16 delta = umin8(subus8(x3, c128), ftC); /* delta if delta>0 */\
	i8x16 ndelta = umin8(subus8(c128, x3), ftC); /* -delta if delta<0 */\
	p0 = subus8(addus8(p0, delta), ndelta);\
	q0 = subus8(addus8(q0, ndelta), delta);}

#define DEBLOCK_CHROMA_SOFT(p1, p0, q0, q1, alpha, beta, tC0) {\
	/* compute the opposite of filterSamplesFlags and apply if to tC */\
	i8x16 c128 = set8(-128);\
	i8x16 pq0 = subus8(p0, q0);\
	i8x16 qp0 = subus8(q0, p0);\
	i8x16 sub0 = subus8(addus8(qp0, c128), pq0); /* save 128+q0-p0 for later */\
	i8x16 abs0 = pq0 | qp0;\
	i8x16 abs1 = subus8(p1, p0) | subus8(p0, p1);\
	i8x16 abs2 = subus8(q1, q0) | subus8(q0, q1);\
	i8x16 and = umin8(subus8(alpha, abs0), subus8(beta, umax8(abs1, abs2)));\
	i8x16 ignoreSamplesFlags = and == 0;\
	i8x16 cm1 = set8(-1);\
	i8x16 ftC = ((i8x16)tC0 - cm1) & ~ignoreSamplesFlags;\
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
#define DEBLOCK_LUMA_HARD(p3, p2, p1, p0, q0, q1, q2, q3, alpha, beta) {\
	/* compute the opposite of filterSamplesFlags, and condition masks for filtering modes */\
	i8x16 abs0 = subus8(p0, q0) | subus8(q0, p0);\
	i8x16 abs1 = subus8(p1, p0) | subus8(p0, p1);\
	i8x16 abs2 = subus8(q1, q0) | subus8(q0, q1);\
	i8x16 abs3 = subus8(p2, p0) | subus8(p0, p2);\
	i8x16 abs4 = subus8(q2, q0) | subus8(q0, q2);\
	i8x16 zero = {};\
	i8x16 c1 = set8(1);\
	i8x16 ignoreSamplesFlags = umin8(subus8(alpha, abs0), subus8(beta, umax8(abs1, abs2))) == zero;\
	i8x16 bm1 = (i8x16)beta - c1;\
	i8x16 condpq = subus8(abs0, avg8(avg8(alpha, c1), zero)); /* abs0-((alpha>>2)+1) */\
	i8x16 condp = (subus8(abs3, bm1) | condpq) == zero;\
	i8x16 condq = (subus8(abs4, bm1) | condpq) == zero;\
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

#define DEBLOCK_CHROMA_HARD(p1, p0, q0, q1, alpha, beta) {\
	/* compute the opposite of filterSamplesFlags */\
	i8x16 abs0 = subus8(p0, q0) | subus8(q0, p0);\
	i8x16 abs1 = subus8(p1, p0) | subus8(p0, p1);\
	i8x16 abs2 = subus8(q1, q0) | subus8(q0, q1);\
	i8x16 and = umin8(subus8(alpha, abs0), subus8(beta, umax8(abs1, abs2)));\
	i8x16 ignoreSamplesFlags = and == 0;\
	/* compute p'0 and q'0 */\
	i8x16 c1 = set8(1);\
	i8x16 pp0b = avg8(p1, avg8(p0, q1) - ((p0 ^ q1) & c1)); /* p'0 (second formula) */\
	i8x16 qp0b = avg8(q1, avg8(q0, p1) - ((q0 ^ p1) & c1)); /* q'0 (second formula) */\
	p0 = ifelse_mask(ignoreSamplesFlags, p0, pp0b);\
	q0 = ifelse_mask(ignoreSamplesFlags, q0, qp0b);}



/**
 * Transpose a 8x16 matrix into 16x8.
 * 
 * With 16 available registers it is expected that most input vectors are on
 * stack, so this macro is to be viewed as "reload and transpose".
 */
#define TRANSPOSE_8x16(src, lohi, dst0, dst1, dst2, dst3, dst4, dst5, dst6, dst7) {\
	i8x16 a0 = unpack##lohi##8(src##0, src##1);\
	i8x16 a1 = unpack##lohi##8(src##2, src##3);\
	i8x16 a2 = unpack##lohi##8(src##4, src##5);\
	i8x16 a3 = unpack##lohi##8(src##6, src##7);\
	i8x16 a4 = unpack##lohi##8(src##8, src##9);\
	i8x16 a5 = unpack##lohi##8(src##A, src##B);\
	i8x16 a6 = unpack##lohi##8(src##C, src##D);\
	i8x16 a7 = unpack##lohi##8(src##E, src##F);\
	i8x16 b0 = unpacklo16(a0, a1);\
	i8x16 b1 = unpackhi16(a0, a1);\
	i8x16 b2 = unpacklo16(a2, a3);\
	i8x16 b3 = unpackhi16(a2, a3);\
	i8x16 b4 = unpacklo16(a4, a5);\
	i8x16 b5 = unpackhi16(a4, a5);\
	i8x16 b6 = unpacklo16(a6, a7);\
	i8x16 b7 = unpackhi16(a6, a7);\
	i8x16 c0 = unpacklo32(b0, b2);\
	i8x16 c1 = unpackhi32(b0, b2);\
	i8x16 c2 = unpacklo32(b1, b3);\
	i8x16 c3 = unpackhi32(b1, b3);\
	i8x16 c4 = unpacklo32(b4, b6);\
	i8x16 c5 = unpackhi32(b4, b6);\
	i8x16 c6 = unpacklo32(b5, b7);\
	i8x16 c7 = unpackhi32(b5, b7);\
	dst0 = unpacklo64(c0, c4);\
	dst1 = unpackhi64(c0, c4);\
	dst2 = unpacklo64(c1, c5);\
	dst3 = unpackhi64(c1, c5);\
	dst4 = unpacklo64(c2, c6);\
	dst5 = unpackhi64(c2, c6);\
	dst6 = unpacklo64(c3, c7);\
	dst7 = unpackhi64(c3, c7);}



/**
 * Helper functions
 */
static always_inline i8x16 expand4(int32_t *a) {
	i32x4 x0 = {*a};
	i8x16 x1 = unpacklo8(x0, x0);
	return unpacklo8(x1, x1);
}
static always_inline i8x16 expand2(int64_t *a) {
	i64x2 x0 = {*a};
	return unpacklo8(x0, x0);
}



/**
 * Deblock the luma plane of the current macroblock in place.
 */
static noinline void FUNC(deblock_Y_8bit, size_t stride, ssize_t nstride, size_t stride7)
{
	i8x16 v0, v1, v2, v3, v4, v5, v6, v7;
	if (mb->filter_edges & 2) {
		// load and transpose the left 12x16 matrix
		uint8_t * restrict px0 = ctx->samples_mb[0];
		i8x16 xa0 = load128(px0               - 8);
		i8x16 xa1 = load128(px0 +  stride     - 8);
		i8x16 xa2 = load128(px0 +  stride * 2 - 8);
		uint8_t * restrict px7 = px0 + stride7;
		i8x16 xa3 = load128(px7 + nstride * 4 - 8);
		i8x16 xa4 = load128(px0 +  stride * 4 - 8);
		i8x16 xa5 = load128(px7 + nstride * 2 - 8);
		i8x16 xa6 = load128(px7 + nstride     - 8);
		i8x16 xa7 = load128(px7               - 8);
		i8x16 xa8 = load128(px7 +  stride     - 8);
		i8x16 xa9 = load128(px7 +  stride * 2 - 8);
		uint8_t * restrict pxE = px7 + stride7;
		i8x16 xaA = load128(pxE + nstride * 4 - 8);
		i8x16 xaB = load128(px7 +  stride * 4 - 8);
		i8x16 xaC = load128(pxE + nstride * 2 - 8);
		i8x16 xaD = load128(pxE + nstride     - 8);
		i8x16 xaE = load128(pxE               - 8);
		i8x16 xaF = load128(pxE +  stride     - 8);
		i8x16 xb0 = unpackhi16(unpacklo8(xa0, xa1), unpacklo8(xa2, xa3));
		i8x16 xb1 = unpackhi16(unpacklo8(xa4, xa5), unpacklo8(xa6, xa7));
		i8x16 xb2 = unpackhi16(unpacklo8(xa8, xa9), unpacklo8(xaA, xaB));
		i8x16 xb3 = unpackhi16(unpacklo8(xaC, xaD), unpacklo8(xaE, xaF));
		i8x16 xb4 = unpacklo32(xb0, xb1);
		i8x16 xb5 = unpackhi32(xb0, xb1);
		i8x16 xb6 = unpacklo32(xb2, xb3);
		i8x16 xb7 = unpackhi32(xb2, xb3);
		i8x16 vW = unpacklo64(xb4, xb6);
		i8x16 vX = unpackhi64(xb4, xb6);
		i8x16 vY = unpacklo64(xb5, xb7);
		i8x16 vZ = unpackhi64(xb5, xb7);
		TRANSPOSE_8x16(xa, hi, v0, v1, v2, v3, v4, v5, v6, v7);
		
		// first vertical edge
		i8x16 alpha_a = set8(ctx->alpha[8]);
		i8x16 beta_a = set8(ctx->beta[8]);
		if (mb[-1].f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			i8x16 tC0a = expand4(ctx->tC0_s + 0);
			if (ctx->tC0_s[0] != -1)
				DEBLOCK_LUMA_SOFT(vX, vY, vZ, v0, v1, v2, alpha_a, beta_a, tC0a);
		} else if (ctx->alpha[8] != 0) {
			DEBLOCK_LUMA_HARD(vW, vX, vY, vZ, v0, v1, v2, v3, alpha_a, beta_a);
		}
		
		// store vW/vX/vY/vZ into the left macroblock
		i8x16 xc0 = unpacklo8(vW, vX);
		i8x16 xc1 = unpackhi8(vW, vX);
		i8x16 xc2 = unpacklo8(vY, vZ);
		i8x16 xc3 = unpackhi8(vY, vZ);
		i32x4 xc4 = unpacklo16(xc0, xc2);
		i32x4 xc5 = unpackhi16(xc0, xc2);
		i32x4 xc6 = unpacklo16(xc1, xc3);
		i32x4 xc7 = unpackhi16(xc1, xc3);
		px0 = ctx->samples_mb[0];
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
		i8x16 xa8 = *(i8x16 *)(px7 +  stride    );
		i8x16 xa9 = *(i8x16 *)(px7 +  stride * 2);
		uint8_t * restrict pxE = px7 + stride7;
		i8x16 xaA = *(i8x16 *)(pxE + nstride * 4);
		i8x16 xaB = *(i8x16 *)(px7 +  stride * 4);
		i8x16 xaC = *(i8x16 *)(pxE + nstride * 2);
		i8x16 xaD = *(i8x16 *)(pxE + nstride    );
		i8x16 xaE = *(i8x16 *)(pxE              );
		i8x16 xaF = *(i8x16 *)(pxE +  stride    );
		TRANSPOSE_8x16(xa, lo, v0, v1, v2, v3, v4, v5, v6, v7);
	}
	
	// second vertical edge
	i8x16 alpha_bcdfgh = set8(ctx->alpha[0]);
	i8x16 beta_bcdfgh = set8(ctx->beta[0]);
	if (!mb->f.transform_size_8x8_flag) {
		i8x16 tC0b = expand4(ctx->tC0_s + 1);
		if (ctx->tC0_s[1] != -1)
			DEBLOCK_LUMA_SOFT(v1, v2, v3, v4, v5, v6, alpha_bcdfgh, beta_bcdfgh, tC0b);
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
	i8x16 xa8 = *(i8x16 *)(px7 +  stride    );
	i8x16 xa9 = *(i8x16 *)(px7 +  stride * 2);
	uint8_t * restrict pxE = px7 + stride7;
	i8x16 xaA = *(i8x16 *)(pxE + nstride * 4);
	i8x16 xaB = *(i8x16 *)(px7 +  stride * 4);
	i8x16 xaC = *(i8x16 *)(pxE + nstride * 2);
	i8x16 xaD = *(i8x16 *)(pxE + nstride    );
	i8x16 xaE = *(i8x16 *)(pxE              );
	i8x16 xaF = *(i8x16 *)(pxE +  stride    );
	i8x16 v8, v9, vA, vB, vC, vD, vE, vF;
	TRANSPOSE_8x16(xa, hi, v8, v9, vA, vB, vC, vD, vE, vF);
	
	// third vertical edge
	i8x16 tC0c = expand4(ctx->tC0_s + 2);
	if (ctx->tC0_s[2] != -1)
		DEBLOCK_LUMA_SOFT(v5, v6, v7, v8, v9, vA, alpha_bcdfgh, beta_bcdfgh, tC0c);
	
	// fourth vertical edge
	if (!mb->f.transform_size_8x8_flag) {
		i8x16 tC0d = expand4(ctx->tC0_s + 3);
		if (ctx->tC0_s[3] != -1)
			DEBLOCK_LUMA_SOFT(v9, vA, vB, vC, vD, vE, alpha_bcdfgh, beta_bcdfgh, tC0d);
	}
	
	// transpose the top 16x8 matrix
	i8x16 h0, h1, h2, h3, h4, h5, h6, h7;
	TRANSPOSE_8x16(v, lo, h0, h1, h2, h3, h4, h5, h6, h7);
	
	// first horizontal edge
	px0 = ctx->samples_mb[0];
	if (mb->filter_edges & 4) {
		i8x16 alpha_e = set8(ctx->alpha[12]);
		i8x16 beta_e = set8(ctx->beta[12]);
		if (mbB->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			i8x16 tC0e = expand4(ctx->tC0_s + 4);
			if (ctx->tC0_s[4] != -1)
				DEBLOCK_LUMA_SOFT(*(i8x16 *)(px0 + nstride * 3), *(i8x16 *)(px0 + nstride * 2), *(i8x16 *)(px0 + nstride    ), h0, h1, h2, alpha_e, beta_e, tC0e);
		} else if (ctx->alpha[12] != 0) {
			DEBLOCK_LUMA_HARD(*(i8x16 *)(px0 + nstride * 4), *(i8x16 *)(px0 + nstride * 3), *(i8x16 *)(px0 + nstride * 2), *(i8x16 *)(px0 + nstride    ), h0, h1, h2, h3, alpha_e, beta_e);
		}
	}
	*(i8x16 *)(px0              ) = h0;
	*(i8x16 *)(px0 +  stride    ) = h1;
	
	// second horizontal edge
	if (!mb->f.transform_size_8x8_flag) {
		i8x16 tC0f = expand4(ctx->tC0_s + 5);
		if (ctx->tC0_s[5] != -1)
			DEBLOCK_LUMA_SOFT(h1, h2, h3, h4, h5, h6, alpha_bcdfgh, beta_bcdfgh, tC0f);
	}
	px7 = px0 + stride7;
	*(i8x16 *)(px0 +  stride * 2) = h2;
	*(i8x16 *)(px7 + nstride * 4) = h3;
	*(i8x16 *)(px0 +  stride * 4) = h4;
	*(i8x16 *)(px7 + nstride * 2) = h5;
	
	// transpose the bottom 16x8 matrix
	i8x16 h8, h9, hA, hB, hC, hD, hE, hF;
	TRANSPOSE_8x16(v, hi, h8, h9, hA, hB, hC, hD, hE, hF);
	pxE = px7 + stride7;
	*(i8x16 *)(pxE              ) = hE;
	*(i8x16 *)(pxE +  stride    ) = hF;
	
	// third horizontal edge
	i8x16 tC0g = expand4(ctx->tC0_s + 6);
	if (ctx->tC0_s[6] != -1)
		DEBLOCK_LUMA_SOFT(h5, h6, h7, h8, h9, hA, alpha_bcdfgh, beta_bcdfgh, tC0g);
	*(i8x16 *)(px7 + nstride    ) = h6;
	*(i8x16 *)(px7              ) = h7;
	*(i8x16 *)(px7 +  stride    ) = h8;
	*(i8x16 *)(px7 +  stride * 2) = h9;
	
	// fourth horizontal edge
	if (!mb->f.transform_size_8x8_flag) {
		i8x16 tC0h = expand4(ctx->tC0_s + 7);
		if (ctx->tC0_s[7] != -1)
			DEBLOCK_LUMA_SOFT(h9, hA, hB, hC, hD, hE, alpha_bcdfgh, beta_bcdfgh, tC0h);
	}
	*(i8x16 *)(pxE + nstride * 4) = hA;
	*(i8x16 *)(px7 +  stride * 4) = hB;
	*(i8x16 *)(pxE + nstride * 2) = hC;
	*(i8x16 *)(pxE + nstride    ) = hD;
}



/**
 * Deblock both chroma planes of the current macroblock in place.
 */
static noinline void FUNC(deblock_CbCr_8bit, size_t stride, ssize_t nstride, size_t stride7)
{
	i8x16 v0, v1, v2, v3, v4, v5, v6, v7;
	if (mb->filter_edges & 2) {
		// load and transpose both 12x8 matrices (with left macroblock)
		uint8_t * restrict Cb0 = ctx->samples_mb[1];
		i8x16 xa0 = load128(Cb0               - 8);
		i8x16 xa1 = load128(Cb0 +  stride     - 8);
		i8x16 xa2 = load128(Cb0 +  stride * 2 - 8);
		uint8_t * restrict Cb7 = Cb0 + stride7;
		i8x16 xa3 = load128(Cb7 + nstride * 4 - 8);
		i8x16 xa4 = load128(Cb0 +  stride * 4 - 8);
		i8x16 xa5 = load128(Cb7 + nstride * 2 - 8);
		i8x16 xa6 = load128(Cb7 + nstride     - 8);
		i8x16 xa7 = load128(Cb7               - 8);
		uint8_t * restrict Cr0 = ctx->samples_mb[2];
		i8x16 xa8 = load128(Cr0               - 8);
		i8x16 xa9 = load128(Cr0 +  stride     - 8);
		i8x16 xaA = load128(Cr0 +  stride * 2 - 8);
		uint8_t * restrict Cr7 = Cr0 + stride7;
		i8x16 xaB = load128(Cr7 + nstride * 4 - 8);
		i8x16 xaC = load128(Cr0 +  stride * 4 - 8);
		i8x16 xaD = load128(Cr7 + nstride * 2 - 8);
		i8x16 xaE = load128(Cr7 + nstride     - 8);
		i8x16 xaF = load128(Cr7               - 8);
		i8x16 xb0 = unpackhi16(unpacklo8(xa0, xa1), unpacklo8(xa2, xa3));
		i8x16 xb1 = unpackhi16(unpacklo8(xa4, xa5), unpacklo8(xa6, xa7));
		i8x16 xb2 = unpackhi16(unpacklo8(xa8, xa9), unpacklo8(xaA, xaB));
		i8x16 xb3 = unpackhi16(unpacklo8(xaC, xaD), unpacklo8(xaE, xaF));
		i8x16 xb4 = unpackhi32(xb0, xb1);
		i8x16 xb5 = unpackhi32(xb2, xb3);
		i8x16 vY = unpacklo64(xb4, xb5);
		i8x16 vZ = unpackhi64(xb4, xb5);
		TRANSPOSE_8x16(xa, hi, v0, v1, v2, v3, v4, v5, v6, v7);
		
		// first vertical edge
		i8x16 shuf_a = {9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10};
		i8x16 alpha_a = shuffle8(ctx->alpha_v, shuf_a);
		i8x16 beta_a = shuffle8(ctx->beta_v, shuf_a);
		if (mb[-1].f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			i8x16 tC0a = expand2(ctx->tC0_l + 4);
			if (ctx->tC0_l[4] != -1)
				DEBLOCK_CHROMA_SOFT(vY, vZ, v0, v1, alpha_a, beta_a, tC0a);
		} else {
			DEBLOCK_CHROMA_HARD(vY, vZ, v0, v1, alpha_a, beta_a);
		}
		
		// store vY/vZ into the left macroblock
		i16x8 xc0 = unpacklo8(vY, vZ);
		i16x8 xc1 = unpackhi8(vY, vZ);
		Cb0 = ctx->samples_mb[1];
		*(int16_t *)(Cb0               - 2) = xc0[0];
		*(int16_t *)(Cb0 +  stride     - 2) = xc0[1];
		*(int16_t *)(Cb0 +  stride * 2 - 2) = xc0[2];
		Cb7 = Cb0 + stride7;
		*(int16_t *)(Cb7 + nstride * 4 - 2) = xc0[3];
		*(int16_t *)(Cb0 +  stride * 4 - 2) = xc0[4];
		*(int16_t *)(Cb7 + nstride * 2 - 2) = xc0[5];
		*(int16_t *)(Cb7 + nstride     - 2) = xc0[6];
		*(int16_t *)(Cb7               - 2) = xc0[7];
		Cr0 = ctx->samples_mb[2];
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
		TRANSPOSE_8x16(xa, lo, v0, v1, v2, v3, v4, v5, v6, v7);
	}
	
	// second vertical edge
	i8x16 shuf_cg = {1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2};
	i8x16 alpha_cg = shuffle8(ctx->alpha_v, shuf_cg);
	i8x16 beta_cg = shuffle8(ctx->beta_v, shuf_cg);
	i8x16 tC0c = expand2(ctx->tC0_l + 5);
	if (ctx->tC0_l[5] != -1)
		DEBLOCK_CHROMA_SOFT(v2, v3, v4, v5, alpha_cg, beta_cg, tC0c);
	
	// transpose both 8x8 matrices
	i8x16 xd0 = unpacklo8(v0, v1);
	i8x16 xd1 = unpackhi8(v0, v1);
	i8x16 xd2 = unpacklo8(v2, v3);
	i8x16 xd3 = unpackhi8(v2, v3);
	i8x16 xd4 = unpacklo8(v4, v5);
	i8x16 xd5 = unpackhi8(v4, v5);
	i8x16 xd6 = unpacklo8(v6, v7);
	i8x16 xd7 = unpackhi8(v6, v7);
	i8x16 xe0 = unpacklo16(xd0, xd2);
	i8x16 xe1 = unpackhi16(xd0, xd2);
	i8x16 xe2 = unpacklo16(xd1, xd3);
	i8x16 xe3 = unpackhi16(xd1, xd3);
	i8x16 xe4 = unpacklo16(xd4, xd6);
	i8x16 xe5 = unpackhi16(xd4, xd6);
	i8x16 xe6 = unpacklo16(xd5, xd7);
	i8x16 xe7 = unpackhi16(xd5, xd7);
	i8x16 xf0 = unpacklo32(xe0, xe4);
	i8x16 xf1 = unpackhi32(xe0, xe4);
	i8x16 xf2 = unpacklo32(xe1, xe5);
	i8x16 xf3 = unpackhi32(xe1, xe5);
	i8x16 xf4 = unpacklo32(xe2, xe6);
	i8x16 xf5 = unpackhi32(xe2, xe6);
	i8x16 xf6 = unpacklo32(xe3, xe7);
	i8x16 xf7 = unpackhi32(xe3, xe7);
	i8x16 h0 = unpacklo64(xf0, xf4);
	i8x16 h1 = unpackhi64(xf0, xf4);
	i8x16 h2 = unpacklo64(xf1, xf5);
	i8x16 h3 = unpackhi64(xf1, xf5);
	i8x16 h4 = unpacklo64(xf2, xf6);
	i8x16 h5 = unpackhi64(xf2, xf6);
	i8x16 h6 = unpacklo64(xf3, xf7);
	i8x16 h7 = unpackhi64(xf3, xf7);
	
	// first horizontal edge
	uint8_t * restrict Cb0 = ctx->samples_mb[1];
	uint8_t * restrict Cr0 = ctx->samples_mb[2];
	if (mb->filter_edges & 4) {
		i8x16 shuf_e = {13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14};
		i8x16 alpha_e = shuffle8(ctx->alpha_v, shuf_e);
		i8x16 beta_e = shuffle8(ctx->beta_v, shuf_e);
		i8x16 hY = (i64x2){*(int64_t *)(Cb0 + nstride * 2), *(int64_t *)(Cr0 + nstride * 2)};
		i8x16 hZ = (i64x2){*(int64_t *)(Cb0 + nstride    ), *(int64_t *)(Cr0 + nstride    )};
		if (mbB->f.mbIsInterFlag & mb->f.mbIsInterFlag) {
			i8x16 tC0e = expand2(ctx->tC0_l + 6);
			if (ctx->tC0_l[6] != -1)
				DEBLOCK_CHROMA_SOFT(hY, hZ, h0, h1, alpha_e, beta_e, tC0e);
		} else {
			DEBLOCK_CHROMA_HARD(hY, hZ, h0, h1, alpha_e, beta_e);
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
	i8x16 tC0g = expand2(ctx->tC0_l + 7);
	if (ctx->tC0_l[7] != -1)
		DEBLOCK_CHROMA_SOFT(h2, h3, h4, h5, alpha_cg, beta_cg, tC0g);
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
 * Loop through an entire frame to apply the deblocking filter on all
 * macroblocks.
 */
void FUNC(deblock_frame, uint8_t *samples)
{
	// point at the first macroblock
	ctx->samples_mb[0] = ctx->samples_row[0] = ctx->samples_pic = samples;
	ctx->samples_mb[1] = ctx->samples_mb[0] + ctx->plane_size_Y;
	ctx->samples_mb[2] = ctx->samples_mb[1] + ctx->plane_size_C;
	mbB = (Edge264_macroblock *)(ctx->samples_mb[2] + ctx->plane_size_C) + 1;
	mb = mbB + ctx->sps.pic_width_in_mbs + 1;
	ctx->CurrMbAddr = 0;
	
	do {
		// deblock a single macroblock
		if (mb->filter_edges & 1) {
			CALL(init_alpha_beta_tC0);
			size_t strideY = ctx->s.stride_Y;
			CALL(deblock_Y_8bit, strideY, -strideY, strideY * 7);
			size_t strideC = ctx->s.stride_C;
			CALL(deblock_CbCr_8bit, strideC, -strideC, strideC * 7);
		}
		
		// point at the next macroblock
		mb++;
		mbB++;
		ctx->samples_mb[0] += 16;
		ctx->samples_mb[1] += 8;
		ctx->samples_mb[2] += 8;
		ctx->CurrMbAddr++;
		
		// end of row
		if (ctx->samples_mb[0] - ctx->samples_row[0] < ctx->s.stride_Y)
			continue;
		mb++;
		mbB++;
		ctx->samples_mb[0] = ctx->samples_row[0] += ctx->s.stride_Y * 16;
		ctx->samples_mb[1] += ctx->s.stride_C * 7;
		ctx->samples_mb[2] += ctx->s.stride_C * 7;
	} while (ctx->samples_mb[0] - ctx->samples_pic < ctx->plane_size_Y);
}
