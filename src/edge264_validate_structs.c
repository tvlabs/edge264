#include "edge264_internal.h"

// This file is compiled twice: once for assume functions and once for assert functions
#undef ASNAME
#undef ASCHECK
#undef ASALL
#ifndef ASSERT
	#define ASSERT 1
#endif
#if !ASSERT
	#define ASNAME(f) assume_ ## f
	#define ASCHECK(cond) __CPROVER_assume(cond)
#else
	#define ASNAME(f) assert_ ## f
	#define ASCHECK(cond) assert(cond)
#endif
#define ASALL(v) ASCHECK(((i64x2)(v))[0] == -1 && ((i64x2)(v))[1] == -1)


// Edge264GetBits CAVLC/CABAC configuration depends on who is using it.
static void ASNAME(Edge264GetBits_CPB)(const Edge264GetBits *gb) {
	ASCHECK(gb->end != NULL);
	ASCHECK(gb->cpb_off >= 0 || (ptrdiff_t)__CPROVER_POINTER_OFFSET(gb->end) + gb->cpb_off >= 0);
}
static void ASNAME(Edge264GetBits_cavlc)(const Edge264GetBits *gb) {
	ASNAME(Edge264GetBits_CPB)(gb);
	ASCHECK(gb->lsb_cache != 0);
}
static void ASNAME(Edge264GetBits_cabac)(const Edge264GetBits *gb) {
	ASNAME(Edge264GetBits_CPB)(gb);
	ASCHECK(gb->range >= 256);
	ASCHECK(gb->offset < gb->range);
}


static void ASNAME(Edge264Macroblock)(const Edge264Macroblock *m) {
	ASCHECK(m->error_probability <= 100);
	ASCHECK(m->recovery_bits <= 3);
	ASCHECK(m->mbIsInterFlag <= 1);
	ASCHECK(m->filter_edges <= 7);
	ASCHECK(m->QP[0] >= -36 && m->QP[0] <= 51 && m->QP[1] >= -36 && m->QP[1] <= 39 && m->QP[2] >= -36 && m->QP[2] <= 39); // 8.5.8
	ASCHECK((m->bits_l & ~0x00ffffff00ffffffull) == 0);
	ASALL((m->Intra4x4PredMode_v >= set8(-2)) & (m->Intra4x4PredMode_v <= set8(8))); // abs(min(A,B)) indexes Intra4x4Modes/Intra8x8Modes[9], with -2 forcing DC
	ASALL(maxu8(maxu8(m->nC_v[0], m->nC_v[1]), m->nC_v[2]) <= (u8x16)set8(16));
	ASALL(maxu8(maxu8(maxu8(m->absMvd_v[0], m->absMvd_v[1]), m->absMvd_v[2]), m->absMvd_v[3]) >= (u8x16)set8(128));
	ASCHECK(m->f.mb_field_decoding_flag <= 1);
	ASCHECK(m->f.mb_skip_flag <= 1);
	ASCHECK(m->f.mb_type_I_NxN <= 1);
	ASCHECK(m->f.mb_type_B_Direct <= 1);
	ASCHECK(m->f.transform_size_8x8_flag <= 1);
	ASCHECK(m->f.intra_chroma_pred_mode_non_zero <= 1);
	ASCHECK(m->f.CodedBlockPatternChromaDC <= 1);
	ASCHECK(m->f.CodedBlockPatternChromaAC <= 1);
	ASCHECK((m->f.coded_block_flags_16x16_s & ~0x010101) == 0);
	ASALL((i8x16)(i64x2){m->refIdx_l} < set8(32)); // negative if unused, otherwise indexes implicit_weights[32][32] and explicit_weights[3][64] (+32 for L1)
	ASALL((i8x16)(i64x2){m->refPic_l} < set8(64)); // negative if unused, otherwise indexes samples_buffers[64] and shuffle4z(MapPicToList0_v)
}

#undef ASSERT
