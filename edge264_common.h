/**
 * Every file should be compilable on its own by including this one.
 */

#ifndef EDGE264_COMMON_H
#define EDGE264_COMMON_H

#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "edge264.h"



typedef int8_t v2qi __attribute__((vector_size(2)));
typedef int8_t v4qi __attribute__((vector_size(4)));
typedef int8_t v8qi __attribute__((vector_size(8)));
typedef int16_t v4hi __attribute__((vector_size(8)));
typedef uint8_t v8qu __attribute__((vector_size(8)));
typedef size_t v16u __attribute__((vector_size(16)));
typedef int8_t v16qi __attribute__((vector_size(16)));
typedef int16_t v8hi __attribute__((vector_size(16)));
typedef int32_t v4si __attribute__((vector_size(16)));
typedef int64_t v2li __attribute__((vector_size(16)));
typedef uint8_t v16qu __attribute__((vector_size(16)));
typedef uint16_t v8hu __attribute__((vector_size(16)));
typedef uint32_t v4su __attribute__((vector_size(16)));
typedef uint64_t v2lu __attribute__((vector_size(16)));
typedef int32_t v8si __attribute__((vector_size(32))); // for alignment of ctx->c
typedef int16_t v16hi __attribute__((vector_size(32))); // for initialization of neighbouring offsets
typedef int32_t v16si __attribute__((vector_size(64))); // for initialization of neighbouring offsets



/**
 * In 9.3.3.1.1, ctxIdxInc is always the result of flagA+flagB or flagA+2*flagB,
 * so we pack macroblock flags together to allow adding them in parallel with
 * flagsA + flagsB + (flagsB & twice).
 */
typedef union {
	struct {
		int8_t unavailable;
		int8_t mb_field_decoding_flag;
		int8_t mb_skip_flag;
		int8_t mb_type_I_NxN;
		int8_t mb_type_B_Direct;
		int8_t transform_size_8x8_flag;
		int8_t intra_chroma_pred_mode_non_zero;
		int8_t CodedBlockPatternChromaDC;
		int8_t CodedBlockPatternChromaAC;
		int8_t mbIsInterFlag;
		union { int8_t coded_block_flags_16x16[3]; int32_t coded_block_flags_16x16_s; };
	};
	v16qi v;
} Edge264_flags;
static const Edge264_flags flags_twice = {
	.unavailable = 1,
	.CodedBlockPatternChromaDC = 1,
	.CodedBlockPatternChromaAC = 1,
	.coded_block_flags_16x16 = {1, 1, 1},
};



/**
 * Although we waste some space by storing some neighbouring values for more
 * than their lifespans, packing everything in a single structure is arguably
 * the simplest to maintain. Arrays of precomputed neighbouring offsets spare
 * the use of local caches, thus minimising memory writes.
 * 
 * For bit values in 8x8 and 4x4 blocks, we use compact bit patterns in bits_v,
 * allowing quick retrieval of neighbouring values for CABAC:
 *             1  8 15 22
 *   1 6    0| 7 14 21  5
 * 0|5 3    6|13 20  4 11
 * 4|2 7   12|19  3 10 17
 *         18| 2  9 16 23
 */
typedef struct {
	Edge264_flags f;
	uint32_t inter_blocks; // bitmask for every index that is the topleft corner of a block, upper half indicates whether each 8x8 block is equal with its right/bottom neighbours
	union { int8_t refIdx[8]; int32_t refIdx_s[2]; int64_t refIdx_l; v8qi refIdx_v; }; // [LX][i8x8]
	union { uint32_t bits[4]; v4su bits_v; }; // {cbf_Y 8x8/4x4 , cbf_Cb 8x8/4x4, cbf_Cr 8x8/4x4, cbp/ref_idx_nz}
	union { int8_t nC[3][16]; v16qi nC_v[3]; }; // for CAVLC
	union { int8_t Intra4x4PredMode[16]; v16qi Intra4x4PredMode_v; }; // [i4x4]
	union { uint8_t absMvdComp[64]; uint64_t absMvdComp_l[8]; v16qu absMvdComp_v[4]; }; // [LX][i4x4][compIdx]
	union { int16_t mvs[64]; int32_t mvs_s[32]; v8hi mvs_v[8]; }; // [LX][i4x4][compIdx]
} Edge264_macroblock;



/**
 * This structure stores the entire decoder state during its operation, such
 * that we can dedicate a single pointer for it.
 * It is separate from Edge264_stream, to distinguish state that lives during
 * a single frame, from state that spans multiple frames.
 */
typedef struct
{
	// small variables and constant parameters
	Edge264_flags inc; // increments for CABAC indices of macroblock syntax elements
	uint8_t nal_ref_flag:1;
	uint8_t IdrPicFlag:1;
	uint8_t field_pic_flag:1;
	uint8_t bottom_field_flag:1;
	uint8_t MbaffFrameFlag:1;
	uint8_t direct_spatial_mv_pred_flag:1;
	int8_t slice_type; // 3 significant bits
	int8_t luma_log2_weight_denom; // 3 significant bits
	int8_t chroma_log2_weight_denom; // 3 significant bits
	int8_t disable_deblocking_filter_idc; // 2 significant bits
	int8_t FilterOffsetA; // 5 significant bits
	int8_t FilterOffsetB;
	int8_t mb_qp_delta_nz;
	int32_t TopFieldOrderCnt;
	int32_t BottomFieldOrderCnt;
	Edge264_parameter_set ps;
	
	// parsing context
	const uint8_t *CPB; // memory address of the next byte to load in lsb_cache
	const uint8_t *end;
	size_t _lsb_cache; // backup storage when not in a Global Register
	size_t _msb_cache;
	size_t _codIRange; // same as _lsb_cache/_msb_cache
	size_t _codIOffset;
	Edge264_macroblock * restrict _mb; // backup storage for macro mb
	Edge264_macroblock * restrict mbB;
	const Edge264_stream *e; // for debugging only
	int32_t CurrMbAddr;
	int32_t mb_skip_run;
	int32_t plane_size_Y;
	int32_t plane_size_C;
	uint8_t *samples_pic; // address of top-left byte of current picture
	uint8_t *samples_row[3]; // address of top-left byte of each plane in current row of macroblocks
	uint8_t *samples_mb[3]; // address of top-left byte of each plane in current macroblock
	uint16_t stride[3]; // [iYCbCr], 16 significant bits (8K, 16bit, field pic)
	int16_t clip[3]; // [iYCbCr], maximum sample value
	union { int8_t unavail[16]; v16qi unavail_v; }; // unavailability of neighbouring A/B/C/D blocks
	uint8_t map_me[48];
	union { uint8_t cabac[1024]; v16qu cabac_v[64]; };
	
	// neighbouring offsets (relative to the start of each array in mb)
	union { int16_t Intra4x4PredMode_A[16]; v16hi Intra4x4PredMode_A_v; };
	union { int32_t Intra4x4PredMode_B[16]; v16si Intra4x4PredMode_B_v; };
	union { int16_t absMvdComp_A[16]; v16hi absMvdComp_A_v; };
	union { int32_t absMvdComp_B[16]; v16si absMvdComp_B_v; };
	union { int8_t refIdx4x4_C[16]; int32_t refIdx4x4_C_s[4]; v16qi refIdx4x4_C_v; }; // shuffle vector for mv prediction
	union { int16_t mvs_A[16]; v16hi mvs_A_v; };
	union { int32_t mvs_B[16]; v16si mvs_B_v; };
	union { int32_t mvs_C[16]; v16si mvs_C_v; };
	union { int32_t mvs_D[16]; v16si mvs_D_v; };
	
	// Intra context
	union { uint8_t intra4x4_modes[9][16]; v16qu intra4x4_modes_v[9]; }; // kept for future 16bit support
	union { uint8_t intra8x8_modes[9][16]; v16qu intra8x8_modes_v[9]; };
	
	// Inter context
	const Edge264_macroblock *mbCol;
	uint8_t num_ref_idx_mask;
	int8_t transform_8x8_mode_flag; // updated during parsing to replace noSubMbPartSizeLessThan8x8Flag
	int8_t col_short_term;
	int8_t MapColToList0[65]; // [refIdxCol + 1]
	union { int8_t clip_ref_idx[8]; v8qi clip_ref_idx_v; };
	union { int8_t RefPicList[2][32]; v16qi RefPicList_v[4]; }; // FIXME store on stack instead?
	const uint8_t *ref_planes[64]; // [lx][refIdx], FIXME store relative to frame
	union { int8_t refIdx4x4_eq[32]; v16qi refIdx4x4_eq_v[2]; }; // FIXME store on stack
	int16_t DistScaleFactor[32]; // [refIdxL0]
	union { int8_t implicit_weights[2][32][32]; v16qi implicit_weights_v[2][32][2]; }; // w1 for [top/bottom][ref0][ref1]
	int8_t explicit_weights[3][64]; // [iYCbCr][LX][RefIdx]
	int8_t explicit_offsets[3][64];
	union { uint8_t edge_buf[2016]; int64_t edge_buf_l[252]; v16qu edge_buf_v[126]; };
	
	// Residuals context
	union { int16_t ctxIdxOffsets[4]; v4hi ctxIdxOffsets_l; }; // {cbf,sig_flag,last_sig_flag,coeff_abs}
	union { int8_t sig_inc[64]; v8qi sig_inc_l; v16qi sig_inc_v[4]; };
	union { int8_t last_inc[64]; v8qi last_inc_l; v16qi last_inc_v[4]; };
	union { int8_t scan[64]; v8qi scan_l; v16qi scan_v[4]; };
	union { int8_t QPprime_C[2][64]; v16qi QPprime_C_v[8]; };
	union { int32_t c[64]; v4si c_v[16]; v8si c_V[8]; }; // non-scaled residual coefficients
} Edge264_ctx;



#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	#define little_endian32(x) (x)
	#define little_endian64(x) (x)
	#define big_endian32 __builtin_bswap32
	#define big_endian64 __builtin_bswap64
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	#define little_endian32 __builtin_bswap32
	#define little_endian64 __builtin_bswap64
	#define big_endian32(x) (x)
	#define big_endian64(x) (x)
#endif

#if SIZE_MAX == 4294967295U
	#define SIZE_BIT 32
	#define big_endian big_endian32
	#define clz clz32
	#define ctz ctz32
#elif SIZE_MAX == 18446744073709551615U
	#define SIZE_BIT 64
	#define big_endian big_endian64
	#define clz clz64
	#define ctz ctz64
#endif

#if UINT_MAX == 4294967295U && ULLONG_MAX == 18446744073709551615U
	#define clz32 __builtin_clz
	#define ctz32 __builtin_ctz
	#define clz64 __builtin_clzll
	#define ctz64 __builtin_ctzll
	#ifndef WORD_BIT
		#define WORD_BIT 32
	#endif
#endif



/**
 * Macro-ed function defs/calls allow removing ctx from args and keeping it in
 * a Global Register Variable if permitted by the compiler. On my machine the
 * speed gain is negligible, but the binary is noticeably smaller.
 * However storing bitstream caches in registers gives a big performance gain.
 */
#if defined(__SSSE3__) && !defined(__clang__)
	register Edge264_ctx * restrict ctx asm("ebx");
	#define SET_CTX(p) Edge264_ctx *old = ctx; ctx = p
	#define RESET_CTX() ctx = old
	#define FUNC(f, ...) f(__VA_ARGS__)
	#define CALL(f, ...) f(__VA_ARGS__)
	#define JUMP(f, ...) {f(__VA_ARGS__); return;}
#else
	#define SET_CTX(p) Edge264_ctx * restrict ctx = p
	#define RESET_CTX()
	#define FUNC(f, ...) f(Edge264_ctx * restrict ctx, ## __VA_ARGS__)
	#define CALL(f, ...) f(ctx, ## __VA_ARGS__)
	#define JUMP(f, ...) {f(ctx, ## __VA_ARGS__); return;}
#endif
#define mb ctx->_mb
#if defined(__SSSE3__) && !defined(__clang__) && SIZE_BIT == 64
	register size_t rbsp_reg0 asm("r14");
	register size_t rbsp_reg1 asm("r15");
	#define codIRange rbsp_reg0
	#define codIOffset rbsp_reg1
	#define lsb_cache rbsp_reg0
	#define msb_cache rbsp_reg1
#else
	#define codIRange ctx->_codIRange
	#define codIOffset ctx->_codIOffset
	#define lsb_cache ctx->_lsb_cache
	#define msb_cache ctx->_msb_cache
#endif



/**
 * Function declarations are put in a single block here instead of .h files
 * because they are so few. Functions used across files are not made static to
 * allow compiling each file individually.
 */
#ifndef noinline
	#define noinline __attribute__((noinline))
#endif
#ifndef always_inline
	#define always_inline inline __attribute__((always_inline))
#endif

#ifdef TRACE
	#include <stdio.h>
	static inline const char *red_if(int cond) { return (cond) ? " style='color:red'" : ""; }
#else
	#define printf(...) ((void)0)
	#define check_stream(e) ((void)0)
	#define check_ctx(...) ((void)0)
#endif
#if TRACE < 2
	#define fprintf(...) ((void)0)
#endif

static inline int min(int a, int b) { return (a < b) ? a : b; }
static inline int max(int a, int b) { return (a > b) ? a : b; }
static inline int clip3(int a, int b, int c) { return min(max(c, a), b); }
static inline unsigned umin(unsigned a, unsigned b) { return (a < b) ? a : b; }
static inline unsigned umax(unsigned a, unsigned b) { return (a > b) ? a : b; }
static inline int median(int a, int b, int c) { return max(min(max(a, b), c), min(a, b)); }

// edge264_bitstream.c
static noinline int FUNC(refill, int ret);
static noinline int FUNC(get_u1);
static noinline unsigned FUNC(get_uv, unsigned v);
static noinline unsigned FUNC(get_ue16, unsigned upper);
static noinline int FUNC(get_se16, int lower, int upper);
#if SIZE_BIT == 32
	static noinline unsigned FUNC(get_ue32, unsigned upper);
	static noinline int FUNC(get_se32, int lower, int upper);
#else
	#define get_ue32 get_ue16
	#define get_se32 get_se16
#endif
static noinline int FUNC(get_ae, int ctxIdx);
static always_inline int FUNC(get_bypass);
static void FUNC(cabac_start);
static int FUNC(cabac_terminate);
static void FUNC(cabac_init, int idc);

// edge264_inter_*.c
static noinline void FUNC(decode_inter, int i, int w, int h);

// edge264_intra_*.c
static always_inline void FUNC(decode_intra4x4, int mode, uint8_t *samples, size_t stride, int clip);
static always_inline void FUNC(decode_intra16x16, int mode, uint8_t *samples, size_t stride, int clip);
static always_inline void FUNC(decode_intraChroma, int mode, uint8_t *samplesCb, uint8_t *samplesCr, size_t stride, int clip);

// edge264_mvpred.c
static inline void FUNC(decode_inter_16x16, v8hi mvd, int lx);
static inline void FUNC(decode_inter_8x16_left, v8hi mvd, int lx);
static inline void FUNC(decode_inter_8x16_right, v8hi mvd, int lx);
static inline void FUNC(decode_inter_16x8_top, v8hi mvd, int lx);
static inline void FUNC(decode_inter_16x8_bottom, v8hi mvd, int lx);
static noinline void FUNC(decode_direct_mv_pred);

// edge264_residual_*.c
static inline void FUNC(add_idct4x4, int iYCbCr, int qP, v16qu wS, int DCidx, uint8_t *samples);
static inline void FUNC(transform_dc4x4, int iYCbCr);
static inline void FUNC(transform_dc2x2);
static inline void FUNC(transform_dc2x4);

// edge264_slice.c
static noinline void FUNC(parse_slice_data_cavlc);
static noinline void FUNC(parse_slice_data_cabac);

// debugging functions
#define print_v16qi(a) {\
	v16qi v = (v16qi)a;\
	printf("<li><code>");\
	for (int i = 0; i < 16; i++)\
		printf("%4d ", v[i]);\
	printf("</code></li>\n");}
#define print_v16qu(a) {\
	v16qu v = (v16qu)a;\
	printf("<li><code>");\
	for (int i = 0; i < 16; i++)\
		printf("%3u ", v[i]);\
	printf("</code></li>\n");}
#define print_v8hi(a) {\
	v8hi v = (v8hi)a;\
	printf("<li><code>");\
	for (int i = 0; i < 8; i++)\
		printf("%6d ", v[i]);\
	printf("</code></li>\n");}
#define print_v4si(a) {\
	v4si v = (v4si)a;\
	printf("<li><code>");\
	for (int i = 0; i < 4; i++)\
		printf("%6d ", v[i]);\
	printf("</code></li>\n");}

// Intel-specific common function declarations
#ifdef __SSSE3__
	#include <x86intrin.h>
	
	// missing intrinsics (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=95483)
	#if defined(__clang__)
		static always_inline __m128i _mm_loadu_si32(const uint8_t *a) {
			return _mm_cvtsi32_si128(*(int32_t *)(a));
		}
	#endif

	// instructions available on later architectures
	#ifndef __AVX_2__
		#define _mm_broadcastw_epi16(a) _mm_shuffle_epi32(_mm_shufflelo_epi16(a, _MM_SHUFFLE(0, 0, 0, 0)), _MM_SHUFFLE(1, 0, 1, 0))
	#endif
	#if defined(__SSE4_1__) && !defined(__clang__)
		static inline __m128i _mm_mullo_epi32(__m128i a, __m128i b) { // FIXME correct
			__m128i c = _mm_shuffle_epi32(a, _MM_SHUFFLE(0, 3, 0, 1));
			__m128i d = _mm_shuffle_epi32(b, _MM_SHUFFLE(0, 3, 0, 1));
			__m128i e = _mm_mul_epu32(a, b);
			__m128i f = _mm_mul_epu32(c, d);
			__m128 g = _mm_shuffle_ps((__m128)e, (__m128)f, _MM_SHUFFLE(2, 0, 2, 0));
			return _mm_shuffle_epi32((__m128i)g, _MM_SHUFFLE(3, 1, 2, 0));
		}
		static inline __m128i _mm_packus_epi32(__m128i a, __m128i b) {
			return _mm_max_epi16(_mm_packs_epi32(a, b), _mm_setzero_si128()); // not strictly equivalent but sufficient for 14bit results
		}
	#endif

	// complements to GCC vector intrinsics
	#ifdef __BMI2__
		static inline unsigned refIdx_to_direct_flags(int64_t f) {
			return _pext_u64(~little_endian64(f), 0x0f0f0f0f0f0f0f0full);
		}
		static inline int mvd_flags2ref_idx(unsigned f) {
			return _pext_u32(f, 0x11111111);
		}
		static inline int extract_neighbours(unsigned f) {
			return _pext_u32(f, 0x30006);
		}
	#else
		static inline unsigned refIdx_to_direct_flags(int64_t f) {
			uint64_t a = ~f & 0x0ff00ff00ff00ff0ull;
			uint64_t b = a >> 4 | a >> 12;
			return (b & 0xffff) | (b >> 16 & 0xffff0000);
		}
		static inline int mvd_flags2ref_idx(unsigned f) {
			int a = f & 0x11111111;
			int b = a | a >> 3;
			int c = b | b >> 6;;
			return (c & 0xf) | (c >> 12 & 0xf0);
		}
		static inline int extract_neighbours(unsigned f) {
			return (f >> 1 & 3) | (f >> 14 & 12);
		}
	#endif
	#ifdef __SSE4_1__
		#define vector_select(mask, t, f) (typeof(t))_mm_blendv_epi8((__m128i)(f), (__m128i)(t), (__m128i)(mask))
		static always_inline __m128i load8x1_8bit(const uint8_t *p, __m128i zero) {
			return _mm_cvtepu8_epi16(_mm_loadu_si64(p));
		}
		static always_inline __m128i load4x1_8bit(const uint8_t *p, __m128i zero) {
			return _mm_cvtepu8_epi16(_mm_loadu_si32(p));
		}
		// FIXME with cvt 8->32 then pack ?
		static always_inline __m128i load4x2_8bit(const uint8_t *r0, const uint8_t *r1, __m128i zero) {
			return _mm_cvtepu8_epi16(_mm_insert_epi32(_mm_cvtsi32_si128(*(int32_t *)r0), *(int *)r1, 1));
		}
		static inline v16qi min_v16qi(v16qi a, v16qi b) {
			return (v16qi)_mm_min_epi8((__m128i)a, (__m128i)b);
		}
	#elif defined __SSSE3__
		#define vector_select(mask, t, f) (((t) & (typeof(t))(mask < 0)) | ((f) & ~(typeof(t))(mask < 0)))
		static inline __m128i load8x1_8bit(const uint8_t *p, __m128i zero) {
			return _mm_unpacklo_epi8(_mm_loadu_si64(p), zero);
		}
		static inline __m128i load4x1_8bit(const uint8_t *p, __m128i zero) {
			return _mm_unpacklo_epi8(_mm_loadu_si32(p), zero);
		}
		static inline __m128i load4x2_8bit(const uint8_t *r0, const uint8_t *r1, __m128i zero) {
			return _mm_unpacklo_epi8(_mm_unpacklo_epi32(_mm_cvtsi32_si128(*(int32_t *)r0), _mm_cvtsi32_si128(*(int32_t *)r1)), zero);
		}
		static inline v16qi min_v16qi(v16qi a, v16qi b) {
			return (v16qi)_mm_xor_si128((__m128i)a, _mm_and_si128(_mm_xor_si128((__m128i)a, (__m128i)b), _mm_cmpgt_epi8((__m128i)a, (__m128i)b)));
		}
	#endif
	static inline size_t lsd(size_t msb, size_t lsb, unsigned shift) {
		__asm__("shld %%cl, %1, %0" : "+rm" (msb) : "r" (lsb), "c" (shift));
		return msb;
	}
	static inline v16qi byte_shuffle(v16qi a, v16qi mask) {
		return (v16qi)_mm_shuffle_epi8((__m128i)a, (__m128i)mask);
	}
	static inline v8hi vector_median(v8hi a, v8hi b, v8hi c) {
		return (v8hi)_mm_max_epi16(_mm_min_epi16(_mm_max_epi16((__m128i)a,
			(__m128i)b), (__m128i)c), _mm_min_epi16((__m128i)a, (__m128i)b));
	}
	static inline v16qu pack_absMvdComp(v8hi a) {
		__m128i x = _mm_abs_epi16(_mm_shuffle_epi32((__m128i)a, _MM_SHUFFLE(0, 0, 0, 0)));
		return (v16qu)_mm_packus_epi16(x, x);
	}
	static inline v8hi mvs_near_zero(v8hi mvCol) {
		return (v8hi)_mm_cmpeq_epi32(_mm_srli_epi16(_mm_abs_epi16((__m128i)mvCol), 1), _mm_setzero_si128());
	}
	static inline int colZero_mask_to_flags(v8hi m0, v8hi m1, v8hi m2, v8hi m3) {
		__m128i x0 = _mm_packs_epi32((__m128i)m0, (__m128i)m1);
		__m128i x1 = _mm_packs_epi32((__m128i)m2, (__m128i)m3);
		return _mm_movemask_epi8(_mm_packs_epi16(x0, x1));
	}
	static inline int first_true(v8hu a) {
		return __builtin_ctz(_mm_movemask_epi8((__m128i)a)) >> 1; // pcmpistri wouldn't help here due to its high latency
	}
	static inline v8hi temporal_scale(v8hi mvCol, int16_t DistScaleFactor) {
		__m128i neg = _mm_set1_epi32(-1);
		__m128i mul = _mm_set1_epi32(DistScaleFactor + 0xff800000u);
		__m128i lo = _mm_madd_epi16(_mm_unpacklo_epi16((__m128i)mvCol, neg), mul);
		__m128i hi = _mm_madd_epi16(_mm_unpackhi_epi16((__m128i)mvCol, neg), mul);
		return (v8hi)_mm_packs_epi32(_mm_srai_epi32(lo, 8), _mm_srai_epi32(hi, 8));
	}
	
	// FIXME remove once we get rid of __m64
	#if defined(__GNUC__) && !defined(__clang__)
	static inline __m128i _mm_movpi64_epi64(__m64 a) {
		__m128i b;
		__asm__("movq2dq %1, %0" : "=x" (b) : "y" (a));
		return b;
	}
	#endif // __GNUC__

#else // add other architectures here
	#error "Add -mssse3 or more recent"
#endif

#ifndef __clang__
#define __builtin_shufflevector(a, b, ...) __builtin_shuffle(a, b, (typeof(a)){__VA_ARGS__})
#endif



/**
 * Constants
 */
static const v16qi sig_inc_4x4 =
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const v16qi sig_inc_8x8[2][4] = {{
	{ 0,  1,  2,  3,  4,  5,  5,  4,  4,  3,  3,  4,  4,  4,  5,  5},
	{ 4,  4,  4,  4,  3,  3,  6,  7,  7,  7,  8,  9, 10,  9,  8,  7},
	{ 7,  6, 11, 12, 13, 11,  6,  7,  8,  9, 14, 10,  9,  8,  6, 11},
	{12, 13, 11,  6,  9, 14, 10,  9, 11, 12, 13, 11, 14, 10, 12,  0},
	}, {
	{ 0,  1,  1,  2,  2,  3,  3,  4,  5,  6,  7,  7,  7,  8,  4,  5},
	{ 6,  9, 10, 10,  8, 11, 12, 11,  9,  9, 10, 10,  8, 11, 12, 11},
	{ 9,  9, 10, 10,  8, 11, 12, 11,  9,  9, 10, 10,  8, 13, 13,  9},
	{ 9, 10, 10,  8, 13, 13,  9,  9, 10, 10, 14, 14, 14, 14, 14,  0},
}};
static const v16qi last_inc_8x8[4] = {
	{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
	{3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4},
	{5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8},
};
static const v16qi sig_inc_chromaDC[2] =
	{{0, 1, 2, 0, 0, 1, 2, 0}, {0, 0, 1, 1, 2, 2, 2, 0, 0, 0, 1, 1, 2, 2, 2, 0}};

// transposed scan tables
static const v16qi scan_4x4[2] = {
	{0, 4, 1, 2, 5, 8, 12, 9, 6, 3, 7, 10, 13, 14, 11, 15},
	{0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
};
static const v16qi scan_8x8[2][4] = {{
	{ 0,  8,  1,  2,  9, 16, 24, 17, 10,  3,  4, 11, 18, 25, 32, 40},
	{33, 26, 19, 12,  5,  6, 13, 20, 27, 34, 41, 48, 56, 49, 42, 35},
	{28, 21, 14,  7, 15, 22, 29, 36, 43, 50, 57, 58, 51, 44, 37, 30},
	{23, 31, 38, 45, 52, 59, 60, 53, 46, 39, 47, 54, 61, 62, 55, 63},
	}, {
	{ 0,  1,  2,  8,  9,  3,  4, 10, 16, 11,  5,  6,  7, 12, 17, 24},
	{18, 13, 14, 15, 19, 25, 32, 26, 20, 21, 22, 23, 27, 33, 40, 34},
	{28, 29, 30, 31, 35, 41, 48, 42, 36, 37, 38, 39, 43, 49, 50, 44},
	{45, 46, 47, 51, 56, 57, 52, 53, 54, 55, 58, 59, 60, 61, 62, 63},
}};
static const v16qi scan_chroma[2] = {
	{0, 1, 2, 3, 4, 5, 6, 7},
	{0, 2, 1, 4, 6, 3, 5, 7, 8, 10, 9, 12, 14, 11, 13, 15}
};

static const v4hi ctxIdxOffsets_16x16DC[3][2] = {
	{{85, 105, 166, 227}, {85, 277, 338, 227}}, // ctxBlockCat==0
	{{460, 484, 572, 952}, {460, 776, 864, 952}}, // ctxBlockCat==6
	{{472, 528, 616, 982}, {472, 820, 908, 982}}, // ctxBlockCat==10
};
static const v4hi ctxIdxOffsets_16x16AC[3][2] = {
	{{89, 119, 180, 237}, {89, 291, 352, 237}}, // ctxBlockCat==1
	{{464, 498, 586, 962}, {464, 790, 878, 962}}, // ctxBlockCat==7
	{{476, 542, 630, 992}, {476, 834, 922, 992}}, // ctxBlockCat==11
};
static const v4hi ctxIdxOffsets_chromaDC[2] =
	{{97, 149, 210, 257}, {97, 321, 382, 257}}; // ctxBlockCat==3
static const v4hi ctxIdxOffsets_chromaAC[2] =
	{{101, 151, 212, 266}, {101, 323, 384, 266}}; // ctxBlockCat==4
static const v4hi ctxIdxOffsets_4x4[3][2] = {
	{{93, 134, 195, 247}, {93, 306, 367, 247}}, // ctxBlockCat==2
	{{468, 528, 616, 972}, {468, 805, 893, 972}}, // ctxBlockCat==8
	{{480, 557, 645, 1002}, {480, 849, 937, 1002}}, // ctxBlockCat==12
};
static const v4hi ctxIdxOffsets_8x8[3][2] = {
	{{1012, 402, 417, 426}, {1012, 436, 451, 426}}, // ctxBlockCat==5
	{{1016, 660, 690, 708}, {1016, 675, 699, 708}}, // ctxBlockCat==9
	{{1020, 718, 748, 766}, {1020, 733, 757, 766}}, // ctxBlockCat==13
};

static const int8_t x444[16] = {0, 4, 0, 4, 8, 12, 8, 12, 0, 4, 0, 4, 8, 12, 8, 12};
static const int8_t y444[16] = {0, 0, 4, 4, 0, 0, 4, 4, 8, 8, 12, 12, 8, 8, 12, 12};
static const int8_t x420[8] = {0, 4, 0, 4, 0, 4, 0, 4};
static const int8_t y420[8] = {0, 0, 4, 4, 0, 0, 4, 4};

static const int8_t inc8x8[12] = {0, 5, 4, 2, 8, 13, 12, 10, 16, 21, 20, 18};
static const int8_t bit8x8[12] = {5, 3, 2, 7, 13, 11, 10, 15, 21, 19, 18, 23};
static const int8_t inc4x4[16] = {8, 15, 14, 21, 22, 29, 28, 12, 20, 27, 26, 10, 11, 18, 17, 24};
static const int8_t bit4x4[16] = {15, 22, 21, 28, 29, 13, 12, 19, 27, 11, 10, 17, 18, 25, 24, 31};



/**
 * Each intraNxN mode is converted to one of these modes right before decoding
 * to select the proper internal routine.
 */
enum Intra4x4_modes {
	I4x4_V_8,
	I4x4_H_8,
	I4x4_DC_8,
	I4x4_DCA_8,
	I4x4_DCB_8,
	I4x4_DCAB_8,
	I4x4_DDL_8,
	I4x4_DDLC_8,
	I4x4_DDR_8,
	I4x4_VR_8,
	I4x4_HD_8,
	I4x4_VL_8,
	I4x4_VLC_8,
	I4x4_HU_8,
};

enum Intra16x16_modes {
	I16x16_V_8,
	I16x16_H_8,
	I16x16_DC_8,
	I16x16_DCA_8,
	I16x16_DCB_8,
	I16x16_DCAB_8,
	I16x16_P_8,
};

enum IntraChroma_modes {
	IC8x8_DC_8,
	IC8x8_DCA_8,
	IC8x8_DCB_8,
	IC8x8_DCAB_8,
	IC8x8_H_8,
	IC8x8_V_8,
	IC8x8_P_8,
};



/**
 * Legacy enum
 */
enum PredModes {
	ADD_RESIDUAL_4x4,
	ADD_RESIDUAL_8x8,
	TRANSFORM_DC_4x4,
	TRANSFORM_DC_2x2,
	TRANSFORM_DC_2x4,
	
	VERTICAL_4x4,
	HORIZONTAL_4x4,
	DC_4x4,
	DC_4x4_A,
	DC_4x4_B,
	DC_4x4_AB,
	DIAGONAL_DOWN_LEFT_4x4,
	DIAGONAL_DOWN_LEFT_4x4_C,
	DIAGONAL_DOWN_RIGHT_4x4,
	VERTICAL_RIGHT_4x4,
	HORIZONTAL_DOWN_4x4,
	VERTICAL_LEFT_4x4,
	VERTICAL_LEFT_4x4_C,
	HORIZONTAL_UP_4x4,
	
	VERTICAL_8x8, // 14
	VERTICAL_8x8_C,
	VERTICAL_8x8_D,
	VERTICAL_8x8_CD,
	HORIZONTAL_8x8,
	HORIZONTAL_8x8_D,
	DC_8x8,
	DC_8x8_C,
	DC_8x8_D,
	DC_8x8_CD,
	DC_8x8_A,
	DC_8x8_AC,
	DC_8x8_AD,
	DC_8x8_ACD,
	DC_8x8_B,
	DC_8x8_BD,
	DC_8x8_AB,
	DIAGONAL_DOWN_LEFT_8x8,
	DIAGONAL_DOWN_LEFT_8x8_C,
	DIAGONAL_DOWN_LEFT_8x8_D,
	DIAGONAL_DOWN_LEFT_8x8_CD,
	DIAGONAL_DOWN_RIGHT_8x8,
	DIAGONAL_DOWN_RIGHT_8x8_C,
	VERTICAL_RIGHT_8x8,
	VERTICAL_RIGHT_8x8_C,
	HORIZONTAL_DOWN_8x8,
	VERTICAL_LEFT_8x8,
	VERTICAL_LEFT_8x8_C,
	VERTICAL_LEFT_8x8_D,
	VERTICAL_LEFT_8x8_CD,
	HORIZONTAL_UP_8x8,
	HORIZONTAL_UP_8x8_D,
	
	VERTICAL_16x16, // 46
	HORIZONTAL_16x16,
	DC_16x16,
	DC_16x16_A,
	DC_16x16_B,
	DC_16x16_AB,
	PLANE_16x16,
	DC_CHROMA_8x8,
	DC_CHROMA_8x8_A,
	DC_CHROMA_8x8_B,
	DC_CHROMA_8x8_AB,
	DC_CHROMA_8x8_Ab,
	DC_CHROMA_8x8_At,
	DC_CHROMA_8x8_AbB,
	DC_CHROMA_8x8_AtB,
	HORIZONTAL_CHROMA_8x8,
	VERTICAL_CHROMA_8x8,
	PLANE_CHROMA_8x8,
	DC_CHROMA_8x16,
	DC_CHROMA_8x16_A,
	DC_CHROMA_8x16_B,
	DC_CHROMA_8x16_AB,
	DC_CHROMA_8x16_Ab,
	DC_CHROMA_8x16_At,
	DC_CHROMA_8x16_AbB,
	DC_CHROMA_8x16_AtB,
	HORIZONTAL_CHROMA_8x16,
	VERTICAL_CHROMA_8x16,
	PLANE_CHROMA_8x16,
	VERTICAL_4x4_BUFFERED,
	HORIZONTAL_4x4_BUFFERED,
	DC_4x4_BUFFERED,
	PLANE_4x4_BUFFERED,
	
	VERTICAL_4x4_16_BIT, // 79
	HORIZONTAL_4x4_16_BIT,
	DC_4x4_16_BIT,
	DC_4x4_A_16_BIT,
	DC_4x4_B_16_BIT,
	DC_4x4_AB_16_BIT,
	DIAGONAL_DOWN_LEFT_4x4_16_BIT,
	DIAGONAL_DOWN_LEFT_4x4_C_16_BIT,
	DIAGONAL_DOWN_RIGHT_4x4_16_BIT,
	VERTICAL_RIGHT_4x4_16_BIT,
	HORIZONTAL_DOWN_4x4_16_BIT,
	VERTICAL_LEFT_4x4_16_BIT,
	VERTICAL_LEFT_4x4_C_16_BIT,
	HORIZONTAL_UP_4x4_16_BIT,
	
	VERTICAL_8x8_16_BIT, // 93
	VERTICAL_8x8_C_16_BIT,
	VERTICAL_8x8_D_16_BIT,
	VERTICAL_8x8_CD_16_BIT,
	HORIZONTAL_8x8_16_BIT,
	HORIZONTAL_8x8_D_16_BIT,
	DC_8x8_16_BIT,
	DC_8x8_C_16_BIT,
	DC_8x8_D_16_BIT,
	DC_8x8_CD_16_BIT,
	DC_8x8_A_16_BIT,
	DC_8x8_AC_16_BIT,
	DC_8x8_AD_16_BIT,
	DC_8x8_ACD_16_BIT,
	DC_8x8_B_16_BIT,
	DC_8x8_BD_16_BIT,
	DC_8x8_AB_16_BIT,
	DIAGONAL_DOWN_LEFT_8x8_16_BIT,
	DIAGONAL_DOWN_LEFT_8x8_C_16_BIT,
	DIAGONAL_DOWN_LEFT_8x8_D_16_BIT,
	DIAGONAL_DOWN_LEFT_8x8_CD_16_BIT,
	DIAGONAL_DOWN_RIGHT_8x8_16_BIT,
	DIAGONAL_DOWN_RIGHT_8x8_C_16_BIT,
	VERTICAL_RIGHT_8x8_16_BIT,
	VERTICAL_RIGHT_8x8_C_16_BIT,
	HORIZONTAL_DOWN_8x8_16_BIT,
	VERTICAL_LEFT_8x8_16_BIT,
	VERTICAL_LEFT_8x8_C_16_BIT,
	VERTICAL_LEFT_8x8_D_16_BIT,
	VERTICAL_LEFT_8x8_CD_16_BIT,
	HORIZONTAL_UP_8x8_16_BIT,
	HORIZONTAL_UP_8x8_D_16_BIT,
	
	VERTICAL_16x16_16_BIT, // 125
	HORIZONTAL_16x16_16_BIT,
	DC_16x16_16_BIT,
	DC_16x16_A_16_BIT,
	DC_16x16_B_16_BIT,
	DC_16x16_AB_16_BIT,
	PLANE_16x16_16_BIT,
	DC_CHROMA_8x8_16_BIT,
	DC_CHROMA_8x8_A_16_BIT,
	DC_CHROMA_8x8_B_16_BIT,
	DC_CHROMA_8x8_AB_16_BIT,
	DC_CHROMA_8x8_Ab_16_BIT,
	DC_CHROMA_8x8_At_16_BIT,
	DC_CHROMA_8x8_AbB_16_BIT,
	DC_CHROMA_8x8_AtB_16_BIT,
	VERTICAL_CHROMA_8x8_16_BIT,
	HORIZONTAL_CHROMA_8x8_16_BIT,
	PLANE_CHROMA_8x8_16_BIT,
	DC_CHROMA_8x16_16_BIT,
	DC_CHROMA_8x16_A_16_BIT,
	DC_CHROMA_8x16_B_16_BIT,
	DC_CHROMA_8x16_AB_16_BIT,
	DC_CHROMA_8x16_Ab_16_BIT,
	DC_CHROMA_8x16_At_16_BIT,
	DC_CHROMA_8x16_AbB_16_BIT,
	DC_CHROMA_8x16_AtB_16_BIT,
	VERTICAL_CHROMA_8x16_16_BIT,
	HORIZONTAL_CHROMA_8x16_16_BIT,
	PLANE_CHROMA_8x16_16_BIT,
	VERTICAL_4x4_BUFFERED_16_BIT,
	HORIZONTAL_4x4_BUFFERED_16_BIT,
	DC_4x4_BUFFERED_16_BIT,
	PLANE_4x4_BUFFERED_16_BIT,
};

#endif
