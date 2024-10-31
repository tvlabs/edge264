/**
 * Every file should be compilable on its own by including this file.
 */

#ifndef edge264_COMMON_H
#define edge264_COMMON_H

#include <assert.h>
#include <limits.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "edge264.h"



typedef int8_t i8x4 __attribute__((vector_size(4)));
typedef int8_t i8x8 __attribute__((vector_size(8)));
typedef int16_t i16x4 __attribute__((vector_size(8)));
typedef int32_t i32x2 __attribute__((vector_size(8)));
typedef int8_t i8x16 __attribute__((vector_size(16)));
typedef int16_t i16x8 __attribute__((vector_size(16)));
typedef int32_t i32x4 __attribute__((vector_size(16)));
typedef int64_t i64x2 __attribute__((vector_size(16)));
typedef uint8_t u8x16 __attribute__((vector_size(16)));
typedef uint16_t u16x8 __attribute__((vector_size(16)));
typedef uint64_t u64x2 __attribute__((vector_size(16)));
typedef int8_t i8x32 __attribute__((vector_size(32))); // alignment for 256-bit extensions
typedef int16_t i16x16 __attribute__((vector_size(32)));
typedef int32_t i32x8 __attribute__((vector_size(32)));
typedef int32_t i32x16 __attribute__((vector_size(64))); // for initialization of neighbouring offsets



/**
 * Bitstream parsing context is put in a struct so that it can be included at
 * the start of both stream and nal structures, thus their pointers can be sent
 * to bitstream functions without offsets.
 */
typedef struct {
	const uint8_t *CPB; // memory address of the next byte to load in caches
	const uint8_t *end; // first byte past end of buffer, capped to 001 or 000 sequence when detected
	union { size_t lsb_cache; size_t codIRange; };
	union { size_t msb_cache; size_t codIOffset; };
} Edge264GetBits;



/**
 * In 9.3.3.1.1, ctxIdxInc is always the result of flagA+flagB or flagA+2*flagB,
 * so we pack macroblock flags together to allow adding them in parallel with
 * flagsA + flagsB + (flagsB & twice).
 */
typedef union {
	struct {
		int8_t filter_edges; // 2 bits to enable deblocking of A/B edges
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
	i8x16 v;
} Edge264Flags;
static const Edge264Flags flags_twice = {
	.CodedBlockPatternChromaDC = 1,
	.CodedBlockPatternChromaAC = 1,
	.coded_block_flags_16x16 = {1, 1, 1},
};



/**
 * Although we waste some space by storing some neighbouring values for more
 * than their lifespans, packing everything in a single structure is arguably
 * the simplest to maintain. Neighbouring values from the same or a different
 * macroblock are obtained with memory offsets precomputed in ctx. With this
 * technique we spare the use of intermediate caches thus reduce memory writes.
 * 
 * CABAC bit values of 8x8 blocks are stored in compact 8-bit patterns that
 * allow retrieving A+B*2 efficiently:
 *   1 6
 * 0|5 3
 * 4|2 7
 */
typedef struct {
	Edge264Flags f;
	union { uint8_t inter_eqs[4]; uint32_t inter_eqs_s; }; // 2 flags per 4x4 block storing right/bottom equality of mvs&ref, in little endian
	union { uint8_t QP[3]; i8x4 QP_s; }; // [iYCbCr]
	union { uint32_t bits[2]; uint64_t bits_l; }; // {cbp/ref_idx_nz, cbf_Y/Cb/Cr 8x8}
	union { int8_t nC[3][16]; int32_t nC_s[3][4]; int64_t nC_l[6]; i8x16 nC_v[3]; }; // for CAVLC and deblocking, 64 if unavailable
	union { int8_t Intra4x4PredMode[16]; i8x16 Intra4x4PredMode_v; }; // [i4x4]
	union { int8_t refIdx[8]; int32_t refIdx_s[2]; int64_t refIdx_l; }; // [LX][i8x8]
	union { int8_t refPic[8]; int32_t refPic_s[2]; int64_t refPic_l; }; // [LX][i8x8]
	union { uint8_t absMvd[64]; uint64_t absMvd_l[8]; i8x16 absMvd_v[4]; }; // [LX][i4x4][compIdx]
	union { int16_t mvs[64]; int32_t mvs_s[32]; int64_t mvs_l[16]; i16x8 mvs_v[8]; }; // [LX][i4x4][compIdx]
} Edge264Macroblock;
static Edge264Macroblock unavail_mb = {
	.f.mb_skip_flag = 1,
	.f.mb_type_I_NxN = 1,
	.f.mb_type_B_Direct = 1,
	.refIdx = {-1, -1, -1, -1, -1, -1, -1, -1},
	.bits[0] = 0xac, // cbp
	.Intra4x4PredMode = {-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2},
};



/**
 * Parameter sets are defined as structs so that we can parse them into
 * temporary storage, then copy them into persistent context after
 * trailing_bits are detected.
 */
typedef struct {
	// The first 8 bytes uniquely determine the frame buffer size and format.
	union {
		struct {
			int8_t chroma_format_idc; // 0..3
			int8_t BitDepth_Y; // 8..14
			int8_t BitDepth_C;
			int8_t max_num_ref_frames; // 0..16
			uint16_t pic_width_in_mbs; // 1..1023
			int16_t pic_height_in_mbs; // 1..1055
		};
		int64_t DPB_format;
	};
	int8_t ChromaArrayType; // 0..3
	int8_t qpprime_y_zero_transform_bypass_flag; // 0..1
	int8_t log2_max_frame_num; // 4..16
	int8_t pic_order_cnt_type; // 0..2
	int8_t log2_max_pic_order_cnt_lsb; // 4..16, pic_order_cnt_type==0
	int8_t delta_pic_order_always_zero_flag; // 0..1, pic_order_cnt_type==1
	uint8_t num_ref_frames_in_pic_order_cnt_cycle; // 0..255, pic_order_cnt_type==1
	int8_t frame_mbs_only_flag; // 0..1
	int8_t mb_adaptive_frame_field_flag; // 0..1
	int8_t direct_8x8_inference_flag; // 0..1
	int8_t num_frame_buffers; // 1..18
	int8_t max_num_reorder_frames; // 0..17
	int8_t mvc; // 0..1
	int16_t offset_for_non_ref_pic; // -32768..32767, pic_order_cnt_type==1
	int16_t offset_for_top_to_bottom_field; // -32768..32767, pic_order_cnt_type==1
	int16_t PicOrderCntDeltas[256]; // -32768..32767, pic_order_cnt_type==1
	union { int16_t frame_crop_offsets[4]; int64_t frame_crop_offsets_l; }; // {top,right,bottom,left}
	union { uint8_t weightScale4x4[6][16]; i8x16 weightScale4x4_v[6]; };
	union { uint8_t weightScale8x8[6][64]; i8x16 weightScale8x8_v[6*4]; };
} Edge264SeqParameterSet;
typedef struct {
	int8_t entropy_coding_mode_flag; // 0..1
	int8_t bottom_field_pic_order_in_frame_present_flag; // 0..1
	int8_t num_ref_idx_active[2]; // 0..32
	int8_t weighted_pred_flag; // 0..1
	int8_t weighted_bipred_idc; // 0..2
	int8_t QPprime_Y; // 0..87
	int8_t chroma_qp_index_offset; // -12..12
	int8_t deblocking_filter_control_present_flag; // 0..1
	int8_t constrained_intra_pred_flag; // 0..1
	int8_t transform_8x8_mode_flag; // 0..1
	int8_t second_chroma_qp_index_offset; // -12..12
	union { uint8_t weightScale4x4[6][16]; i8x16 weightScale4x4_v[6]; };
	union { uint8_t weightScale8x8[6][64]; i8x16 weightScale8x8_v[6*4]; };
} Edge264PicParameterSet;



/**
 * This structure stores all the data necessary to decode a slice, such that it
 * can be copied into Edge264Context when a worker starts decoding it.
 */
typedef struct {
	Edge264GetBits _gb; // must be first in struct to use the same pointer for bitstream functions
	int8_t slice_type; // 0..2
	int8_t field_pic_flag; // 0..1
	int8_t bottom_field_flag; // 0..1
	int8_t MbaffFrameFlag; // 0..1
	int8_t direct_spatial_mv_pred_flag; // 0..1
	int8_t luma_log2_weight_denom; // 0..7
	int8_t chroma_log2_weight_denom; // 0..7
	int8_t disable_deblocking_filter_idc; // 0..2
	int8_t FilterOffsetA; // -12..12
	int8_t FilterOffsetB;
	int8_t ChromaArrayType; // 0..3
	int8_t direct_8x8_inference_flag; // 0..1
	int8_t cabac_init_idc; // 0..3
	int8_t next_deblock_idc; // -1..31, -1 if next_deblock_addr is not written back to dec, currPic otherwise
	int16_t pic_width_in_mbs; // 0..1023
	int16_t pic_height_in_mbs; // 0..1055
	uint16_t stride[3]; // 0..65472 (at max width, 16bit & field pic), [iYCbCr]
	int32_t plane_size_Y;
	int32_t plane_size_C;
	int32_t next_deblock_addr; // INT_MIN..INT_MAX
	uint32_t first_mb_in_slice; // 0..139263
	uint32_t long_term_flags;
	union { int8_t QP[3]; i8x4 QP_s; }; // same as mb
	uint8_t *samples_base;
	uint8_t *frame_buffers[32];
   void (*free_cb)(void *free_arg, int ret); // copy from decode_NAL
   void *free_arg; // copy from decode_NAL
	union { uint16_t samples_clip[3][8]; i16x8 samples_clip_v[3]; }; // [iYCbCr], maximum sample value
	union { int8_t RefPicList[2][32]; int64_t RefPicList_l[8]; i8x16 RefPicList_v[4]; };
	union { int16_t diff_poc[32]; i16x8 diff_poc_v[4]; };
	Edge264PicParameterSet pps;
	int16_t explicit_weights[3][64]; // [iYCbCr][LX][RefIdx]
	int8_t explicit_offsets[3][64];
} Edge264Task;



/**
 * This structure stores the context data needed by each thread to decode
 * a slice, such that we can dedicate a single register pointer to it.
 */
typedef struct Edge264Context {
	Edge264Task t; // must be first in struct to use the same pointer for bitstream functions
	int8_t n_threads;
	int8_t mb_qp_delta_nz; // 0..1
	int8_t col_short_term; // 0..1
	int16_t mbx;
	int16_t mby;
	int32_t CurrMbAddr;
	int32_t PicOrderCnt;
	int32_t mb_skip_run;
	uint8_t *samples_mb[3]; // address of top-left byte of each plane in current macroblock
	Edge264Macroblock * _mb; // backup storage for macro mb
	const Edge264Macroblock * _mbA; // backup storage for macro mbA
	const Edge264Macroblock * _mbB; // backup storage for macro mbB
	const Edge264Macroblock * _mbC; // backup storage for macro mbC
	const Edge264Macroblock * _mbD; // backup storage for macro mbD
	const Edge264Macroblock *mbCol;
	Edge264Decoder *d;
	Edge264Flags inc; // increments for CABAC indices of macroblock syntax elements
	union { int8_t unavail4x4[16]; i8x16 unavail4x4_v; }; // unavailability of neighbouring A/B/C/D blocks
	union { int8_t nC_inc[3][16]; i8x16 nC_inc_v[3]; }; // stores the intra/inter default increment from unavailable neighbours (9.3.3.1.1.9)
	union { uint8_t cabac[1024]; i8x16 cabac_v[64]; };
	
	// neighbouring offsets (relative to the start of each array in mb)
	union { int16_t A4x4_int8[16]; i16x16 A4x4_int8_v; };
	union { int32_t B4x4_int8[16]; i32x16 B4x4_int8_v; };
	union { int16_t ACbCr_int8[16]; i16x8 ACbCr_int8_v[2]; };
	union { int32_t BCbCr_int8[16]; i32x8 BCbCr_int8_v[2]; };
	union { int8_t refIdx4x4_C[16]; int32_t refIdx4x4_C_s[4]; i8x16 refIdx4x4_C_v; }; // shuffle vector for mv prediction
	union { int16_t absMvd_A[16]; i16x16 absMvd_A_v; };
	union { int32_t absMvd_B[16]; i32x16 absMvd_B_v; };
	union { int16_t mvs_A[16]; i16x16 mvs_A_v; };
	union { int32_t mvs_B[16]; i32x16 mvs_B_v; };
	union { int32_t mvs_C[16]; i32x16 mvs_C_v; };
	union { int32_t mvs_D[16]; i32x16 mvs_D_v; };
	
	// Inter context
	int8_t transform_8x8_mode_flag; // updated during parsing to account for noSubMbPartSizeLessThan8x8Flag
	uint8_t num_ref_idx_mask;
	int16_t DistScaleFactor[32]; // [refIdxL0]
	union { int8_t clip_ref_idx[8]; i8x8 clip_ref_idx_v; };
	union { int8_t MapPicToList0[32]; i8x16 MapPicToList0_v[2]; };
	union { int8_t implicit_weights[32][32]; i8x16 implicit_weights_v[32][2]; }; // w1 for [ref0][ref1]
	union { uint8_t edge_buf[2016]; int64_t edge_buf_l[252]; i8x16 edge_buf_v[126]; };
	
	// Residuals context
	union { int16_t ctxIdxOffsets[4]; i16x4 ctxIdxOffsets_l; }; // {cbf,sig_flag,last_sig_flag,coeff_abs}
	union { int8_t coeff_abs_inc[8]; i8x8 coeff_abs_inc_l; };
	union { int8_t sig_inc[64]; i8x8 sig_inc_l; i8x16 sig_inc_v[4]; };
	union { int8_t last_inc[64]; i8x8 last_inc_l; i8x16 last_inc_v[4]; };
	union { int8_t scan[64]; i8x4 scan_s; i8x8 scan_l; i8x16 scan_v[4]; };
	union { int8_t QP_C[2][64]; i8x16 QP_C_v[8]; };
	union { int32_t c[64]; i32x4 c_v[16]; i32x8 c_V[8]; }; // non-scaled residual coefficients
	
	// Deblocking context
	union { uint8_t alpha[16]; i8x16 alpha_v; }; // {internal_Y,internal_Cb,internal_Cr,0,0,0,0,0,left_Y,left_Cb,left_Cr,0,top_Y,top_Cb,top_Cr,0}
	union { uint8_t beta[16]; i8x16 beta_v; };
	union { int32_t tC0_s[16]; int64_t tC0_l[8]; i8x16 tC0_v[4]; i8x32 tC0_V[2]; }; // 4 bytes per edge in deblocking order -> 8 luma edges then 8 alternating Cb/Cr edges
} Edge264Context;
#define mb ctx->_mb
#define mbA ctx->_mbA
#define mbB ctx->_mbB
#define mbC ctx->_mbC
#define mbD ctx->_mbD



/**
 * This structure stores all variables scoped to the entire stream.
 */
typedef int (*Parser)(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg);
typedef struct Edge264Decoder {
	Edge264GetBits _gb; // must be first in the struct to use the same pointer for bitstream functions
	int8_t n_threads; // 0 to disable multithreading
	int8_t nal_ref_idc; // 2 significant bits
	int8_t nal_unit_type; // 5 significant bits
	int8_t IdrPicFlag; // 1 significant bit
	int8_t currPic; // index of current incomplete frame, or -1
	int8_t basePic; // index of last MVC base view, or -1
	int32_t plane_size_Y;
	int32_t plane_size_C;
	int32_t frame_size;
	int32_t FrameNum; // value for the current incomplete frame, unaffected by mmco5
	int32_t prevRefFrameNum[2];
	int32_t TopFieldOrderCnt; // same
	int32_t BottomFieldOrderCnt;
	int32_t prevPicOrderCnt;
	int32_t dispPicOrderCnt; // all POCs lower or equal than this are ready for output
	int32_t FrameNums[32];
	uint32_t reference_flags; // bitfield for indices of reference frames/views
	uint32_t long_term_flags; // bitfield for indices of long-term frames/views
	uint32_t output_flags; // bitfield for frames waiting to be output
	uint32_t borrow_flags; // bitfield for frames that are owned by the caller after get_frame and not yet returned
	uint32_t pic_reference_flags; // to be applied after decoding all slices of the current picture
	uint32_t pic_long_term_flags; // to be applied after decoding all slices of the current picture
	int64_t DPB_format; // should match format in SPS otherwise triggers resize
	uint8_t *frame_buffers[32];
	Parser parse_nal_unit[32];
	union { int8_t LongTermFrameIdx[32]; i8x16 LongTermFrameIdx_v[2]; };
	union { int8_t pic_LongTermFrameIdx[32]; i8x16 pic_LongTermFrameIdx_v[2]; }; // to be applied after decoding all slices of the current frame
	union { int32_t FieldOrderCnt[2][32]; i32x4 FieldOrderCnt_v[2][8]; }; // lower/higher half for top/bottom fields
	Edge264Frame out;
	Edge264SeqParameterSet sps;
	Edge264PicParameterSet PPS[4];
	pthread_t threads[16];
	
	// fields accessed concurrently from multiple threads
	pthread_mutex_t lock;
	pthread_cond_t task_ready;
	pthread_cond_t task_progress; // signals next_deblock_addr has been updated
	pthread_cond_t task_complete;
	uint16_t busy_tasks; // bitmask for tasks that are either pending or processed in a thread
	uint16_t pending_tasks;
	uint16_t ready_tasks;
	int32_t remaining_mbs[32]; // when zero the picture is complete
	union { int32_t next_deblock_addr[32]; i32x4 next_deblock_addr_v[8]; }; // next CurrMbAddr value for which mbB will be deblocked
	volatile union { uint32_t task_dependencies[16]; i32x4 task_dependencies_v[4]; }; // frames on which each task depends to start
	union { int8_t taskPics[16]; i8x16 taskPics_v; }; // values of currPic for each task
	Edge264Task tasks[16];
} Edge264Decoder;



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

#if UINT_MAX == 4294967295U && ULLONG_MAX == 18446744073709551615U
	#define clz32 __builtin_clz
	#define ctz32 __builtin_ctz
	#define clz64 __builtin_clzll
	#define ctz64 __builtin_ctzll
	#ifndef WORD_BIT
		#define WORD_BIT 32
	#endif
#endif

#if SIZE_MAX == 4294967295U
	#define SIZE_BIT 32
	#define clz clz32
	#define ctz ctz32
#elif SIZE_MAX == 18446744073709551615U
	#define SIZE_BIT 64
	#define clz clz64
	#define ctz ctz64
#endif

#ifndef noinline
	#define noinline __attribute__((noinline))
#endif
#ifndef always_inline
	#define always_inline inline __attribute__((always_inline))
#endif



/**
 * Constants that are common to several files
 */
static const i8x16 scan_4x4[2] = {
	{0, 4, 1, 2, 5, 8, 12, 9, 6, 3, 7, 10, 13, 14, 11, 15},
	{0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
};
static const i8x16 scan_8x8_cavlc[2][4] = {{
	{ 0,  9, 10, 18, 33,  5, 27, 56, 28, 15, 43, 51, 23, 52, 46, 61},
	{ 8, 16,  3, 25, 26,  6, 34, 49, 21, 22, 50, 44, 31, 59, 39, 62},
	{ 1, 24,  4, 32, 19, 13, 41, 42, 14, 29, 57, 37, 38, 60, 47, 55},
	{ 2, 17, 11, 40, 12, 20, 48, 35,  7, 36, 58, 30, 45, 53, 54, 63},
}, {
	{ 0,  9, 16,  7, 18, 19, 20, 27, 28, 35, 36, 43, 45, 56, 54, 60},
	{ 1,  3, 11, 12, 13, 25, 21, 33, 29, 41, 37, 49, 46, 57, 55, 61},
	{ 2,  4,  5, 17, 14, 32, 22, 40, 30, 48, 38, 50, 47, 52, 58, 62},
	{ 8, 10,  6, 24, 15, 26, 23, 34, 31, 42, 39, 44, 51, 53, 59, 63},
}};
static const i8x16 scan_8x8_cabac[2][4] = {{
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
static const i8x16 scan_chroma[2] = {
	{0, 1, 2, 3, 4, 5, 6, 7},
	{0, 2, 1, 4, 6, 3, 5, 7, 8, 10, 9, 12, 14, 11, 13, 15}
};

static const int8_t inc8x8[12] = {0, 5, 4, 2, 8, 13, 12, 10, 16, 21, 20, 18};
static const int8_t bit8x8[12] = {5, 3, 2, 7, 13, 11, 10, 15, 21, 19, 18, 23};

static const int8_t x444[16] = {0, 4, 0, 4, 8, 12, 8, 12, 0, 4, 0, 4, 8, 12, 8, 12};
static const int8_t y444[16] = {0, 0, 4, 4, 0, 0, 4, 4, 8, 8, 12, 12, 8, 8, 12, 12};
static const int8_t x420[8] = {0, 4, 0, 4, 0, 4, 0, 4};
static const int8_t y420[8] = {0, 0, 4, 4, 0, 0, 4, 4};

// really big, defined in edge264.c
extern const int8_t cabac_context_init[4][1024][2] __attribute__((aligned(16)));



/**
 * Each intraNxN mode is converted to one of these modes right before decoding
 * to select the proper internal routine.
 */
enum Intra4x4Modes {
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

enum Intra8x8Modes {
	I8x8_V_8,
	I8x8_V_C_8,
	I8x8_V_D_8,
	I8x8_V_CD_8,
	I8x8_H_8,
	I8x8_H_D_8,
	I8x8_DC_8,
	I8x8_DC_A_8,
	I8x8_DC_AC_8,
	I8x8_DC_AD_8,
	I8x8_DC_ACD_8,
	I8x8_DC_B_8,
	I8x8_DC_BD_8,
	I8x8_DC_C_8,
	I8x8_DC_D_8,
	I8x8_DC_CD_8,
	I8x8_DC_AB_8,
	I8x8_DDL_8,
	I8x8_DDL_C_8,
	I8x8_DDL_D_8,
	I8x8_DDL_CD_8,
	I8x8_DDR_8,
	I8x8_DDR_C_8,
	I8x8_VR_8,
	I8x8_VR_C_8,
	I8x8_HD_8,
	I8x8_VL_8,
	I8x8_VL_C_8,
	I8x8_VL_D_8,
	I8x8_VL_CD_8,
	I8x8_HU_8,
	I8x8_HU_D_8,
};

enum Intra16x16Modes {
	I16x16_V_8,
	I16x16_H_8,
	I16x16_DC_8,
	I16x16_DCA_8,
	I16x16_DCB_8,
	I16x16_DCAB_8,
	I16x16_P_8,
};

enum IntraChromaModes {
	IC8x8_DC_8,
	IC8x8_DCA_8,
	IC8x8_DCB_8,
	IC8x8_DCAB_8,
	IC8x8_H_8,
	IC8x8_V_8,
	IC8x8_P_8,
};



/**
 * Debugging functions
 */
#ifdef TRACE
	#include <stdio.h>
	static inline const char *red_if(int cond) { return (cond) ? " style='background-color:#fee'" : ""; }
#else
	#define printf(...) ((void)0)
#endif
#if TRACE < 2
	#define fprintf(...) ((void)0)
#endif

#define print_i8x16(a) {\
	i8x16 _v = a;\
	printf("<k>" #a "</k><v>");\
	for (int _i = 0; _i < 16; _i++)\
		printf("%4d ", _v[_i]);\
	printf("</v>\n");}
#define print_u8x16(a) {\
	u8x16 _v = a;\
	printf("<k>" #a "</k><v>");\
	for (int _i = 0; _i < 16; _i++)\
		printf("%3u ", _v[_i]);\
	printf("</v>\n");}
#define print_i16x8(a) {\
	i16x8 _v = a;\
	printf("<k>" #a "</k><v>");\
	for (int _i = 0; _i < 8; _i++)\
		printf("%6d ", _v[_i]);\
	printf("</v>\n");}
#define print_i32x4(a) {\
	i32x4 _v = a;\
	printf("<k>" #a "</k><v>");\
	for (int _i = 0; _i < 4; _i++)\
		printf("%6d ", _v[_i]);\
	printf("</v>\n");}



/**
 * These custom vector intrinsics are an attempt to shorten those from Intel,
 * as well as to facilitate the future ARM support by listing all of the
 * instructions actually used.
 */
#ifdef __SSE2__
	#include <x86intrin.h>
	static const i8x16 mul_15 = {1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5};
	static const i8x16 mul_20 = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20};
	static const i8x16 mul_51 = {-5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1, -5, 1};
	
	// basic shortened intrinsics
	#define adds16(a, b) (i16x8)_mm_adds_epi16(a, b)
	#define avg8(a, b) (i8x16)_mm_avg_epu8(a, b)
	#define avg16(a, b) (i16x8)_mm_avg_epu16(a, b)
	#define load32(p) ((i32x4){*(int32_t *)(p)}) // GCC < 12 doesn't define _mm_loadu_si32
	#define load64(p) (i64x2)_mm_loadl_epi64((__m128i*)(p))
	#define loadh64(a, p) (i64x2)_mm_loadh_pd((__m128d)(a), (double*)(p))
	#define load128(p) (i8x16)_mm_loadu_si128((__m128i*)(p))
	#define madd16(a, b) (i32x4)_mm_madd_epi16(a, b)
	#define max16(a, b) (i16x8)_mm_max_epi16(a, b)
	#define min16(a, b) (i16x8)_mm_min_epi16(a, b)
	#define movemask(a) _mm_movemask_epi8(a)
	#define packs16(a, b) (i8x16)_mm_packs_epi16(a, b)
	#define packs32(a, b) (i16x8)_mm_packs_epi32(a, b)
	#define packus16(a, b) (i8x16)_mm_packus_epi16(a, b)
	#define sad8(a, b) (i16x8)_mm_sad_epu8(a, b)
	#define set8(i) (i8x16)_mm_set1_epi8(i)
	#define set16(i) (i16x8)_mm_set1_epi16(i)
	#define set32(i) (i32x4)_mm_set1_epi32(i)
	#define shlc(a, i) (i8x16)_mm_slli_si128(a, i)
	#define shl16(a, b) (i16x8)_mm_sll_epi16(a, b) // FIXME naming
	#define shl32(a, b) (i32x4)_mm_sll_epi32(a, b)
	#define shrc(a, i) (i8x16)_mm_srli_si128(a, i)
	#define shr16(a, b) (i16x8)_mm_sra_epi16(a, b)
	#define shr32(a, b) (i32x4)_mm_sra_epi32(a, b)
	#define shuffle32(a, i, j, k, l) (i32x4)_mm_shuffle_epi32(a, _MM_SHUFFLE(l, k, j, i))
	#define shufflehi(a, i, j, k, l) (i16x8)_mm_shufflehi_epi16(a, _MM_SHUFFLE(l, k, j, i))
	#define shufflelo(a, i, j, k, l) (i16x8)_mm_shufflelo_epi16(a, _MM_SHUFFLE(l, k, j, i))
	#define shuffleps(a, b, i, j, k, l) (i32x4)_mm_shuffle_ps((__m128)(a), (__m128)(b), _MM_SHUFFLE(l, k, j, i))
	#define usubs8(a, b) (i8x16)_mm_subs_epu8(a, b)
	#define uadds8(a, b) (i8x16)_mm_adds_epu8(a, b)
	#define umax8(a, b) (i8x16)_mm_max_epu8(a, b)
	#define umin8(a, b) (i8x16)_mm_min_epu8(a, b)
	#define unpacklo8(a, b) (i8x16)_mm_unpacklo_epi8(a, b)
	#define unpacklo16(a, b) (i16x8)_mm_unpacklo_epi16(a, b)
	#define unpacklo32(a, b) (i32x4)_mm_unpacklo_epi32(a, b)
	#define unpacklo64(a, b) (i64x2)_mm_unpacklo_epi64(a, b)
	#define unpackhi8(a, b) (i8x16)_mm_unpackhi_epi8(a, b)
	#define unpackhi16(a, b) (i16x8)_mm_unpackhi_epi16(a, b)
	#define unpackhi32(a, b) (i32x4)_mm_unpackhi_epi32(a, b)
	#define unpackhi64(a, b) (i64x2)_mm_unpackhi_epi64(a, b)
	#ifdef __SSSE3__
		#define abs8(a) (u8x16)_mm_abs_epi8(a)
		#define abs16(a) (u16x8)_mm_abs_epi16(a)
		#define alignr(l, h, i) (i8x16)_mm_alignr_epi8(h, l, i)
		#define hadd16(a, b) (i16x8)_mm_hadd_epi16(a, b)
		#define maddubs(a, b) (i16x8)_mm_maddubs_epi16(a, b)
		static always_inline i8x16 shuffle(i8x16 a, i8x16 m) { return (i8x16)_mm_shuffle_epi8(a, m); } // out-of-bounds values yield anything
	#else
		static always_inline u8x16 abs8(i8x16 a) { return _mm_min_epu8(-a, a); }
		static always_inline u16x8 abs16(i16x8 a) { return _mm_max_epi16(-a, a); }
		#define alignr(l, h, i) (shrc(l, i) | shlc(h, 16 - (i)))
		static always_inline i16x8 hadd16(i16x8 a, i16x8 b) { return _mm_packs_epi32((i32x4)((i16x8)((i32x4)a << 16) + a) >> 16, (i32x4)((i16x8)((i32x4)b << 16) + b) >> 16); }
		static always_inline i16x8 maddubs(u8x16 a, i8x16 b) { return adds16(((u16x8)a << 8 >> 8) * ((i16x8)b << 8 >> 8), ((u16x8)a >> 8) * ((i16x8)b >> 8)); }
		// no way to make clang use __builtin_shufflevector, and GCC performs worse with __builtin_shuffle :(
		static always_inline i8x16 shuffle(i8x16 a, i8x16 m) { m &= 15; return (i8x16){a[m[0]], a[m[1]], a[m[2]], a[m[3]], a[m[4]], a[m[5]], a[m[6]], a[m[7]], a[m[8]], a[m[9]], a[m[10]], a[m[11]], a[m[12]], a[m[13]], a[m[14]], a[m[15]]}; }
	#endif
	#ifdef __SSE4_1__
		#define cvt8zx16(a) (i16x8)_mm_cvtepu8_epi16(a)
		#define cvt16zx32(a) (i32x4)_mm_cvtepu16_epi32(a)
		#define ifelse_mask(v, t, f) (i8x16)_mm_blendv_epi8(f, t, v)
		#define ifelse_msb(v, t, f) (i8x16)_mm_blendv_epi8(f, t, v)
		#define load8zx16(p) (i16x8)_mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i*)(p)))
		#define min8(a, b) (i8x16)_mm_min_epi8(a, b)
		#define min32(a, b) (i32x4)_mm_min_epi32(a, b)
		#define max8(a, b) (i8x16)_mm_max_epi8(a, b)
	#else
		#define cvt8zx16(a) (i16x8)unpacklo8(a, (i8x16){})
		#define cvt16zx32(a) (i32x4)unpacklo16(a, (i16x8){})
		static always_inline i8x16 ifelse_mask(i8x16 v, i8x16 t, i8x16 f) { return t & v | f & ~v; }
		static always_inline i8x16 ifelse_msb(i8x16 v, i8x16 t, i8x16 f) { v = (0 > v); return t & v | f & ~v; }
		#define load8zx16(p) (i16x8)unpacklo8(load64(p), (i8x16){})
		static always_inline i8x16 min8(i8x16 a, i8x16 b) { i8x16 v = b > a; return a & v | b & ~v; }
		static always_inline i32x4 min32(i32x4 a, i32x4 b) { i32x4 v = b > a; return a & v | b & ~v; }
		static always_inline i8x16 max8(i8x16 a, i8x16 b) { i8x16 v = a > b; return a & v | b & ~v; }
	#endif
	#ifdef __AVX2__
		#define broadcast8(a) (i8x16)_mm_broadcastb_epi8(a)
		#define broadcast16(a) (i16x8)_mm_broadcastw_epi16(a)
	#else
		#ifdef __SSSE3__
			#define broadcast8(a) shuffle(a, (i8x16){})
		#else
			static always_inline i8x16 broadcast8(i8x16 a) { return shuffle32(shufflelo(unpacklo8(a, a), 0, 0, 0, 0), 0, 0, 0, 0); }
		#endif
		#define broadcast16(a) (i16x8)_mm_shuffle_epi32(_mm_shufflelo_epi16(a, _MM_SHUFFLE(0, 0, 0, 0)), _MM_SHUFFLE(1, 0, 1, 0))
	#endif
	
	// hardware-specific helper functions
	static always_inline size_t lsd(size_t msb, size_t lsb, unsigned shift) {
		__asm__("shld %%cl, %1, %0" : "+rm" (msb) : "r" (lsb), "c" (shift));
		return msb;
	}
	static always_inline unsigned refs_to_mask(Edge264Task *t) {
		u8x16 a = t->RefPicList_v[0] + 127;
		u8x16 b = t->RefPicList_v[2] + 127;
		i8x16 zero = {};
		i16x8 c = (i16x8)unpacklo8(a, zero) << 7;
		i16x8 d = (i16x8)unpackhi8(a, zero) << 7;
		i16x8 e = (i16x8)unpacklo8(b, zero) << 7;
		i16x8 f = (i16x8)unpackhi8(b, zero) << 7;
		i32x4 g = _mm_cvtps_epi32((__m128)unpacklo16(zero, c)) | _mm_cvtps_epi32((__m128)unpackhi16(zero, c)) |
		          _mm_cvtps_epi32((__m128)unpacklo16(zero, d)) | _mm_cvtps_epi32((__m128)unpackhi16(zero, d)) |
		          _mm_cvtps_epi32((__m128)unpacklo16(zero, e)) | _mm_cvtps_epi32((__m128)unpackhi16(zero, e)) |
		          _mm_cvtps_epi32((__m128)unpacklo16(zero, f)) | _mm_cvtps_epi32((__m128)unpackhi16(zero, f));
		i32x4 h = g | shuffle32(g, 2, 3, 0, 1);
		i32x4 i = h | shuffle32(h, 1, 0, 3, 2);
		return i[0];
	}
	static always_inline i8x16 shufflen(i8x16 a, i8x16 m) {
		return shuffle(a, m) | (0 > m); // -1 yields -1, other out-of-bounds values yield anything
	}
	#ifdef __SSSE3__
		static const int8_t sh_data[48] = {
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
			 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
			-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		};
		#define shl(a, i) shuffle(a, load128(sh_data + 16 - (i)))
		#define shr(a, i) shuffle(a, load128(sh_data + 16 + (i)))
		#define shufflez shuffle // negative values yield zero, positive out-of-bounds values yield anything
		static always_inline i8x16 shuffle2(const i8x16 *p, i8x16 m) {
			return ifelse_mask(m > 15, shuffle(p[1], m), shuffle(p[0], m));
		}
		#define shufflez2 shuffle2
		static always_inline i8x16 shuffle3(const i8x16 *p, i8x16 m) {
			return ifelse_mask(m > 15, ifelse_mask(m > 31, shuffle(p[2], m), shuffle(p[1], m)), shuffle(p[0], m));
		}
		static always_inline i16x8 sixtapU8(i8x16 ab, i8x16 cd, i8x16 ef) {
			i16x8 x0 = maddubs(ab, mul_15);
			i16x8 x1 = maddubs(cd, mul_20);
			i16x8 x2 = maddubs(ef, mul_51);
			return x0 + x1 + x2;
		}
		static always_inline u8x16 weight_add8(u8x16 q, u8x16 p, i8x16 w, i16x8 o, i64x2 logWD) {
			i16x8 x0 = adds16(maddubs(unpacklo8(q, p), w), o);
			i16x8 x1 = adds16(maddubs(unpackhi8(q, p), w), o);
			return packus16(shr16(x0, logWD), shr16(x1, logWD));
		}
	#else
		// for SSE2 shl/shr we rely on defined behaviour for out-of-bounds shifts
		static always_inline i8x16 shl(i8x16 a, int i) {
			i *= 8;
			u64x2 lo = shlc(a, 8);
			return _mm_sll_epi64(a, (i32x4){i}) | _mm_srl_epi64(lo, (i32x4){64 - i}) | _mm_sll_epi64(lo, (i32x4){i - 64});
		}
		static always_inline i8x16 shr(i8x16 a, int i) {
			i *= 8;
			u64x2 hi = shrc(a, 8);
			return _mm_srl_epi64(a, (i32x4){i}) | _mm_sll_epi64(hi, (i32x4){64 - i}) | _mm_srl_epi64(hi, (i32x4){i - 64});
		}
		static always_inline i8x16 shufflez(i8x16 a, i8x16 m) { return shuffle(a, m) & ~(0 > m); }
		static i8x16 shuffle2(const i8x16 *p, i8x16 m) {
			union { int8_t q[16]; i8x16 v; } _m = {.v = m & 31};
			for (int i = 0; i < 16; i++)
				_m.q[i] = ((int8_t *)p)[_m.q[i]];
			return _m.v;
		}
		static always_inline i8x16 shufflez2(const i8x16 *p, i8x16 m) { return shuffle2(p, m) & ~(0 > m); }
		static i8x16 shuffle3(const i8x16 *p, i8x16 m) {
			union { int8_t q[16]; i8x16 v; } _m = {.v = umin8(m, set8(47))};
			for (int i = 0; i < 16; i++)
				_m.q[i] = ((int8_t *)p)[_m.q[i]];
			return _m.v;
		}
		static always_inline i16x8 sixtapU8(i8x16 ab, i8x16 cd, i8x16 ef) {
			i16x8 a = (u16x8)ab << 8 >> 8;
			i16x8 b = (u16x8)ab >> 8;
			i16x8 c = (u16x8)cd << 8 >> 8;
			i16x8 d = (u16x8)cd >> 8;
			i16x8 e = (u16x8)ef << 8 >> 8;
			i16x8 f = (u16x8)ef >> 8;
			return a + (((c + d) << 2) -b - e) * 5 + f;
		}
		static always_inline u8x16 weight_add8(u8x16 q, u8x16 p, i8x16 w, i16x8 o, i64x2 logWD) {
			i8x16 z = {};
			i16x8 wq = (i16x8)w << 8 >> 8;
			i16x8 wp = (i16x8)w >> 8;
			i16x8 x0 = adds16(adds16((i16x8)unpacklo8(q, z) * wq, (i16x8)unpacklo8(p, z) * wp), o);
			i16x8 x1 = adds16(adds16((i16x8)unpackhi8(q, z) * wq, (i16x8)unpackhi8(p, z) * wp), o);
			return packus16(shr16(x0, logWD), shr16(x1, logWD));
		}
	#endif
	#ifdef __BMI2__ // FIXME and not AMD pre-Zen3
		static always_inline int extract_neighbours(unsigned f) { return _pext_u32(f, 0x27); }
		static always_inline int mvd_flags2ref_idx(unsigned f) { return _pext_u32(f, 0x11111111); }
	#else
		static always_inline int extract_neighbours(unsigned f) {
			return (f & 7) | (f >> 2 & 8);
		}
		static always_inline int mvd_flags2ref_idx(unsigned f) {
			int a = f & 0x11111111;
			int b = a | a >> 3;
			int c = b | b >> 6;
			return (c & 0xf) | (c >> 12 & 0xf0);
		}
	#endif
#else // add other architectures here
	#error "Add -msse2 or more recent"
#endif



/**
 * Function declarations are put in a single block here instead of .h files
 * because they are so few.
 */

// helper functions
static always_inline int min(int a, int b) { return (a < b) ? a : b; }
static always_inline int max(int a, int b) { return (a > b) ? a : b; }
static always_inline int clip3(int a, int b, int c) { return min(max(c, a), b); }
static always_inline unsigned umin(unsigned a, unsigned b) { return (a < b) ? a : b; }
static always_inline unsigned umax(unsigned a, unsigned b) { return (a > b) ? a : b; }
static always_inline i16x8 median16(i16x8 a, i16x8 b, i16x8 c) {
	return max16(min16(max16(a, b), c), min16(a, b));
}
static always_inline i8x16 pack_absMvd(i16x8 a) {
	i16x8 x = abs16(shuffle32(a, 0, 0, 0, 0));
	return packus16(x, x);
}
static always_inline unsigned depended_frames(Edge264Decoder *dec) {
	i32x4 a = dec->task_dependencies_v[0] | dec->task_dependencies_v[1] |
	          dec->task_dependencies_v[2] | dec->task_dependencies_v[3];
	i32x4 b = a | shuffle32(a, 2, 3, 0, 1);
	i32x4 c = b | shuffle32(b, 1, 0, 3, 2);
	return c[0];
}
static always_inline unsigned ready_frames(Edge264Decoder *c) {
	i32x4 last = set32(INT_MAX);
	i16x8 a = packs32(c->next_deblock_addr_v[0] == last, c->next_deblock_addr_v[1] == last);
	i16x8 b = packs32(c->next_deblock_addr_v[2] == last, c->next_deblock_addr_v[3] == last);
	i16x8 d = packs32(c->next_deblock_addr_v[4] == last, c->next_deblock_addr_v[5] == last);
	i16x8 e = packs32(c->next_deblock_addr_v[6] == last, c->next_deblock_addr_v[7] == last);
	return movemask(packs16(a, b)) | movemask(packs16(d, e)) << 16;
}
static always_inline unsigned ready_tasks(Edge264Decoder *c) {
	i32x4 rf = set32(ready_frames(c));
	i32x4 a = (i32x4)_mm_andnot_si128(rf, c->task_dependencies_v[0]) == 0;
	i32x4 b = (i32x4)_mm_andnot_si128(rf, c->task_dependencies_v[1]) == 0;
	i32x4 d = (i32x4)_mm_andnot_si128(rf, c->task_dependencies_v[2]) == 0;
	i32x4 e = (i32x4)_mm_andnot_si128(rf, c->task_dependencies_v[3]) == 0;
	return c->pending_tasks & movemask(packs16(packs32(a, b), packs32(d, e)));
}

// edge264_bitstream.c
static inline size_t get_bytes(Edge264GetBits *gb, int nbytes);
static noinline int refill(Edge264GetBits *gb, int ret);
static noinline int get_u1(Edge264GetBits *gb);
static noinline unsigned get_uv(Edge264GetBits *gb, unsigned v);
static noinline unsigned get_ue16(Edge264GetBits *gb, unsigned upper);
static noinline int get_se16(Edge264GetBits *gb, int lower, int upper);
#if SIZE_BIT == 32
	static noinline unsigned get_ue32(Edge264GetBits *gb, unsigned upper);
	static noinline int get_se32(Edge264GetBits *gb, int lower, int upper);
#else
	#define get_ue32 get_ue16
	#define get_se32 get_se16
#endif
static noinline int get_ae(Edge264Context *ctx, int ctxIdx);
static inline int get_bypass(Edge264Context *ctx);
static int cabac_start(Edge264Context *ctx);
static int cabac_terminate(Edge264Context *ctx);
static void cabac_init(Edge264Context *ctx);

// edge264_deblock.c
static noinline void deblock_mb(Edge264Context *ctx);

// edge264_inter.c
static void noinline decode_inter(Edge264Context *ctx, int i, int w, int h);

// edge264_intra.c
static void noinline _decode_intra4x4(int mode, uint8_t *px1, size_t stride, ssize_t nstride, i16x8 clip, i8x16 zero);
static void noinline _decode_intra8x8(int mode, uint8_t *px0, uint8_t *px7, size_t stride, ssize_t nstride, i16x8 clip);
static void noinline _decode_intra16x16(int mode, uint8_t *px0, uint8_t *px7, uint8_t *pxE, size_t stride, ssize_t nstride, i16x8 clip);
static void noinline _decode_intraChroma(int mode, uint8_t *Cb0, uint8_t *Cb7, uint8_t *Cr0, uint8_t *Cr7, size_t stride, ssize_t nstride, i16x8 clip);
#define decode_intra4x4(ctx, mode, samples, stride, iYCbCr) _decode_intra4x4(mode, (samples) + (stride), stride, -(stride), ctx->t.samples_clip_v[iYCbCr], (i8x16){})
#define decode_intra8x8(ctx, mode, samples, stride, iYCbCr) _decode_intra8x8(mode, samples, (samples) + (stride) * 7, stride, -(stride), ctx->t.samples_clip_v[iYCbCr])
#define decode_intra16x16(ctx, mode, samples, stride, iYCbCr) _decode_intra16x16(mode, samples, (samples) + (stride) * 7, (samples) + (stride) * 14, stride, -(stride), ctx->t.samples_clip_v[iYCbCr])
#define decode_intraChroma(ctx, mode, samplesCb, samplesCr, stride) _decode_intraChroma(mode, samplesCb, (samplesCb) + (stride) * 7, samplesCr, (samplesCr) + (stride) * 7, stride, -(stride), ctx->t.samples_clip_v[1])

// edge264_mvpred.c
static inline void decode_inter_16x16(Edge264Context *ctx, i16x8 mvd, int lx);
static inline void decode_inter_8x16_left(Edge264Context *ctx, i16x8 mvd, int lx);
static inline void decode_inter_8x16_right(Edge264Context *ctx, i16x8 mvd, int lx);
static inline void decode_inter_16x8_top(Edge264Context *ctx, i16x8 mvd, int lx);
static inline void decode_inter_16x8_bottom(Edge264Context *ctx, i16x8 mvd, int lx);
static noinline void decode_direct_mv_pred(Edge264Context *ctx, unsigned direct_mask);

// edge264_residual.c
static void noinline add_idct4x4(Edge264Context *ctx, int iYCbCr, int qP, i8x16 wS, int DCidx, uint8_t *samples);
static void noinline add_dc4x4(Edge264Context *ctx, int iYCbCr, int DCidx, uint8_t *samples);
static void noinline add_idct8x8(Edge264Context *ctx, int iYCbCr, uint8_t *samples);
static void noinline transform_dc4x4(Edge264Context *ctx, int iYCbCr);
static void noinline transform_dc2x2(Edge264Context *ctx);

// edge264_slice.c (compiled 2x)
static void parse_slice_data_cavlc(Edge264Context *ctx);
static void parse_slice_data_cabac(Edge264Context *ctx);

// edge264_headers.c (compiled 3x)
void *worker_loop(Edge264Decoder *d);
void *worker_loop_v2(Edge264Decoder *d);
void *worker_loop_v3(Edge264Decoder *d);
int parse_slice_layer_without_partitioning(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg);
int parse_slice_layer_without_partitioning_v2(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg);
int parse_slice_layer_without_partitioning_v3(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg);
int parse_pic_parameter_set(Edge264Decoder *dec, int non_blocking,  void(*free_cb)(void*,int), void *free_arg);
int parse_pic_parameter_set_v2(Edge264Decoder *dec, int non_blocking,  void(*free_cb)(void*,int), void *free_arg);
int parse_pic_parameter_set_v3(Edge264Decoder *dec, int non_blocking,  void(*free_cb)(void*,int), void *free_arg);
int parse_seq_parameter_set(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg);
int parse_seq_parameter_set_v2(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg);
int parse_seq_parameter_set_v3(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg);
int parse_seq_parameter_set_extension(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg);
int parse_seq_parameter_set_extension_v2(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg);
int parse_seq_parameter_set_extension_v3(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg);

#endif
