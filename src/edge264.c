/** MAYDO:
 * _ receiving a different SPS should reset SSPS
 * _ Replace P and INIT_P with PX versions
 * _ try to optimize shld with extr on ARM when shift is constant
 * _ add support for https://gcc.gnu.org/wiki/Visibility ?
 * _ swap numbers and text in header logging (ex: 7 (Sequence Parameter Set)) to facilitate automated extraction
 * _ Try to replace errno_on_fail with a mechanism to manage alloc'ed frames with 2 parameters preferred_num_frames and max_num_frames?
 * _ Plugins
 * 	_ Read https://tldp.org/HOWTO/Program-Library-HOWTO/shared-libraries.html
 * 	_ Make a test target that builds locally and runs edge264_test
 * 	_ Make a default target that builds locally
 * 	_ Make a install target that installs on host machine
 * 	_ Make a uninstall target
 * 	_ Add virtualization support and update the release target to test all cross-compiled files
 * _ Multithreading
 * 	_ pass thread id in alignment of context pointer, along with aligned_alloc to guarantee enough thread space
 *    _ Fix bunny.264 looping on unavailable buffer in multithreading
 * 	_ add an option to forbid slice threading, to make redundant slices reliable
 * 	_ measure the time it takes to decode each type of slice
 * 	_ initialize next_deblock_idc at context_init rather than task to catch the latest nda value
 * 	_ make tasks start without waiting for availabilities, and wait inside all separate mv parsers
 * 	_ progressively replace waits with per-mb waits
 * 	_ limit n_threads and inferred CPUs to 16
 * 	_ remove taskPic now to remove a source of false sharing
 * 	_ try to improve parallel decoding of frames with disable_deblocking_idc==2
 * 	_ Update DPB availability checks to take deps into account, and make sure we wait until there is a frame ready before returning -2
 * 	_ Add currPic to task_dependencies
 * 	_ Add a mask of pending tasks
 * 	_ Add an option for number of threads and max_frame_delay
 * 	_ Give back frame delay when returning a frame
 * 	_ Add a notion of reference ownership for CPB pointer, and return last CPB byte rather than next start code
 * 	_ Create a pool of worker threads and make them consume tasks as soon as possible
 * 	_ Add a loop when picking an available slot to wait until enough tasks are done
 * 	_ Add a wait when looking for an available task slot
 * 	_ Create a single worker thread and use it to decode each slice
 * 	_ Add debug output to signal start and end of worker assignment
 * 	_ add an option to store N more frames, to tolerate lags in process scheduling
 * 	_ make reference dependencies be waited in each mb with conditions on minimum values of next_deblock_addr, like ffmpeg does
 * 	_ Windows fallback functions
 * 	_ Switch back convention to never allow CPB past end because of risk of pointer overflow!
 * 	_ Change edge264_test to avoid counting mmap time in benchmark (check if ffmpeg does it too to be fair)
 * 	_ fix segfault on videos/geek.264, mvc.264 and shrinkage.264
 * _ Fuzzing and bug hunting
 * 	_ reducing DPB size to 16 entries and int16 should never crash
 * 	_ find a mechanism like assert to exit on coding error, but that can be caught by front end software
 * 	_ check with clang-tidy
 * 	_ check with CScout
 * 	_ Protect again for possibility of not enough ref frames in RefPicList
 * 	_ fuzz with H26Forge
 * 	_ replace calloc with malloc+memset(127), determine a policy for ensuring the validity of variables over time, and setup a solver (ex. KLEE, Crest, Triton) to test their intervals
 * 	_ check that gaps in frame_num cannot result in using NULL buffers in inter pred
 * 	_ Review the entire inlining scheme (in particular bitstream functions)
 * 	_ make a debugging pass by looking at shall/"shall not" clauses in spec and checking that we are robust against each violation
 * 	_ limit width to 16384 to prevent overflow in stride with 16bit+field (check also relative offsets to mbD!)
 * 	_ check on https://kodi.wiki/view/Samples#3D_Test_Clips
 * _ Optimizations
 * 	_ improve column loads in Intra for SSE4.1 by using PINSRB
 * 	_ reduce binary size of Inter by replacing macros with selection masks
 * 	_ Setup AMD CodeXL IBS to monitor pipeline stalls and cache misses
 * 	_ Do GCC/Clang use different call conventions for static functions? If not, do it!
 * 	_ set COLD and hot functions
 * 	_ Reintroduce the intra to residual passing of predicted samples in registers
 * 	_ try vectorizing loops on get_ae with movemask trick, starting with residual block parsing
 * 	_ Group dec fields by frequency of accesses and force them manually into L1/L2/L3
 * 	_ store Cb & Cr by interleaving rows instead of separate planes (check it does not overflow stride for future 4:2:2)
 * 	_ try combining clang and gcc over decoding and parsing
 * 	_ try reordering cases in intra to gather those that may belong to same active cache lines at any time (ex. all executed when top row is absent)
 * _ Documentation
 * 	_ add an FAQ with (1) how to optimize latency, (2) what can be removed from stream without issue, (3) how to finish a frame with an AUD
 * 	_ Don't assume that stride_Y is multiple of stride_C, and give alignment of pointers
 * _ add an option to get_frame to poll without consuming
 * _ when implementing fields and MBAFF, keep the same pic coding struct (no FLD/AFRM) and just add mb_field_decoding_flag
 */

/** Notes:
 * _ to benchmark ffmpeg: ffmpeg -hide_banner -benchmark -threads 1 -i video.264 -f null -
 */

#include "edge264_internal.h"

#include "edge264_headers.c"



const uint8_t *edge264_find_start_code(const uint8_t *buf, const uint8_t *end, int four_byte) {
	four_byte = four_byte != 0;
	buf += four_byte;
	if ((intptr_t)(end - buf) <= 0)
		return end;
	const i8x16 *p = (i8x16 *)((uintptr_t)buf & -16);
	i8x16 zero = {};
	i8x16 c1 = set8(1);
	i8x16 lo0 = {};
	i8x16 v = *p;
	i8x16 hi0 = (v == zero) & shlv128(set8(-1), (uintptr_t)buf & 15);
	while (1) {
		#if defined(__SSE2__)
			unsigned m = movemask(shrd128(lo0, hi0, 14) & shrd128(lo0, hi0, 15) & (v == c1));
			if (m) {
				const uint8_t *res = (uint8_t *)p - 2 - four_byte + __builtin_ctz(m);
				if (*res == 0)
					return minp(res, end);
			}
		#elif defined(__ARM_NEON)
			uint64_t m = (uint64_t)vshrn_n_u16(shrd128(lo0, hi0, 14) & shrd128(lo0, hi0, 15) & (v == c1), 4);
			if (m) {
				const uint8_t *res = (uint8_t *)p - 2 - four_byte + (__builtin_ctzll(m) >> 2);
				if (*res == 0)
					return minp(res, end);
			}
		#endif
		if ((intptr_t)(end - (uint8_t *)++p) <= 0)
			return end;
		lo0 = hi0;
		hi0 = ((v = *p) == zero);
	}
}



static void internal_alloc(void **samples, unsigned samples_size, void **mbs, unsigned mbs_size, int errno_on_fail, void *alloc_arg) {
	*samples = malloc(samples_size + mbs_size);
	*mbs = *samples + samples_size;
}

static void internal_free(void *samples, void *mbs, void *alloc_arg) {
	free(samples);
}

// These could be optimized into log/nolog functions but be less simple then
static int ignore_NAL(Edge264Decoder *dec, int non_blocking, Edge264UnrefCb unref_cb, void *unref_arg) {
	if (dec->log_cb) {
		dec->log_pos = 0;
		dec->log_cb(dec->log_buf, dec->log_arg);
	}
	return 0;
}

static int unsupp_NAL(Edge264Decoder *dec, int non_blocking, Edge264UnrefCb unref_cb, void *unref_arg) {
	if (dec->log_cb) {
		dec->log_pos = 0;
		dec->log_cb(dec->log_buf, dec->log_arg);
	}
	return ENOTSUP;
}

static int parse_end_of_sequence(Edge264Decoder *dec, int non_blocking, Edge264UnrefCb unref_cb, void *unref_arg) {
	if (dec->log_cb) {
		dec->log_pos = 0;
		dec->log_cb(dec->log_buf, dec->log_arg);
	}
	int ret = bump_all_frames(dec, non_blocking);
	if (ret)
		return ret;
	clear_decoder(dec);
	return 0;
}



Edge264Decoder *edge264_alloc(int n_threads, Edge264LogCb log_cb, void *log_arg, int log_mbs, Edge264AllocCb alloc_cb, Edge264FreeCb free_cb, void *alloc_arg) {
	Edge264Decoder *dec = calloc(1, sizeof(Edge264Decoder));
	if (dec == NULL)
		return NULL;
	dec->currPic = dec->basePic = -1;
	dec->PrevRefFrameNum[0] = dec->PrevRefFrameNum[1] = dec->prevFrameId = -1;
	dec->taskPics_v = dec->get_frame_queue_v[0] = dec->get_frame_queue_v[1] = set8(-1);
	dec->n_threads = n_threads;
	dec->log_cb = log_cb;
	dec->log_arg = log_arg;
	dec->alloc_cb = alloc_cb && free_cb ? alloc_cb : internal_alloc;
	dec->free_cb = alloc_cb && free_cb ? free_cb : internal_free;
	dec->alloc_arg = alloc_arg;
	
	// select parser functions based on CPU capabilities and logs mode
	dec->worker_loop = ADD_VARIANT(worker_loop);
	for (int i = 0; i < 32; i++)
		dec->parse_nal_unit[i] = unsupp_NAL;
	dec->parse_nal_unit[1] = dec->parse_nal_unit[5] = ADD_VARIANT(parse_slice_layer_without_partitioning);
	dec->parse_nal_unit[6] = dec->parse_nal_unit[9] = dec->parse_nal_unit[11] = dec->parse_nal_unit[12] = ignore_NAL;
	dec->parse_nal_unit[7] = dec->parse_nal_unit[15] = ADD_VARIANT(parse_seq_parameter_set);
	dec->parse_nal_unit[8] = ADD_VARIANT(parse_pic_parameter_set);
	dec->parse_nal_unit[10] = parse_end_of_sequence;
	dec->parse_nal_unit[14] = dec->parse_nal_unit[20] = ADD_VARIANT(parse_nal_unit_header_extension);
	#if defined(HAS_X86_64_V2) || defined(HAS_X86_64_V3)
		__builtin_cpu_init();
	#endif
	#ifdef HAS_X86_64_V2
		// macOS's clang does not support x86-64-v3/v2 feature string yet
		if (__builtin_cpu_supports("popcnt") &&
			__builtin_cpu_supports("sse3") &&
			__builtin_cpu_supports("sse4.1") &&
			__builtin_cpu_supports("sse4.2") &&
			__builtin_cpu_supports("ssse3")) {
			dec->worker_loop = worker_loop_v2;
			dec->parse_nal_unit[1] = dec->parse_nal_unit[5] = parse_slice_layer_without_partitioning_v2;
			dec->parse_nal_unit[7] = dec->parse_nal_unit[15] = parse_seq_parameter_set_v2;
			dec->parse_nal_unit[8] = parse_pic_parameter_set_v2;
			dec->parse_nal_unit[14] = dec->parse_nal_unit[20] = parse_nal_unit_header_extension_v2;
		}
	#endif
	#ifdef HAS_X86_64_V3
		if (__builtin_cpu_supports("avx") &&
			__builtin_cpu_supports("avx2") &&
			__builtin_cpu_supports("bmi") &&
			__builtin_cpu_supports("bmi2") &&
			__builtin_cpu_supports("fma")) {
			dec->worker_loop = worker_loop_v3;
			dec->parse_nal_unit[1] = dec->parse_nal_unit[5] = parse_slice_layer_without_partitioning_v3;
			dec->parse_nal_unit[7] = dec->parse_nal_unit[15] = parse_seq_parameter_set_v3;
			dec->parse_nal_unit[8] = parse_pic_parameter_set_v3;
			dec->parse_nal_unit[14] = dec->parse_nal_unit[20] = parse_nal_unit_header_extension_v3;
		}
	#endif
	#ifdef HAS_LOGS
		if (log_cb) {
			dec->parse_nal_unit[1] = dec->parse_nal_unit[5] = parse_slice_layer_without_partitioning_log;
			dec->parse_nal_unit[6] = parse_sei_log;
			dec->parse_nal_unit[7] = dec->parse_nal_unit[15] = parse_seq_parameter_set_log;
			dec->parse_nal_unit[8] = parse_pic_parameter_set_log;
			dec->parse_nal_unit[9] = parse_access_unit_delimiter_log;
			dec->parse_nal_unit[14] = dec->parse_nal_unit[20] = parse_nal_unit_header_extension_log;
			if (log_mbs)
				dec->worker_loop = worker_loop_log;
		}
	#else
		if (log_cb)
			return free(dec), NULL;
	#endif
	
	// get the number of logical cores if requested
	if (n_threads < 0) {
		#ifdef _WIN32
			int n_cpus = atoi(getenv("NUMBER_OF_PROCESSORS"));
		#else
			int n_cpus = sysconf(_SC_NPROCESSORS_ONLN);
		#endif
		n_threads = min(n_cpus, 16);
	}
	
	// if multithreading is disabled we are done, otherwise initialize all
	if (n_threads == 0)
		return dec;
	if (pthread_mutex_init(&dec->lock, NULL) == 0) {
		if (pthread_cond_init(&dec->task_ready, NULL) == 0) {
			if (pthread_cond_init(&dec->task_progress, NULL) == 0) {
				if (pthread_cond_init(&dec->task_complete, NULL) == 0) {
					int i = 0;
					while (i < n_threads && pthread_create(&dec->threads[i], NULL, (void*(*)(void*))dec->worker_loop, dec) == 0)
						i++;
					if (i == n_threads) {
						return dec;
					}
					while (i-- > 0)
						pthread_cancel(dec->threads[i]);
					pthread_cond_destroy(&dec->task_complete);
				}
				pthread_cond_destroy(&dec->task_progress);
			}
			pthread_cond_destroy(&dec->task_ready);
		}
		pthread_mutex_destroy(&dec->lock);
	}
	free(dec);
	return NULL;
}



void edge264_flush(Edge264Decoder *dec) {
	if (dec == NULL)
		return;
	if (dec->n_threads)
		pthread_mutex_lock(&dec->lock);
	flush_frames(dec);
	clear_decoder(dec);
	if (dec->n_threads)
		pthread_mutex_unlock(&dec->lock);
}



void edge264_free(Edge264Decoder **pdec) {
	Edge264Decoder *dec;
	if (pdec != NULL && (dec = *pdec) != NULL) {
		*pdec = NULL;
		if (dec->n_threads) {
			for (int i = 0; i < dec->n_threads; i++)
				pthread_cancel(dec->threads[i]);
			pthread_mutex_destroy(&dec->lock);
			pthread_cond_destroy(&dec->task_ready);
			pthread_cond_destroy(&dec->task_progress);
			pthread_cond_destroy(&dec->task_complete);
		}
		for (int i = 0; i < 32; i++) {
			if (dec->samples_buffers[i] != NULL)
				dec->free_cb(dec->samples_buffers[i], dec->mb_buffers[i], dec->alloc_arg);
		}
		free(dec);
	}
}



/**
 * Maximum buffer size is 2^(SIZE_BIT-1)-1, and pointer comparisons are coded
 * to allow wrapping around memory, so the buffer may be close to end of memory
 * without risk.
 */
int edge264_decode_NAL(Edge264Decoder *dec, const uint8_t *buf, const uint8_t *end, int non_blocking, Edge264UnrefCb unref_cb, void *unref_arg, const uint8_t **next_NAL)
{
	static const char * const nal_unit_type_names[32] = {
		[0 ... 31] = "Unknown",
		[1] = "Coded slice of a non-IDR picture",
		[2] = "Coded slice data partition A",
		[3] = "Coded slice data partition B",
		[4] = "Coded slice data partition C",
		[5] = "Coded slice of an IDR picture",
		[6] = "Supplemental enhancement information (SEI)",
		[7] = "Sequence parameter set",
		[8] = "Picture parameter set",
		[9] = "Access unit delimiter",
		[10] = "End of sequence",
		[11] = "End of stream",
		[12] = "Filler data",
		[13] = "Sequence parameter set extension",
		[14] = "Prefix NAL unit",
		[15] = "Subset sequence parameter set",
		[16] = "Depth parameter set",
		[19] = "Coded slice of an auxiliary coded picture without partitioning",
		[20] = "Coded slice extension",
		[21] = "Coded slice extension for a depth view component or a 3D-AVC texture view component",
	};
	
	// initial checks before parsing
	if (dec == NULL || (buf == NULL && end != NULL))
		return EINVAL;
	if (dec->n_threads)
		pthread_mutex_lock(&dec->lock);
	
	// there has to be enough buffer space for any NAL to flush the entire DPB
	int queued0 = __builtin_ctz(movemask(dec->get_frame_queue_v[0]) | 1 << 16);
	int queued1 = __builtin_ctz(movemask(dec->get_frame_queue_v[1]) | 1 << 16);
	int bumpable = max(1, __builtin_popcount(dec->to_get_frames & ~dec->output_frames));
	if (queued0 + queued1 + bumpable > 16) {
		if (dec->n_threads)
			pthread_mutex_unlock(&dec->lock);
		return ENOBUFS;
	}
	
	// bump all frames at the end of buffer
	if (__builtin_expect((intptr_t)(end - buf) <= 0, 0)) {
		int ret = bump_all_frames(dec, non_blocking);
		if (dec->n_threads)
			pthread_mutex_unlock(&dec->lock);
		return ret ?: ENODATA;
	}
	
	// NAL byte header
	dec->nal_ref_idc = buf[0] >> 5;
	dec->nal_unit_type = buf[0] & 0x1f;
	Parser parser = dec->parse_nal_unit[dec->nal_unit_type];
	if (dec->log_cb) {
		dec->log_pos = snprintf(dec->log_buf, sizeof(dec->log_buf),
			"\n- nal_ref_idc: %u\n"
			"  nal_unit_type: %u # %s%s\n",
			dec->nal_ref_idc,
			dec->nal_unit_type, nal_unit_type_names[dec->nal_unit_type], unsup_if(!parser));
	}
	
	// prefill the bitstream cache
	if ((intptr_t)(end - buf) >= 2) {
		dec->gb.msb_cache = (size_t)buf[1] << (SIZE_BIT - 8) | (size_t)1 << (SIZE_BIT - 9);
		dec->gb.CPB = buf + 2;
	} else {
		dec->gb.msb_cache = (size_t)1 << (SIZE_BIT - 1);
		dec->gb.CPB = buf + 1;
	}
	dec->gb.lsb_cache = 0;
	dec->gb.end = end;
	refill(&dec->gb, 0);
	int ret = parser(dec, non_blocking, unref_cb, unref_arg);
	// printf("nal_unit_type=%d, ret=%d\n\n", dec->nal_unit_type, ret);
	
	// for 0, ENOTSUP and EBADMSG we may free or advance the buffer pointer
	if (ret == 0 || ret == ENOTSUP || ret == EBADMSG) {
		if (unref_cb && !(ret == 0 && 1048610 & 1 << dec->nal_unit_type)) // 1, 5 or 20
			unref_cb(ret, unref_arg);
		// end may have been set to the next start code thanks to escape code detection in get_bytes
		if (next_NAL)
			*next_NAL = edge264_find_start_code(minp(dec->gb.CPB - 2, dec->gb.end), end, 0) + 3;
	}
	if (dec->n_threads)
		pthread_mutex_unlock(&dec->lock);
	return ret;
}



/**
 * By default all frames with POC lower or equal with the last non-reference
 * picture or lower than the last IDR picture are considered for output.
 * This function will consider all frames instead if either:
 * _ there are more frames to output than max_num_reorder_frames
 * _ there is no empty slot for the next frame
 */
int edge264_get_frame(Edge264Decoder *dec, Edge264Frame *out, int borrow) {
	if (dec == NULL || out == NULL)
		return EINVAL;
	if (dec->n_threads)
		pthread_mutex_lock(&dec->lock);
	int idx0 = __builtin_ctz(movemask(dec->get_frame_queue_v[0]) | 1 << 16) - 1;
	int idx1 = __builtin_ctz(movemask(dec->get_frame_queue_v[1]) | 1 << 16) - 1;
	int pic0, pic1, res = ENOMSG;
	if (idx0 >= 0 && dec->next_deblock_addr[pic0 = dec->get_frame_queue[0][idx0]] == INT_MAX &&
		(dec->ssps.BitDepth_Y == 0 || (idx1 >= 0 && dec->next_deblock_addr[pic1 = dec->get_frame_queue[1][idx1]] == INT_MAX))) {
		dec->get_frame_queue[0][idx0] = -1;
		memcpy(out, &dec->out, sizeof(*out)); // GCC-14 crashes on dec->out = format
		int top = dec->out.frame_crop_offsets[0];
		int left = dec->out.frame_crop_offsets[3];
		int offY = top * dec->out.stride_Y + (dec->out.bit_depth_Y == 8 ? left : left << 1);
		int topC = dec->sps.chroma_format_idc == 3 ? top : top >> 1;
		int leftC = dec->sps.chroma_format_idc == 1 ? left >> 1 : left;
		int offC = dec->plane_size_Y + topC * dec->out.stride_C + (dec->out.bit_depth_C == 8 ? leftC : leftC << 1);
		assert(dec->to_get_frames & dec->output_frames & 1 << pic0);
		dec->to_get_frames &= ~(1 << pic0);
		out->samples[0] = dec->samples_buffers[pic0] + offY;
		out->samples[1] = dec->samples_buffers[pic0] + offC;
		out->samples[2] = dec->samples_buffers[pic0] + offC + (dec->out.stride_C >> 1);
		out->FrameId = dec->FrameIds[pic0];
		out->return_arg = (void *)((uintptr_t)1 << pic0);
		if (idx1 >= 0) {
			dec->get_frame_queue[1][idx1] = -1;
			assert(dec->to_get_frames & dec->output_frames & 1 << pic1);
			dec->to_get_frames ^= 1 << pic1;
			out->samples_mvc[0] = dec->samples_buffers[pic1] + offY;
			out->samples_mvc[1] = dec->samples_buffers[pic1] + offC;
			out->samples_mvc[2] = dec->samples_buffers[pic1] + offC + (dec->out.stride_C >> 1);
			out->FrameId_mvc = dec->FrameIds[pic1];
			out->return_arg = (void *)((uintptr_t)1 << pic0 | (uintptr_t)1 << pic1);
		}
		res = 0;
		if (!borrow)
			dec->output_frames &= ~(uintptr_t)out->return_arg;
	}
	if (dec->n_threads)
		pthread_mutex_unlock(&dec->lock);
	return res;
}



void edge264_return_frame(Edge264Decoder *dec, void *return_arg) {
	if (dec != NULL)
		dec->output_frames &= ~(size_t)return_arg;
}



const int8_t cabac_context_init[4][1024][2] __attribute__((aligned(16))) = {{
	{  20, -15}, {   2,  54}, {   3,  74}, {  20, -15}, {   2,  54}, {   3,  74},
	{ -28, 127}, { -23, 104}, {  -6,  53}, {  -1,  54}, {   7,  51}, {   0,   0},
	{   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0},
	{   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0},
	{   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0},
	{   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0},
	{   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0},
	{   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0},
	{   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0},
	{   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0}, {   0,   0},
	{   0,  41}, {   0,  63}, {   0,  63}, {   0,  63}, {  -9,  83}, {   4,  86},
	{   0,  97}, {  -7,  72}, {  13,  41}, {   3,  62}, {   0,  11}, {   1,  55},
	{   0,  69}, { -17, 127}, { -13, 102}, {   0,  82}, {  -7,  74}, { -21, 107},
	{ -27, 127}, { -31, 127}, { -24, 127}, { -18,  95}, { -27, 127}, { -21, 114},
	{ -30, 127}, { -17, 123}, { -12, 115}, { -16, 122}, { -11, 115}, { -12,  63},
	{  -2,  68}, { -15,  84}, { -13, 104}, {  -3,  70}, {  -8,  93}, { -10,  90},
	{ -30, 127}, {  -1,  74}, {  -6,  97}, {  -7,  91}, { -20, 127}, {  -4,  56},
	{  -5,  82}, {  -7,  76}, { -22, 125}, {  -7,  93}, { -11,  87}, {  -3,  77},
	{  -5,  71}, {  -4,  63}, {  -4,  68}, { -12,  84}, {  -7,  62}, {  -7,  65},
	{   8,  61}, {   5,  56}, {  -2,  66}, {   1,  64}, {   0,  61}, {  -2,  78},
	{   1,  50}, {   7,  52}, {  10,  35}, {   0,  44}, {  11,  38}, {   1,  45},
	{   0,  46}, {   5,  44}, {  31,  17}, {   1,  51}, {   7,  50}, {  28,  19},
	{  16,  33}, {  14,  62}, { -13, 108}, { -15, 100}, { -13, 101}, { -13,  91},
	{ -12,  94}, { -10,  88}, { -16,  84}, { -10,  86}, {  -7,  83}, { -13,  87},
	{ -19,  94}, {   1,  70}, {   0,  72}, {  -5,  74}, {  18,  59}, {  -8, 102},
	{ -15, 100}, {   0,  95}, {  -4,  75}, {   2,  72}, { -11,  75}, {  -3,  71},
	{  15,  46}, { -13,  69}, {   0,  62}, {   0,  65}, {  21,  37}, { -15,  72},
	{   9,  57}, {  16,  54}, {   0,  62}, {  12,  72}, {  24,   0}, {  15,   9},
	{   8,  25}, {  13,  18}, {  15,   9}, {  13,  19}, {  10,  37}, {  12,  18},
	{   6,  29}, {  20,  33}, {  15,  30}, {   4,  45}, {   1,  58}, {   0,  62},
	{   7,  61}, {  12,  38}, {  11,  45}, {  15,  39}, {  11,  42}, {  13,  44},
	{  16,  45}, {  12,  41}, {  10,  49}, {  30,  34}, {  18,  42}, {  10,  55},
	{  17,  51}, {  17,  46}, {   0,  89}, {  26, -19}, {  22, -17}, {  26, -17},
	{  30, -25}, {  28, -20}, {  33, -23}, {  37, -27}, {  33, -23}, {  40, -28},
	{  38, -17}, {  33, -11}, {  40, -15}, {  41,  -6}, {  38,   1}, {  41,  17},
	{  30,  -6}, {  27,   3}, {  26,  22}, {  37, -16}, {  35,  -4}, {  38,  -8},
	{  38,  -3}, {  37,   3}, {  38,   5}, {  42,   0}, {  35,  16}, {  39,  22},
	{  14,  48}, {  27,  37}, {  21,  60}, {  12,  68}, {   2,  97}, {  -3,  71},
	{  -6,  42}, {  -5,  50}, {  -3,  54}, {  -2,  62}, {   0,  58}, {   1,  63},
	{  -2,  72}, {  -1,  74}, {  -9,  91}, {  -5,  67}, {  -5,  27}, {  -3,  39},
	{  -2,  44}, {   0,  46}, { -16,  64}, {  -8,  68}, { -10,  78}, {  -6,  77},
	{ -10,  86}, { -12,  92}, { -15,  55}, { -10,  60}, {  -6,  62}, {  -4,  65},
	{ -12,  73}, {  -8,  76}, {  -7,  80}, {  -9,  88}, { -17, 110}, { -11,  97},
	{ -20,  84}, { -11,  79}, {  -6,  73}, {  -4,  74}, { -13,  86}, { -13,  96},
	{ -11,  97}, { -19, 117}, {  -8,  78}, {  -5,  33}, {  -4,  48}, {  -2,  53},
	{  -3,  62}, { -13,  71}, { -10,  79}, { -12,  86}, { -13,  90}, { -14,  97},
	{   0,   0}, {  -6,  93}, {  -6,  84}, {  -8,  79}, {   0,  66}, {  -1,  71},
	{   0,  62}, {  -2,  60}, {  -2,  59}, {  -5,  75}, {  -3,  62}, {  -4,  58},
	{  -9,  66}, {  -1,  79}, {   0,  71}, {   3,  68}, {  10,  44}, {  -7,  62},
	{  15,  36}, {  14,  40}, {  16,  27}, {  12,  29}, {   1,  44}, {  20,  36},
	{  18,  32}, {   5,  42}, {   1,  48}, {  10,  62}, {  17,  46}, {   9,  64},
	{ -12, 104}, { -11,  97}, { -16,  96}, {  -7,  88}, {  -8,  85}, {  -7,  85},
	{  -9,  85}, { -13,  88}, {   4,  66}, {  -3,  77}, {  -3,  76}, {  -6,  76},
	{  10,  58}, {  -1,  76}, {  -1,  83}, {  -7,  99}, { -14,  95}, {   2,  95},
	{   0,  76}, {  -5,  74}, {   0,  70}, { -11,  75}, {   1,  68}, {   0,  65},
	{ -14,  73}, {   3,  62}, {   4,  62}, {  -1,  68}, { -13,  75}, {  11,  55},
	{   5,  64}, {  12,  70}, {  15,   6}, {   6,  19}, {   7,  16}, {  12,  14},
	{  18,  13}, {  13,  11}, {  13,  15}, {  15,  16}, {  12,  23}, {  13,  23},
	{  15,  20}, {  14,  26}, {  14,  44}, {  17,  40}, {  17,  47}, {  24,  17},
	{  21,  21}, {  25,  22}, {  31,  27}, {  22,  29}, {  19,  35}, {  14,  50},
	{  10,  57}, {   7,  63}, {  -2,  77}, {  -4,  82}, {  -3,  94}, {   9,  69},
	{ -12, 109}, {  36, -35}, {  36, -34}, {  32, -26}, {  37, -30}, {  44, -32},
	{  34, -18}, {  34, -15}, {  40, -15}, {  33,  -7}, {  35,  -5}, {  33,   0},
	{  38,   2}, {  33,  13}, {  23,  35}, {  13,  58}, {  29,  -3}, {  26,   0},
	{  22,  30}, {  31,  -7}, {  35, -15}, {  34,  -3}, {  34,   3}, {  36,  -1},
	{  34,   5}, {  32,  11}, {  35,   5}, {  34,  12}, {  39,  11}, {  30,  29},
	{  34,  26}, {  29,  39}, {  19,  66}, {  31,  21}, {  31,  31}, {  25,  50},
	{ -17, 120}, { -20, 112}, { -18, 114}, { -11,  85}, { -15,  92}, { -14,  89},
	{ -26,  71}, { -15,  81}, { -14,  80}, {   0,  68}, { -14,  70}, { -24,  56},
	{ -23,  68}, { -24,  50}, { -11,  74}, {  23, -13}, {  26, -13}, {  40, -15},
	{  49, -14}, {  44,   3}, {  45,   6}, {  44,  34}, {  33,  54}, {  19,  82},
	{  -3,  75}, {  -1,  23}, {   1,  34}, {   1,  43}, {   0,  54}, {  -2,  55},
	{   0,  61}, {   1,  64}, {   0,  68}, {  -9,  92}, { -14, 106}, { -13,  97},
	{ -15,  90}, { -12,  90}, { -18,  88}, { -10,  73}, {  -9,  79}, { -14,  86},
	{ -10,  73}, { -10,  70}, { -10,  69}, {  -5,  66}, {  -9,  64}, {  -5,  58},
	{   2,  59}, {  21, -10}, {  24, -11}, {  28,  -8}, {  28,  -1}, {  29,   3},
	{  29,   9}, {  35,  20}, {  29,  36}, {  14,  67}, { -17, 123}, { -12, 115},
	{ -16, 122}, { -11, 115}, { -12,  63}, {  -2,  68}, { -15,  84}, { -13, 104},
	{  -3,  70}, {  -8,  93}, { -10,  90}, { -30, 127}, { -17, 123}, { -12, 115},
	{ -16, 122}, { -11, 115}, { -12,  63}, {  -2,  68}, { -15,  84}, { -13, 104},
	{  -3,  70}, {  -8,  93}, { -10,  90}, { -30, 127}, {  -7,  93}, { -11,  87},
	{  -3,  77}, {  -5,  71}, {  -4,  63}, {  -4,  68}, { -12,  84}, {  -7,  62},
	{  -7,  65}, {   8,  61}, {   5,  56}, {  -2,  66}, {   1,  64}, {   0,  61},
	{  -2,  78}, {   1,  50}, {   7,  52}, {  10,  35}, {   0,  44}, {  11,  38},
	{   1,  45}, {   0,  46}, {   5,  44}, {  31,  17}, {   1,  51}, {   7,  50},
	{  28,  19}, {  16,  33}, {  14,  62}, { -13, 108}, { -15, 100}, { -13, 101},
	{ -13,  91}, { -12,  94}, { -10,  88}, { -16,  84}, { -10,  86}, {  -7,  83},
	{ -13,  87}, { -19,  94}, {   1,  70}, {   0,  72}, {  -5,  74}, {  18,  59},
	{  -7,  93}, { -11,  87}, {  -3,  77}, {  -5,  71}, {  -4,  63}, {  -4,  68},
	{ -12,  84}, {  -7,  62}, {  -7,  65}, {   8,  61}, {   5,  56}, {  -2,  66},
	{   1,  64}, {   0,  61}, {  -2,  78}, {   1,  50}, {   7,  52}, {  10,  35},
	{   0,  44}, {  11,  38}, {   1,  45}, {   0,  46}, {   5,  44}, {  31,  17},
	{   1,  51}, {   7,  50}, {  28,  19}, {  16,  33}, {  14,  62}, { -13, 108},
	{ -15, 100}, { -13, 101}, { -13,  91}, { -12,  94}, { -10,  88}, { -16,  84},
	{ -10,  86}, {  -7,  83}, { -13,  87}, { -19,  94}, {   1,  70}, {   0,  72},
	{  -5,  74}, {  18,  59}, {  24,   0}, {  15,   9}, {   8,  25}, {  13,  18},
	{  15,   9}, {  13,  19}, {  10,  37}, {  12,  18}, {   6,  29}, {  20,  33},
	{  15,  30}, {   4,  45}, {   1,  58}, {   0,  62}, {   7,  61}, {  12,  38},
	{  11,  45}, {  15,  39}, {  11,  42}, {  13,  44}, {  16,  45}, {  12,  41},
	{  10,  49}, {  30,  34}, {  18,  42}, {  10,  55}, {  17,  51}, {  17,  46},
	{   0,  89}, {  26, -19}, {  22, -17}, {  26, -17}, {  30, -25}, {  28, -20},
	{  33, -23}, {  37, -27}, {  33, -23}, {  40, -28}, {  38, -17}, {  33, -11},
	{  40, -15}, {  41,  -6}, {  38,   1}, {  41,  17}, {  24,   0}, {  15,   9},
	{   8,  25}, {  13,  18}, {  15,   9}, {  13,  19}, {  10,  37}, {  12,  18},
	{   6,  29}, {  20,  33}, {  15,  30}, {   4,  45}, {   1,  58}, {   0,  62},
	{   7,  61}, {  12,  38}, {  11,  45}, {  15,  39}, {  11,  42}, {  13,  44},
	{  16,  45}, {  12,  41}, {  10,  49}, {  30,  34}, {  18,  42}, {  10,  55},
	{  17,  51}, {  17,  46}, {   0,  89}, {  26, -19}, {  22, -17}, {  26, -17},
	{  30, -25}, {  28, -20}, {  33, -23}, {  37, -27}, {  33, -23}, {  40, -28},
	{  38, -17}, {  33, -11}, {  40, -15}, {  41,  -6}, {  38,   1}, {  41,  17},
	{ -17, 120}, { -20, 112}, { -18, 114}, { -11,  85}, { -15,  92}, { -14,  89},
	{ -26,  71}, { -15,  81}, { -14,  80}, {   0,  68}, { -14,  70}, { -24,  56},
	{ -23,  68}, { -24,  50}, { -11,  74}, { -14, 106}, { -13,  97}, { -15,  90},
	{ -12,  90}, { -18,  88}, { -10,  73}, {  -9,  79}, { -14,  86}, { -10,  73},
	{ -10,  70}, { -10,  69}, {  -5,  66}, {  -9,  64}, {  -5,  58}, {   2,  59},
	{  23, -13}, {  26, -13}, {  40, -15}, {  49, -14}, {  44,   3}, {  45,   6},
	{  44,  34}, {  33,  54}, {  19,  82}, {  21, -10}, {  24, -11}, {  28,  -8},
	{  28,  -1}, {  29,   3}, {  29,   9}, {  35,  20}, {  29,  36}, {  14,  67},
	{  -3,  75}, {  -1,  23}, {   1,  34}, {   1,  43}, {   0,  54}, {  -2,  55},
	{   0,  61}, {   1,  64}, {   0,  68}, {  -9,  92}, { -17, 120}, { -20, 112},
	{ -18, 114}, { -11,  85}, { -15,  92}, { -14,  89}, { -26,  71}, { -15,  81},
	{ -14,  80}, {   0,  68}, { -14,  70}, { -24,  56}, { -23,  68}, { -24,  50},
	{ -11,  74}, { -14, 106}, { -13,  97}, { -15,  90}, { -12,  90}, { -18,  88},
	{ -10,  73}, {  -9,  79}, { -14,  86}, { -10,  73}, { -10,  70}, { -10,  69},
	{  -5,  66}, {  -9,  64}, {  -5,  58}, {   2,  59}, {  23, -13}, {  26, -13},
	{  40, -15}, {  49, -14}, {  44,   3}, {  45,   6}, {  44,  34}, {  33,  54},
	{  19,  82}, {  21, -10}, {  24, -11}, {  28,  -8}, {  28,  -1}, {  29,   3},
	{  29,   9}, {  35,  20}, {  29,  36}, {  14,  67}, {  -3,  75}, {  -1,  23},
	{   1,  34}, {   1,  43}, {   0,  54}, {  -2,  55}, {   0,  61}, {   1,  64},
	{   0,  68}, {  -9,  92}, {  -6,  93}, {  -6,  84}, {  -8,  79}, {   0,  66},
	{  -1,  71}, {   0,  62}, {  -2,  60}, {  -2,  59}, {  -5,  75}, {  -3,  62},
	{  -4,  58}, {  -9,  66}, {  -1,  79}, {   0,  71}, {   3,  68}, {  10,  44},
	{  -7,  62}, {  15,  36}, {  14,  40}, {  16,  27}, {  12,  29}, {   1,  44},
	{  20,  36}, {  18,  32}, {   5,  42}, {   1,  48}, {  10,  62}, {  17,  46},
	{   9,  64}, { -12, 104}, { -11,  97}, { -16,  96}, {  -7,  88}, {  -8,  85},
	{  -7,  85}, {  -9,  85}, { -13,  88}, {   4,  66}, {  -3,  77}, {  -3,  76},
	{  -6,  76}, {  10,  58}, {  -1,  76}, {  -1,  83}, {  -6,  93}, {  -6,  84},
	{  -8,  79}, {   0,  66}, {  -1,  71}, {   0,  62}, {  -2,  60}, {  -2,  59},
	{  -5,  75}, {  -3,  62}, {  -4,  58}, {  -9,  66}, {  -1,  79}, {   0,  71},
	{   3,  68}, {  10,  44}, {  -7,  62}, {  15,  36}, {  14,  40}, {  16,  27},
	{  12,  29}, {   1,  44}, {  20,  36}, {  18,  32}, {   5,  42}, {   1,  48},
	{  10,  62}, {  17,  46}, {   9,  64}, { -12, 104}, { -11,  97}, { -16,  96},
	{  -7,  88}, {  -8,  85}, {  -7,  85}, {  -9,  85}, { -13,  88}, {   4,  66},
	{  -3,  77}, {  -3,  76}, {  -6,  76}, {  10,  58}, {  -1,  76}, {  -1,  83},
	{  15,   6}, {   6,  19}, {   7,  16}, {  12,  14}, {  18,  13}, {  13,  11},
	{  13,  15}, {  15,  16}, {  12,  23}, {  13,  23}, {  15,  20}, {  14,  26},
	{  14,  44}, {  17,  40}, {  17,  47}, {  24,  17}, {  21,  21}, {  25,  22},
	{  31,  27}, {  22,  29}, {  19,  35}, {  14,  50}, {  10,  57}, {   7,  63},
	{  -2,  77}, {  -4,  82}, {  -3,  94}, {   9,  69}, { -12, 109}, {  36, -35},
	{  36, -34}, {  32, -26}, {  37, -30}, {  44, -32}, {  34, -18}, {  34, -15},
	{  40, -15}, {  33,  -7}, {  35,  -5}, {  33,   0}, {  38,   2}, {  33,  13},
	{  23,  35}, {  13,  58}, {  15,   6}, {   6,  19}, {   7,  16}, {  12,  14},
	{  18,  13}, {  13,  11}, {  13,  15}, {  15,  16}, {  12,  23}, {  13,  23},
	{  15,  20}, {  14,  26}, {  14,  44}, {  17,  40}, {  17,  47}, {  24,  17},
	{  21,  21}, {  25,  22}, {  31,  27}, {  22,  29}, {  19,  35}, {  14,  50},
	{  10,  57}, {   7,  63}, {  -2,  77}, {  -4,  82}, {  -3,  94}, {   9,  69},
	{ -12, 109}, {  36, -35}, {  36, -34}, {  32, -26}, {  37, -30}, {  44, -32},
	{  34, -18}, {  34, -15}, {  40, -15}, {  33,  -7}, {  35,  -5}, {  33,   0},
	{  38,   2}, {  33,  13}, {  23,  35}, {  13,  58}, {  -3,  71}, {  -6,  42},
	{  -5,  50}, {  -3,  54}, {  -2,  62}, {   0,  58}, {   1,  63}, {  -2,  72},
	{  -1,  74}, {  -9,  91}, {  -5,  67}, {  -5,  27}, {  -3,  39}, {  -2,  44},
	{   0,  46}, { -16,  64}, {  -8,  68}, { -10,  78}, {  -6,  77}, { -10,  86},
	{ -12,  92}, { -15,  55}, { -10,  60}, {  -6,  62}, {  -4,  65}, { -12,  73},
	{  -8,  76}, {  -7,  80}, {  -9,  88}, { -17, 110}, {  -3,  71}, {  -6,  42},
	{  -5,  50}, {  -3,  54}, {  -2,  62}, {   0,  58}, {   1,  63}, {  -2,  72},
	{  -1,  74}, {  -9,  91}, {  -5,  67}, {  -5,  27}, {  -3,  39}, {  -2,  44},
	{   0,  46}, { -16,  64}, {  -8,  68}, { -10,  78}, {  -6,  77}, { -10,  86},
	{ -12,  92}, { -15,  55}, { -10,  60}, {  -6,  62}, {  -4,  65}, { -12,  73},
	{  -8,  76}, {  -7,  80}, {  -9,  88}, { -17, 110}, {  -3,  70}, {  -8,  93},
	{ -10,  90}, { -30, 127}, {  -3,  70}, {  -8,  93}, { -10,  90}, { -30, 127},
	{  -3,  70}, {  -8,  93}, { -10,  90}, { -30, 127},
	}, {
	{  20, -15}, {   2,  54}, {   3,  74}, {  20, -15}, {   2,  54}, {   3,  74},
	{ -28, 127}, { -23, 104}, {  -6,  53}, {  -1,  54}, {   7,  51}, {  23,  33},
	{  23,   2}, {  21,   0}, {   1,   9}, {   0,  49}, { -37, 118}, {   5,  57},
	{ -13,  78}, { -11,  65}, {   1,  62}, {  12,  49}, {  -4,  73}, {  17,  50},
	{  18,  64}, {   9,  43}, {  29,   0}, {  26,  67}, {  16,  90}, {   9, 104},
	{ -46, 127}, { -20, 104}, {   1,  67}, { -13,  78}, { -11,  65}, {   1,  62},
	{  -6,  86}, { -17,  95}, {  -6,  61}, {   9,  45}, {  -3,  69}, {  -6,  81},
	{ -11,  96}, {   6,  55}, {   7,  67}, {  -5,  86}, {   2,  88}, {   0,  58},
	{  -3,  76}, { -10,  94}, {   5,  54}, {   4,  69}, {  -3,  81}, {   0,  88},
	{  -7,  67}, {  -5,  74}, {  -4,  74}, {  -5,  80}, {  -7,  72}, {   1,  58},
	{   0,  41}, {   0,  63}, {   0,  63}, {   0,  63}, {  -9,  83}, {   4,  86},
	{   0,  97}, {  -7,  72}, {  13,  41}, {   3,  62}, {   0,  45}, {  -4,  78},
	{  -3,  96}, { -27, 126}, { -28,  98}, { -25, 101}, { -23,  67}, { -28,  82},
	{ -20,  94}, { -16,  83}, { -22, 110}, { -21,  91}, { -18, 102}, { -13,  93},
	{ -29, 127}, {  -7,  92}, {  -5,  89}, {  -7,  96}, { -13, 108}, {  -3,  46},
	{  -1,  65}, {  -1,  57}, {  -9,  93}, {  -3,  74}, {  -9,  92}, {  -8,  87},
	{ -23, 126}, {   5,  54}, {   6,  60}, {   6,  59}, {   6,  69}, {  -1,  48},
	{   0,  68}, {  -4,  69}, {  -8,  88}, {  -2,  85}, {  -6,  78}, {  -1,  75},
	{  -7,  77}, {   2,  54}, {   5,  50}, {  -3,  68}, {   1,  50}, {   6,  42},
	{  -4,  81}, {   1,  63}, {  -4,  70}, {   0,  67}, {   2,  57}, {  -2,  76},
	{  11,  35}, {   4,  64}, {   1,  61}, {  11,  35}, {  18,  25}, {  12,  24},
	{  13,  29}, {  13,  36}, { -10,  93}, {  -7,  73}, {  -2,  73}, {  13,  46},
	{   9,  49}, {  -7, 100}, {   9,  53}, {   2,  53}, {   5,  53}, {  -2,  61},
	{   0,  56}, {   0,  56}, { -13,  63}, {  -5,  60}, {  -1,  62}, {   4,  57},
	{  -6,  69}, {   4,  57}, {  14,  39}, {   4,  51}, {  13,  68}, {   3,  64},
	{   1,  61}, {   9,  63}, {   7,  50}, {  16,  39}, {   5,  44}, {   4,  52},
	{  11,  48}, {  -5,  60}, {  -1,  59}, {   0,  59}, {  22,  33}, {   5,  44},
	{  14,  43}, {  -1,  78}, {   0,  60}, {   9,  69}, {  11,  28}, {   2,  40},
	{   3,  44}, {   0,  49}, {   0,  46}, {   2,  44}, {   2,  51}, {   0,  47},
	{   4,  39}, {   2,  62}, {   6,  46}, {   0,  54}, {   3,  54}, {   2,  58},
	{   4,  63}, {   6,  51}, {   6,  57}, {   7,  53}, {   6,  52}, {   6,  55},
	{  11,  45}, {  14,  36}, {   8,  53}, {  -1,  82}, {   7,  55}, {  -3,  78},
	{  15,  46}, {  22,  31}, {  -1,  84}, {  25,   7}, {  30,  -7}, {  28,   3},
	{  28,   4}, {  32,   0}, {  34,  -1}, {  30,   6}, {  30,   6}, {  32,   9},
	{  31,  19}, {  26,  27}, {  26,  30}, {  37,  20}, {  28,  34}, {  17,  70},
	{   1,  67}, {   5,  59}, {   9,  67}, {  16,  30}, {  18,  32}, {  18,  35},
	{  22,  29}, {  24,  31}, {  23,  38}, {  18,  43}, {  20,  41}, {  11,  63},
	{   9,  59}, {   9,  64}, {  -1,  94}, {  -2,  89}, {  -9, 108}, {  -6,  76},
	{  -2,  44}, {   0,  45}, {   0,  52}, {  -3,  64}, {  -2,  59}, {  -4,  70},
	{  -4,  75}, {  -8,  82}, { -17, 102}, {  -9,  77}, {   3,  24}, {   0,  42},
	{   0,  48}, {   0,  55}, {  -6,  59}, {  -7,  71}, { -12,  83}, { -11,  87},
	{ -30, 119}, {   1,  58}, {  -3,  29}, {  -1,  36}, {   1,  38}, {   2,  43},
	{  -6,  55}, {   0,  58}, {   0,  64}, {  -3,  74}, { -10,  90}, {   0,  70},
	{  -4,  29}, {   5,  31}, {   7,  42}, {   1,  59}, {  -2,  58}, {  -3,  72},
	{  -3,  81}, { -11,  97}, {   0,  58}, {   8,   5}, {  10,  14}, {  14,  18},
	{  13,  27}, {   2,  40}, {   0,  58}, {  -3,  70}, {  -6,  79}, {  -8,  85},
	{   0,   0}, { -13, 106}, { -16, 106}, { -10,  87}, { -21, 114}, { -18, 110},
	{ -14,  98}, { -22, 110}, { -21, 106}, { -18, 103}, { -21, 107}, { -23, 108},
	{ -26, 112}, { -10,  96}, { -12,  95}, {  -5,  91}, {  -9,  93}, { -22,  94},
	{  -5,  86}, {   9,  67}, {  -4,  80}, { -10,  85}, {  -1,  70}, {   7,  60},
	{   9,  58}, {   5,  61}, {  12,  50}, {  15,  50}, {  18,  49}, {  17,  54},
	{  10,  41}, {   7,  46}, {  -1,  51}, {   7,  49}, {   8,  52}, {   9,  41},
	{   6,  47}, {   2,  55}, {  13,  41}, {  10,  44}, {   6,  50}, {   5,  53},
	{  13,  49}, {   4,  63}, {   6,  64}, {  -2,  69}, {  -2,  59}, {   6,  70},
	{  10,  44}, {   9,  31}, {  12,  43}, {   3,  53}, {  14,  34}, {  10,  38},
	{  -3,  52}, {  13,  40}, {  17,  32}, {   7,  44}, {   7,  38}, {  13,  50},
	{  10,  57}, {  26,  43}, {  14,  11}, {  11,  14}, {   9,  11}, {  18,  11},
	{  21,   9}, {  23,  -2}, {  32, -15}, {  32, -15}, {  34, -21}, {  39, -23},
	{  42, -33}, {  41, -31}, {  46, -28}, {  38, -12}, {  21,  29}, {  45, -24},
	{  53, -45}, {  48, -26}, {  65, -43}, {  43, -19}, {  39, -10}, {  30,   9},
	{  18,  26}, {  20,  27}, {   0,  57}, { -14,  82}, {  -5,  75}, { -19,  97},
	{ -35, 125}, {  27,   0}, {  28,   0}, {  31,  -4}, {  27,   6}, {  34,   8},
	{  30,  10}, {  24,  22}, {  33,  19}, {  22,  32}, {  26,  31}, {  21,  41},
	{  26,  44}, {  23,  47}, {  16,  65}, {  14,  71}, {   8,  60}, {   6,  63},
	{  17,  65}, {  21,  24}, {  23,  20}, {  26,  23}, {  27,  32}, {  28,  23},
	{  28,  24}, {  23,  40}, {  24,  32}, {  28,  29}, {  23,  42}, {  19,  57},
	{  22,  53}, {  22,  61}, {  11,  86}, {  12,  40}, {  11,  51}, {  14,  59},
	{  -4,  79}, {  -7,  71}, {  -5,  69}, {  -9,  70}, {  -8,  66}, { -10,  68},
	{ -19,  73}, { -12,  69}, { -16,  70}, { -15,  67}, { -20,  62}, { -19,  70},
	{ -16,  66}, { -22,  65}, { -20,  63}, {   9,  -2}, {  26,  -9}, {  33,  -9},
	{  39,  -7}, {  41,  -2}, {  45,   3}, {  49,   9}, {  45,  27}, {  36,  59},
	{  -6,  66}, {  -7,  35}, {  -7,  42}, {  -8,  45}, {  -5,  48}, { -12,  56},
	{  -6,  60}, {  -5,  62}, {  -8,  66}, {  -8,  76}, {  -5,  85}, {  -6,  81},
	{ -10,  77}, {  -7,  81}, { -17,  80}, { -18,  73}, {  -4,  74}, { -10,  83},
	{  -9,  71}, {  -9,  67}, {  -1,  61}, {  -8,  66}, { -14,  66}, {   0,  59},
	{   2,  59}, {  21, -13}, {  33, -14}, {  39,  -7}, {  46,  -2}, {  51,   2},
	{  60,   6}, {  61,  17}, {  55,  34}, {  42,  62}, {  -7,  92}, {  -5,  89},
	{  -7,  96}, { -13, 108}, {  -3,  46}, {  -1,  65}, {  -1,  57}, {  -9,  93},
	{  -3,  74}, {  -9,  92}, {  -8,  87}, { -23, 126}, {  -7,  92}, {  -5,  89},
	{  -7,  96}, { -13, 108}, {  -3,  46}, {  -1,  65}, {  -1,  57}, {  -9,  93},
	{  -3,  74}, {  -9,  92}, {  -8,  87}, { -23, 126}, {  -2,  85}, {  -6,  78},
	{  -1,  75}, {  -7,  77}, {   2,  54}, {   5,  50}, {  -3,  68}, {   1,  50},
	{   6,  42}, {  -4,  81}, {   1,  63}, {  -4,  70}, {   0,  67}, {   2,  57},
	{  -2,  76}, {  11,  35}, {   4,  64}, {   1,  61}, {  11,  35}, {  18,  25},
	{  12,  24}, {  13,  29}, {  13,  36}, { -10,  93}, {  -7,  73}, {  -2,  73},
	{  13,  46}, {   9,  49}, {  -7, 100}, {   9,  53}, {   2,  53}, {   5,  53},
	{  -2,  61}, {   0,  56}, {   0,  56}, { -13,  63}, {  -5,  60}, {  -1,  62},
	{   4,  57}, {  -6,  69}, {   4,  57}, {  14,  39}, {   4,  51}, {  13,  68},
	{  -2,  85}, {  -6,  78}, {  -1,  75}, {  -7,  77}, {   2,  54}, {   5,  50},
	{  -3,  68}, {   1,  50}, {   6,  42}, {  -4,  81}, {   1,  63}, {  -4,  70},
	{   0,  67}, {   2,  57}, {  -2,  76}, {  11,  35}, {   4,  64}, {   1,  61},
	{  11,  35}, {  18,  25}, {  12,  24}, {  13,  29}, {  13,  36}, { -10,  93},
	{  -7,  73}, {  -2,  73}, {  13,  46}, {   9,  49}, {  -7, 100}, {   9,  53},
	{   2,  53}, {   5,  53}, {  -2,  61}, {   0,  56}, {   0,  56}, { -13,  63},
	{  -5,  60}, {  -1,  62}, {   4,  57}, {  -6,  69}, {   4,  57}, {  14,  39},
	{   4,  51}, {  13,  68}, {  11,  28}, {   2,  40}, {   3,  44}, {   0,  49},
	{   0,  46}, {   2,  44}, {   2,  51}, {   0,  47}, {   4,  39}, {   2,  62},
	{   6,  46}, {   0,  54}, {   3,  54}, {   2,  58}, {   4,  63}, {   6,  51},
	{   6,  57}, {   7,  53}, {   6,  52}, {   6,  55}, {  11,  45}, {  14,  36},
	{   8,  53}, {  -1,  82}, {   7,  55}, {  -3,  78}, {  15,  46}, {  22,  31},
	{  -1,  84}, {  25,   7}, {  30,  -7}, {  28,   3}, {  28,   4}, {  32,   0},
	{  34,  -1}, {  30,   6}, {  30,   6}, {  32,   9}, {  31,  19}, {  26,  27},
	{  26,  30}, {  37,  20}, {  28,  34}, {  17,  70}, {  11,  28}, {   2,  40},
	{   3,  44}, {   0,  49}, {   0,  46}, {   2,  44}, {   2,  51}, {   0,  47},
	{   4,  39}, {   2,  62}, {   6,  46}, {   0,  54}, {   3,  54}, {   2,  58},
	{   4,  63}, {   6,  51}, {   6,  57}, {   7,  53}, {   6,  52}, {   6,  55},
	{  11,  45}, {  14,  36}, {   8,  53}, {  -1,  82}, {   7,  55}, {  -3,  78},
	{  15,  46}, {  22,  31}, {  -1,  84}, {  25,   7}, {  30,  -7}, {  28,   3},
	{  28,   4}, {  32,   0}, {  34,  -1}, {  30,   6}, {  30,   6}, {  32,   9},
	{  31,  19}, {  26,  27}, {  26,  30}, {  37,  20}, {  28,  34}, {  17,  70},
	{  -4,  79}, {  -7,  71}, {  -5,  69}, {  -9,  70}, {  -8,  66}, { -10,  68},
	{ -19,  73}, { -12,  69}, { -16,  70}, { -15,  67}, { -20,  62}, { -19,  70},
	{ -16,  66}, { -22,  65}, { -20,  63}, {  -5,  85}, {  -6,  81}, { -10,  77},
	{  -7,  81}, { -17,  80}, { -18,  73}, {  -4,  74}, { -10,  83}, {  -9,  71},
	{  -9,  67}, {  -1,  61}, {  -8,  66}, { -14,  66}, {   0,  59}, {   2,  59},
	{   9,  -2}, {  26,  -9}, {  33,  -9}, {  39,  -7}, {  41,  -2}, {  45,   3},
	{  49,   9}, {  45,  27}, {  36,  59}, {  21, -13}, {  33, -14}, {  39,  -7},
	{  46,  -2}, {  51,   2}, {  60,   6}, {  61,  17}, {  55,  34}, {  42,  62},
	{  -6,  66}, {  -7,  35}, {  -7,  42}, {  -8,  45}, {  -5,  48}, { -12,  56},
	{  -6,  60}, {  -5,  62}, {  -8,  66}, {  -8,  76}, {  -4,  79}, {  -7,  71},
	{  -5,  69}, {  -9,  70}, {  -8,  66}, { -10,  68}, { -19,  73}, { -12,  69},
	{ -16,  70}, { -15,  67}, { -20,  62}, { -19,  70}, { -16,  66}, { -22,  65},
	{ -20,  63}, {  -5,  85}, {  -6,  81}, { -10,  77}, {  -7,  81}, { -17,  80},
	{ -18,  73}, {  -4,  74}, { -10,  83}, {  -9,  71}, {  -9,  67}, {  -1,  61},
	{  -8,  66}, { -14,  66}, {   0,  59}, {   2,  59}, {   9,  -2}, {  26,  -9},
	{  33,  -9}, {  39,  -7}, {  41,  -2}, {  45,   3}, {  49,   9}, {  45,  27},
	{  36,  59}, {  21, -13}, {  33, -14}, {  39,  -7}, {  46,  -2}, {  51,   2},
	{  60,   6}, {  61,  17}, {  55,  34}, {  42,  62}, {  -6,  66}, {  -7,  35},
	{  -7,  42}, {  -8,  45}, {  -5,  48}, { -12,  56}, {  -6,  60}, {  -5,  62},
	{  -8,  66}, {  -8,  76}, { -13, 106}, { -16, 106}, { -10,  87}, { -21, 114},
	{ -18, 110}, { -14,  98}, { -22, 110}, { -21, 106}, { -18, 103}, { -21, 107},
	{ -23, 108}, { -26, 112}, { -10,  96}, { -12,  95}, {  -5,  91}, {  -9,  93},
	{ -22,  94}, {  -5,  86}, {   9,  67}, {  -4,  80}, { -10,  85}, {  -1,  70},
	{   7,  60}, {   9,  58}, {   5,  61}, {  12,  50}, {  15,  50}, {  18,  49},
	{  17,  54}, {  10,  41}, {   7,  46}, {  -1,  51}, {   7,  49}, {   8,  52},
	{   9,  41}, {   6,  47}, {   2,  55}, {  13,  41}, {  10,  44}, {   6,  50},
	{   5,  53}, {  13,  49}, {   4,  63}, {   6,  64}, { -13, 106}, { -16, 106},
	{ -10,  87}, { -21, 114}, { -18, 110}, { -14,  98}, { -22, 110}, { -21, 106},
	{ -18, 103}, { -21, 107}, { -23, 108}, { -26, 112}, { -10,  96}, { -12,  95},
	{  -5,  91}, {  -9,  93}, { -22,  94}, {  -5,  86}, {   9,  67}, {  -4,  80},
	{ -10,  85}, {  -1,  70}, {   7,  60}, {   9,  58}, {   5,  61}, {  12,  50},
	{  15,  50}, {  18,  49}, {  17,  54}, {  10,  41}, {   7,  46}, {  -1,  51},
	{   7,  49}, {   8,  52}, {   9,  41}, {   6,  47}, {   2,  55}, {  13,  41},
	{  10,  44}, {   6,  50}, {   5,  53}, {  13,  49}, {   4,  63}, {   6,  64},
	{  14,  11}, {  11,  14}, {   9,  11}, {  18,  11}, {  21,   9}, {  23,  -2},
	{  32, -15}, {  32, -15}, {  34, -21}, {  39, -23}, {  42, -33}, {  41, -31},
	{  46, -28}, {  38, -12}, {  21,  29}, {  45, -24}, {  53, -45}, {  48, -26},
	{  65, -43}, {  43, -19}, {  39, -10}, {  30,   9}, {  18,  26}, {  20,  27},
	{   0,  57}, { -14,  82}, {  -5,  75}, { -19,  97}, { -35, 125}, {  27,   0},
	{  28,   0}, {  31,  -4}, {  27,   6}, {  34,   8}, {  30,  10}, {  24,  22},
	{  33,  19}, {  22,  32}, {  26,  31}, {  21,  41}, {  26,  44}, {  23,  47},
	{  16,  65}, {  14,  71}, {  14,  11}, {  11,  14}, {   9,  11}, {  18,  11},
	{  21,   9}, {  23,  -2}, {  32, -15}, {  32, -15}, {  34, -21}, {  39, -23},
	{  42, -33}, {  41, -31}, {  46, -28}, {  38, -12}, {  21,  29}, {  45, -24},
	{  53, -45}, {  48, -26}, {  65, -43}, {  43, -19}, {  39, -10}, {  30,   9},
	{  18,  26}, {  20,  27}, {   0,  57}, { -14,  82}, {  -5,  75}, { -19,  97},
	{ -35, 125}, {  27,   0}, {  28,   0}, {  31,  -4}, {  27,   6}, {  34,   8},
	{  30,  10}, {  24,  22}, {  33,  19}, {  22,  32}, {  26,  31}, {  21,  41},
	{  26,  44}, {  23,  47}, {  16,  65}, {  14,  71}, {  -6,  76}, {  -2,  44},
	{   0,  45}, {   0,  52}, {  -3,  64}, {  -2,  59}, {  -4,  70}, {  -4,  75},
	{  -8,  82}, { -17, 102}, {  -9,  77}, {   3,  24}, {   0,  42}, {   0,  48},
	{   0,  55}, {  -6,  59}, {  -7,  71}, { -12,  83}, { -11,  87}, { -30, 119},
	{   1,  58}, {  -3,  29}, {  -1,  36}, {   1,  38}, {   2,  43}, {  -6,  55},
	{   0,  58}, {   0,  64}, {  -3,  74}, { -10,  90}, {  -6,  76}, {  -2,  44},
	{   0,  45}, {   0,  52}, {  -3,  64}, {  -2,  59}, {  -4,  70}, {  -4,  75},
	{  -8,  82}, { -17, 102}, {  -9,  77}, {   3,  24}, {   0,  42}, {   0,  48},
	{   0,  55}, {  -6,  59}, {  -7,  71}, { -12,  83}, { -11,  87}, { -30, 119},
	{   1,  58}, {  -3,  29}, {  -1,  36}, {   1,  38}, {   2,  43}, {  -6,  55},
	{   0,  58}, {   0,  64}, {  -3,  74}, { -10,  90}, {  -3,  74}, {  -9,  92},
	{  -8,  87}, { -23, 126}, {  -3,  74}, {  -9,  92}, {  -8,  87}, { -23, 126},
	{  -3,  74}, {  -9,  92}, {  -8,  87}, { -23, 126},
	}, {
	{  20, -15}, {   2,  54}, {   3,  74}, {  20, -15}, {   2,  54}, {   3,  74},
	{ -28, 127}, { -23, 104}, {  -6,  53}, {  -1,  54}, {   7,  51}, {  22,  25},
	{  34,   0}, {  16,   0}, {  -2,   9}, {   4,  41}, { -29, 118}, {   2,  65},
	{  -6,  71}, { -13,  79}, {   5,  52}, {   9,  50}, {  -3,  70}, {  10,  54},
	{  26,  34}, {  19,  22}, {  40,   0}, {  57,   2}, {  41,  36}, {  26,  69},
	{ -45, 127}, { -15, 101}, {  -4,  76}, {  -6,  71}, { -13,  79}, {   5,  52},
	{   6,  69}, { -13,  90}, {   0,  52}, {   8,  43}, {  -2,  69}, {  -5,  82},
	{ -10,  96}, {   2,  59}, {   2,  75}, {  -3,  87}, {  -3, 100}, {   1,  56},
	{  -3,  74}, {  -6,  85}, {   0,  59}, {  -3,  81}, {  -7,  86}, {  -5,  95},
	{  -1,  66}, {  -1,  77}, {   1,  70}, {  -2,  86}, {  -5,  72}, {   0,  61},
	{   0,  41}, {   0,  63}, {   0,  63}, {   0,  63}, {  -9,  83}, {   4,  86},
	{   0,  97}, {  -7,  72}, {  13,  41}, {   3,  62}, {  13,  15}, {   7,  51},
	{   2,  80}, { -39, 127}, { -18,  91}, { -17,  96}, { -26,  81}, { -35,  98},
	{ -24, 102}, { -23,  97}, { -27, 119}, { -24,  99}, { -21, 110}, { -18, 102},
	{ -36, 127}, {   0,  80}, {  -5,  89}, {  -7,  94}, {  -4,  92}, {   0,  39},
	{   0,  65}, { -15,  84}, { -35, 127}, {  -2,  73}, { -12, 104}, {  -9,  91},
	{ -31, 127}, {   3,  55}, {   7,  56}, {   7,  55}, {   8,  61}, {  -3,  53},
	{   0,  68}, {  -7,  74}, {  -9,  88}, { -13, 103}, { -13,  91}, {  -9,  89},
	{ -14,  92}, {  -8,  76}, { -12,  87}, { -23, 110}, { -24, 105}, { -10,  78},
	{ -20, 112}, { -17,  99}, { -78, 127}, { -70, 127}, { -50, 127}, { -46, 127},
	{  -4,  66}, {  -5,  78}, {  -4,  71}, {  -8,  72}, {   2,  59}, {  -1,  55},
	{  -7,  70}, {  -6,  75}, {  -8,  89}, { -34, 119}, {  -3,  75}, {  32,  20},
	{  30,  22}, { -44, 127}, {   0,  54}, {  -5,  61}, {   0,  58}, {  -1,  60},
	{  -3,  61}, {  -8,  67}, { -25,  84}, { -14,  74}, {  -5,  65}, {   5,  52},
	{   2,  57}, {   0,  61}, {  -9,  69}, { -11,  70}, {  18,  55}, {  -4,  71},
	{   0,  58}, {   7,  61}, {   9,  41}, {  18,  25}, {   9,  32}, {   5,  43},
	{   9,  47}, {   0,  44}, {   0,  51}, {   2,  46}, {  19,  38}, {  -4,  66},
	{  15,  38}, {  12,  42}, {   9,  34}, {   0,  89}, {   4,  45}, {  10,  28},
	{  10,  31}, {  33, -11}, {  52, -43}, {  18,  15}, {  28,   0}, {  35, -22},
	{  38, -25}, {  34,   0}, {  39, -18}, {  32, -12}, { 102, -94}, {   0,   0},
	{  56, -15}, {  33,  -4}, {  29,  10}, {  37,  -5}, {  51, -29}, {  39,  -9},
	{  52, -34}, {  69, -58}, {  67, -63}, {  44,  -5}, {  32,   7}, {  55, -29},
	{  32,   1}, {   0,   0}, {  27,  36}, {  33, -25}, {  34, -30}, {  36, -28},
	{  38, -28}, {  38, -27}, {  34, -18}, {  35, -16}, {  34, -14}, {  32,  -8},
	{  37,  -6}, {  35,   0}, {  30,  10}, {  28,  18}, {  26,  25}, {  29,  41},
	{   0,  75}, {   2,  72}, {   8,  77}, {  14,  35}, {  18,  31}, {  17,  35},
	{  21,  30}, {  17,  45}, {  20,  42}, {  18,  45}, {  27,  26}, {  16,  54},
	{   7,  66}, {  16,  56}, {  11,  73}, {  10,  67}, { -10, 116}, { -23, 112},
	{ -15,  71}, {  -7,  61}, {   0,  53}, {  -5,  66}, { -11,  77}, {  -9,  80},
	{  -9,  84}, { -10,  87}, { -34, 127}, { -21, 101}, {  -3,  39}, {  -5,  53},
	{  -7,  61}, { -11,  75}, { -15,  77}, { -17,  91}, { -25, 107}, { -25, 111},
	{ -28, 122}, { -11,  76}, { -10,  44}, { -10,  52}, { -10,  57}, {  -9,  58},
	{ -16,  72}, {  -7,  69}, {  -4,  69}, {  -5,  74}, {  -9,  86}, {   2,  66},
	{  -9,  34}, {   1,  32}, {  11,  31}, {   5,  52}, {  -2,  55}, {  -2,  67},
	{   0,  73}, {  -8,  89}, {   3,  52}, {   7,   4}, {  10,   8}, {  17,   8},
	{  16,  19}, {   3,  37}, {  -1,  61}, {  -5,  73}, {  -1,  70}, {  -4,  78},
	{   0,   0}, { -21, 126}, { -23, 124}, { -20, 110}, { -26, 126}, { -25, 124},
	{ -17, 105}, { -27, 121}, { -27, 117}, { -17, 102}, { -26, 117}, { -27, 116},
	{ -33, 122}, { -10,  95}, { -14, 100}, {  -8,  95}, { -17, 111}, { -28, 114},
	{  -6,  89}, {  -2,  80}, {  -4,  82}, {  -9,  85}, {  -8,  81}, {  -1,  72},
	{   5,  64}, {   1,  67}, {   9,  56}, {   0,  69}, {   1,  69}, {   7,  69},
	{  -7,  69}, {  -6,  67}, { -16,  77}, {  -2,  64}, {   2,  61}, {  -6,  67},
	{  -3,  64}, {   2,  57}, {  -3,  65}, {  -3,  66}, {   0,  62}, {   9,  51},
	{  -1,  66}, {  -2,  71}, {  -2,  75}, {  -1,  70}, {  -9,  72}, {  14,  60},
	{  16,  37}, {   0,  47}, {  18,  35}, {  11,  37}, {  12,  41}, {  10,  41},
	{   2,  48}, {  12,  41}, {  13,  41}, {   0,  59}, {   3,  50}, {  19,  40},
	{   3,  66}, {  18,  50}, {  19,  -6}, {  18,  -6}, {  14,   0}, {  26, -12},
	{  31, -16}, {  33, -25}, {  33, -22}, {  37, -28}, {  39, -30}, {  42, -30},
	{  47, -42}, {  45, -36}, {  49, -34}, {  41, -17}, {  32,   9}, {  69, -71},
	{  63, -63}, {  66, -64}, {  77, -74}, {  54, -39}, {  52, -35}, {  41, -10},
	{  36,   0}, {  40,  -1}, {  30,  14}, {  28,  26}, {  23,  37}, {  12,  55},
	{  11,  65}, {  37, -33}, {  39, -36}, {  40, -37}, {  38, -30}, {  46, -33},
	{  42, -30}, {  40, -24}, {  49, -29}, {  38, -12}, {  40, -10}, {  38,  -3},
	{  46,  -5}, {  31,  20}, {  29,  30}, {  25,  44}, {  12,  48}, {  11,  49},
	{  26,  45}, {  22,  22}, {  23,  22}, {  27,  21}, {  33,  20}, {  26,  28},
	{  30,  24}, {  27,  34}, {  18,  42}, {  25,  39}, {  18,  50}, {  12,  70},
	{  21,  54}, {  14,  71}, {  11,  83}, {  25,  32}, {  21,  49}, {  21,  54},
	{  -5,  85}, {  -6,  81}, { -10,  77}, {  -7,  81}, { -17,  80}, { -18,  73},
	{  -4,  74}, { -10,  83}, {  -9,  71}, {  -9,  67}, {  -1,  61}, {  -8,  66},
	{ -14,  66}, {   0,  59}, {   2,  59}, {  17, -10}, {  32, -13}, {  42,  -9},
	{  49,  -5}, {  53,   0}, {  64,   3}, {  68,  10}, {  66,  27}, {  47,  57},
	{  -5,  71}, {   0,  24}, {  -1,  36}, {  -2,  42}, {  -2,  52}, {  -9,  57},
	{  -6,  63}, {  -4,  65}, {  -4,  67}, {  -7,  82}, {  -3,  81}, {  -3,  76},
	{  -7,  72}, {  -6,  78}, { -12,  72}, { -14,  68}, {  -3,  70}, {  -6,  76},
	{  -5,  66}, {  -5,  62}, {   0,  57}, {  -4,  61}, {  -9,  60}, {   1,  54},
	{   2,  58}, {  17, -10}, {  32, -13}, {  42,  -9}, {  49,  -5}, {  53,   0},
	{  64,   3}, {  68,  10}, {  66,  27}, {  47,  57}, {   0,  80}, {  -5,  89},
	{  -7,  94}, {  -4,  92}, {   0,  39}, {   0,  65}, { -15,  84}, { -35, 127},
	{  -2,  73}, { -12, 104}, {  -9,  91}, { -31, 127}, {   0,  80}, {  -5,  89},
	{  -7,  94}, {  -4,  92}, {   0,  39}, {   0,  65}, { -15,  84}, { -35, 127},
	{  -2,  73}, { -12, 104}, {  -9,  91}, { -31, 127}, { -13, 103}, { -13,  91},
	{  -9,  89}, { -14,  92}, {  -8,  76}, { -12,  87}, { -23, 110}, { -24, 105},
	{ -10,  78}, { -20, 112}, { -17,  99}, { -78, 127}, { -70, 127}, { -50, 127},
	{ -46, 127}, {  -4,  66}, {  -5,  78}, {  -4,  71}, {  -8,  72}, {   2,  59},
	{  -1,  55}, {  -7,  70}, {  -6,  75}, {  -8,  89}, { -34, 119}, {  -3,  75},
	{  32,  20}, {  30,  22}, { -44, 127}, {   0,  54}, {  -5,  61}, {   0,  58},
	{  -1,  60}, {  -3,  61}, {  -8,  67}, { -25,  84}, { -14,  74}, {  -5,  65},
	{   5,  52}, {   2,  57}, {   0,  61}, {  -9,  69}, { -11,  70}, {  18,  55},
	{ -13, 103}, { -13,  91}, {  -9,  89}, { -14,  92}, {  -8,  76}, { -12,  87},
	{ -23, 110}, { -24, 105}, { -10,  78}, { -20, 112}, { -17,  99}, { -78, 127},
	{ -70, 127}, { -50, 127}, { -46, 127}, {  -4,  66}, {  -5,  78}, {  -4,  71},
	{  -8,  72}, {   2,  59}, {  -1,  55}, {  -7,  70}, {  -6,  75}, {  -8,  89},
	{ -34, 119}, {  -3,  75}, {  32,  20}, {  30,  22}, { -44, 127}, {   0,  54},
	{  -5,  61}, {   0,  58}, {  -1,  60}, {  -3,  61}, {  -8,  67}, { -25,  84},
	{ -14,  74}, {  -5,  65}, {   5,  52}, {   2,  57}, {   0,  61}, {  -9,  69},
	{ -11,  70}, {  18,  55}, {   4,  45}, {  10,  28}, {  10,  31}, {  33, -11},
	{  52, -43}, {  18,  15}, {  28,   0}, {  35, -22}, {  38, -25}, {  34,   0},
	{  39, -18}, {  32, -12}, { 102, -94}, {   0,   0}, {  56, -15}, {  33,  -4},
	{  29,  10}, {  37,  -5}, {  51, -29}, {  39,  -9}, {  52, -34}, {  69, -58},
	{  67, -63}, {  44,  -5}, {  32,   7}, {  55, -29}, {  32,   1}, {   0,   0},
	{  27,  36}, {  33, -25}, {  34, -30}, {  36, -28}, {  38, -28}, {  38, -27},
	{  34, -18}, {  35, -16}, {  34, -14}, {  32,  -8}, {  37,  -6}, {  35,   0},
	{  30,  10}, {  28,  18}, {  26,  25}, {  29,  41}, {   4,  45}, {  10,  28},
	{  10,  31}, {  33, -11}, {  52, -43}, {  18,  15}, {  28,   0}, {  35, -22},
	{  38, -25}, {  34,   0}, {  39, -18}, {  32, -12}, { 102, -94}, {   0,   0},
	{  56, -15}, {  33,  -4}, {  29,  10}, {  37,  -5}, {  51, -29}, {  39,  -9},
	{  52, -34}, {  69, -58}, {  67, -63}, {  44,  -5}, {  32,   7}, {  55, -29},
	{  32,   1}, {   0,   0}, {  27,  36}, {  33, -25}, {  34, -30}, {  36, -28},
	{  38, -28}, {  38, -27}, {  34, -18}, {  35, -16}, {  34, -14}, {  32,  -8},
	{  37,  -6}, {  35,   0}, {  30,  10}, {  28,  18}, {  26,  25}, {  29,  41},
	{  -5,  85}, {  -6,  81}, { -10,  77}, {  -7,  81}, { -17,  80}, { -18,  73},
	{  -4,  74}, { -10,  83}, {  -9,  71}, {  -9,  67}, {  -1,  61}, {  -8,  66},
	{ -14,  66}, {   0,  59}, {   2,  59}, {  -3,  81}, {  -3,  76}, {  -7,  72},
	{  -6,  78}, { -12,  72}, { -14,  68}, {  -3,  70}, {  -6,  76}, {  -5,  66},
	{  -5,  62}, {   0,  57}, {  -4,  61}, {  -9,  60}, {   1,  54}, {   2,  58},
	{  17, -10}, {  32, -13}, {  42,  -9}, {  49,  -5}, {  53,   0}, {  64,   3},
	{  68,  10}, {  66,  27}, {  47,  57}, {  17, -10}, {  32, -13}, {  42,  -9},
	{  49,  -5}, {  53,   0}, {  64,   3}, {  68,  10}, {  66,  27}, {  47,  57},
	{  -5,  71}, {   0,  24}, {  -1,  36}, {  -2,  42}, {  -2,  52}, {  -9,  57},
	{  -6,  63}, {  -4,  65}, {  -4,  67}, {  -7,  82}, {  -5,  85}, {  -6,  81},
	{ -10,  77}, {  -7,  81}, { -17,  80}, { -18,  73}, {  -4,  74}, { -10,  83},
	{  -9,  71}, {  -9,  67}, {  -1,  61}, {  -8,  66}, { -14,  66}, {   0,  59},
	{   2,  59}, {  -3,  81}, {  -3,  76}, {  -7,  72}, {  -6,  78}, { -12,  72},
	{ -14,  68}, {  -3,  70}, {  -6,  76}, {  -5,  66}, {  -5,  62}, {   0,  57},
	{  -4,  61}, {  -9,  60}, {   1,  54}, {   2,  58}, {  17, -10}, {  32, -13},
	{  42,  -9}, {  49,  -5}, {  53,   0}, {  64,   3}, {  68,  10}, {  66,  27},
	{  47,  57}, {  17, -10}, {  32, -13}, {  42,  -9}, {  49,  -5}, {  53,   0},
	{  64,   3}, {  68,  10}, {  66,  27}, {  47,  57}, {  -5,  71}, {   0,  24},
	{  -1,  36}, {  -2,  42}, {  -2,  52}, {  -9,  57}, {  -6,  63}, {  -4,  65},
	{  -4,  67}, {  -7,  82}, { -21, 126}, { -23, 124}, { -20, 110}, { -26, 126},
	{ -25, 124}, { -17, 105}, { -27, 121}, { -27, 117}, { -17, 102}, { -26, 117},
	{ -27, 116}, { -33, 122}, { -10,  95}, { -14, 100}, {  -8,  95}, { -17, 111},
	{ -28, 114}, {  -6,  89}, {  -2,  80}, {  -4,  82}, {  -9,  85}, {  -8,  81},
	{  -1,  72}, {   5,  64}, {   1,  67}, {   9,  56}, {   0,  69}, {   1,  69},
	{   7,  69}, {  -7,  69}, {  -6,  67}, { -16,  77}, {  -2,  64}, {   2,  61},
	{  -6,  67}, {  -3,  64}, {   2,  57}, {  -3,  65}, {  -3,  66}, {   0,  62},
	{   9,  51}, {  -1,  66}, {  -2,  71}, {  -2,  75}, { -21, 126}, { -23, 124},
	{ -20, 110}, { -26, 126}, { -25, 124}, { -17, 105}, { -27, 121}, { -27, 117},
	{ -17, 102}, { -26, 117}, { -27, 116}, { -33, 122}, { -10,  95}, { -14, 100},
	{  -8,  95}, { -17, 111}, { -28, 114}, {  -6,  89}, {  -2,  80}, {  -4,  82},
	{  -9,  85}, {  -8,  81}, {  -1,  72}, {   5,  64}, {   1,  67}, {   9,  56},
	{   0,  69}, {   1,  69}, {   7,  69}, {  -7,  69}, {  -6,  67}, { -16,  77},
	{  -2,  64}, {   2,  61}, {  -6,  67}, {  -3,  64}, {   2,  57}, {  -3,  65},
	{  -3,  66}, {   0,  62}, {   9,  51}, {  -1,  66}, {  -2,  71}, {  -2,  75},
	{  19,  -6}, {  18,  -6}, {  14,   0}, {  26, -12}, {  31, -16}, {  33, -25},
	{  33, -22}, {  37, -28}, {  39, -30}, {  42, -30}, {  47, -42}, {  45, -36},
	{  49, -34}, {  41, -17}, {  32,   9}, {  69, -71}, {  63, -63}, {  66, -64},
	{  77, -74}, {  54, -39}, {  52, -35}, {  41, -10}, {  36,   0}, {  40,  -1},
	{  30,  14}, {  28,  26}, {  23,  37}, {  12,  55}, {  11,  65}, {  37, -33},
	{  39, -36}, {  40, -37}, {  38, -30}, {  46, -33}, {  42, -30}, {  40, -24},
	{  49, -29}, {  38, -12}, {  40, -10}, {  38,  -3}, {  46,  -5}, {  31,  20},
	{  29,  30}, {  25,  44}, {  19,  -6}, {  18,  -6}, {  14,   0}, {  26, -12},
	{  31, -16}, {  33, -25}, {  33, -22}, {  37, -28}, {  39, -30}, {  42, -30},
	{  47, -42}, {  45, -36}, {  49, -34}, {  41, -17}, {  32,   9}, {  69, -71},
	{  63, -63}, {  66, -64}, {  77, -74}, {  54, -39}, {  52, -35}, {  41, -10},
	{  36,   0}, {  40,  -1}, {  30,  14}, {  28,  26}, {  23,  37}, {  12,  55},
	{  11,  65}, {  37, -33}, {  39, -36}, {  40, -37}, {  38, -30}, {  46, -33},
	{  42, -30}, {  40, -24}, {  49, -29}, {  38, -12}, {  40, -10}, {  38,  -3},
	{  46,  -5}, {  31,  20}, {  29,  30}, {  25,  44}, { -23, 112}, { -15,  71},
	{  -7,  61}, {   0,  53}, {  -5,  66}, { -11,  77}, {  -9,  80}, {  -9,  84},
	{ -10,  87}, { -34, 127}, { -21, 101}, {  -3,  39}, {  -5,  53}, {  -7,  61},
	{ -11,  75}, { -15,  77}, { -17,  91}, { -25, 107}, { -25, 111}, { -28, 122},
	{ -11,  76}, { -10,  44}, { -10,  52}, { -10,  57}, {  -9,  58}, { -16,  72},
	{  -7,  69}, {  -4,  69}, {  -5,  74}, {  -9,  86}, { -23, 112}, { -15,  71},
	{  -7,  61}, {   0,  53}, {  -5,  66}, { -11,  77}, {  -9,  80}, {  -9,  84},
	{ -10,  87}, { -34, 127}, { -21, 101}, {  -3,  39}, {  -5,  53}, {  -7,  61},
	{ -11,  75}, { -15,  77}, { -17,  91}, { -25, 107}, { -25, 111}, { -28, 122},
	{ -11,  76}, { -10,  44}, { -10,  52}, { -10,  57}, {  -9,  58}, { -16,  72},
	{  -7,  69}, {  -4,  69}, {  -5,  74}, {  -9,  86}, {  -2,  73}, { -12, 104},
	{  -9,  91}, { -31, 127}, {  -2,  73}, { -12, 104}, {  -9,  91}, { -31, 127},
	{  -2,  73}, { -12, 104}, {  -9,  91}, { -31, 127},
	}, {
	{  20, -15}, {   2,  54}, {   3,  74}, {  20, -15}, {   2,  54}, {   3,  74},
	{ -28, 127}, { -23, 104}, {  -6,  53}, {  -1,  54}, {   7,  51}, {  29,  16},
	{  25,   0}, {  14,   0}, { -10,  51}, {  -3,  62}, { -27,  99}, {  26,  16},
	{  -4,  85}, { -24, 102}, {   5,  57}, {   6,  57}, { -17,  73}, {  14,  57},
	{  20,  40}, {  20,  10}, {  29,   0}, {  54,   0}, {  37,  42}, {  12,  97},
	{ -32, 127}, { -22, 117}, {  -2,  74}, {  -4,  85}, { -24, 102}, {   5,  57},
	{  -6,  93}, { -14,  88}, {  -6,  44}, {   4,  55}, { -11,  89}, { -15, 103},
	{ -21, 116}, {  19,  57}, {  20,  58}, {   4,  84}, {   6,  96}, {   1,  63},
	{  -5,  85}, { -13, 106}, {   5,  63}, {   6,  75}, {  -3,  90}, {  -1, 101},
	{   3,  55}, {  -4,  79}, {  -2,  75}, { -12,  97}, {  -7,  50}, {   1,  60},
	{   0,  41}, {   0,  63}, {   0,  63}, {   0,  63}, {  -9,  83}, {   4,  86},
	{   0,  97}, {  -7,  72}, {  13,  41}, {   3,  62}, {   7,  34}, {  -9,  88},
	{ -20, 127}, { -36, 127}, { -17,  91}, { -14,  95}, { -25,  84}, { -25,  86},
	{ -12,  89}, { -17,  91}, { -31, 127}, { -14,  76}, { -18, 103}, { -13,  90},
	{ -37, 127}, {  11,  80}, {   5,  76}, {   2,  84}, {   5,  78}, {  -6,  55},
	{   4,  61}, { -14,  83}, { -37, 127}, {  -5,  79}, { -11, 104}, { -11,  91},
	{ -30, 127}, {   0,  65}, {  -2,  79}, {   0,  72}, {  -4,  92}, {  -6,  56},
	{   3,  68}, {  -8,  71}, { -13,  98}, {  -4,  86}, { -12,  88}, {  -5,  82},
	{  -3,  72}, {  -4,  67}, {  -8,  72}, { -16,  89}, {  -9,  69}, {  -1,  59},
	{   5,  66}, {   4,  57}, {  -4,  71}, {  -2,  71}, {   2,  58}, {  -1,  74},
	{  -4,  44}, {  -1,  69}, {   0,  62}, {  -7,  51}, {  -4,  47}, {  -6,  42},
	{  -3,  41}, {  -6,  53}, {   8,  76}, {  -9,  78}, { -11,  83}, {   9,  52},
	{   0,  67}, {  -5,  90}, {   1,  67}, { -15,  72}, {  -5,  75}, {  -8,  80},
	{ -21,  83}, { -21,  64}, { -13,  31}, { -25,  64}, { -29,  94}, {   9,  75},
	{  17,  63}, {  -8,  74}, {  -5,  35}, {  -2,  27}, {  13,  91}, {   3,  65},
	{  -7,  69}, {   8,  77}, { -10,  66}, {   3,  62}, {  -3,  68}, { -20,  81},
	{   0,  30}, {   1,   7}, {  -3,  23}, { -21,  74}, {  16,  66}, { -23, 124},
	{  17,  37}, {  44, -18}, {  50, -34}, { -22, 127}, {   4,  39}, {   0,  42},
	{   7,  34}, {  11,  29}, {   8,  31}, {   6,  37}, {   7,  42}, {   3,  40},
	{   8,  33}, {  13,  43}, {  13,  36}, {   4,  47}, {   3,  55}, {   2,  58},
	{   6,  60}, {   8,  44}, {  11,  44}, {  14,  42}, {   7,  48}, {   4,  56},
	{   4,  52}, {  13,  37}, {   9,  49}, {  19,  58}, {  10,  48}, {  12,  45},
	{   0,  69}, {  20,  33}, {   8,  63}, {  35, -18}, {  33, -25}, {  28,  -3},
	{  24,  10}, {  27,   0}, {  34, -14}, {  52, -44}, {  39, -24}, {  19,  17},
	{  31,  25}, {  36,  29}, {  24,  33}, {  34,  15}, {  30,  20}, {  22,  73},
	{  20,  34}, {  19,  31}, {  27,  44}, {  19,  16}, {  15,  36}, {  15,  36},
	{  21,  28}, {  25,  21}, {  30,  20}, {  31,  12}, {  27,  16}, {  24,  42},
	{   0,  93}, {  14,  56}, {  15,  57}, {  26,  38}, { -24, 127}, { -24, 115},
	{ -22,  82}, {  -9,  62}, {   0,  53}, {   0,  59}, { -14,  85}, { -13,  89},
	{ -13,  94}, { -11,  92}, { -29, 127}, { -21, 100}, { -14,  57}, { -12,  67},
	{ -11,  71}, { -10,  77}, { -21,  85}, { -16,  88}, { -23, 104}, { -15,  98},
	{ -37, 127}, { -10,  82}, {  -8,  48}, {  -8,  61}, {  -8,  66}, {  -7,  70},
	{ -14,  75}, { -10,  79}, {  -9,  83}, { -12,  92}, { -18, 108}, {  -4,  79},
	{ -22,  69}, { -16,  75}, {  -2,  58}, {   1,  58}, { -13,  78}, {  -9,  83},
	{  -4,  81}, { -13,  99}, { -13,  81}, {  -6,  38}, { -13,  62}, {  -6,  58},
	{  -2,  59}, { -16,  73}, { -10,  76}, { -13,  86}, {  -9,  83}, { -10,  87},
	{   0,   0}, { -22, 127}, { -25, 127}, { -25, 120}, { -27, 127}, { -19, 114},
	{ -23, 117}, { -25, 118}, { -26, 117}, { -24, 113}, { -28, 118}, { -31, 120},
	{ -37, 124}, { -10,  94}, { -15, 102}, { -10,  99}, { -13, 106}, { -50, 127},
	{  -5,  92}, {  17,  57}, {  -5,  86}, { -13,  94}, { -12,  91}, {  -2,  77},
	{   0,  71}, {  -1,  73}, {   4,  64}, {  -7,  81}, {   5,  64}, {  15,  57},
	{   1,  67}, {   0,  68}, { -10,  67}, {   1,  68}, {   0,  77}, {   2,  64},
	{   0,  68}, {  -5,  78}, {   7,  55}, {   5,  59}, {   2,  65}, {  14,  54},
	{  15,  44}, {   5,  60}, {   2,  70}, {  -2,  76}, { -18,  86}, {  12,  70},
	{   5,  64}, { -12,  70}, {  11,  55}, {   5,  56}, {   0,  69}, {   2,  65},
	{  -6,  74}, {   5,  54}, {   7,  54}, {  -6,  76}, { -11,  82}, {  -2,  77},
	{  -2,  77}, {  25,  42}, {  17, -13}, {  16,  -9}, {  17, -12}, {  27, -21},
	{  37, -30}, {  41, -40}, {  42, -41}, {  48, -47}, {  39, -32}, {  46, -40},
	{  52, -51}, {  46, -41}, {  52, -39}, {  43, -19}, {  32,  11}, {  61, -55},
	{  56, -46}, {  62, -50}, {  81, -67}, {  45, -20}, {  35,  -2}, {  28,  15},
	{  34,   1}, {  39,   1}, {  30,  17}, {  20,  38}, {  18,  45}, {  15,  54},
	{   0,  79}, {  36, -16}, {  37, -14}, {  37, -17}, {  32,   1}, {  34,  15},
	{  29,  15}, {  24,  25}, {  34,  22}, {  31,  16}, {  35,  18}, {  31,  28},
	{  33,  41}, {  36,  28}, {  27,  47}, {  21,  62}, {  18,  31}, {  19,  26},
	{  36,  24}, {  24,  23}, {  27,  16}, {  24,  30}, {  31,  29}, {  22,  41},
	{  22,  42}, {  16,  60}, {  15,  52}, {  14,  60}, {   3,  78}, { -16, 123},
	{  21,  53}, {  22,  56}, {  25,  61}, {  21,  33}, {  19,  50}, {  17,  61},
	{  -3,  78}, {  -8,  74}, {  -9,  72}, { -10,  72}, { -18,  75}, { -12,  71},
	{ -11,  63}, {  -5,  70}, { -17,  75}, { -14,  72}, { -16,  67}, {  -8,  53},
	{ -14,  59}, {  -9,  52}, { -11,  68}, {   9,  -2}, {  30, -10}, {  31,  -4},
	{  33,  -1}, {  33,   7}, {  31,  12}, {  37,  23}, {  31,  38}, {  20,  64},
	{  -9,  71}, {  -7,  37}, {  -8,  44}, { -11,  49}, { -10,  56}, { -12,  59},
	{  -8,  63}, {  -9,  67}, {  -6,  68}, { -10,  79}, {  -3,  78}, {  -8,  74},
	{  -9,  72}, { -10,  72}, { -18,  75}, { -12,  71}, { -11,  63}, {  -5,  70},
	{ -17,  75}, { -14,  72}, { -16,  67}, {  -8,  53}, { -14,  59}, {  -9,  52},
	{ -11,  68}, {   9,  -2}, {  30, -10}, {  31,  -4}, {  33,  -1}, {  33,   7},
	{  31,  12}, {  37,  23}, {  31,  38}, {  20,  64}, {  11,  80}, {   5,  76},
	{   2,  84}, {   5,  78}, {  -6,  55}, {   4,  61}, { -14,  83}, { -37, 127},
	{  -5,  79}, { -11, 104}, { -11,  91}, { -30, 127}, {  11,  80}, {   5,  76},
	{   2,  84}, {   5,  78}, {  -6,  55}, {   4,  61}, { -14,  83}, { -37, 127},
	{  -5,  79}, { -11, 104}, { -11,  91}, { -30, 127}, {  -4,  86}, { -12,  88},
	{  -5,  82}, {  -3,  72}, {  -4,  67}, {  -8,  72}, { -16,  89}, {  -9,  69},
	{  -1,  59}, {   5,  66}, {   4,  57}, {  -4,  71}, {  -2,  71}, {   2,  58},
	{  -1,  74}, {  -4,  44}, {  -1,  69}, {   0,  62}, {  -7,  51}, {  -4,  47},
	{  -6,  42}, {  -3,  41}, {  -6,  53}, {   8,  76}, {  -9,  78}, { -11,  83},
	{   9,  52}, {   0,  67}, {  -5,  90}, {   1,  67}, { -15,  72}, {  -5,  75},
	{  -8,  80}, { -21,  83}, { -21,  64}, { -13,  31}, { -25,  64}, { -29,  94},
	{   9,  75}, {  17,  63}, {  -8,  74}, {  -5,  35}, {  -2,  27}, {  13,  91},
	{  -4,  86}, { -12,  88}, {  -5,  82}, {  -3,  72}, {  -4,  67}, {  -8,  72},
	{ -16,  89}, {  -9,  69}, {  -1,  59}, {   5,  66}, {   4,  57}, {  -4,  71},
	{  -2,  71}, {   2,  58}, {  -1,  74}, {  -4,  44}, {  -1,  69}, {   0,  62},
	{  -7,  51}, {  -4,  47}, {  -6,  42}, {  -3,  41}, {  -6,  53}, {   8,  76},
	{  -9,  78}, { -11,  83}, {   9,  52}, {   0,  67}, {  -5,  90}, {   1,  67},
	{ -15,  72}, {  -5,  75}, {  -8,  80}, { -21,  83}, { -21,  64}, { -13,  31},
	{ -25,  64}, { -29,  94}, {   9,  75}, {  17,  63}, {  -8,  74}, {  -5,  35},
	{  -2,  27}, {  13,  91}, {   4,  39}, {   0,  42}, {   7,  34}, {  11,  29},
	{   8,  31}, {   6,  37}, {   7,  42}, {   3,  40}, {   8,  33}, {  13,  43},
	{  13,  36}, {   4,  47}, {   3,  55}, {   2,  58}, {   6,  60}, {   8,  44},
	{  11,  44}, {  14,  42}, {   7,  48}, {   4,  56}, {   4,  52}, {  13,  37},
	{   9,  49}, {  19,  58}, {  10,  48}, {  12,  45}, {   0,  69}, {  20,  33},
	{   8,  63}, {  35, -18}, {  33, -25}, {  28,  -3}, {  24,  10}, {  27,   0},
	{  34, -14}, {  52, -44}, {  39, -24}, {  19,  17}, {  31,  25}, {  36,  29},
	{  24,  33}, {  34,  15}, {  30,  20}, {  22,  73}, {   4,  39}, {   0,  42},
	{   7,  34}, {  11,  29}, {   8,  31}, {   6,  37}, {   7,  42}, {   3,  40},
	{   8,  33}, {  13,  43}, {  13,  36}, {   4,  47}, {   3,  55}, {   2,  58},
	{   6,  60}, {   8,  44}, {  11,  44}, {  14,  42}, {   7,  48}, {   4,  56},
	{   4,  52}, {  13,  37}, {   9,  49}, {  19,  58}, {  10,  48}, {  12,  45},
	{   0,  69}, {  20,  33}, {   8,  63}, {  35, -18}, {  33, -25}, {  28,  -3},
	{  24,  10}, {  27,   0}, {  34, -14}, {  52, -44}, {  39, -24}, {  19,  17},
	{  31,  25}, {  36,  29}, {  24,  33}, {  34,  15}, {  30,  20}, {  22,  73},
	{  -3,  78}, {  -8,  74}, {  -9,  72}, { -10,  72}, { -18,  75}, { -12,  71},
	{ -11,  63}, {  -5,  70}, { -17,  75}, { -14,  72}, { -16,  67}, {  -8,  53},
	{ -14,  59}, {  -9,  52}, { -11,  68}, {  -3,  78}, {  -8,  74}, {  -9,  72},
	{ -10,  72}, { -18,  75}, { -12,  71}, { -11,  63}, {  -5,  70}, { -17,  75},
	{ -14,  72}, { -16,  67}, {  -8,  53}, { -14,  59}, {  -9,  52}, { -11,  68},
	{   9,  -2}, {  30, -10}, {  31,  -4}, {  33,  -1}, {  33,   7}, {  31,  12},
	{  37,  23}, {  31,  38}, {  20,  64}, {   9,  -2}, {  30, -10}, {  31,  -4},
	{  33,  -1}, {  33,   7}, {  31,  12}, {  37,  23}, {  31,  38}, {  20,  64},
	{  -9,  71}, {  -7,  37}, {  -8,  44}, { -11,  49}, { -10,  56}, { -12,  59},
	{  -8,  63}, {  -9,  67}, {  -6,  68}, { -10,  79}, {  -3,  78}, {  -8,  74},
	{  -9,  72}, { -10,  72}, { -18,  75}, { -12,  71}, { -11,  63}, {  -5,  70},
	{ -17,  75}, { -14,  72}, { -16,  67}, {  -8,  53}, { -14,  59}, {  -9,  52},
	{ -11,  68}, {  -3,  78}, {  -8,  74}, {  -9,  72}, { -10,  72}, { -18,  75},
	{ -12,  71}, { -11,  63}, {  -5,  70}, { -17,  75}, { -14,  72}, { -16,  67},
	{  -8,  53}, { -14,  59}, {  -9,  52}, { -11,  68}, {   9,  -2}, {  30, -10},
	{  31,  -4}, {  33,  -1}, {  33,   7}, {  31,  12}, {  37,  23}, {  31,  38},
	{  20,  64}, {   9,  -2}, {  30, -10}, {  31,  -4}, {  33,  -1}, {  33,   7},
	{  31,  12}, {  37,  23}, {  31,  38}, {  20,  64}, {  -9,  71}, {  -7,  37},
	{  -8,  44}, { -11,  49}, { -10,  56}, { -12,  59}, {  -8,  63}, {  -9,  67},
	{  -6,  68}, { -10,  79}, { -22, 127}, { -25, 127}, { -25, 120}, { -27, 127},
	{ -19, 114}, { -23, 117}, { -25, 118}, { -26, 117}, { -24, 113}, { -28, 118},
	{ -31, 120}, { -37, 124}, { -10,  94}, { -15, 102}, { -10,  99}, { -13, 106},
	{ -50, 127}, {  -5,  92}, {  17,  57}, {  -5,  86}, { -13,  94}, { -12,  91},
	{  -2,  77}, {   0,  71}, {  -1,  73}, {   4,  64}, {  -7,  81}, {   5,  64},
	{  15,  57}, {   1,  67}, {   0,  68}, { -10,  67}, {   1,  68}, {   0,  77},
	{   2,  64}, {   0,  68}, {  -5,  78}, {   7,  55}, {   5,  59}, {   2,  65},
	{  14,  54}, {  15,  44}, {   5,  60}, {   2,  70}, { -22, 127}, { -25, 127},
	{ -25, 120}, { -27, 127}, { -19, 114}, { -23, 117}, { -25, 118}, { -26, 117},
	{ -24, 113}, { -28, 118}, { -31, 120}, { -37, 124}, { -10,  94}, { -15, 102},
	{ -10,  99}, { -13, 106}, { -50, 127}, {  -5,  92}, {  17,  57}, {  -5,  86},
	{ -13,  94}, { -12,  91}, {  -2,  77}, {   0,  71}, {  -1,  73}, {   4,  64},
	{  -7,  81}, {   5,  64}, {  15,  57}, {   1,  67}, {   0,  68}, { -10,  67},
	{   1,  68}, {   0,  77}, {   2,  64}, {   0,  68}, {  -5,  78}, {   7,  55},
	{   5,  59}, {   2,  65}, {  14,  54}, {  15,  44}, {   5,  60}, {   2,  70},
	{  17, -13}, {  16,  -9}, {  17, -12}, {  27, -21}, {  37, -30}, {  41, -40},
	{  42, -41}, {  48, -47}, {  39, -32}, {  46, -40}, {  52, -51}, {  46, -41},
	{  52, -39}, {  43, -19}, {  32,  11}, {  61, -55}, {  56, -46}, {  62, -50},
	{  81, -67}, {  45, -20}, {  35,  -2}, {  28,  15}, {  34,   1}, {  39,   1},
	{  30,  17}, {  20,  38}, {  18,  45}, {  15,  54}, {   0,  79}, {  36, -16},
	{  37, -14}, {  37, -17}, {  32,   1}, {  34,  15}, {  29,  15}, {  24,  25},
	{  34,  22}, {  31,  16}, {  35,  18}, {  31,  28}, {  33,  41}, {  36,  28},
	{  27,  47}, {  21,  62}, {  17, -13}, {  16,  -9}, {  17, -12}, {  27, -21},
	{  37, -30}, {  41, -40}, {  42, -41}, {  48, -47}, {  39, -32}, {  46, -40},
	{  52, -51}, {  46, -41}, {  52, -39}, {  43, -19}, {  32,  11}, {  61, -55},
	{  56, -46}, {  62, -50}, {  81, -67}, {  45, -20}, {  35,  -2}, {  28,  15},
	{  34,   1}, {  39,   1}, {  30,  17}, {  20,  38}, {  18,  45}, {  15,  54},
	{   0,  79}, {  36, -16}, {  37, -14}, {  37, -17}, {  32,   1}, {  34,  15},
	{  29,  15}, {  24,  25}, {  34,  22}, {  31,  16}, {  35,  18}, {  31,  28},
	{  33,  41}, {  36,  28}, {  27,  47}, {  21,  62}, { -24, 115}, { -22,  82},
	{  -9,  62}, {   0,  53}, {   0,  59}, { -14,  85}, { -13,  89}, { -13,  94},
	{ -11,  92}, { -29, 127}, { -21, 100}, { -14,  57}, { -12,  67}, { -11,  71},
	{ -10,  77}, { -21,  85}, { -16,  88}, { -23, 104}, { -15,  98}, { -37, 127},
	{ -10,  82}, {  -8,  48}, {  -8,  61}, {  -8,  66}, {  -7,  70}, { -14,  75},
	{ -10,  79}, {  -9,  83}, { -12,  92}, { -18, 108}, { -24, 115}, { -22,  82},
	{  -9,  62}, {   0,  53}, {   0,  59}, { -14,  85}, { -13,  89}, { -13,  94},
	{ -11,  92}, { -29, 127}, { -21, 100}, { -14,  57}, { -12,  67}, { -11,  71},
	{ -10,  77}, { -21,  85}, { -16,  88}, { -23, 104}, { -15,  98}, { -37, 127},
	{ -10,  82}, {  -8,  48}, {  -8,  61}, {  -8,  66}, {  -7,  70}, { -14,  75},
	{ -10,  79}, {  -9,  83}, { -12,  92}, { -18, 108}, {  -5,  79}, { -11, 104},
	{ -11,  91}, { -30, 127}, {  -5,  79}, { -11, 104}, { -11,  91}, { -30, 127},
	{  -5,  79}, { -11, 104}, { -11,  91}, { -30, 127},
}};
