#include "edge264_internal.h"


/* Covered by formal proof */
const uint8_t *edge264_find_start_code(
		const uint8_t *buf,
		const uint8_t *end,
		int four_byte)
{
	if (buf >= end) return end;
	int nb_zeros = four_byte ? 3 : 2;
	#if SIMD != NONE
		const uint8_t *base = (const uint8_t *)((uintptr_t)buf & -16);
		const uint8_t *p = base;
		i8x16 v = *(const i8x16 *)p;
		i8x16 zero = {};
		i8x16 lo0 = zero;
		i8x16 hi0 = (v == zero) & shlv128(set8(-1), (uintptr_t)buf & 15);
		i8x16 c1 = set8(1);
		while (1) {
			i8x16 mask = shrd128(lo0, hi0, 14) & shrd128(lo0, hi0, 15) & (v == c1);
			if (four_byte)
				mask &= shrd128(lo0, hi0, 13);
			#if SIMD == SSE
				unsigned m = movemask(mask);
				if (m) {
					ptrdiff_t i = p - base + __builtin_ctz(m) - nb_zeros;
					p = base + maxt(i < end - base - nb_zeros ? i : end - base, buf - base);
					break;
				}
			#else
				uint64_t m = shrlou16(mask, 4);
				if (m) {
					ptrdiff_t i = p - base + (__builtin_ctzll(m) >> 2) - nb_zeros;
					p = base + maxt(i < end - base - nb_zeros ? i : end - base, buf - base);
					break;
				}
			#endif
			p += 16;
			if (p >= end) {
				p = end;
				break;
			}
			v = *(const i8x16 *)p;
			lo0 = hi0;
			hi0 = (v == zero);
		}
	#endif
	
	#if SIMD == CBMC || SIMD == NONE
		const uint8_t *ref_p = end;
		for (ptrdiff_t i = 0; i < end - buf - nb_zeros; i++) {
			int match = four_byte
				? (buf[i]==0 && buf[i+1]==0 && buf[i+2]==0 && buf[i+3]==1)
				: (buf[i]==0 && buf[i+1]==0 && buf[i+2]==1);
			if (match) {
				ref_p = buf + i;
				break;
			}
		}
		#if SIMD == CBMC
			assert(p == ref_p);
		#else
			const uint8_t *p = ref_p;
		#endif
	#endif
	return p;
}



static void internal_alloc(void **samples, unsigned samples_size, void **mbs, unsigned mbs_size, int errno_on_fail, void *alloc_arg) {
	*samples = aligned_alloc(16, samples_size + mbs_size);
	*mbs = *samples + samples_size;
}

static void internal_free(void *samples, void *mbs, void *alloc_arg) {
	aligned_free(samples);
}

static int ignore_NAL(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg) {
	return 0;
}

static int unsup_NAL(Edge264Decoder *dec, Edge264UnrefCb unref_cb, void *unref_arg) {
	return ENOTSUP;
}



Edge264Decoder *edge264_alloc(int n_threads, Edge264LogCb log_cb, void *log_arg, int log_mbs, Edge264AllocCb alloc_cb, Edge264FreeCb free_cb, void *alloc_arg) {
	Edge264Decoder *dec = aligned_alloc(64, sizeof(*dec)); // maximal SIMD type alignment used in edge264
	if (dec == NULL)
		return NULL;
	memset(dec, 0, sizeof(*dec));
	dec->log_base_us = get_relative_time_us();
	dec->currPic = dec->basePic = -1;
	dec->PrevRefFrameNum[0] = dec->PrevRefFrameNum[1] = dec->prevFrameId = -1;
	dec->taskPics_v = dec->output_queue_v[0][0] = dec->output_queue_v[0][1] =
		dec->output_queue_v[1][0] = dec->output_queue_v[1][1] = set8(-1);
	dec->n_threads = n_threads;
	dec->max_output_latency = 16;
	dec->alloc_cb = alloc_cb && free_cb ? alloc_cb : internal_alloc;
	dec->free_cb = alloc_cb && free_cb ? free_cb : internal_free;
	dec->alloc_arg = alloc_arg;
	dec->log_cb = log_cb;
	dec->log_arg = log_arg;
	
	// select parser functions based on CPU capabilities and logs mode
	dec->worker_loop = ADD_VARIANT(worker_loop);
	for (int i = 0; i < 32; i++)
		dec->parse_nal_unit[i] = unsup_NAL;
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
			dec->parse_nal_unit[10] = parse_end_of_sequence_v2;
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
			dec->parse_nal_unit[10] = parse_end_of_sequence_v3;
			dec->parse_nal_unit[14] = dec->parse_nal_unit[20] = parse_nal_unit_header_extension_v3;
		}
	#endif
	#ifdef HAS_LOGS
		if (log_cb) {
			for (int i = 0; i < 32; i++)
				dec->parse_nal_unit[i] = unsup_NAL_log;
			dec->parse_nal_unit[1] = dec->parse_nal_unit[5] = parse_slice_layer_without_partitioning_log;
			dec->parse_nal_unit[6] = parse_sei_log;
			dec->parse_nal_unit[7] = dec->parse_nal_unit[15] = parse_seq_parameter_set_log;
			dec->parse_nal_unit[8] = parse_pic_parameter_set_log;
			dec->parse_nal_unit[9] = parse_access_unit_delimiter_log;
			dec->parse_nal_unit[10] = parse_end_of_sequence_log;
			dec->parse_nal_unit[11] = dec->parse_nal_unit[12] = ignore_NAL_log;
			dec->parse_nal_unit[14] = dec->parse_nal_unit[20] = parse_nal_unit_header_extension_log;
			if (log_mbs)
				dec->worker_loop = worker_loop_log;
		}
	#else
		if (log_cb)
			return aligned_free(dec), NULL;
	#endif
	
	// get the number of logical cores if requested
	if (n_threads < 0) {
		#ifdef _WIN32
			int n_cpus = atoi(getenv("NUMBER_OF_PROCESSORS"));
		#else
			int n_cpus = sysconf(_SC_NPROCESSORS_ONLN);
		#endif
		n_threads = mins(n_cpus, 16);
	}
	
	// if multithreading is disabled we are done, otherwise initialize all
	if (n_threads == 0)
		return dec;
	if (pthread_mutex_init(&dec->lock, NULL) == 0) {
		if (pthread_cond_init(&dec->task_ready, NULL) == 0) {
			if (pthread_cond_init(&dec->task_progress, NULL) == 0) {
				if (pthread_cond_init(&dec->task_complete, NULL) == 0) {
					int i = 0;
					while (i < n_threads && pthread_create(&dec->threads[i], NULL, dec->worker_loop, (void *)((uintptr_t)dec + i)) == 0)
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
	aligned_free(dec);
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
		for (int i = 0; i < 64; i++) {
			if (dec->samples_buffers[i] != NULL)
				dec->free_cb(dec->samples_buffers[i], dec->mb_buffers[i], dec->alloc_arg);
		}
		aligned_free(dec);
	}
}



int edge264_decode_NAL(Edge264Decoder *dec, const uint8_t *buf, const uint8_t *end, Edge264UnrefCb unref_cb, void *unref_arg)
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
	if (dec == NULL || buf == NULL)
		return EINVAL;
	if (dec->n_threads)
		pthread_mutex_lock(&dec->lock);
	
	// bump all frames at the end of buffer
	if (__builtin_expect(buf >= end, 0)) {
		int ret = bump_all_frames(dec);
		if (dec->n_threads)
			pthread_mutex_unlock(&dec->lock);
		return ret ?: ENODATA;
	}
	
	// prefill the bitstream cache while parsing the NAL byte header
	dec->gb.CPB = buf;
	dec->gb.end = end;
	dec->gb.msb_cache = (size_t)1 << (SIZE_BIT - 1);
	refill(&dec->gb, 0);
	dec->nal_ref_idc = dec->gb.msb_cache >> (SIZE_BIT - 3);
	dec->nal_unit_type = dec->gb.msb_cache >> (SIZE_BIT - 8) & 0x1f;
	dec->gb.msb_cache = dec->gb.msb_cache << 8 | 1 << 7;
	refill(&dec->gb, 0);
	Parser parser = dec->parse_nal_unit[dec->nal_unit_type];
	if (dec->log_cb) {
		dec->log_pos = snprintf(dec->log_buf, sizeof(dec->log_buf),
			"\n- nal_ref_idc: %u\n"
			"  nal_unit_type: %u # %s%s\n",
			dec->nal_ref_idc,
			dec->nal_unit_type, nal_unit_type_names[dec->nal_unit_type], unsup_if(!parser));
	}
	int ret = parser(dec, unref_cb, unref_arg);
	// printf("nal_unit_type=%d, ret=%d\n\n", dec->nal_unit_type, ret);
	
	// on non-slice successful return we release the NAL buffer
	if (unref_cb && ret == 0 && !(0x100022 & 1 << dec->nal_unit_type)) // 1, 5 or 20
		unref_cb(ret, unref_arg);
	if (dec->n_threads)
		pthread_mutex_unlock(&dec->lock);
	return ret;
}



int edge264_get_frame(Edge264Decoder *dec, Edge264Frame *out, int borrow) {
	if (dec == NULL || out == NULL)
		return EINVAL;
	if (dec->n_threads)
		pthread_mutex_lock(&dec->lock);
	int idx0 = __builtin_ctzg(movemask(dec->output_queue_v[0][0]) | movemask(dec->output_queue_v[0][1]) << 16, 32) - 1;
	int idx1 = __builtin_ctzg(movemask(dec->output_queue_v[1][0]) | movemask(dec->output_queue_v[1][1]) << 16, 32) - 1;
	int pic0, pic1, res = ENOMSG;
	if (idx0 >= 0 && dec->next_deblock_addr[pic0 = dec->output_queue[0][idx0]] == INT_MAX &&
		(dec->ssps.BitDepth_Y == 0 || (idx1 >= 0 && dec->next_deblock_addr[pic1 = dec->output_queue[1][idx1]] == INT_MAX))) {
		dec->output_queue[0][idx0] = -1;
		memcpy(out, &dec->out, sizeof(*out)); // GCC-14 crashes on dec->out = format
		int top = dec->out.frame_crop_offsets[0];
		int left = dec->out.frame_crop_offsets[3];
		int offY = top * dec->out.stride_Y + (dec->out.bit_depth_Y == 8 ? left : left << 1);
		int topC = dec->sps.chroma_format_idc == 3 ? top : top >> 1;
		int leftC = dec->sps.chroma_format_idc == 1 ? left >> 1 : left;
		int offC = dec->plane_size_Y + topC * dec->out.stride_C + (dec->out.bit_depth_C == 8 ? leftC : leftC << 1);
		assert(dec->to_get_frames_l & dec->output_frames_l & 1ull << pic0);
		dec->to_get_frames_l &= ~(1ull << pic0);
		out->samples[0] = dec->samples_buffers[pic0] + offY;
		out->samples[1] = dec->samples_buffers[pic0] + offC;
		out->samples[2] = dec->samples_buffers[pic0] + offC + (dec->out.stride_C >> 1);
		out->FrameId = dec->FrameIds[pic0];
		if (idx1 >= 0) {
			dec->output_queue[1][idx1] = -1;
			assert(dec->to_get_frames_l & dec->output_frames_l & 1ull << pic1);
			dec->to_get_frames_l ^= 1ull << pic1;
			out->samples_mvc[0] = dec->samples_buffers[pic1] + offY;
			out->samples_mvc[1] = dec->samples_buffers[pic1] + offC;
			out->samples_mvc[2] = dec->samples_buffers[pic1] + offC + (dec->out.stride_C >> 1);
			out->FrameId_mvc = dec->FrameIds[pic1];
		} else {
			pic1 = pic0;
		}
		out->return_arg = pic0 | pic1 << 8; // pic1=pic0 without second view, so that both can always be returned
		res = 0;
		if (!borrow)
			dec->output_frames_l &= ~(1ull << pic0 | 1ull << pic1);
	}
	if (dec->n_threads)
		pthread_mutex_unlock(&dec->lock);
	return res;
}



void edge264_return_frame(Edge264Decoder *dec, uintptr_t return_arg) {
	if (dec != NULL)
		dec->output_frames_l &= ~(1ull << (return_arg & 63) | 1ull << (return_arg >> 8 & 63));
}
