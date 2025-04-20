static int parse_buffering_period(Edge264Decoder *dec) {
	if (get_ue16(&dec->_gb, 31))
		return ENOTSUP;
	log_dec(dec, "    nal_hrd_parameters:\n");
	for (int i = dec->sps.nal_hrd_cpb_cnt; i--; ) {
		int initial_cpb_removal_delay = get_uv(&dec->_gb, dec->sps.initial_cpb_removal_delay_length);
		int initial_cpb_removal_delay_offset = get_uv(&dec->_gb, dec->sps.initial_cpb_removal_delay_length);
		log_dec(dec, "      - initial_cpb_removal_delay: %u\n"
			"        initial_cpb_removal_delay_offset: %u\n",
			initial_cpb_removal_delay, initial_cpb_removal_delay_offset);
	}
	log_dec(dec, "    vcl_hrd_parameters:\n");
	for (int i = dec->sps.vcl_hrd_cpb_cnt; i--; ) {
		int initial_cpb_removal_delay = get_uv(&dec->_gb, dec->sps.initial_cpb_removal_delay_length);
		int initial_cpb_removal_delay_offset = get_uv(&dec->_gb, dec->sps.initial_cpb_removal_delay_length);
		log_dec(dec, "      - initial_cpb_removal_delay: %u\n"
			"        initial_cpb_removal_delay_offset: %u\n",
			initial_cpb_removal_delay, initial_cpb_removal_delay_offset);
	}
	return 0;
}



static int parse_pic_timing(Edge264Decoder *dec) {
	static const char * const pic_struct_names[16] = {
		"progressive frame", "top field", "bottom field", "top then bottom",
		"bottom then top", "top then bottom then top",
		"bottom then top then bottom", "frame doubling", "frame tripling",
		[9 ... 15] = "Unknown"};
	static const char * const ct_type_names[4] = {
		"progressive", "interlaced", [2 ... 3] = "unknown"};
	
	if (dec->sps.nal_hrd_cpb_cnt | dec->sps.vcl_hrd_cpb_cnt) {
		int cpb_removal_delay = get_uv(&dec->_gb, dec->sps.cpb_removal_delay_length);
		int dpb_output_delay = get_uv(&dec->_gb, dec->sps.dpb_output_delay_length);
		log_dec(dec, "    cpb_removal_delay: %u\n"
			"    dpb_output_delay: %u\n",
			cpb_removal_delay, dpb_output_delay);
	}
	if (dec->sps.pic_struct_present_flag) {
		int pic_struct = get_uv(&dec->_gb, 4);
		int NumClockTS = 0x3be95 >> (pic_struct * 2) & 3;
		log_dec(dec, "    pic_struct: %s (%u)\n",
			pic_struct_names[pic_struct], pic_struct);
		log_dec(dec, "    clock_timestamps:\n");
		while (NumClockTS--) {
			if (!get_u1(&dec->_gb)) // clock_timestamp_flag
				continue;
			unsigned u = get_uv(&dec->_gb, 19);
			if (u & 1 << 10) {
				unsigned v = get_uv(&dec->_gb, 17);
				dec->sS = v >> 11;
				dec->mM = v >> 5 & 0x3f;
				dec->hH = v & 0x1f;
			} else if (get_u1(&dec->_gb)) { // seconds_flag
				unsigned w = get_uv(&dec->_gb, 7);
				dec->sS = w >> 1;
				if (w & 1) {
					unsigned x = get_uv(&dec->_gb, 7);
					dec->mM = x >> 1;
					if (x & 1)
						dec->hH = get_uv(&dec->_gb, 5);
				}
			}
			int tOffset = 0;
			if (dec->sps.time_offset_length)
				tOffset = get_uv(&dec->_gb, dec->sps.time_offset_length);
			log_dec(dec, "      - scan_type: %s (%u)\n"
				"        discontinuity_flag: %u\n"
				"        clockTimestamp: \"%02u:%02u:%02u+%u/%u\"\n",
				ct_type_names[u >> 17], u >> 17,
				u >> 9 & 1,
				dec->hH, dec->mM, dec->sS, (u & 0xff) * (dec->sps.num_units_in_tick * (1 + (u >> 16 & 1))) + tOffset, dec->sps.time_scale);
		}
	}
	return 0;
}



static int parse_pan_scan_rect(Edge264Decoder *dec) {
	int pan_scan_rect_id = get_ue32(&dec->_gb, 4294967294);
	int pan_scan_rect_cancel_flag = get_u1(&dec->_gb);
	log_dec(dec, "    pan_scan_rect_id: %u\n"
		"    pan_scan_rect_cancel_flag: %u\n",
		pan_scan_rect_id, pan_scan_rect_cancel_flag);
	if (!pan_scan_rect_cancel_flag) {
		log_dec(dec, "    fields:\n");
		for (int i = get_ue16(&dec->_gb, 2) + 1; i--; ) {
			int pan_scan_rect_left_offset = get_se32(&dec->_gb, -2147483647, 2147483647);
			int pan_scan_rect_right_offset = get_se32(&dec->_gb, -2147483647, 2147483647);
			int pan_scan_rect_top_offset = get_se32(&dec->_gb, -2147483647, 2147483647);
			int pan_scan_rect_bottom_offset = get_se32(&dec->_gb, -2147483647, 2147483647);
			log_dec(dec, "      - {left: %d, right: %d, top: %d, bottom: %d}\n",
				pan_scan_rect_left_offset, pan_scan_rect_right_offset, pan_scan_rect_top_offset, pan_scan_rect_bottom_offset);
		}
		int pan_scan_rect_repetition_period = get_ue16(&dec->_gb, 16384);
		log_dec(dec, "    pan_scan_rect_repetition_period: %u\n",
			pan_scan_rect_repetition_period);
	}
	return 0;
}



typedef int (*SEI_Parser)(Edge264Decoder *dec);
int ADD_VARIANT(parse_sei)(Edge264Decoder *dec, int non_blocking, void(*free_cb)(void*,int), void *free_arg)
{
	static const char * const payloadType_names[206] = {
		[0 ... 205] = "Unknown",
		[0] = "Buffering period",
		[1] = "Picture timing",
		[2] = "Pan-scan rectangle",
	};
	static const SEI_Parser parse_sei_message[206] = {
		[0] = parse_buffering_period,
		[1] = parse_pic_timing,
		[2] = parse_pan_scan_rect,
	};
	
	refill(&dec->_gb, 0);
	log_dec(dec, "  sei_messages:\n");
	if (print_dec(dec, dec->log_header_arg))
		return ENOTSUP;
	int ret = 0;
	do {
		int byte, payloadType = 0, payloadSize = 0;
		do {
			byte = get_uv(&dec->_gb, 8);
			payloadType += byte;
		} while (byte == 255);
		do {
			byte = get_uv(&dec->_gb, 8);
			payloadSize += byte;
		} while (byte == 255);
		log_dec(dec, "  - payloadType: %s (%u)\n",
			payloadType_names[payloadType], payloadType);
		Edge264GetBits start = dec->_gb;
		int res = ENOTSUP;
		if (payloadType <= 205 && parse_sei_message[payloadType])
			res = parse_sei_message[payloadType](dec);
		if (res) {
			ret = ENOTSUP;
			dec->_gb = start;
			while (payloadSize-- > 0)
				get_uv(&dec->_gb, 8);
		} else {
			int skip = (SIZE_BIT - 1 - ctz(dec->_gb.lsb_cache)) & 7;
			if (skip)
				get_uv(&dec->_gb, skip);
		}
		if (print_dec(dec, dec->log_sei_arg))
			return ENOTSUP;
	} while (dec->_gb.msb_cache << 1 || (dec->_gb.lsb_cache & (dec->_gb.lsb_cache - 1)) || (intptr_t)(dec->_gb.end - dec->_gb.CPB) > 0);
	return (dec->_gb.msb_cache & (size_t)1 << (SIZE_BIT - 1)) ? ret : EBADMSG;
}
