#!/usr/bin/env python3
from functools import reduce
from os import path
import re
import sys
from time import process_time
from types import SimpleNamespace
import yaml

# make yaml use SimpleNamespace instead of dict
class SafeConstructor(yaml.loader.SafeConstructor):
	def construct_yaml_map(self, node):
		data = SimpleNamespace()
		yield data
		value = self.construct_mapping(node)
		vars(data).update(value)
SafeConstructor.add_constructor(u'tag:yaml.org,2002:map', SafeConstructor.construct_yaml_map)
class SafeLoader(yaml.loader.Reader, yaml.loader.Scanner, yaml.loader.Parser, yaml.loader.Composer, SafeConstructor, yaml.loader.Resolver):
	def __init__(self, stream):
		yaml.loader.Reader.__init__(self, stream)
		yaml.loader.Scanner.__init__(self)
		yaml.loader.Parser.__init__(self)
		yaml.loader.Composer.__init__(self)
		SafeConstructor.__init__(self)
		yaml.loader.Resolver.__init__(self)



def ctz(v):
	return (v & -v).bit_length() - 1

def gen_ue(bits, v):
	v += 1
	return bits << (v.bit_length() * 2 - 1) | v

def gen_se(bits, v):
	v = v * 2 if v > 0 else -v * 2 + 1
	return bits << (v.bit_length() * 2 - 1) | v



def gen_residual_block_cavlc(bits, nC, coeffs):
	ct02 = [['1'], ['000101', '01'], ['00000111', '000100', '001'], ['000000111', '00000110', '0000101', '00011'], ['0000000111', '000000110', '00000101', '000011'], ['00000000111', '0000000110', '000000101', '0000100'], ['0000000001111', '00000000110', '0000000101', '00000100'], ['0000000001011', '0000000001110', '00000000101', '000000100'], ['0000000001000', '0000000001010', '0000000001101', '0000000100'], ['00000000001111', '00000000001110', '0000000001001', '00000000100'], ['00000000001011', '00000000001010', '00000000001101', '0000000001100'], ['000000000001111', '000000000001110', '00000000001001', '00000000001100'], ['000000000001011', '000000000001010', '000000000001101', '00000000001000'], ['0000000000001111', '000000000000001', '000000000001001', '000000000001100'], ['0000000000001011', '0000000000001110', '0000000000001101', '000000000001000'], ['0000000000000111', '0000000000001010', '0000000000001001', '0000000000001100'], ['0000000000000100', '0000000000000110', '0000000000000101', '0000000000001000']]
	ct24 = [['11'], ['001011', '10'], ['000111', '00111', '011'], ['0000111', '001010', '001001', '0101'], ['00000111', '000110', '000101', '0100'], ['00000100', '0000110', '0000101', '00110'], ['000000111', '00000110', '00000101', '001000'], ['00000001111', '000000110', '000000101', '000100'], ['00000001011', '00000001110', '00000001101', '0000100'], ['000000001111', '00000001010', '00000001001', '000000100'], ['000000001011', '000000001110', '000000001101', '00000001100'], ['000000001000', '000000001010', '000000001001', '00000001000'], ['0000000001111', '0000000001110', '0000000001101', '000000001100'], ['0000000001011', '0000000001010', '0000000001001', '0000000001100'], ['0000000000111', '00000000001011', '0000000000110', '0000000001000'], ['00000000001001', '00000000001000', '00000000001010', '0000000000001'], ['00000000000111', '00000000000110', '00000000000101', '00000000000100']]
	ct48 = [['1111'], ['001111', '1110'], ['001011', '01111', '1101'], ['001000', '01100', '01110', '1100'], ['0001111', '01010', '01011', '1011'], ['0001011', '01000', '01001', '1010'], ['0001001', '001110', '001101', '1001'], ['0001000', '001010', '001001', '1000'], ['00001111', '0001110', '0001101', '01101'], ['00001011', '00001110', '0001010', '001100'], ['000001111', '00001010', '00001101', '0001100'], ['000001011', '000001110', '00001001', '00001100'], ['000001000', '000001010', '000001101', '00001000'], ['0000001101', '000000111', '000001001', '000001100'], ['0000001001', '0000001100', '0000001011', '0000001010'], ['0000000101', '0000001000', '0000000111', '0000000110'], ['0000000001', '0000000100', '0000000011', '0000000010']]
	ct8 = [['000011'], ['000000', '000001'], ['000100', '000101', '000110'], ['001000', '001001', '001010', '001011'], ['001100', '001101', '001110', '001111'], ['010000', '010001', '010010', '010011'], ['010100', '010101', '010110', '010111'], ['011000', '011001', '011010', '011011'], ['011100', '011101', '011110', '011111'], ['100000', '100001', '100010', '100011'], ['100100', '100101', '100110', '100111'], ['101000', '101001', '101010', '101011'], ['101100', '101101', '101110', '101111'], ['110000', '110001', '110010', '110011'], ['110100', '110101', '110110', '110111'], ['111000', '111001', '111010', '111011'], ['111100', '111101', '111110', '111111']]
	ctn1 = [['01'],['000111','1'],['000100','000110','001'],['000011','0000011','0000010','000101'],['000010','00000011','00000010','0000000']]
	ctn2 = [['1'],['0001111','01'],['0001110','0001101','001'],['000000111','0001100','0001011','00001'],['000000110','000000101','0001010','000001'],['0000000111','0000000110','000000100','0001001'],['00000000111','00000000110','0000000101','0001000'],['000000000111','000000000110','00000000101','0000000100'],['0000000000111','000000000101','000000000100','00000000100']]
	tz4x4 = [['1', '011', '010', '0011', '0010', '00011', '00010', '000011', '000010', '0000011', '0000010', '00000011', '00000010', '000000011', '000000010', '000000001'], ['111', '110', '101', '100', '011', '0101', '0100', '0011', '0010', '00011', '00010', '000011', '000010', '000001', '000000'], ['0101', '111', '110', '101', '0100', '0011', '100', '011', '0010', '00011', '00010', '000001', '00001', '000000'], ['00011', '111', '0101', '0100', '110', '101', '100', '0011', '011', '0010', '00010', '00001', '00000'], ['0101', '0100', '0011', '111', '110', '101', '100', '011', '0010', '00001', '0001', '00000'], ['000001', '00001', '111', '110', '101', '100', '011', '010', '0001', '001', '000000'], ['000001', '00001', '101', '100', '011', '11', '010', '0001', '001', '000000'], ['000001', '0001', '00001', '011', '11', '10', '010', '001', '000000'], ['000001', '000000', '0001', '11', '10', '001', '01', '00001'], ['00001', '00000', '001', '11', '10', '01', '0001'], ['0000', '0001', '001', '010', '1', '011'], ['0000', '0001', '01', '1', '001'], ['000', '001', '1', '01'], ['00', '01', '1'], ['0', '1']]
	tz2x4 = [['1', '010', '011', '0010', '0011', '0001', '00001', '00000'], ['000', '01', '001', '100', '101', '110', '111'], ['000', '001', '01', '10', '110', '111'], ['110', '00', '01', '10', '111'], ['00', '01', '10', '11'], ['00', '01', '1'], ['0', '1']]
	tz2x2 = [['1', '01', '001', '000'], ['1', '01', '00'], ['1', '0']]
	rb = [['1', '0'], ['1', '01', '00'], ['11', '10', '01', '00'], ['11', '10', '01', '001', '000'], ['11', '10', '011', '010', '001', '000'], ['11', '000', '001', '011', '010', '101', '100'], ['111', '110', '101', '100', '011', '010', '001', '0001', '00001', '000001', '0000001', '00000001', '000000001', '0000000001', '00000000001']]
	icoeffs = [i for i, c in enumerate(coeffs) if c != 0]
	nzcoeffs = [c for c in coeffs if c != 0]
	TotalCoeff = len(nzcoeffs)
	TrailingOnes = min(3, next((s for s, c in enumerate(reversed(nzcoeffs)) if abs(c) > 1), TotalCoeff))
	vlc = (ctn2 if nC == -2 else ctn1 if nC == -1 else ct02 if nC < 2 else ct24 if nC < 4 else ct48 if nC < 8 else ct8)[TotalCoeff][TrailingOnes]
	bits = bits << len(vlc) | int(vlc, 2)
	if TotalCoeff > 0:
		suffixLength = int(TotalCoeff > 10 and TrailingOnes < 3)
		for i, c in enumerate(reversed(nzcoeffs)):
			if i < TrailingOnes:
				bits = bits << 1 | int(c < 0)
			else:
				levelCode = abs(c) * 2 - 2 + int(c < 0) - 2 * (i == TrailingOnes < 3)
				level_prefix = levelCode >> suffixLength
				levelSuffixSize = suffixLength
				if suffixLength == 0 and 14 <= levelCode < 30:
					level_prefix = 14
					levelCode -= 14
					levelSuffixSize = 4
				if levelCode >= 15 << max(1, suffixLength):
					levelCode += 4096 - (15 << max(1, suffixLength))
					level_prefix = levelCode.bit_length() + 2
					levelSuffixSize = level_prefix - 3
				bits = bits << level_prefix << 1 | 1
				bits = bits << levelSuffixSize | levelCode & ((1 << levelSuffixSize) - 1)
				suffixLength += suffixLength == 0
				suffixLength = min(6, suffixLength + (abs(c) > (3 << (suffixLength - 1))))
		zerosLeft = 0
		if TotalCoeff < len(coeffs):
			zerosLeft = icoeffs[-1] - len(icoeffs) + 1
			vlc = (tz4x4 if len(coeffs) >= 15 else tz2x4 if len(coeffs) == 8 else tz2x2)[TotalCoeff - 1][zerosLeft]
			bits = bits << len(vlc) | int(vlc, 2)
		for i in range(TotalCoeff - 1, 0, -1):
			if zerosLeft > 0:
				run_before = icoeffs[i] - icoeffs[i - 1] - 1
				vlc = rb[min(7, zerosLeft) - 1][run_before]
				bits = bits << len(vlc) | int(vlc, 2)
				zerosLeft -= run_before
	return bits

def gen_slice_data_cavlc(bits, f, slice, slice_type):
	me_intra = [3, 29, 30, 17, 31, 18, 37, 8, 32, 38, 19, 9, 20, 10, 11, 2, 16, 33, 34, 21, 35, 22, 39, 4, 36, 40, 23, 5, 24, 6, 7, 1, 41, 42, 43, 25, 44, 26, 46, 12, 45, 47, 27, 13, 28, 14, 15, 0]
	me_inter = [0, 2, 3, 7, 4, 8, 17, 13, 5, 18, 9, 14, 10, 15, 16, 11, 1, 32, 33, 36, 34, 37, 44, 40, 35, 45, 38, 41, 39, 42, 43, 19, 6, 24, 25, 20, 26, 21, 46, 28, 27, 47, 22, 29, 23, 30, 31, 12]
	skip_run = 0
	for mb in slice.macroblocks_cavlc:
		# flush the bits buffer to file
		num = bits.bit_length() - 1
		bits ^= 1 << num
		f.write((bits >> (num % 8)).to_bytes(num // 8, byteorder="big"))
		bits = bits & ((1 << (num % 8)) - 1) | 1 << (num % 8)
		if "mb_skip_run" in vars(mb):
			bits = gen_ue(bits, mb.mb_skip_run)
			skip_run = mb.mb_skip_run
		skip_run -= 1
		if skip_run >= 0: continue
		if "mb_field_decoding_flag" in vars(mb):
			bits = bits << 1 | mb.mb_field_decoding_flag
		# macroblock_layer()
		bits = gen_ue(bits, mb.mb_type)
		if mb.mb_type == [30, 48, 25][slice_type]: # I_PCM
			num = bits.bit_length() - 1
			bits <<= -num % 8 # pcm_alignment_zero_bit
			for sample in mb.pcm_samples.Y:
				bits = bits << mb.pcm_samples.bits_Y | sample
			for sample in mb.pcm_samples.Cb + mb.pcm_samples.Cr:
				bits = bits << mb.pcm_samples.bits_C | sample
		if mb.mb_type == [5, 23, 0][slice_type] and "transform_size_8x8_flag" in vars(mb): # I_NxN
			bits = bits << 1 | mb.transform_size_8x8_flag
		for mode in vars(mb).get("rem_intra4x4_pred_modes", vars(mb).get("rem_intra8x8_pred_modes", [])):
			bits = bits << 1 | int(mode < 0)
			if mode >= 0:
				bits = bits << 3 | mode
		if "intra_chroma_pred_mode" in vars(mb):
			bits = gen_ue(bits, mb.intra_chroma_pred_mode)
		if mb.mb_type in [[3, 4], [22], []][slice_type]: # P/B and sub-8x8 mb shape
			for sub_mb_type in mb.sub_mb_types:
				bits = gen_ue(bits, sub_mb_type)
		if mb.mb_type in [range(5), range(1, 23), []][slice_type]: # non-Direct Inter
			for i, ref_idx in vars(mb.ref_idx).items():
				if vars(slice.num_ref_idx_active)[f"l{int(i)//4}"] == 2:
					bits = bits << 1 | ref_idx ^ 1
				else:
					bits = gen_ue(bits, ref_idx)
			for x, y in mb.mvds:
				bits = gen_se(bits, x)
				bits = gen_se(bits, y)
		if mb.mb_type in [range(6), range(24), [0]][slice_type]: # not Intra_16x16 nor PCM
			bits = gen_ue(bits, (me_inter if mb.mb_type < [5, 23, 0][slice_type] else me_intra)[mb.coded_block_pattern])
		if mb.mb_type != [5, 23, 0][slice_type] and "transform_size_8x8_flag" in vars(mb): # not I_NxN
			bits = bits << 1 | mb.transform_size_8x8_flag
		if "mb_qp_delta" in vars(mb):
			bits = gen_se(bits, mb.mb_qp_delta)
			for block in mb.coeffLevels:
				bits = gen_residual_block_cavlc(bits, block.nC, vars(block).get("c", []))
	return bits

def gen_slice_layer_without_partitioning(bits, f, slice):
	bits = gen_ue(bits, slice.first_mb_in_slice)
	bits = gen_ue(bits, slice.slice_type)
	slice_type = slice.slice_type % 5
	bits = gen_ue(bits, slice.pic_parameter_set_id)
	if "colour_plane_id" in vars(slice):
		bits = bits << 2 | slice.colour_plane_id
	bits = bits << slice.frame_num.bits | (slice.frame_num.absolute & (1 << slice.frame_num.bits) - 1)
	if "field_pic_flag" in vars(slice):
		bits = bits << 1 | slice.field_pic_flag
		if slice.field_pic_flag:
			bits = bits << 1 | slice.bottom_field_flag
	IdrPicFlag = slice.nal_unit_type == 5 or "non_idr_flag" in vars(slice) and not slice.non_idr_flag
	if IdrPicFlag:
		bits = gen_ue(bits, slice.idr_pic_id)
	if slice.pic_order_cnt.type == 0:
		bits = bits << slice.pic_order_cnt.bits | (slice.pic_order_cnt.absolute & (1 << slice.pic_order_cnt.bits) - 1)
		if "bottom" in vars(slice.pic_order_cnt):
			bits = gen_se(bits, slice.pic_order_cnt.bottom - slice.pic_order_cnt.absolute)
	if slice.pic_order_cnt.type == 1 and "delta0" in vars(slice.pic_order_cnt):
		bits = gen_se(bits, slice.pic_order_cnt.delta0)
		if "delta1" in vars(slice.pic_order_cnt):
			bits = gen_se(bits, slice.pic_order_cnt.delta1)
	if slice_type == 1:
		bits = bits << 1 | slice.direct_spatial_mv_pred_flag
	if slice_type <= 1:
		bits = bits << 1 | slice.num_ref_idx_active.override_flag
		if slice.num_ref_idx_active.override_flag:
			bits = gen_ue(bits, slice.num_ref_idx_active.l0 - 1)
			if slice_type == 1:
				bits = gen_ue(bits, slice.num_ref_idx_active.l1 - 1)
		field_to_idc = {"sref": 1, "lref": 2, "view": 5}
		for i in range(slice_type + 1):
			bits = bits << 1 | int(f"ref_pic_list_modification_l{i}" in vars(slice))
			if f"ref_pic_list_modification_l{i}" in vars(slice):
				for field, diff in vars(slice)[f"ref_pic_list_modification_l{i}"]:
					bits = gen_ue(bits, field_to_idc[field] - (diff < 0))
					bits = gen_ue(bits, diff if field == "lref" else abs(diff) - 1)
				bits = gen_ue(bits, 3)
		if "explicit_weights_l0" in vars(slice):
			bits = gen_ue(bits, int(re.findall(r"\d+", slice.explicit_weights_l0[0].Y)[1]))
			bits = gen_ue(bits, int(re.findall(r"\d+", slice.explicit_weights_l0[0].Cb)[1]))
			for i in range(slice_type + 1):
				for ref in vars(slice)[f"explicit_weights_l{i}"]:
					for plane in ("Y", "Cb", "Cr"):
						weight, denom, offset = map(int, re.findall(r"\-?\d+", vars(ref)[plane]))
						bits = bits << 1 | int(weight != 2 ** denom or offset != 0)
						if weight != 2 ** denom or offset != 0:
							bits = gen_se(bits, weight)
							bits = gen_se(bits, offset)
	if slice.nal_ref_idc:
		if IdrPicFlag:
			bits = bits << 1 | slice.no_output_of_prior_pics_flag
			bits = bits << 1 | slice.long_term_reference_flag
		else:
			bits = bits << 1 | int("memory_management_control_operations" in vars(slice))
			if "memory_management_control_operations" in vars(slice):
				for mmco in slice.memory_management_control_operations:
					bits = gen_ue(bits, mmco.idc)
					if "sref" in mmco:
						bits = gen_ue(bits, -mmco.sref)
					if "lref" in mmco:
						bits = gen_ue(bits, mmco.lref)
				bits = bits << 1 | 1 # memory_management_control_operation == 0
	if "cabac_init_idc" in vars(slice):
		bits = gen_ue(bits, slice.cabac_init_idc)
	bits = gen_se(bits, slice.slice_qp_delta)
	if "disable_deblocking_filter_idc" in vars(slice):
		bits = gen_ue(bits, slice.disable_deblocking_filter_idc)
		if slice.disable_deblocking_filter_idc != 1:
			bits = gen_se(bits, slice.slice_alpha_c0_offset >> 1)
			bits = gen_se(bits, slice.slice_beta_offset >> 1)
	if "macroblocks_cavlc" in vars(slice):
		bits = gen_slice_data_cavlc(bits, f, slice, slice_type)
	else:
		bits = gen_slice_data_cabac(bits, f, slice, slice_type)
	return bits



def gen_hrd_parameters(bits, hrd):
	bits = gen_ue(bits, len(hrd.CPBs) - 1)
	bit_rate_scale = min(ctz(reduce(lambda a, b: a | b, (cpb.bit_rate for cpb in hrd.CPBs))) - 6, 15)
	cpb_size_scale = min(ctz(reduce(lambda a, b: a | b, (cpb.size for cpb in hrd.CPBs))) - 4, 15)
	bits = bits << 4 | bit_rate_scale
	bits = bits << 4 | cpb_size_scale
	for cpb in hrd.CPBs:
		bits = gen_ue(bits, (cpb.bit_rate >> 6 >> bit_rate_scale) - 1)
		bits = gen_ue(bits, (cpb.size >> 4 >> cpb_size_scale) - 1)
		bits = bits << 1 | cpb.cbr_flag
	bits = bits << 5 | hrd.initial_cpb_removal_delay_length - 1
	bits = bits << 5 | hrd.cpb_removal_delay_length - 1
	bits = bits << 5 | hrd.dpb_output_delay_length - 1
	bits = bits << 5 | hrd.time_offset_length
	return bits

def gen_vui_parameters(bits, sps, vui):
	bits = bits << 1 | int("aspect_ratio" in vars(vui))
	if "aspect_ratio" in vars(vui):
		bits = bits << 8 | vui.aspect_ratio.idc
		if vui.aspect_ratio.idc == 255:
			bits = bits << 16 | vui.aspect_ratio.width
			bits = bits << 16 | vui.aspect_ratio.height
	bits = bits << 1 | min(vui.overscan_appropriate_flag + 1, 1)
	if vui.overscan_appropriate_flag >= 0:
		bits = bits << 1 | vui.overscan_appropriate_flag
	bits = bits << 1 | int("video_format" in vars(vui))
	if "video_format" in vars(vui):
		bits = bits << 3 | vui.video_format
		bits = bits << 1 | vui.video_full_range_flag
		bits = bits << 1 | int("colour_primaries" in vars(vui))
		if "colour_primaries" in vars(vui):
			bits = bits << 8 | vui.colour_primaries
			bits = bits << 8 | vui.transfer_characteristics
			bits = bits << 8 | vui.matrix_coefficients
	bits = bits << 1 | int("chroma_sample_loc" in vars(vui))
	if "chroma_sample_loc" in vars(vui):
		bits = gen_ue(bits, vui.chroma_sample_loc.top)
		bits = gen_ue(bits, vui.chroma_sample_loc.bottom)
	bits = bits << 1 | int("num_units_in_tick" in vars(vui))
	if "num_units_in_tick" in vars(vui):
		bits = bits << 32 | vui.num_units_in_tick
		bits = bits << 32 | vui.time_scale
		bits = bits << 1 | vui.fixed_frame_rate_flag
	bits = bits << 1 | int("nal_hrd_parameters" in vars(vui))
	if "nal_hrd_parameters" in vars(vui):
		bits = gen_hrd_parameters(bits, vui.nal_hrd_parameters)
	bits = bits << 1 | int("vcl_hrd_parameters" in vars(vui))
	if "vcl_hrd_parameters" in vars(vui):
		bits = gen_hrd_parameters(bits, vui.vcl_hrd_parameters)
	if "nal_hrd_parameters" in vars(vui) or "vcl_hrd_parameters" in vars(vui):
		bits = bits << 1 | vui.low_delay_hrd_flag
	bits = bits << 1 | vui.pic_struct_present_flag
	bits = bits << 1 | int("motion_vectors_over_pic_boundaries_flag" in vars(vui))
	if "motion_vectors_over_pic_boundaries_flag" in vars(vui):
		bits = bits << 1 | vui.motion_vectors_over_pic_boundaries_flag
		PicSizeInMbs = sps.pic_size_in_mbs.width * sps.pic_size_in_mbs.height
		RawMbBits = 256 * sps.bit_depth.luma + (64 << sps.chroma_format_idc & ~64) * sps.bit_depth.chroma
		bits = gen_ue(bits, (PicSizeInMbs * RawMbBits) // vui.max_bytes_per_pic // 8 if vui.max_bytes_per_pic else 0)
		bits = gen_ue(bits, (128 + RawMbBits) // vui.max_bits_per_mb if vui.max_bits_per_mb else 0)
		bits = gen_ue(bits, vui.log2_max_mv_length_horizontal)
		bits = gen_ue(bits, vui.log2_max_mv_length_vertical)
		bits = gen_ue(bits, vui.max_num_reorder_frames)
		bits = gen_ue(bits, vui.max_dec_frame_buffering)
	return bits

def gen_seq_parameter_set(bits, f, sps):
	bits = bits << 8 | sps.profile_idc
	bits = bits << 8 | sum(f << (7 - i) for i, f in enumerate(sps.constraint_set_flags))
	bits = bits << 8 | int(sps.level_idc * 10)
	bits = bits << 1 | 1 # seq_parameter_set_id
	if sps.profile_idc in (100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135):
		bits = gen_ue(bits, sps.chroma_format_idc)
		if sps.chroma_format_idc == 3:
			bits = bits << 1 | sps.separate_colour_plane_flag
		bits = gen_ue(bits, sps.bit_depth.luma - 8)
		bits = gen_ue(bits, sps.bit_depth.chroma - 8)
		bits = bits << 1 | sps.qpprime_y_zero_transform_bypass_flag
		bits = bits << 1 | int("seq_scaling_matrix" in vars(sps))
		if "seq_scaling_matrix" in vars(sps):
			for scaling_list in sps.seq_scaling_matrix:
				bits = bits << 1 | min(len(scaling_list), 1) # seq_scaling_list_present_flag
				for lastScale, nextScale in zip([8] + scaling_list, scaling_list):
					bits = gen_se(bits, (nextScale - lastScale + 128) % 256 - 128) # delta_scale
	bits = gen_ue(bits, sps.log2_max_frame_num - 4)
	bits = gen_ue(bits, sps.pic_order_cnt_type)
	if sps.pic_order_cnt_type == 0:
		bits = gen_ue(bits, sps.log2_max_pic_order_cnt_lsb - 4)
	elif sps.pic_order_cnt_type == 1:
		bits = bits << 1 | sps.delta_pic_order_always_zero_flag
		bits = gen_se(bits, sps.offset_for_non_ref_pic)
		bits = gen_se(bits, sps.offset_for_top_to_bottom_field)
		bits = gen_ue(bits, len(sps.offsets_for_ref_frames))
		for offset in sps.offsets_for_ref_frames:
			bits = gen_se(bits, offset)
	bits = gen_ue(bits, sps.max_num_ref_frames)
	bits = bits << 1 | sps.gaps_in_frame_num_value_allowed_flag
	bits = gen_ue(bits, sps.pic_size_in_mbs.width - 1)
	bits = gen_ue(bits, sps.pic_size_in_mbs.height - 1)
	bits = bits << 1 | sps.frame_mbs_only_flag
	if not sps.frame_mbs_only_flag:
		bits = bits << 1 | sps.mb_adaptive_frame_field_flag
	bits = bits << 1 | sps.direct_8x8_inference_flag
	bits = bits << 1 | int("frame_crop_offsets" in vars(sps))
	if "frame_crop_offsets" in vars(sps):
		for side in ("left", "right", "top", "bottom"):
			bits = gen_ue(bits, vars(sps.frame_crop_offsets)[side])
	bits = bits << 1 | int("vui_parameters" in vars(sps))
	if "vui_parameters" in vars(sps):
		bits = gen_vui_parameters(bits, sps, sps.vui_parameters)
	return bits



def gen_pic_parameter_set(bits, f, pps):
	bits = gen_ue(bits, pps.pic_parameter_set_id)
	bits = bits << 1 | 1 # seq_parameter_set_id
	bits = bits << 1 | pps.entropy_coding_mode_flag
	bits = bits << 1 | pps.bottom_field_pic_order_in_frame_present_flag
	bits = bits << 1 | 1 # num_slice_groups
	bits = gen_ue(bits, pps.num_ref_idx_default_active.l0 - 1)
	bits = gen_ue(bits, pps.num_ref_idx_default_active.l1 - 1)
	bits = bits << 1 | pps.weighted_pred_flag
	bits = bits << 2 | pps.weighted_bipred_idc
	bits = gen_se(bits, pps.pic_init_qp - 26)
	bits = bits << 1 | 1 # pic_init_qs
	bits = gen_se(bits, pps.chroma_qp_index_offset)
	bits = bits << 1 | pps.deblocking_filter_control_present_flag
	bits = bits << 1 | pps.constrained_intra_pred_flag
	bits = bits << 1 # redundant_pic_cnt_present_flag
	if "transform_8x8_mode_flag" in vars(pps):
		bits = bits << 1 | pps.transform_8x8_mode_flag
		bits = bits << 1 | int("pic_scaling_matrix" in vars(pps))
		if "pic_scaling_matrix" in vars(pps):
			for scaling_list in pps.pic_scaling_matrix:
				bits = bits << 1 | min(len(scaling_list), 1) # pic_scaling_list_present_flag
				for lastScale, nextScale in zip([8] + scaling_list, scaling_list):
					bits = gen_se(bits, (nextScale - lastScale + 128) % 256 - 128) # delta_scale
		bits = gen_se(bits, pps.second_chroma_qp_index_offset)
	return bits



def gen_access_unit_delimiter(bits, f, aud):
	bits = bits << 3 | aud.primary_pic_type
	return bits



def gen_seq_parameter_set_extension(bits, f, spse):
	bits = bits << 1 | 1 # seq_parameter_set_id
	bits = gen_ue(bits, spse.aux_format_idc)
	if spse.aux_format_idc:
		bits = gen_ue(bits, spse.bit_depth_aux - 8)
		bits = bits << 1 | spse.alpha_incr_flag
		bits = bits << spse.bit_depth_aux << 1 | spse.alpha_opaque_value
		bits = bits << spse.bit_depth_aux << 1 | spse.alpha_transparent_value
	bits = bits << 1 # additional_extension_flag
	return bits



def gen_mvc_vui_parameters_extension(bits, ssps, vui):
	bits = gen_ue(bits, len(vui.vui_mvc_operation_points) - 1)
	for op in vui.vui_mvc_operation_points:
		bits = bits << 3 | op.temporal_id
		bits = gen_ue(bits, len(op.target_views) - 1)
		for view_id in op.target_views:
			bits = gen_ue(bits, view_id)
		bits = bits << 1 | int("num_units_in_tick" in vars(vui))
		if "num_units_in_tick" in vars(vui):
			bits = bits << 32 | vui.num_units_in_tick
			bits = bits << 32 | vui.time_scale
			bits = bits << 1 | vui.fixed_frame_rate_flag
		bits = bits << 1 | int("nal_hrd_parameters" in vars(vui))
		if "nal_hrd_parameters" in vars(vui):
			bits = gen_hrd_parameters(bits, vui.nal_hrd_parameters)
		bits = bits << 1 | int("vcl_hrd_parameters" in vars(vui))
		if "vcl_hrd_parameters" in vars(vui):
			bits = gen_hrd_parameters(bits, vui.vcl_hrd_parameters)
		if "nal_hrd_parameters" in vars(vui) or "vcl_hrd_parameters" in vars(vui):
			bits = bits << 1 | vui.low_delay_hrd_flag
		bits = bits << 1 | vui.pic_struct_present_flag
	return bits

def gen_subset_seq_parameter_set(bits, f, ssps):
	bits = gen_seq_parameter_set(bits, f, ssps)
	if ssps.profile_idc in (118, 128, 134):
		bits = bits << 1 | 1 # bit_equal_to_one
		bits = gen_ue(bits, 1) # num_views_minus1
		bits = gen_ue(bits, ssps.view_ids[0])
		bits = gen_ue(bits, ssps.view_ids[1])
		bits = gen_ue(bits, ssps.num_anchor_refs.l0)
		if ssps.num_anchor_refs.l0:
			bits = gen_ue(bits, ssps.view_ids[0])
		bits = gen_ue(bits, ssps.num_anchor_refs.l1)
		if ssps.num_anchor_refs.l1:
			bits = gen_ue(bits, ssps.view_ids[0])
		bits = gen_ue(bits, ssps.num_non_anchor_refs.l0)
		if ssps.num_non_anchor_refs.l0:
			bits = gen_ue(bits, ssps.view_ids[0])
		bits = gen_ue(bits, ssps.num_non_anchor_refs.l1)
		if ssps.num_non_anchor_refs.l1:
			bits = gen_ue(bits, ssps.view_ids[0])
		bits = gen_ue(bits, len(ssps.level_values_signalled) - 1)
		for level in ssps.level_values_signalled:
			bits = bits << 8 | level.idc
			bits = gen_ue(bits, len(level.operation_points) - 1)
			for op in level.operation_points:
				bits = bits << 3 | op.temporal_id
				bits = gen_ue(bits, len(op.target_views) - 1)
				for view_id in op.target_views:
					bits = gen_ue(bits, view_id)
				bits = gen_ue(bits, op.num_views - 1)
		bits = bits << 1 | int("mvc_vui_parameters" in vars(ssps))
		if "mvc_vui_parameters" in vars(ssps):
			bits = gen_mvc_vui_parameters_extension(bits, ssps, ssps.mvc_vui_parameters)
	bits = bits << 1 # additional_extension2_flag
	return bits



gen_bits = {
	1: gen_slice_layer_without_partitioning,
	5: gen_slice_layer_without_partitioning,
	7: gen_seq_parameter_set,
	8: gen_pic_parameter_set,
	9: gen_access_unit_delimiter,
	13: gen_seq_parameter_set_extension,
	15: gen_subset_seq_parameter_set
}

def main():
	if len(sys.argv) != 3:
		print(f"Usage: {sys.argv[0]} input.yaml output.264")
		exit()
	print(f"Loading {sys.argv[1]}... (estimated {round(1.7437078849318695e-05 * path.getsize(sys.argv[1]))}s)")
	with open(sys.argv[1], "r") as f:
		nals = yaml.load(f, Loader=SafeLoader)
	print(f"Generating {sys.argv[2]}...")
	with open(sys.argv[2], "wb") as f:
		for nal in nals:
			bits = 1 # leading set bit
			bits = bits << 32 | 1 # start code
			bits <<= 1 # forbidden_zero_bit
			bits = bits << 2 | nal.nal_ref_idc
			bits = bits << 5 | nal.nal_unit_type
			bits = gen_bits[nal.nal_unit_type](bits, f, nal)
			bits = bits << 1 | 1 # rbsp_stop_one_bit
			num = bits.bit_length() - 1
			bits ^= 1 << num
			bits <<= -num % 8
			f.write(bits.to_bytes((num + 7) // 8, byteorder="big"))

if __name__ == "__main__":
	main()