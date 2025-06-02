#!/usr/bin/env python3
from functools import reduce
import re
import sys
import types
import yaml

# make yaml use SimpleNamespace instead of dict
class SafeConstructor(yaml.loader.SafeConstructor):
	def construct_yaml_map(self, node):
		data = types.SimpleNamespace()
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



def gen_slice_layer_without_partitioning(bits, slice):
	bits = gen_ue(bits, slice.first_mb_in_slice)
	bits = gen_ue(bits, slice.slice_type)
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
		bits = bits << slice.pic_order_cnt.bits | (slice.pic_order_cnt.absolute & (1 << slice.pic_order_cnt) - 1)
		if "bottom" in vars(slice.pic_order_cnt):
			bits = gen_se(bits, slice.pic_order_cnt.bottom - slice.pic_order_cnt.absolute)
	if slice.pic_order_cnt.type == 1 and "delta0" in vars(slice.pic_order_cnt):
		bits = gen_se(bits, slice.pic_order_cnt.delta0)
		if "delta1" in vars(slice.pic_order_cnt):
			bits = gen_se(bits, slice.pic_order_cnt.delta1)
	if slice.slice_type in (1, 6):
		bits = bits << 1 | slice.direct_spatial_mv_pred_flag
	if slice.slice_type in (0, 1, 5, 6):
		bits = bits << 1 | int("num_ref_idx_active" in vars(slice))
		if "num_ref_idx_active" in vars(slice):
			bits = gen_ue(bits, slice.num_ref_idx_active.l0)
			if slice.slice_type in (1, 6):
				bits = gen_ue(bits, slice.num_ref_idx_active.l1)
	field_to_idc = {"sref": 1, "lref": 2, "view": 5}
	for i in range(slice.slice_type % 5 + 1):
		bits = bits << 1 | int(f"ref_pic_list_modification_l{i}" in vars(slice))
		if f"ref_pic_list_modification_l{i}" in vars(slice):
			for field, diff in vars(slice)[f"ref_pic_list_modification_l{i}"]:
				bits = gen_ue(bits, field_to_idc[field] - (diff < 0))
				bits = gen_ue(bits, diff if field == "lref" else abs(diff) - 1)
			bits = gen_ue(bits, 3)
	if "explicit_weights_l0" in vars(slice):
		bits = gen_ue(bits, int(re.findall(r"\d+", slice.explicit_weights_l0[0].Y)[1]))
		bits = gen_ue(bits, int(re.findall(r"\d+", slice.explicit_weights_l0[0].Cb)[1]))
		for i in range(slice.slice_type % 5 + 1):
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

def gen_seq_parameter_set(bits, sps):
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



def gen_pic_parameter_set(bits, pps):
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
				bits = bits << 1 | min(len(scaling_list), 1) # seq_scaling_list_present_flag
				for lastScale, nextScale in zip([8] + scaling_list, scaling_list):
					bits = gen_se(bits, (nextScale - lastScale + 128) % 256 - 128) # delta_scale
		bits = gen_se(bits, pps.second_chroma_qp_index_offset)
	return bits



def gen_access_unit_delimiter(bits, aud):
	bits = bits << 3 | aud.primary_pic_type
	return bits



def gen_seq_parameter_set_extension(bits, spse):
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

def gen_subset_seq_parameter_set(bits, ssps):
	bits = gen_seq_parameter_set(bits, ssps)
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
	with open(sys.argv[1], "r") as f:
		nals = yaml.load(f, Loader=SafeLoader)
	with open(sys.argv[2], "wb") as f:
		for nal in nals:
			bits = 1 # start code leading set bit
			bits <<= 1 # forbidden_zero_bit
			bits = bits << 2 | nal.nal_ref_idc
			bits = bits << 5 | nal.nal_unit_type
			bits = gen_bits[nal.nal_unit_type](bits, nal)
			bits = bits << 1 | 1 # rbsp_stop_one_bit
			num = bits.bit_length() + 31
			bits <<= -num % 8
			f.write(bits.to_bytes((num + 7) // 8, byteorder="big"))

if __name__ == "__main__":
	main()