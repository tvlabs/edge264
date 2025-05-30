#!/usr/bin/env python3
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



def gen_ue(bits, v):
	v += 1
	return bits << (v.bit_length() * 2 - 1) | v

def gen_se(bits, v):
	v = v * 2 if v > 0 else -v * 2 + 1
	return bits << (v.bit_length() * 2 - 1) | v



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
	bits = bits << 1 | int("chroma_sample_loc_type_top_field" in vars(vui))
	if "chroma_sample_loc_type_top_field" in vars(vui):
		bits = gen_ue(bits, vui.chroma_sample_loc_type_top_field)
		bits = gen_ue(bits, vui.chroma_sample_loc_type_bottom_field)
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
		PicSizeInMbs = sps.pic_width_in_mbs * sps.pic_height_in_mbs
		RawMbBits = 256 * sps.bit_depth_luma + (64 << sps.chroma_format_idc & ~64) * sps.bit_depth_chroma
		bits = gen_ue(bits, (PicSizeInMbs * RawMbBits) // vui.max_bytes_per_pic // 8 if "max_bytes_per_pic" in vars(vui) else 0)
		bits = gen_ue(bits, (128 + RawMbBits) // vui.max_bits_per_mb if "max_bits_per_mb" in vars(vui) else 0)
		bits = gen_ue(bits, vui.log2_max_mv_length_horizontal)
		bits = gen_ue(bits, vui.log2_max_mv_length_vertical)
		bits = gen_ue(bits, vui.max_num_reorder_frames)
		bits = gen_ue(bits, vui.max_dec_frame_buffering)
	return bits



def gen_seq_parameter_set(bits, sps):
	bits = bits << 8 | sps.profile_idc
	bits = bits << 8 | sum(f << (7 - i) for i, f in enumerate(sps.constraint_set_flags))
	bits = bits << 8 | int(sps.level_idc * 10)
	bits = gen_ue(bits, sps.seq_parameter_set_id)
	if sps.profile_idc in (100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135):
		bits = gen_ue(bits, sps.chroma_format_idc)
		if sps.chroma_format_idc == 3:
			bits = bits << 1 | sps.separate_colour_plane_flag
		bits = gen_ue(bits, sps.bit_depth_luma - 8)
		bits = gen_ue(bits, sps.bit_depth_chroma - 8)
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
	bits = gen_ue(bits, sps.pic_width_in_mbs - 1)
	bits = gen_ue(bits, sps.pic_height_in_mbs - 1)
	bits = bits << 1 | sps.frame_mbs_only_flag
	if not sps.frame_mbs_only_flag:
		bits = bits << 1 | sps.mb_adaptive_frame_field_flag
	bits = bits << 1 | sps.direct_8x8_inference_flag
	bits = bits << 1 | int("frame_crop_offsets" in vars(sps))
	if "frame_crop_offsets" in vars(sps):
		for side in ("left", "right", "top", "bottom"):
			bits = gen_ue(bits, sps.frame_crop_offsets[side])
	bits = bits << 1 | int("vui_parameters" in vars(sps))
	if "vui_parameters" in vars(sps):
		bits = gen_vui_parameters(bits, sps, sps.vui_parameters)
	return bits



gen_bits = {
	7: gen_seq_parameter_set
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
			num = bits.bit_length() + 31
			bits <<= -num % 8
			f.write(bits.to_bytes((num + 7) // 8, byteorder="big"))

if __name__ == "__main__":
	main()