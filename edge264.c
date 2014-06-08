/* TODO: Vérifier dans les SPS, PPS, slice header ET leurs sous-fonctions,
   qu'aucune variable ne soit initialisée que dans un bloc conditionnel. */
/* TODO: Vérifier que deux appels de fonction ne coexistent pas sur la même ligne. */
/* TODO: Tester sur lint. */
/* TODO: In Debug mode, initialise each image in plain red. */
/* TODO: Dimensionner correctement la "safe zone". */
/* TODO: Si unsigned ne sert qu'à étendre le range, le virer des déclarations. */
/* TODO: Supprimer FrameSizeInSamples, qui s'il diffère de width*height pose problème. */
/* TODO: Utiliser signed par défaut, et unsigned lorsque l'overflow est souhaité ? */
/* TODO: Corriger les valeurs de crop selon ChromaArrayType. */
/* TODO: Effacer la liste d'images stockées si les caractéristiques du SPS ont changé. */
/* TODO: Traiter no_output_of_prior_pics_flag, et l'inférer à 1 lorsque la résolution change. */
/* TODO: Réintégrer transform_bypass_flag. */
/* TODO: Intégrer le flag mmco5. */
/* TODO: Supprimer les CPB lors d'un end of sequence. */
/* TODO: A la fin de chaque ligne (même incomplète, cf first_mb_in_slice), le thread appelle un callback. */
/* TODO: Remplacer les 0x%x par des %#x. */

#include <limits.h>
#include <stdlib.h>
#include <string.h>

/**
 * Order of inclusion matters, lower files may use functions and variables from
 * higher files. Compiling all dependencies along with the main library has
 * advantages not met by Whole Program Optimisation:
 * _ simplicity: no need to create internal APIs between compilation units,
 *   avoiding the .c/.h couples thus reducing the number of files;
 * _ transparency: compilation needs not be hidden behind a Makefile, and the
 *   simple command used to build the library can be tuned as will;
 * _ output control: static functions do not appear in the resulting archive,
 *   without having to strip them afterwards.
 */
#include "H264_common.h"
#ifdef __SSSE3__
//#include "H264_deblock_ssse3.c"
//#include "H264_residual_ssse3.c"
//#include "H264_intra_ssse3.c"
//#include "H264_inter_ssse3.c"
#endif
//#include "H264_CABAC.c"



// TODO: Which scan order was used??
static const uint8_t Default_4x4_Intra[16] __attribute__((aligned(16))) =
    {6, 13, 20, 28, 13, 20, 28, 32, 20, 28, 32, 37, 28, 32, 37, 42};
static const uint8_t Default_4x4_Inter[16] __attribute__((aligned(16))) =
    {10, 14, 20, 24, 14, 20, 24, 27, 20, 24, 27, 30, 24, 27, 30, 34};
static const uint8_t Default_8x8_Intra[64] __attribute__((aligned(16))) =
    {6, 10, 13, 16, 18, 23, 25, 27, 10, 11, 16, 18, 23, 25, 27, 29, 13, 16, 18,
    23, 25, 27, 29, 31, 16, 18, 23, 25, 27, 29, 31, 33, 18, 23, 25, 27, 29, 31,
    33, 36, 23, 25, 27, 29, 31, 33, 36, 38, 25, 27, 29, 31, 33, 36, 38, 40, 27,
    29, 31, 33, 36, 38, 40, 42};
static const uint8_t Default_8x8_Inter[64] __attribute__((aligned(16))) =
    {9, 13, 15, 17, 19, 21, 22, 24, 13, 13, 17, 19, 21, 22, 24, 25, 15, 17, 19,
    21, 22, 24, 25, 27, 17, 19, 21, 22, 24, 25, 27, 28, 19, 21, 22, 24, 25, 27,
    28, 30, 21, 22, 24, 25, 27, 28, 30, 32, 22, 24, 25, 27, 28, 30, 32, 33, 24,
    25, 27, 28, 30, 32, 33, 35};



/**
 * Computes the Picture Order Count (8.2.1), and returns a target Frame_ctx.
 *
 * Computation of every POC type differs from the (overly complex) spec:
 * _ for type 0, PicOrderCntMsb and pic_order_cnt_lsb are never stored separate;
 * _ for type 1 and the rest of the parsing, FrameNumOffset and frame_num are
 *   stored together in absFrameNum. Also, the prefix sums for
 *   offset_for_ref_frame are precomputed in PicOrderCntDeltas.
 */
static Frame_ctx *parse_pic_order_cnt(Video_ctx *v, const PPS_ctx *p,
    Thread_ctx *t, unsigned int field_pic_flag, unsigned int bottom_field_flag)
{
    int FieldOrderCnt[2];
    if (v->pic_order_cnt_type == 0) {
        int pic_order_cnt_lsb = get_uv(&t->CPB, v->log2_max_pic_order_cnt_lsb);
        int MaxPicOrderCntLsb = 1 << v->log2_max_pic_order_cnt_lsb;
        t->PicOrderCnt = (v->prevPicOrderCnt & -MaxPicOrderCntLsb) +
            pic_order_cnt_lsb;
        if (t->PicOrderCnt - v->prevPicOrderCnt <= -MaxPicOrderCntLsb / 2)
            t->PicOrderCnt += MaxPicOrderCntLsb;
        if (t->PicOrderCnt - v->prevPicOrderCnt > MaxPicOrderCntLsb / 2)
            t->PicOrderCnt -= MaxPicOrderCntLsb;
        FieldOrderCnt[0] = FieldOrderCnt[1] = v->prevPicOrderCnt = t->PicOrderCnt;
        printf("<li>pic_order_cnt_lsb: <code>%u</code></li>\n", pic_order_cnt_lsb);
        if (!field_pic_flag) {
            int delta_pic_order_cnt_bottom = 0;
            if (p->bottom_field_pic_order_in_frame_present_flag) {
                delta_pic_order_cnt_bottom = get_se(&t->CPB, -2147483647, 2147483647);
                printf("<li>delta_pic_order_cnt_bottom: <code>%d</code></li>\n",
                    delta_pic_order_cnt_bottom);
            }
            FieldOrderCnt[1] += delta_pic_order_cnt_bottom;
            t->PicOrderCnt = min(t->PicOrderCnt, FieldOrderCnt[1]);
        }
    } else if (v->pic_order_cnt_type == 1) {
        t->PicOrderCnt = (v->nal_ref_idc == 0) ? v->offset_for_non_ref_pic : 0;
        unsigned int absFrameNum = v->prevAbsFrameNum +
            v->num_ref_frames_in_pic_order_cnt_cycle - (v->nal_ref_idc == 0);
        if (v->num_ref_frames_in_pic_order_cnt_cycle > 0) {
            unsigned int picOrderCntCycleCnt = (absFrameNum - 1) /
                v->num_ref_frames_in_pic_order_cnt_cycle;
            unsigned int frameNumInPicOrderCntCycle = (absFrameNum - 1) %
                v->num_ref_frames_in_pic_order_cnt_cycle;
            t->PicOrderCnt = picOrderCntCycleCnt *
                v->PicOrderCntDeltas[v->num_ref_frames_in_pic_order_cnt_cycle - 1] +
                v->PicOrderCntDeltas[frameNumInPicOrderCntCycle];
        }
        int delta_pic_order_cnt[2] = {0};
        if (!v->delta_pic_order_always_zero_flag) {
            delta_pic_order_cnt[0] = get_se(&t->CPB, -2147483647, 2147483647);
            printf("<li>delta_pic_order_cnt[0]: <code>%d</code></li>\n",
                delta_pic_order_cnt[0]);
            if (p->bottom_field_pic_order_in_frame_present_flag && !field_pic_flag) {
                delta_pic_order_cnt[1] = get_se(&t->CPB, -2147483647, 2147483647);
                printf("<li>delta_pic_order_cnt[1]: <code>%d</code></li>\n",
                    delta_pic_order_cnt[1]);
            }
        }
        FieldOrderCnt[0] = t->PicOrderCnt + delta_pic_order_cnt[0];
        FieldOrderCnt[1] = t->PicOrderCnt + v->offset_for_top_to_bottom_field +
            delta_pic_order_cnt[bottom_field_flag];
    } else if (v->pic_order_cnt_type == 2) {
        t->PicOrderCnt = 2 * v->prevAbsFrameNum - (v->nal_ref_idc == 0);
        FieldOrderCnt[0] = FieldOrderCnt[1] = t->PicOrderCnt;
    }
    
    /* Select a Frame_ctx to draw to. */
    Frame_ctx *f = NULL, *lowestPOC = NULL;
    int equal = max(FieldOrderCnt[0], FieldOrderCnt[1]);
    int lower = INT_MAX;
    for (int i = 0; i < v->max_num_ref_frames + MAX_THREADS; i++) {
        int POC = max(v->frames[i].FieldOrderCnt[0], v->frames[i].FieldOrderCnt[1]);
        if (POC == equal)
            f = &v->frames[i];
        if (POC < lower) {
            lower = POC;
            lowestPOC = &v->frames[i];
        }
    }
    assert(lowestPOC!=NULL);
    if (f == NULL) {
        if (!field_pic_flag) {
            v->unpaired_field = NULL;
            f = lowestPOC;
            f->FieldOrderCnt[0] = FieldOrderCnt[0];
            f->FieldOrderCnt[1] = FieldOrderCnt[1];
        } else {
            if (v->unpaired_field == NULL) {
                v->unpaired_field = f = lowestPOC;
                f->FieldOrderCnt[1 ^ bottom_field_flag] = INT_MIN;
            } else {
                f = v->unpaired_field;
                v->unpaired_field = NULL;
            }
            f->FieldOrderCnt[bottom_field_flag] = FieldOrderCnt[bottom_field_flag];
            f->used_for_reference |= 1 << bottom_field_flag;
        }
    }
    return f;
}



/**
 * Dedicated function to parse one half of ref_pic_list_modification().
 */
static void H264_parse_ref_pic_list_modification(Bit_ctx *b)
{
    unsigned int ref_pic_list_modification_flag = get_u1(b);
    printf("<li>ref_pic_list_modification_flag: <code>%x</code></li>\n"
        "<ul>\n",
        ref_pic_list_modification_flag);
    if (ref_pic_list_modification_flag) {
        unsigned int modification_of_pic_nums_idc;
        while ((modification_of_pic_nums_idc = get_ue(b, 3)) != 3) {
            unsigned int diff_pic_num = get_ue(b, 131071) + 1;
            printf("<li%s>diff_pic_num: <code>%c%u</code></li>\n",
                red_if(modification_of_pic_nums_idc == 2), '-' - modification_of_pic_nums_idc * 2, diff_pic_num);
        }
    }
    printf("</ul>\n");
}



/**
 * Dedicated function to parse one half of pred_weight_table().
 */
static void H264_parse_pred_weight_table(Bit_ctx *b, unsigned int num)
{
    printf("<ul>\n");
    for (int i = 0; i < num; i++) {
        unsigned int luma_weight_flag = get_u1(b);
        printf("<li>luma_weight_flag: <code>%x</code></li>\n",
            luma_weight_flag);
        if (luma_weight_flag) {
            int luma_weight = get_se(b, -128, 127);
            int luma_offset = get_se(b, -128, 127);
            printf("<li>luma_weight: <code>%d</code></li>\n"
                "<li>luma_offset: <code>%d</code></li>\n",
                luma_weight,
                luma_offset);
        }
        unsigned int chroma_weight_flag = get_u1(b);
        printf("<li>chroma_weight_flag: <code>%x</code></li>\n",
            chroma_weight_flag);
        if (chroma_weight_flag) {
            for (int iCbCr = 0; iCbCr < 2; iCbCr++) {
                int chroma_weight = get_se(b, -128, 127);
                int chroma_offset = get_se(b, -128, 127);
                printf("<li>chroma_weight: <code>%d</code></li>\n"
                    "<li>chroma_offset: <code>%d</code></li>\n",
                    chroma_weight,
                    chroma_offset);
            }
        }
    }
    printf("</ul>\n");
}



/**
 * Dedicated function to parse dec_ref_pic_marking().
 */
static void H264_parse_dec_ref_pic_marking(Bit_ctx *b, const Video_ctx *v)
{
    if (v->nal_unit_type == 5) {
        unsigned int no_output_of_prior_pics_flag = get_u1(b);
        unsigned int long_term_reference_flag = get_u1(b);
        printf("<li>no_output_of_prior_pics_flag: <code>%x</code></li>\n"
            "<li%s>long_term_reference_flag: <code>%x</code></li>\n",
            no_output_of_prior_pics_flag,
            red_if(long_term_reference_flag), long_term_reference_flag);
    } else {
        unsigned int adaptive_ref_pic_marking_mode_flag = get_u1(b);
        printf("<li>adaptive_ref_pic_marking_mode_flag: <code>%x</code></li>\n",
            adaptive_ref_pic_marking_mode_flag);
        if (adaptive_ref_pic_marking_mode_flag) {
            printf("<ul>\n");
            unsigned int memory_management_control_operation;
            while ((memory_management_control_operation = get_ue(b, 6)) != 0) {
                unsigned int difference_of_pic_nums = get_ue(b, 131071) + 1;
                printf("<li%s>difference_of_pic_nums: <code>%u</code></li>\n",
                    red_if(memory_management_control_operation != 1), difference_of_pic_nums);
            }
            printf("</ul>\n");
        }
    }
}



/**
 * Interprets a slice RBSP and decodes its picture.
 */
static inline unsigned int H264_parse_slice_RBSP(Video_ctx *v, Thread_ctx *t)
{
    static const char * const slice_type_names[5] = {
        [0] = "P",
        [1] = "B",
        [2] = "I",
        [3] = "SP",
        [4] = "SI",
    };
    
    unsigned int error_flags = v->error_flags;
    t->FrameWidthInMbs = v->width / 16;
    t->FrameHeightInMbs = v->height / 16;
    t->ChromaArrayType = v->ChromaArrayType;
    unsigned int first_mb_in_slice = get_ue(&t->CPB, 36863);
    if (first_mb_in_slice > t->FrameWidthInMbs * t->FrameHeightInMbs - 1)
        first_mb_in_slice = t->FrameWidthInMbs * t->FrameHeightInMbs - 1;
    t->mb_x = first_mb_in_slice % t->FrameWidthInMbs;
    t->mb_y = first_mb_in_slice / t->FrameWidthInMbs;
    t->slice_type = get_ue(&t->CPB, 9) % 5;
    error_flags |= (t->slice_type > 2) << H264_UNSUPPORTED_SWITCHING_SLICES;
    unsigned int pic_parameter_set_id = get_ue(&t->CPB, 255);
    error_flags |= (pic_parameter_set_id >= 4) <<
        H264_UNSUPPORTED_MORE_THAN_FOUR_PPS;
    unsigned int frame_num = get_uv(&t->CPB, v->log2_max_frame_num);
    unsigned int MaxFrameNum = 1 << v->log2_max_frame_num;
    unsigned int absFrameNum = (v->prevAbsFrameNum & -MaxFrameNum) + frame_num;
    if (absFrameNum < v->prevAbsFrameNum)
        absFrameNum += MaxFrameNum;
    v->prevAbsFrameNum = absFrameNum;
    printf("<li>first_mb_in_slice: <code>%u</code></li>\n"
        "<li%s>slice_type: <code>%u (%s)</code></li>\n"
        "<li%s>pic_parameter_set_id: <code>%u</code></li>\n"
        "<li>frame_num: <code>%u</code></li>\n",
        first_mb_in_slice,
        red_if(t->slice_type > 2), t->slice_type, slice_type_names[t->slice_type],
        red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
        frame_num);
    const PPS_ctx *p = &v->PPSs[pic_parameter_set_id % 4];
    unsigned int field_pic_flag = 0;
    unsigned int bottom_field_flag = 0;
    t->stride_Y = v->width;
    t->stride_C = v->width >> ((4 - t->ChromaArrayType) / 2);
    t->MbaffFrameFlag = 0;
    if (!v->frame_mbs_only_flag) {
        field_pic_flag = get_u1(&t->CPB);
        printf("<li>field_pic_flag: <code>%x</code></li>\n", field_pic_flag);
        if (field_pic_flag) {
            bottom_field_flag = get_u1(&t->CPB);
            printf("<li>bottom_field_flag: <code>%x</code></li>\n",
                bottom_field_flag);
            t->stride_Y *= 2;
            t->stride_C *= 2;
        } else if ((t->MbaffFrameFlag = v->mb_adaptive_frame_field_flag)) {
            t->stride_Y *= 2;
            t->stride_C *= 2;
        }
    }
    if (v->nal_unit_type == 5) {
        unsigned int idr_pic_id = get_ue(&t->CPB, 65535);
        printf("<li>idr_pic_id: <code>%u</code></li>\n", idr_pic_id);
    }
    Frame_ctx *f = parse_pic_order_cnt(v, p, t, field_pic_flag, bottom_field_flag);
    if (t->slice_type == 1) { /* B */
        unsigned int direct_spatial_mv_pred_flag = get_u1(&t->CPB);
        unsigned int num_ref_idx_active_override_flag = get_u1(&t->CPB);
        printf("<li>direct_spatial_mv_pred_flag: <code>%x</code></li>\n"
            "<li>num_ref_idx_active_override_flag: <code>%x</code></li>\n",
            direct_spatial_mv_pred_flag,
            num_ref_idx_active_override_flag);
        t->num_ref_idx_l0_active = p->num_ref_idx_l0_default_active;
        t->num_ref_idx_l1_active = p->num_ref_idx_l1_default_active;
        if (num_ref_idx_active_override_flag) {
            t->num_ref_idx_l0_active = get_ue(&t->CPB, 31) + 1;
            t->num_ref_idx_l1_active = get_ue(&t->CPB, 31) + 1;
            printf("<li>num_ref_idx_l0_active: <code>%u</code></li>\n"
                "<li>num_ref_idx_l1_active: <code>%u</code></li>\n",
                t->num_ref_idx_l0_active,
                t->num_ref_idx_l1_active);
        }
        H264_parse_ref_pic_list_modification(&t->CPB);
        H264_parse_ref_pic_list_modification(&t->CPB);
        if (p->weighted_bipred_idc == 1) {
            unsigned int luma_log2_weight_denom = get_ue(&t->CPB, 7);
            unsigned int chroma_log2_weight_denom = get_ue(&t->CPB, 7);
            printf("<li>luma_log2_weight_denom: <code>%u</code></li>\n"
                "<li>chroma_log2_weight_denom: <code>%u</code></li>\n",
                luma_log2_weight_denom,
                chroma_log2_weight_denom);
            H264_parse_pred_weight_table(&t->CPB, t->num_ref_idx_l0_active);
            H264_parse_pred_weight_table(&t->CPB, t->num_ref_idx_l1_active);
        }
    } else if (t->slice_type == 0) { /* P */
        unsigned int num_ref_idx_active_override_flag = get_u1(&t->CPB);
        printf("<li>num_ref_idx_active_override_flag: <code>%x</code></li>\n",
            num_ref_idx_active_override_flag);
        t->num_ref_idx_l0_active = p->num_ref_idx_l0_default_active;
        if (num_ref_idx_active_override_flag) {
            t->num_ref_idx_l0_active = get_ue(&t->CPB, 31) + 1;
            printf("<li>num_ref_idx_l0_active: <code>%u</code></li>\n",
                t->num_ref_idx_l0_active);
        }
        H264_parse_ref_pic_list_modification(&t->CPB);
        if (p->weighted_pred_flag) {
            unsigned int luma_log2_weight_denom = get_ue(&t->CPB, 7);
            unsigned int chroma_log2_weight_denom = get_ue(&t->CPB, 7);
            printf("<li>luma_log2_weight_denom: <code>%u</code></li>\n"
                "<li>chroma_log2_weight_denom: <code>%u</code></li>\n",
                luma_log2_weight_denom,
                chroma_log2_weight_denom);
            H264_parse_pred_weight_table(&t->CPB, t->num_ref_idx_l0_active);
        }
    }
    if (v->nal_ref_idc != 0)
        H264_parse_dec_ref_pic_marking(&t->CPB, v);
    t->cabac_init_idc = 3;
    if (p->entropy_coding_mode_flag && t->slice_type != 2) {
        t->cabac_init_idc = get_ue(&t->CPB, 2);
        printf("<li>cabac_init_idc: <code>%u</code></li>\n",
            t->cabac_init_idc);
    }
    t->BitDepth_Y = v->BitDepth_Y;
    t->BitDepth_C = v->BitDepth_C;
    t->QpBdOffset_Y = 6 * (t->BitDepth_Y - 8);
    t->QpBdOffset_C = 6 * (t->BitDepth_C - 8);
    int slice_qp_delta = get_se(&t->CPB, -87, 87);
    int SliceQP_Y = p->pic_init_qp + slice_qp_delta;
    t->QP_Y = (SliceQP_Y < -t->QpBdOffset_Y) ? -t->QpBdOffset_Y :
        (SliceQP_Y > 51) ? 51 : SliceQP_Y;
    printf("<li>slice_qp_delta: <code>%d</code></li>\n",
        slice_qp_delta);
    t->disable_deblocking_filter_idc = 0;
    if (p->deblocking_filter_control_present_flag) {
        t->disable_deblocking_filter_idc = get_ue(&t->CPB, 2);
        printf("<li>disable_deblocking_filter_idc: <code>%u</code></li>\n",
            t->disable_deblocking_filter_idc);
        if (t->disable_deblocking_filter_idc != 1) {
            int FilterOffsetA = get_se(&t->CPB, -6, 6) << 1;
            int FilterOffsetB = get_se(&t->CPB, -6, 6) << 1;
            printf("<li>FilterOffsetA: <code>%d</code></li>\n"
                "<li>FilterOffsetB: <code>%d</code></li>\n",
                FilterOffsetA,
                FilterOffsetB);
        }
    }
    if (error_flags == 0) {
        t->mb_init = mb_void;
        t->mb_init.inter_top_flag = t->mb_init.inter_bot_flag =
            p->constrained_intra_pred_flag;
        t->mb_init.mb_type_B_inc = 1;
        t->mb_init.not_available = 0;
    }
    return error_flags | (t->disable_deblocking_filter_idc == 0 &&
        first_mb_in_slice > 0) << H264_UNSUPPORTED_DEBLOCK_ACROSS_SLICES;
}



/**
 * Dedicated function to parse scaling_list().
 */
static void H264_parse_scaling_list(Bit_ctx *b, uint8_t *weightScale, int num,
    const uint8_t *fallback, const uint8_t *def, const int *invScan)
{
    typedef struct { uint8_t q[16]; } v16qi __attribute__((aligned(16)));
    const v16qi *src;
    int nextScale;
    if (!get_u1(b) || (src = (const v16qi *)def, nextScale = 8 + get_se(b, -128, 127)) == 0) {
        for (int i = 0; i < num; i++)
            ((v16qi *)weightScale)[i] = src[i];
    } else {
        const int *end = invScan + num * 16;
        int lastScale = 0;
        while (nextScale != 0) {
            weightScale[*invScan++] = lastScale = nextScale;
            if (invScan == end)
                break;
            nextScale += get_se(b, -128, 127);
        }
        while (invScan < end)
            weightScale[*invScan++] = lastScale;
    }
    printf("<li>weightScale: <code>");
    for (int i = 0; i < num * 16; i++)
        printf(" %u", weightScale[i]);
    printf("</code></li>\n");
}



/**
 * Interprets and saves a PPS RBSP for the current sequence. Returns 0 if the
 * PPS and all of its features can be decoded.
 */
static inline unsigned int H264_parse_PPS_RBSP(Video_ctx *v, Thread_ctx *t)
{
    unsigned int pic_parameter_set_id = get_ue(&t->CPB, 255);
    v->error_flags |= (pic_parameter_set_id >= 4) <<
        H264_UNSUPPORTED_MORE_THAN_FOUR_PPS;
    unsigned int seq_parameter_set_id = get_ue(&t->CPB, 31);
    v->error_flags |= (seq_parameter_set_id != 0) << H264_UNSUPPORTED_MULTIPLE_SPS;
    printf("<li%s>pic_parameter_set_id: <code>%u</code></li>\n"
        "<li%s>seq_parameter_set_id: <code>%u</code></li>\n",
        red_if(pic_parameter_set_id >= 4), pic_parameter_set_id,
        red_if(seq_parameter_set_id != 0), seq_parameter_set_id);
    if (v->error_flags != 0)
        return v->error_flags;
    PPS_ctx *p = &v->PPSs[pic_parameter_set_id];
    p->entropy_coding_mode_flag = get_u1(&t->CPB);
    p->bottom_field_pic_order_in_frame_present_flag = get_u1(&t->CPB);
    unsigned int num_slice_groups = get_ue(&t->CPB, 7) + 1;
    v->error_flags |= (num_slice_groups > 1) << H264_UNSUPPORTED_FMO_ASO;
    p->num_ref_idx_l0_default_active = get_ue(&t->CPB, 31) + 1;
    p->num_ref_idx_l1_default_active = get_ue(&t->CPB, 31) + 1;
    unsigned int weighted_pred = get_uv(&t->CPB, 3);
    p->weighted_pred_flag = weighted_pred >> 2;
    p->weighted_bipred_idc = weighted_pred & 0x3;
    p->pic_init_qp = get_se(&t->CPB, -62, 25) + 26;
    unsigned int pic_init_qs = get_se(&t->CPB, -26, 25) + 26;
    p->chroma_qp_index_offset = get_se(&t->CPB, -12, 12);
    p->second_chroma_qp_index_offset = p->chroma_qp_index_offset;
    p->deblocking_filter_control_present_flag = get_u1(&t->CPB);
    p->constrained_intra_pred_flag = get_u1(&t->CPB);
    unsigned int redundant_pic_cnt_present_flag = get_u1(&t->CPB);
    v->error_flags |= redundant_pic_cnt_present_flag
        << H264_UNSUPPORTED_REDUNDANT_SLICES;
    printf("<li>entropy_coding_mode_flag: <code>%x</code></li>\n"
        "<li>bottom_field_pic_order_in_frame_present_flag: <code>%x</code></li>\n"
        "<li%s>num_slice_groups: <code>%u</code></li>\n"
        "<li>num_ref_idx_l0_default_active: <code>%u</code></li>\n"
        "<li>num_ref_idx_l1_default_active: <code>%u</code></li>\n"
        "<li>weighted_pred_flag: <code>%x</code></li>\n"
        "<li>weighted_bipred_idc: <code>%u</code></li>\n"
        "<li>pic_init_qp: <code>%u</code></li>\n"
        "<li>pic_init_qs: <code>%u</code></li>\n"
        "<li>chroma_qp_index_offset: <code>%d</code></li>\n"
        "<li>deblocking_filter_control_present_flag: <code>%x</code></li>\n"
        "<li>constrained_intra_pred_flag: <code>%x</code></li>\n"
        "<li%s>redundant_pic_cnt_present_flag: <code>%x</code></li>\n",
        p->entropy_coding_mode_flag,
        p->bottom_field_pic_order_in_frame_present_flag,
        red_if(num_slice_groups > 1), num_slice_groups,
        p->num_ref_idx_l0_default_active,
        p->num_ref_idx_l1_default_active,
        p->weighted_pred_flag,
        p->weighted_bipred_idc,
        p->pic_init_qp,
        pic_init_qs,
        p->chroma_qp_index_offset,
        p->deblocking_filter_control_present_flag,
        p->constrained_intra_pred_flag,
        red_if(redundant_pic_cnt_present_flag), redundant_pic_cnt_present_flag);
    p->transform_8x8_mode_flag = 0;
    unsigned int pic_scaling_matrix_present_flag = 0;
    if (t->CPB.shift < t->lim) {
        p->transform_8x8_mode_flag = get_u1(&t->CPB);
        pic_scaling_matrix_present_flag = get_u1(&t->CPB);
        printf("<li>transform_8x8_mode_flag: <code>%x</code></li>\n"
            "<li>pic_scaling_matrix_present_flag: <code>%x</code></li>\n",
            p->transform_8x8_mode_flag,
            pic_scaling_matrix_present_flag);
        if (pic_scaling_matrix_present_flag) {
            printf("<ul>\n");
            H264_parse_scaling_list(&t->CPB, p->weightScale4x4[0], 1,
                v->weightScale4x4[0], Default_4x4_Intra, invScan4x4[0]);
            H264_parse_scaling_list(&t->CPB, p->weightScale4x4[1], 1,
                p->weightScale4x4[0], Default_4x4_Intra, invScan4x4[0]);
            H264_parse_scaling_list(&t->CPB, p->weightScale4x4[2], 1,
                p->weightScale4x4[1], Default_4x4_Intra, invScan4x4[0]);
            H264_parse_scaling_list(&t->CPB, p->weightScale4x4[3], 1,
                v->weightScale4x4[3], Default_4x4_Inter, invScan4x4[0]);
            H264_parse_scaling_list(&t->CPB, p->weightScale4x4[4], 1,
                p->weightScale4x4[3], Default_4x4_Inter, invScan4x4[0]);
            H264_parse_scaling_list(&t->CPB, p->weightScale4x4[5], 1,
                p->weightScale4x4[4], Default_4x4_Inter, invScan4x4[0]);
            if (p->transform_8x8_mode_flag) {
                H264_parse_scaling_list(&t->CPB, p->weightScale8x8[0], 4,
                    v->weightScale8x8[0], Default_4x4_Intra, invScan8x8[0]);
                H264_parse_scaling_list(&t->CPB, p->weightScale8x8[1], 4,
                    v->weightScale8x8[1], Default_4x4_Inter, invScan8x8[0]);
                if (v->ChromaArrayType == 3) {
                    H264_parse_scaling_list(&t->CPB, p->weightScale8x8[2], 4,
                        p->weightScale8x8[0], Default_4x4_Intra, invScan8x8[0]);
                    H264_parse_scaling_list(&t->CPB, p->weightScale8x8[3], 4,
                        p->weightScale8x8[1], Default_4x4_Inter, invScan8x8[0]);
                    H264_parse_scaling_list(&t->CPB, p->weightScale8x8[4], 4,
                        p->weightScale8x8[2], Default_4x4_Intra, invScan8x8[0]);
                    H264_parse_scaling_list(&t->CPB, p->weightScale8x8[5], 4,
                        p->weightScale8x8[3], Default_4x4_Inter, invScan8x8[0]);
                }
            }
            printf("</ul>\n");
        }
        p->second_chroma_qp_index_offset = get_se(&t->CPB, -12, 12);
        printf("<li>second_chroma_qp_index_offset: <code>%d</code></li>\n",
            p->second_chroma_qp_index_offset);
    }
    if (!pic_scaling_matrix_present_flag) {
        memcpy(p->weightScale4x4, v->weightScale4x4, sizeof(p->weightScale4x4));
        memcpy(p->weightScale8x8, v->weightScale8x8, sizeof(p->weightScale8x8));
    }
    v->error_flags |= (t->CPB.shift != t->lim) << H264_ERROR_PARSING_BITSTREAM;
    return v->error_flags;
}



/**
 * Dedicated function to parse hrd_parameters().
 */
static inline void H264_parse_hrd_parameters(Bit_ctx *b)
{
    unsigned int cpb_cnt = get_ue(b, 31) + 1;
    unsigned int scale = get_uv(b, 8);
    unsigned int bit_rate_scale = scale >> 4;
    unsigned int cpb_size_scale = scale & 0xf;
    printf("<li>cpb_cnt: <code>%u</code></li>\n"
        "<li>bit_rate_scale: <code>%u</code></li>\n"
        "<li>cpb_size_scale: <code>%u</code></li>\n",
        cpb_cnt,
        bit_rate_scale,
        cpb_size_scale);
    do {
        unsigned int bit_rate_value = get_ue(b, 4294967294) + 1;
        unsigned int cpb_size_value = get_ue(b, 4294967294) + 1;
        unsigned int cbr_flag = get_u1(b);
        printf("<ul>\n"
            "<li>bit_rate_value: <code>%u</code></li>\n"
            "<li>cpb_size_value: <code>%u</code></li>\n"
            "<li>cbr_flag: <code>%x</code></li>\n"
            "</ul>\n",
            bit_rate_value,
            cpb_size_value,
            cbr_flag);
    } while (--cpb_cnt != 0);
    unsigned int delays = get_uv(b, 20);
    unsigned int initial_cpb_removal_delay_length = (delays >> 15) + 1;
    unsigned int cpb_removal_delay_length = ((delays >> 10) & 0x1f) + 1;
    unsigned int dpb_output_delay_length = ((delays >> 5) & 0x1f) + 1;
    unsigned int time_offset_length = delays & 0x1f;
    printf("<li>initial_cpb_removal_delay_length: <code>%u</code></li>\n"
        "<li>cpb_removal_delay_length: <code>%u</code></li>\n"
        "<li>dpb_output_delay_length: <code>%u</code></li>\n"
        "<li>time_offset_length: <code>%u</code></li>\n",
        initial_cpb_removal_delay_length,
        cpb_removal_delay_length,
        dpb_output_delay_length,
        time_offset_length);
}



/**
 * Dedicated function to parse vui_parameters(), for readability.
 */
static inline void H264_parse_vui_parameters(Bit_ctx *b)
{
    static const char * const aspect_ratio_idc_names[256] = {
        [0] = "Unspecified",
        [1] = "1:1",
        [2] = "12:11",
        [3] = "10:11",
        [4] = "16:11",
        [5] = "40:33",
        [6] = "24:11",
        [7] = "20:11",
        [8] = "32:11",
        [9] = "80:33",
        [10] = "18:11",
        [11] = "15:11",
        [12] = "64:33",
        [13] = "160:99",
        [14] = "4:3",
        [15] = "3:2",
        [16] = "2:1",
        [17 ... 254] = "unknown",
        [255] = "Extended_SAR",
    };
    static const char * const video_format_names[8] = {
        [0] = "Component",
        [1] = "PAL",
        [2] = "NTSC",
        [3] = "SECAM",
        [4] = "MAC",
        [5] = "Unspecified video format",
        [6 ... 7] = "unknown",
    };
    static const char * const colour_primaries_names[256] = {
        [0] = "unknown",
        [1] = "green(0.300,0.600) blue(0.150,0.060) red(0.640,0.330) whiteD65(0.3127,0.3290)",
        [2] = "Unspecified",
        [3] = "unknown",
        [4] = "green(0.21,0.71) blue(0.14,0.08) red(0.67,0.33) whiteC(0.310,0.316)",
        [5] = "green(0.29,0.60) blue(0.15,0.06) red(0.64,0.33) whiteD65(0.3127,0.3290)",
        [6 ... 7] = "green(0.310,0.595) blue(0.155,0.070) red(0.630,0.340) whiteD65(0.3127,0.3290)",
        [8] = "green(0.243,0.692) blue(0.145,0.049) red(0.681,0.319) whiteC(0.310,0.316)",
        [9] = "green(0.170,0.797) blue(0.131,0.046) red(0.708,0.292) whiteD65(0.3127,0.3290)",
        [10 ... 255] = "unknown",
    };
    static const char * const transfer_characteristics_names[256] = {
        [0] = "unknown",
        [1] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1], V=4.500*Lc for Lc in [0,0.018[",
        [2] = "Unspecified",
        [3] = "unknown",
        [4] = "Assumed display gamma 2.2",
        [5] = "Assumed display gamma 2.8",
        [6] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1], V=4.500*Lc for Lc in [0,0.018[",
        [7] = "V=1.1115*Lc^0.45-0.1115 for Lc in [0.0228,1], V=4.0*Lc for Lc in [0,0.0228[",
        [8] = "V=Lc for Lc in [0,1[",
        [9] = "V=1.0+Log10(Lc)/2 for Lc in [0.01,1], V=0.0 for Lc in [0,0.01[",
        [10] = "V=1.0+Log10(Lc)/2.5 for Lc in [Sqrt(10)/1000,1], V=0.0 for Lc in [0,Sqrt(10)/1000[",
        [11] = "V=1.099*Lc^0.45-0.099 for Lc>=0.018, V=4.500*Lc for Lc in ]-0.018,0.018[, V=-1.099*(-Lc)^0.45+0.099 for Lc<=-0.018",
        [12] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1.33[, V=4.500*Lc for Lc in [-0.0045,0.018[, V=-(1.099*(-4*Lc)^0.45-0.099)/4 for Lc in [-0.25,-0.0045[",
        [13] = "V=1.055*Lc^(1/2.4)-0.055 for Lc in [0.0031308,1[, V=12.92*Lc for Lc in [0,0.0031308[",
        [14] = "V=1.099*Lc^0.45-0.099 for Lc in [0.018,1], V=4.500*Lc for Lc in [0,0.018[",
        [15] = "V=1.0993*Lc^0.45-0.0993 for Lc in [0.0181,1], V=4.500*Lc for Lc in [0,0.0181[",
        [16 ... 255] = "unknown",
    };
    static const char * const matrix_coefficients_names[256] = {
        [0] = "unknown",
        [1] = "Kr = 0.2126; Kb = 0.0722",
        [2] = "Unspecified",
        [3] = "unknown",
        [4] = "Kr = 0.30; Kb = 0.11",
        [5 ... 6] = "Kr = 0.299; Kb = 0.114",
        [7] = "Kr = 0.212; Kb = 0.087",
        [8] = "YCgCo",
        [9] = "Kr = 0.2627; Kb = 0.0593 (non-constant luminance)",
        [10] = "Kr = 0.2627; Kb = 0.0593 (constant luminance)",
        [11 ... 255] = "unknown",
    };
    
    unsigned int aspect_ratio_info_present_flag = get_u1(b);
    printf("<li>aspect_ratio_info_present_flag: <code>%x</code></li>\n",
        aspect_ratio_info_present_flag);
    if (aspect_ratio_info_present_flag) {
        unsigned int aspect_ratio_idc = get_uv(b, 8);
        printf("<li>aspect_ratio_idc: <code>%u (%s)</code></li>\n",
            aspect_ratio_idc, aspect_ratio_idc_names[aspect_ratio_idc]);
        if (aspect_ratio_idc == 255) {
            unsigned int sar = get_uv(b, 32);
            unsigned int sar_width = sar >> 16;
            unsigned int sar_height = sar & 0xffff;
            printf("<li>sar_width: <code>%u</code></li>\n"
                "<li>sar_height: <code>%u</code></li>\n",
                sar_width,
                sar_height);
        }
    }
    unsigned int overscan_info_present_flag = get_u1(b);
    printf("<li>overscan_info_present_flag: <code>%x</code></li>\n",
        overscan_info_present_flag);
    if (overscan_info_present_flag) {
        unsigned int overscan_appropriate_flag = get_u1(b);
        printf("<li>overscan_appropriate_flag: <code>%x</code></li>\n",
            overscan_appropriate_flag);
    }
    unsigned int video_signal_type_present_flag = get_u1(b);
    printf("<li>video_signal_type_present_flag: <code>%x</code></li>\n",
        video_signal_type_present_flag);
    if (video_signal_type_present_flag) {
        unsigned int video_format = get_uv(b, 3);
        unsigned int video_full_range_flag = get_u1(b);
        unsigned int colour_description_present_flag = get_u1(b);
        printf("<li>video_format: <code>%u (%s)</code></li>\n"
            "<li>video_full_range_flag: <code>%x</code></li>\n"
            "<li>colour_description_present_flag: <code>%x</code></li>\n",
            video_format, video_format_names[video_format],
            video_full_range_flag,
            colour_description_present_flag);
        if (colour_description_present_flag) {
            unsigned int desc = get_uv(b, 24);
            unsigned int colour_primaries = desc >> 16;
            unsigned int transfer_characteristics = (desc >> 8) & 0xff;
            unsigned int matrix_coefficients = desc & 0xff;
            printf("<li>colour_primaries: <code>%u (%s)</code></li>\n"
                "<li>transfer_characteristics: <code>%u (%s)</code></li>\n"
                "<li>matrix_coefficients: <code>%u (%s)</code></li>\n",
                colour_primaries, colour_primaries_names[colour_primaries],
                transfer_characteristics, transfer_characteristics_names[transfer_characteristics],
                matrix_coefficients, matrix_coefficients_names[matrix_coefficients]);
        }
    }
    unsigned int chroma_loc_info_present_flag = get_u1(b);
    printf("<li>chroma_loc_info_present_flag: <code>%x</code></li>\n",
        chroma_loc_info_present_flag);
    if (chroma_loc_info_present_flag) {
        unsigned int chroma_sample_loc_type_top_field = get_ue(b, 5);
        unsigned int chroma_sample_loc_type_bottom_field = get_ue(b, 5);
        printf("<li>chroma_sample_loc_type_top_field: <code>%x</code></li>\n"
            "<li>chroma_sample_loc_type_bottom_field: <code>%x</code></li>\n",
            chroma_sample_loc_type_top_field,
            chroma_sample_loc_type_bottom_field);
    }
    unsigned int timing_info_present_flag = get_u1(b);
    printf("<li>timing_info_present_flag: <code>%x</code></li>\n",
        timing_info_present_flag);
    if (timing_info_present_flag) {
        unsigned int num_units_in_tick = get_uv(b, 32);
        unsigned int time_scale = get_uv(b, 32);
        unsigned int fixed_frame_rate_flag = get_u1(b);
        printf("<li>num_units_in_tick: <code>%u</code></li>\n"
            "<li>time_scale: <code>%u</code></li>\n"
            "<li>fixed_frame_rate_flag: <code>%x</code></li>\n",
            num_units_in_tick,
            time_scale,
            fixed_frame_rate_flag);
    }
    unsigned int nal_hrd_parameters_present_flag = get_u1(b);
    printf("<li>nal_hrd_parameters_present_flag: <code>%x</code></li>\n",
        nal_hrd_parameters_present_flag);
    if (nal_hrd_parameters_present_flag)
        H264_parse_hrd_parameters(b);
    unsigned int vcl_hrd_parameters_present_flag = get_u1(b);
    printf("<li>vcl_hrd_parameters_present_flag: <code>%x</code></li>\n",
        vcl_hrd_parameters_present_flag);
    if (vcl_hrd_parameters_present_flag)
        H264_parse_hrd_parameters(b);
    if (nal_hrd_parameters_present_flag || vcl_hrd_parameters_present_flag) {
        unsigned int low_delay_hrd_flag = get_u1(b);
        printf("<li>low_delay_hrd_flag: <code>%x</code></li>\n",
            low_delay_hrd_flag);
    }
    unsigned int pic_struct_present_flag = get_u1(b);
    unsigned int bitstream_restriction_flag = get_u1(b);
    printf("<li>pic_struct_present_flag: <code>%x</code></li>\n"
        "<li>bitstream_restriction_flag: <code>%x</code></li>\n",
        pic_struct_present_flag,
        bitstream_restriction_flag);
    if (bitstream_restriction_flag) {
        unsigned int motion_vectors_over_pic_boundaries_flag = get_u1(b);
        unsigned int max_bytes_per_pic_denom = get_ue(b, 16);
        unsigned int max_bits_per_mb_denom = get_ue(b, 16);
        unsigned int log2_max_mv_length_horizontal = get_ue(b, 5);
        unsigned int log2_max_mv_length_vertical = get_ue(b, 16);
        unsigned int max_num_reorder_frames = get_ue(b, 16);
        unsigned int max_dec_frame_buffering = get_ue(b, 16);
        printf("<li>motion_vectors_over_pic_boundaries_flag: <code>%x</code></li>\n"
            "<li>max_bytes_per_pic_denom: <code>%u</code></li>\n"
            "<li>max_bits_per_mb_denom: <code>%u</code></li>\n"
            "<li>max_mv_length_horizontal: <code>%u</code></li>\n"
            "<li>max_mv_length_vertical: <code>%u</code></li>\n"
            "<li>max_num_reorder_frames: <code>%u</code></li>\n"
            "<li>max_dec_frame_buffering: <code>%u</code></li>\n",
            motion_vectors_over_pic_boundaries_flag,
            max_bytes_per_pic_denom,
            max_bits_per_mb_denom,
            1 << log2_max_mv_length_horizontal,
            1 << log2_max_mv_length_vertical,
            max_num_reorder_frames,
            max_dec_frame_buffering);
    }
}



/**
 * Interprets and saves a SPS RBSP for the current sequence. Returns 0 if the
 * SPS and all of its features can be decoded (unsupported features are coloured
 * red in the trace output), pursue the decoding at your own risk!
 * Transmission errors receive no special care: considering the size of an SPS
 * they will most likely affect slices.
 * On-the-fly changes to chroma format, bit depth, max_num_ref_frames or
 * resolution cause the DPB to be flushed. To display the delayed frames before
 * the new SPS one should issue end-of-sequence NAL units until no output frame
 * is returned.
 */
static inline unsigned int H264_parse_SPS_RBSP(Video_ctx *v, Thread_ctx *t)
{
    static const char * const profile_idc_names[256] = {
        [44] = "CAVLC 4:4:4 Intra",
        [66] = "Baseline",
        [77] = "Main",
        [83] = "Scalable Baseline",
        [86] = "Scalable High",
        [88] = "Extended",
        [100] = "High",
        [110] = "High 10",
        [118] = "Multiview High",
        [122] = "High 4:2:2",
        [128] = "Stereo High",
        [138] = "Multiview Depth High",
        [244] = "High 4:4:4 Predictive",
    };
    static const char * const chroma_format_idc_names[31] = {
        [0] = "4:0:0",
        [1] = "4:2:0",
        [2] = "4:2:2",
        [3] = "4:4:4",
        [4 ... 30] = "unknown",
    };
    
    unsigned int profile_idc = ((uint8_t *)t->CPB.buf)[0];
    v->error_flags = (profile_idc == 66 || profile_idc == 88) <<
        H264_UNSUPPORTED_PROFILE_BASELINE_EXTENDED;
    unsigned int constraint_set_flags = ((uint8_t *)t->CPB.buf)[1];
    unsigned int constraint_set0_flag = constraint_set_flags >> 7;
    unsigned int constraint_set1_flag = (constraint_set_flags >> 6) & 1;
    unsigned int constraint_set2_flag = (constraint_set_flags >> 5) & 1;
    unsigned int constraint_set3_flag = (constraint_set_flags >> 4) & 1;
    unsigned int constraint_set4_flag = (constraint_set_flags >> 3) & 1;
    unsigned int constraint_set5_flag = (constraint_set_flags >> 2) & 1;
    unsigned int level_idc = ((uint8_t *)t->CPB.buf)[2];
    t->CPB.shift = 24;
    unsigned int seq_parameter_set_id = get_ue(&t->CPB, 31);
    printf("<li>profile_idc: <code>%u (%s)</code></li>\n"
        "<li>constraint_set0_flag: <code>%x</code></li>\n"
        "<li>constraint_set1_flag: <code>%x</code></li>\n"
        "<li>constraint_set2_flag: <code>%x</code></li>\n"
        "<li>constraint_set3_flag: <code>%x</code></li>\n"
        "<li>constraint_set4_flag: <code>%x</code></li>\n"
        "<li>constraint_set5_flag: <code>%x</code></li>\n"
        "<li>level_idc: <code>%u</code></li>\n"
        "<li%s>seq_parameter_set_id: <code>%u</code></li>\n",
        profile_idc, profile_idc_names[profile_idc],
        constraint_set0_flag,
        constraint_set1_flag,
        constraint_set2_flag,
        constraint_set3_flag,
        constraint_set4_flag,
        constraint_set5_flag,
        level_idc,
        red_if(seq_parameter_set_id != 0), seq_parameter_set_id);
    if (seq_parameter_set_id != 0)
        return v->error_flags |= 1 << H264_UNSUPPORTED_MULTIPLE_SPS;
    unsigned int ChromaArrayType = 1;
    unsigned int BitDepth_Y = 8;
    unsigned int BitDepth_C = 8;
    v->qpprime_y_zero_transform_bypass_flag = 0;
    unsigned int seq_scaling_matrix_present_flag = 0;
    if (profile_idc != 66 && profile_idc != 77 && profile_idc != 88) {
        ChromaArrayType = get_ue(&t->CPB, 3);
        printf("<li>chroma_format_idc: <code>%u (%s)</code></li>\n",
            ChromaArrayType, chroma_format_idc_names[ChromaArrayType]);
        if (ChromaArrayType == 3) { // Fix ChromaArrayType
            unsigned int separate_colour_plane_flag = get_u1(&t->CPB);
            v->error_flags |= separate_colour_plane_flag <<
                H264_UNSUPPORTED_SEPARATE_COLOUR_PLANES;
            printf("<li%s>separate_colour_plane_flag: <code>%x</code></li>\n",
                red_if(separate_colour_plane_flag), separate_colour_plane_flag);
        }
        BitDepth_Y = get_ue(&t->CPB, 6) + 8;
        BitDepth_C = get_ue(&t->CPB, 6) + 8;
        v->qpprime_y_zero_transform_bypass_flag = get_u1(&t->CPB);
        unsigned int seq_scaling_matrix_present_flag = get_u1(&t->CPB);
        printf("<li>bit_depth_luma: <code>%u</code></li>\n"
            "<li>bit_depth_chroma: <code>%u</code></li>\n"
            "<li>qpprime_y_zero_transform_bypass_flag: <code>%x</code></li>\n"
            "<li>seq_scaling_matrix_present_flag: <code>%x</code></li>\n",
            BitDepth_Y,
            BitDepth_C,
            v->qpprime_y_zero_transform_bypass_flag,
            seq_scaling_matrix_present_flag);
    }
    unsigned int reinit = ChromaArrayType != v->ChromaArrayType |
        BitDepth_Y != v->BitDepth_Y | BitDepth_C != v->BitDepth_C;
    v->ChromaArrayType = ChromaArrayType;
    v->BitDepth_Y = BitDepth_Y;
    v->BitDepth_C = BitDepth_C;
    if (!seq_scaling_matrix_present_flag) {
        memset(v->weightScale4x4, 16, sizeof(v->weightScale4x4));
        memset(v->weightScale8x8, 16, sizeof(v->weightScale8x8));
    } else {
        printf("<ul>\n");
        H264_parse_scaling_list(&t->CPB, v->weightScale4x4[0], 1,
            Default_4x4_Intra, Default_4x4_Intra, invScan4x4[0]);
        H264_parse_scaling_list(&t->CPB, v->weightScale4x4[1], 1,
            v->weightScale4x4[0], Default_4x4_Intra, invScan4x4[0]);
        H264_parse_scaling_list(&t->CPB, v->weightScale4x4[2], 1,
            v->weightScale4x4[1], Default_4x4_Intra, invScan4x4[0]);
        H264_parse_scaling_list(&t->CPB, v->weightScale4x4[3], 1,
            Default_4x4_Inter, Default_4x4_Inter, invScan4x4[0]);
        H264_parse_scaling_list(&t->CPB, v->weightScale4x4[4], 1,
            v->weightScale4x4[3], Default_4x4_Inter, invScan4x4[0]);
        H264_parse_scaling_list(&t->CPB, v->weightScale4x4[5], 1,
            v->weightScale4x4[4], Default_4x4_Inter, invScan4x4[0]);
        H264_parse_scaling_list(&t->CPB, v->weightScale8x8[0], 4,
            Default_8x8_Intra, Default_4x4_Intra, invScan8x8[0]);
        H264_parse_scaling_list(&t->CPB, v->weightScale8x8[1], 4,
            Default_8x8_Inter, Default_4x4_Inter, invScan8x8[0]);
        if (v->ChromaArrayType == 3) {
            H264_parse_scaling_list(&t->CPB, v->weightScale8x8[2], 4,
                v->weightScale8x8[0], Default_4x4_Intra, invScan8x8[0]);
            H264_parse_scaling_list(&t->CPB, v->weightScale8x8[3], 4,
                v->weightScale8x8[1], Default_4x4_Inter, invScan8x8[0]);
            H264_parse_scaling_list(&t->CPB, v->weightScale8x8[4], 4,
                v->weightScale8x8[2], Default_4x4_Intra, invScan8x8[0]);
            H264_parse_scaling_list(&t->CPB, v->weightScale8x8[5], 4,
                v->weightScale8x8[3], Default_4x4_Inter, invScan8x8[0]);
        }
        printf("</ul>\n");
    }
    v->log2_max_frame_num = get_ue(&t->CPB, 12) + 4;
    v->pic_order_cnt_type = get_ue(&t->CPB, 2);
    printf("<li>max_frame_num: <code>%u</code></li>\n"
        "<li>pic_order_cnt_type: <code>%u</code></li>\n",
        1 << v->log2_max_frame_num,
        v->pic_order_cnt_type);
    if (v->pic_order_cnt_type == 0) {
        v->log2_max_pic_order_cnt_lsb = get_ue(&t->CPB, 12) + 4;
        printf("<li>max_pic_order_cnt_lsb: <code>%u</code></li>\n",
            1 << v->log2_max_pic_order_cnt_lsb);
    } else if (v->pic_order_cnt_type == 1) {
        v->delta_pic_order_always_zero_flag = get_u1(&t->CPB);
        v->offset_for_non_ref_pic = get_se(&t->CPB, -2147483647, 2147483647);
        v->offset_for_top_to_bottom_field = get_se(&t->CPB, -2147483647, 2147483647);
        v->num_ref_frames_in_pic_order_cnt_cycle = get_ue(&t->CPB, 255);
        printf("<li>delta_pic_order_always_zero_flag: <code>%x</code></li>\n"
            "<li>offset_for_non_ref_pic: <code>%d</code></li>\n"
            "<li>offset_for_top_to_bottom: <code>%d</code></li>\n"
            "<li>num_ref_frames_in_pic_order_cnt_cycle: <code>%u</code></li>\n"
            "<ul>\n",
            v->delta_pic_order_always_zero_flag,
            v->offset_for_non_ref_pic,
            v->offset_for_top_to_bottom_field,
            v->num_ref_frames_in_pic_order_cnt_cycle);
        for (int i = 0, delta = 0; i < v->num_ref_frames_in_pic_order_cnt_cycle; i++) {
            int offset_for_ref_frame = get_se(&t->CPB, -2147483647, 2147483647);
            v->PicOrderCntDeltas[i] = delta += offset_for_ref_frame;
            printf("<li>offset_for_ref_frame: <code>%d</code></li>\n",
                offset_for_ref_frame);
        }
        printf("</ul>\n");
    }
    unsigned int max_num_ref_frames = get_ue(&t->CPB, 16);
    reinit |= max_num_ref_frames != v->max_num_ref_frames;
    v->max_num_ref_frames = max_num_ref_frames;
    unsigned int gaps_in_frame_num_value_allowed_flag = get_u1(&t->CPB);
    unsigned int pic_width_in_mbs = get_ue(&t->CPB, 543) + 1;
    unsigned int pic_height_in_map_units = get_ue(&t->CPB, 543) + 1;
    v->frame_mbs_only_flag = get_u1(&t->CPB);
    unsigned int FrameHeightInMbs = pic_height_in_map_units <<
        (v->frame_mbs_only_flag ^ 1);
    if (FrameHeightInMbs > 543)
        FrameHeightInMbs = 543;
    unsigned int FrameSizeInMbs = pic_width_in_mbs * FrameHeightInMbs;
    if (FrameSizeInMbs > 36864) {
        FrameHeightInMbs = 36864 / pic_width_in_mbs;
        FrameSizeInMbs = pic_width_in_mbs * FrameHeightInMbs;
    }
    unsigned int width = pic_width_in_mbs * 16;
    unsigned int height = FrameHeightInMbs * 16;
    reinit |= width != v->width | height != v->height;
    v->width = width;
    v->height = height;
    printf("<li>max_num_ref_frames: <code>%u</code></li>\n"
        "<li>gaps_in_frame_num_value_allowed_flag: <code>%x</code></li>\n"
        "<li>pic_width_in_mbs: <code>%u</code></li>\n"
        "<li>pic_height_in_map_units: <code>%u</code></li>\n"
        "<li>frame_mbs_only_flag: <code>%x</code></li>\n",
        v->max_num_ref_frames,
        gaps_in_frame_num_value_allowed_flag,
        pic_width_in_mbs,
        pic_height_in_map_units,
        v->frame_mbs_only_flag);
    v->mb_adaptive_frame_field_flag = 0;
    if (v->frame_mbs_only_flag == 0) {
        v->mb_adaptive_frame_field_flag = get_u1(&t->CPB);
        printf("<li>mb_adaptive_frame_field_flag: <code>%x</code></li>\n",
            v->mb_adaptive_frame_field_flag);
    }
    v->direct_8x8_inference_flag = get_u1(&t->CPB);
    unsigned int frame_cropping_flag = get_u1(&t->CPB);
    printf("<li>direct_8x8_inference_flag: <code>%x</code></li>\n"
        "<li>frame_cropping_flag: <code>%x</code></li>\n",
        v->direct_8x8_inference_flag,
        frame_cropping_flag);
    v->frame_crop_left_offset = v->frame_crop_right_offset =
        v->frame_crop_top_offset = v->frame_crop_bottom_offset = 0;
    if (frame_cropping_flag) {
        unsigned int shiftX = v->ChromaArrayType == 1 | v->ChromaArrayType == 2;
        unsigned int shiftY = (v->frame_mbs_only_flag ^ 1) +
            (v->ChromaArrayType == 1);
        v->frame_crop_left_offset = get_ue(&t->CPB, 8687) << shiftX;
        v->frame_crop_right_offset = get_ue(&t->CPB, 8687) << shiftX;
        v->frame_crop_top_offset = get_ue(&t->CPB, 8687) << shiftY;
        v->frame_crop_bottom_offset = get_ue(&t->CPB, 8687) << shiftY;
        printf("<li>frame_crop_left_offset: <code>%u</code></li>\n"
            "<li>frame_crop_right_offset: <code>%u</code></li>\n"
            "<li>frame_crop_top_offset: <code>%u</code></li>\n"
            "<li>frame_crop_bottom_offset: <code>%u</code></li>\n",
            v->frame_crop_left_offset >> shiftX,
            v->frame_crop_right_offset >> shiftX,
            v->frame_crop_top_offset >> shiftY,
            v->frame_crop_bottom_offset >> shiftY);
    }
    unsigned int vui_parameters_present_flag = get_u1(&t->CPB);
    printf("<li>vui_parameters_present_flag: <code>%x</code></li>\n",
        vui_parameters_present_flag);
    if (vui_parameters_present_flag)
        H264_parse_vui_parameters(&t->CPB);
    v->error_flags |= (t->CPB.shift != t->lim) << H264_ERROR_PARSING_BITSTREAM;
    
    /* When the frame characteristics changed, clear the internal state. */
    if (reinit) {
        pthread_mutex_lock(&v->lock);
        for (int i = 0; i < MAX_THREADS; i++) {
            if (v->threads[i].CPB.buf != NULL) {
                while (v->threads[i].target != NULL)
                    pthread_cond_wait(&v->threads[i].update_target, &v->lock);
                free((uintptr_t *)v->threads[i].CPB.buf);
                v->threads[i].CPB.buf = NULL;
                v->threads[i].CPB_size = 0;
            }
        }
        pthread_mutex_unlock(&v->lock);
        if (v->DPB == NULL) {
            v->lock = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
            v->thread_available = (pthread_cond_t)PTHREAD_COND_INITIALIZER;
        } else {
            free(v->DPB);
            v->DPB = NULL;
        }
        unsigned int luma_size = FrameSizeInMbs * 256 * sizeof(uint16_t);
        unsigned int chroma_size = FrameSizeInMbs * 64 * sizeof(uint16_t) *
            (1 << v->ChromaArrayType >> 1) * (v->ChromaArrayType > 0);
        unsigned int frame_size = luma_size + 2 * chroma_size +
            FrameSizeInMbs * sizeof(Macroblock_motion);
        unsigned int size = (v->max_num_ref_frames + MAX_THREADS) * frame_size;
        if (posix_memalign((void **)&v->DPB, 32, size) != 0)
            v->error_flags |= 1 << H264_ERROR_NO_MEMORY;
        for (int i = 0; i < v->max_num_ref_frames + MAX_THREADS; i++) {
            v->frames[i].planes[0] = v->DPB + frame_size * i;
            v->frames[i].planes[1] = (void *)v->frames[i].planes[0] + luma_size;
            v->frames[i].planes[2] = (void *)v->frames[i].planes[1] + chroma_size;
            v->frames[i].motion_matrix = (void *)v->frames[i].planes[2] + chroma_size;
            v->frames[i].FieldOrderCnt[0] = v->frames[i].FieldOrderCnt[1] = INT32_MIN;
            v->frames[i].used_for_reference = 0;
        }
    }
    return v->error_flags;
}



/**
 * Parses one NAL unit. The whole buffer is internally duplicated on the CPB
 * for removal of emulation_prevention_three_bytes, and to append a "safe zone"
 * speeding up Exp-Golomb parsing.
 *
 * lim shall not be merged with Bit_ctx to form a SODB_ctx, since it would make
 * a BITSTREAM_OVERFLOW_CHECK option tempting, which in turn would mitigate the
 * need for a "safe zone", whick itself justifies the mandatory CPB memcpy which
 * makes it simplest and smoothes performance.
 */
unsigned int H264_parse_NAL(Video_ctx *v, const uint8_t *cur,
    const uint8_t *end)
{
    static const char * const nal_unit_type_names[32] = {
        [0] = "Unspecified",
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
        [16 ... 18] = "unknown",
        [19] = "Coded slice of an auxiliary coded picture",
        [20] = "Coded slice extension",
        [21] = "Coded slice extension for depth view components",
        [22 ... 23] = "unknown",
        [24 ... 31] = "Unspecified",
    };
    
    /* Read the one-byte NAL header. */
    if (cur + 1 >= end)
        return 1 << H264_ERROR_PARSING_BITSTREAM;
    v->nal_ref_idc = *cur >> 5;
    v->nal_unit_type = *cur & 0x1f;
    
    /* Wait for a thread context to become available. */
    pthread_mutex_lock(&v->lock);
    int idx = MAX_THREADS;
    while (idx == MAX_THREADS) {
        for (idx = 0; idx < MAX_THREADS && v->threads[idx].target != NULL; idx++)
            continue;
        if (idx == MAX_THREADS)
            pthread_cond_wait(&v->thread_available, &v->lock);
    }
    Thread_ctx *t = &v->threads[idx];
    pthread_mutex_unlock(&v->lock);
    
    /* Prepare the CPB. */
    const int safety_size = 8;
    uintptr_t size = end - ++cur + safety_size;
    if (size > 36000000 + safety_size) { // 240000 * 1200 * 8
        size = 36000000 + safety_size;
        end = cur + 36000000;
    }
    if (size > t->CPB_size) {
        t->CPB_size = size;
        if (t->CPB.buf != NULL)
            free((uintptr_t *)t->CPB.buf);
        t->CPB.buf = malloc(size);
        if (t->CPB.buf == NULL)
            return 1 << H264_ERROR_NO_MEMORY;
    }
    t->CPB.shift = 0;
    
    /* Seek and remove each emulation_prevention_three_byte. */
    uint8_t *dst = (uint8_t *)t->CPB.buf;
    const uintptr_t *wlim = (uintptr_t *)((uintptr_t)(end - 2) & -sizeof(*wlim));
    const uint8_t *src = cur;
    while (cur < end - 2) {
        const uintptr_t *p = (uintptr_t *)((uintptr_t)cur & -sizeof(*p)) + 1;
        const uint8_t *blim = ((uint8_t *)p < end - 2) ? (uint8_t *)p : end - 2;
        for (unsigned int u = (cur[0] << 8) | cur[1]; cur < blim; cur++) {
            if ((u = ((u & 0xffff) << 8) | cur[2]) == 0x000003) {
                memcpy(dst, src, cur + 2 - src);
                dst += cur + 2 - src;
                src = cur + 3; // incrementing cur would break u!
            }
        }
        
        /* Skip words without a zero odd byte (from Bit Twiddling Hacks). */
        const uintptr_t ones = htobe(0x0001000100010001);
        const uintptr_t negs = htobe(0x0080008000800080);
        while (p < wlim && ((*p - ones) & ~*p & negs) == 0)
            p++;
        cur = (uint8_t *)p;
    }
    memcpy(dst, src, end - src);
    dst += end - src - 1;
    
    /* Skip any cabac_zero_word, delimit the SODB and append the "safe zone". */
    while (*dst == 0)
        dst -= 2;
    t->lim = 8 * (dst - (uint8_t *)t->CPB.buf) + 7 - __builtin_ctz(*dst);
    memset(dst + 1, 0xff, safety_size);
    
    /* Execute the relevant parser. */
    printf("<ul class=\"frame\">\n"
        "<li>nal_ref_idc: <code>%u</code></li>\n"
        "<li%s>nal_unit_type: <code>%u (%s)</code></li>\n",
        v->nal_ref_idc,
        red_if((0xfffffe5d >> v->nal_unit_type) & 1), v->nal_unit_type, nal_unit_type_names[v->nal_unit_type]);
    unsigned int error_flags = 0;
    switch (v->nal_unit_type) {
    case 5:
        v->prevAbsFrameNum = 0;
        v->prevPicOrderCnt = 0;
    case 1:
        error_flags = H264_parse_slice_RBSP(v, t);
        break;
    case 7:
        error_flags = H264_parse_SPS_RBSP(v, t);
        break;
    case 8:
        error_flags = H264_parse_PPS_RBSP(v, t);
        break;
    }
    if (error_flags != 0)
        printf("<li style=\"color: red\">Error 0x%x</li>\n", error_flags);
    printf("</ul>\n");
    return error_flags;
}



/**
 * Parses one NAL unit from a H.264 stream formatted according to Annex B, and
 * updates the buf pointer.
 */
unsigned int H264_parse_annexB_stream(Video_ctx *v, const uint8_t **buf,
    const uint8_t *end)
{
    /* Find the start of the NAL unit. */
    const uint8_t *cur = *buf;
    while (cur < end && *cur == 0x00)
        cur++;
    unsigned int error_flags = 0;
    if (cur < end - 2) {
        const uintptr_t *wlim = (uintptr_t *)((uintptr_t)(end - 2) & -sizeof(*wlim));
        const uint8_t *nal = ++cur;
        for (;;) {
            /* Search for the 00n pattern, n<=1, until cur is aligned. */
            unsigned int u = (cur[0] << 8) | cur[1];
            const uintptr_t *p = (uintptr_t *)((uintptr_t)cur & -sizeof(*p)) + 1;
            const uint8_t *blim = ((uint8_t *)p < end - 2) ? (uint8_t *)p : end - 2;
            while (cur < blim && (u = ((u & 0xffff) << 8) | cur[2]) > 0x000001)
                cur++;
            if (cur != (uint8_t *)p)
                break;
            
            /* Skip words without a zero odd byte (from Bit Twiddling Hacks). */
            const uintptr_t ones = htobe(0x0001000100010001);
            const uintptr_t negs = htobe(0x0080008000800080);
            while (p < wlim && ((*p - ones) & ~*p & negs) == 0)
                p++;
            cur = (uint8_t *)p;
        }
        
        /* cur points to the first byte after rbsp_trailing_bits(). */
        error_flags = H264_parse_NAL(v, nal, (cur < end - 2) ? cur : end);
    }
    *buf = cur + 2;
    return error_flags;
}
