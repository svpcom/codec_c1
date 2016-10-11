#define LOG_TAG "MTVENC_MV"
#include <utils/Log.h>
#include "mtvenclib.h"
#include "enc_define.h"

#define MIN_GOP     1 

#define ALL_CAND_EQUAL  10  /*  any number greater than 5 will work */

const static int tab_exclude[9][9] =  // [last_loc][curr_loc]
{
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 1, 1, 0, 0},
    {0, 0, 0, 0, 1, 1, 1, 1, 1},
    {0, 0, 0, 0, 0, 0, 1, 1, 1},
    {0, 1, 1, 0, 0, 0, 1, 1, 1},
    {0, 1, 1, 0, 0, 0, 0, 0, 1},
    {0, 1, 1, 1, 1, 0, 0, 0, 1},
    {0, 0, 1, 1, 1, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 0, 0}
}; //to decide whether to continue or compute

const static int refine_next[8][2] =    /* [curr_k][increment] */
{
    {0, 0}, {2, 0}, {1, 1}, {0, 2}, { -1, 1}, { -2, 0}, { -1, -1}, {0, -2}
};

const static int QP2QUANT[40] =
{
    1, 1, 1, 1, 2, 2, 2, 2,
    3, 3, 3, 4, 4, 4, 5, 6,
    6, 7, 8, 9, 10, 11, 13, 14,
    16, 18, 20, 23, 25, 29, 32, 36,
    40, 45, 51, 57, 64, 72, 81, 91
};

#define MAX_VMVR  80 //512

#define MIN_QP          0
#define MAX_QP          51
#define SHIFT_QP        12
#define  LAMBDA_ACCURACY_BITS         16
#define  LAMBDA_FACTOR(lambda)        ((int)((double)(1<<LAMBDA_ACCURACY_BITS)*lambda+0.5))

#define  WEIGHTED_COST(factor,bits)   (((factor)*(bits))>>LAMBDA_ACCURACY_BITS)
#define  MV_COST(f,s,cx,cy,px,py)     (WEIGHTED_COST(f,mvbits[((cx)<<(s))-px]+mvbits[((cy)<<(s))-py]))
#define  MV_COST_S(f,cx,cy,px,py)     (WEIGHTED_COST(f,mvbits[cx-px]+mvbits[cy-py]))

static unsigned SAD_Macroblock_0(unsigned char* cur, unsigned char* ref, unsigned stride)
{
    unsigned total = 0;
    unsigned cur_stride  = stride;
    unsigned ref_stride = stride;

    asm volatile(
        "mov r5, #0                                           \n\t"
        "vdup.32 q2, r5                                      \n\t"
        "mov r5, #15                                         \n\t"
        "0:                                                        \n\t"
        "vld1.8 {q0}, [%[cur]], %[cur_stride]      \n\t"
        "vld1.8 {q1}, [%[ref]], %[ref_stride]       \n\t"
        "vrev64.8 q1,q1                                     \n\t"
        "vabd.u8     q0, q0, q1                            \n\t"
        "vpaddl.u8  q0, q0                                  \n\t"
        "vadd.u16   q2, q2, q0                            \n\t"
	 "subs r5, #1                                          \n\t"	
        "bpl 0b                                                  \n\t"
        "vaddl.u16   q2, d4, d5                           \n\t"
        "vadd.u32    d4, d4, d5                           \n\t"
        "vpaddl.u32  d4, d4                                \n\t"
        "vmov.32 %[total], d4[0]                        \n\t"

        : [cur] "+r" (cur), [ref] "+r" (ref) ,[total] "+r" (total)
        : [cur_stride] "r" (cur_stride), [ref_stride] "r" (ref_stride)
        : "cc", "memory", "q0", "q1", "q2", "r5"
    );
    return total;
}

static unsigned SAD_Macroblock_1(unsigned char* cur, unsigned char* ref, unsigned stride)
{
    unsigned total = 0;
    unsigned cur_stride  = stride;
    unsigned ref_stride = stride-16;

    asm volatile(
        "mov r5, #0                                           \n\t"
        "vdup.32 q3, r5                                      \n\t"
        "mov r5, #15                                         \n\t"
        "0:                                                        \n\t"
        "vld1.8 {q0}, [%[cur]], %[cur_stride]      \n\t"
        "vld1.8 {q1}, [%[ref]]!                          \n\t"
        "vld1.8 {d4}, [%[ref]], %[ref_stride]      \n\t"
        "vext.8 d2, d3, d2, #7                            \n\t"
        "vext.8 d3, d4, d3, #7                            \n\t"
        "vrev64.8 q1,q1                                     \n\t"
        "vabd.u8     q0, q0, q1                            \n\t"
        "vpaddl.u8  q0, q0                                  \n\t"
        "vadd.u16   q3, q3, q0                            \n\t"
	 "subs r5, #1                                          \n\t"	
        "bpl 0b                                                  \n\t"
        "vaddl.u16   q3, d6, d7                           \n\t"
        "vadd.u32    d6, d6, d7                           \n\t"
        "vpaddl.u32  d6, d6                                \n\t"
        "vmov.32 %[total], d6[0]                        \n\t"

        : [cur] "+r" (cur), [ref] "+r" (ref) ,[total] "+r" (total)
        : [cur_stride] "r" (cur_stride), [ref_stride] "r" (ref_stride)
        : "cc", "memory", "q0", "q1", "q2", "q3", "r5"
    );
    return total;
}

static unsigned SAD_Macroblock_2(unsigned char* cur, unsigned char* ref, unsigned stride)
{
    unsigned total = 0;
    unsigned cur_stride  = stride;
    unsigned ref_stride = stride-16;

    asm volatile(
        "mov r5, #0                                           \n\t"
        "vdup.32 q3, r5                                      \n\t"
        "mov r5, #15                                         \n\t"
        "0:                                                        \n\t"
        "vld1.8 {q0}, [%[cur]], %[cur_stride]      \n\t"
        "vld1.8 {q1}, [%[ref]]!                          \n\t"
        "vld1.8 {d4}, [%[ref]], %[ref_stride]      \n\t"
        "vext.8 d2, d3, d2, #6                            \n\t"
        "vext.8 d3, d4, d3, #6                            \n\t"
        "vrev64.8 q1,q1                                     \n\t"
        "vabd.u8     q0, q0, q1                            \n\t"
        "vpaddl.u8  q0, q0                                  \n\t"
        "vadd.u16   q3, q3, q0                            \n\t"
	 "subs r5, #1                                          \n\t"	
        "bpl 0b                                                  \n\t"
        "vaddl.u16   q3, d6, d7                           \n\t"
        "vadd.u32    d6, d6, d7                           \n\t"
        "vpaddl.u32  d6, d6                                \n\t"
        "vmov.32 %[total], d6[0]                        \n\t"

        : [cur] "+r" (cur), [ref] "+r" (ref) ,[total] "+r" (total)
        : [cur_stride] "r" (cur_stride), [ref_stride] "r" (ref_stride)
        : "cc", "memory", "q0", "q1", "q2", "q3", "r5"
    );
    return total;
}

static unsigned SAD_Macroblock_3(unsigned char* cur, unsigned char* ref, unsigned stride)
{
    unsigned total = 0;
    unsigned cur_stride  = stride;
    unsigned ref_stride = stride-16;

    asm volatile(
        "mov r5, #0                                           \n\t"
        "vdup.32 q3, r5                                      \n\t"
        "mov r5, #15                                         \n\t"
        "0:                                                        \n\t"
        "vld1.8 {q0}, [%[cur]], %[cur_stride]      \n\t"
        "vld1.8 {q1}, [%[ref]]!                          \n\t"
        "vld1.8 {d4}, [%[ref]], %[ref_stride]      \n\t"
        "vext.8 d2, d3, d2, #5                            \n\t"
        "vext.8 d3, d4, d3, #5                            \n\t"
        "vrev64.8 q1,q1                                     \n\t"
        "vabd.u8     q0, q0, q1                            \n\t"
        "vpaddl.u8  q0, q0                                  \n\t"
        "vadd.u16   q3, q3, q0                            \n\t"
	 "subs r5, #1                                          \n\t"	
        "bpl 0b                                                  \n\t"
        "vaddl.u16   q3, d6, d7                           \n\t"
        "vadd.u32    d6, d6, d7                           \n\t"
        "vpaddl.u32  d6, d6                                \n\t"
        "vmov.32 %[total], d6[0]                        \n\t"

        : [cur] "+r" (cur), [ref] "+r" (ref) ,[total] "+r" (total)
        : [cur_stride] "r" (cur_stride), [ref_stride] "r" (ref_stride)
        : "cc", "memory", "q0", "q1", "q2", "q3", "r5"
    );
    return total;
}

static unsigned SAD_Macroblock_4(unsigned char* cur, unsigned char* ref, unsigned stride)
{
    unsigned total = 0;
    unsigned cur_stride  = stride;
    unsigned ref_stride = stride-16;

    asm volatile(
        "mov r5, #0                                           \n\t"
        "vdup.32 q3, r5                                      \n\t"
        "mov r5, #15                                         \n\t"
        "0:                                                        \n\t"
        "vld1.8 {q0}, [%[cur]], %[cur_stride]      \n\t"
        "vld1.8 {q1}, [%[ref]]!                          \n\t"
        "vld1.8 {d4}, [%[ref]], %[ref_stride]      \n\t"
        "vext.8 d2, d3, d2, #4                            \n\t"
        "vext.8 d3, d4, d3, #4                            \n\t"
        "vrev64.8 q1,q1                                     \n\t"
        "vabd.u8     q0, q0, q1                            \n\t"
        "vpaddl.u8  q0, q0                                  \n\t"
        "vadd.u16   q3, q3, q0                            \n\t"
	 "subs r5, #1                                          \n\t"	
        "bpl 0b                                                  \n\t"
        "vaddl.u16   q3, d6, d7                           \n\t"
        "vadd.u32    d6, d6, d7                           \n\t"
        "vpaddl.u32  d6, d6                                \n\t"
        "vmov.32 %[total], d6[0]                        \n\t"

        : [cur] "+r" (cur), [ref] "+r" (ref) ,[total] "+r" (total)
        : [cur_stride] "r" (cur_stride), [ref_stride] "r" (ref_stride)
        : "cc", "memory", "q0", "q1", "q2", "q3", "r5"
    );
    return total;
}

static unsigned SAD_Macroblock_5(unsigned char* cur, unsigned char* ref, unsigned stride)
{
    unsigned total = 0;
    unsigned cur_stride  = stride;
    unsigned ref_stride = stride-16;

    asm volatile(
        "mov r5, #0                                           \n\t"
        "vdup.32 q3, r5                                      \n\t"
        "mov r5, #15                                         \n\t"
        "0:                                                        \n\t"
        "vld1.8 {q0}, [%[cur]], %[cur_stride]      \n\t"
        "vld1.8 {q1}, [%[ref]]!                          \n\t"
        "vld1.8 {d4}, [%[ref]], %[ref_stride]      \n\t"
        "vext.8 d2, d3, d2, #3                            \n\t"
        "vext.8 d3, d4, d3, #3                            \n\t"
        "vrev64.8 q1,q1                                     \n\t"
        "vabd.u8     q0, q0, q1                            \n\t"
        "vpaddl.u8  q0, q0                                  \n\t"
        "vadd.u16   q3, q3, q0                            \n\t"
	 "subs r5, #1                                          \n\t"	
        "bpl 0b                                                  \n\t"
        "vaddl.u16   q3, d6, d7                           \n\t"
        "vadd.u32    d6, d6, d7                           \n\t"
        "vpaddl.u32  d6, d6                                \n\t"
        "vmov.32 %[total], d6[0]                        \n\t"

        : [cur] "+r" (cur), [ref] "+r" (ref) ,[total] "+r" (total)
        : [cur_stride] "r" (cur_stride), [ref_stride] "r" (ref_stride)
        : "cc", "memory", "q0", "q1", "q2", "q3", "r5"
    );
    return total;
}
static unsigned SAD_Macroblock_6(unsigned char* cur, unsigned char* ref, unsigned stride)
{
    unsigned total = 0;
    unsigned cur_stride  = stride;
    unsigned ref_stride = stride-16;

    asm volatile(
        "mov r5, #0                                           \n\t"
        "vdup.32 q3, r5                                      \n\t"
        "mov r5, #15                                         \n\t"
        "0:                                                        \n\t"
        "vld1.8 {q0}, [%[cur]], %[cur_stride]      \n\t"
        "vld1.8 {q1}, [%[ref]]!                          \n\t"
        "vld1.8 {d4}, [%[ref]], %[ref_stride]      \n\t"
        "vext.8 d2, d3, d2, #2                            \n\t"
        "vext.8 d3, d4, d3, #2                            \n\t"
        "vrev64.8 q1,q1                                     \n\t"
        "vabd.u8     q0, q0, q1                            \n\t"
        "vpaddl.u8  q0, q0                                  \n\t"
        "vadd.u16   q3, q3, q0                            \n\t"
	 "subs r5, #1                                          \n\t"	
        "bpl 0b                                                  \n\t"
        "vaddl.u16   q3, d6, d7                           \n\t"
        "vadd.u32    d6, d6, d7                           \n\t"
        "vpaddl.u32  d6, d6                                \n\t"
        "vmov.32 %[total], d6[0]                        \n\t"

        : [cur] "+r" (cur), [ref] "+r" (ref) ,[total] "+r" (total)
        : [cur_stride] "r" (cur_stride), [ref_stride] "r" (ref_stride)
        : "cc", "memory", "q0", "q1", "q2", "q3", "r5"
    );
    return total;
}

static unsigned SAD_Macroblock_7(unsigned char* cur, unsigned char* ref, unsigned stride)
{
    unsigned total = 0;
    unsigned cur_stride  = stride;
    unsigned ref_stride = stride-16;

    asm volatile(
        "mov r5, #0                                           \n\t"
        "vdup.32 q3, r5                                      \n\t"
        "mov r5, #15                                         \n\t"
        "0:                                                        \n\t"
        "vld1.8 {q0}, [%[cur]], %[cur_stride]      \n\t"
        "vld1.8 {q1}, [%[ref]]!                          \n\t"
        "vld1.8 {d4}, [%[ref]], %[ref_stride]      \n\t"
        "vext.8 d2, d3, d2, #1                            \n\t"
        "vext.8 d3, d4, d3, #1                            \n\t"
        "vrev64.8 q1,q1                                     \n\t"
        "vabd.u8     q0, q0, q1                            \n\t"
        "vpaddl.u8  q0, q0                                  \n\t"
        "vadd.u16   q3, q3, q0                            \n\t"
	 "subs r5, #1                                          \n\t"	
        "bpl 0b                                                  \n\t"
        "vaddl.u16   q3, d6, d7                           \n\t"
        "vadd.u32    d6, d6, d7                           \n\t"
        "vpaddl.u32  d6, d6                                \n\t"
        "vmov.32 %[total], d6[0]                        \n\t"

        : [cur] "+r" (cur), [ref] "+r" (ref) ,[total] "+r" (total)
        : [cur_stride] "r" (cur_stride), [ref_stride] "r" (ref_stride)
        : "cc", "memory", "q0", "q1", "q2", "q3", "r5"
    );
    return total;
}

static unsigned SAD_Macroblock(unsigned char* cur, unsigned char* ref, unsigned stride  ,unsigned char i_off, unsigned dmin)
{
    unsigned total = 0x7fffffff;
    if(i_off == 1)
        total = SAD_Macroblock_1(cur, ref, stride);
    else if(i_off == 2)
        total = SAD_Macroblock_2(cur, ref, stride);
    else if(i_off == 3)
        total = SAD_Macroblock_3(cur, ref, stride);
    else if(i_off == 4)
        total = SAD_Macroblock_4(cur, ref, stride);
    else if(i_off == 5)
        total = SAD_Macroblock_5(cur, ref, stride);
    else if(i_off == 6)
        total = SAD_Macroblock_6(cur, ref, stride);
    else if(i_off == 7)
        total = SAD_Macroblock_7(cur, ref, stride);
    else if(i_off == 0)
        total = SAD_Macroblock_0(cur, ref, stride);
    return total;
}

void CleanMotionSearchMoudle(amvenc_motionsearch_t *motion_search)
{
    if(motion_search){
        if(motion_search->mvbits_array)
            free(motion_search->mvbits_array);
        if(motion_search->mot16x16)
            free(motion_search->mot16x16);
        if(motion_search->min_cost)
            free(motion_search->min_cost);
        if(motion_search->mb_array)
            free(motion_search->mb_array);
        if(motion_search->yc_convert)
            free(motion_search->yc_convert);

        motion_search->mvbits_array = NULL;
        motion_search->mvbits_array = NULL;
	 motion_search->mot16x16 = NULL;
	 motion_search->min_cost = NULL;
	 motion_search->yc_convert = NULL;
    }
    return;
}

AMVEnc_Status InitMotionSearchModule(amvenc_drv_t *p)
{
    amvenc_motionsearch_t *motion_search = (amvenc_motionsearch_t*)(&p->motion_search);
    int search_range = motion_search->mvRange;
    int number_of_subpel_positions = 4 * (2 * search_range + 3);
    int max_mv_bits, max_mvd;
    int temp_bits = 0;
    uint8_t *mvbits;
    int bits, imax, imin, i;

    motion_search->min_cost = (int*)calloc(1,sizeof(int)*p->src.mb_width*p->src.mb_height);
    motion_search->mot16x16 = (mvs_t*)calloc(1,sizeof(mvs_t)*p->src.mb_width*p->src.mb_height);
    motion_search->mb_array = (mbinfo_t*)calloc(1,sizeof(mbinfo_t)*p->src.mb_width*p->src.mb_height);
    motion_search->yc_convert = (uint8_t*)calloc(1,256);

    while (number_of_subpel_positions > 0){
        temp_bits++;
        number_of_subpel_positions >>= 1;
    }
    max_mv_bits = 3 + 2 * temp_bits;
    max_mvd  = (1 << (max_mv_bits >> 1)) - 1;
    motion_search->mvbits_array = (uint8_t*)calloc(1,sizeof(uint8_t) * (2 * max_mvd + 1));
    if ((motion_search->mvbits_array == NULL)
        ||(motion_search->yc_convert == NULL)
        ||(motion_search->mb_array == NULL)||(motion_search->mot16x16 == NULL)||(motion_search->min_cost == NULL)){
        CleanMotionSearchMoudle(motion_search);
        return AMVENC_MEMORY_FAIL;
    }
    mvbits = motion_search->mvbits  = motion_search->mvbits_array + max_mvd;

    mvbits[0] = 1;
    for (bits = 3; bits <= max_mv_bits; bits += 2)
    {
        imax = 1    << (bits >> 1);
        imin = imax >> 1;

        for (i = imin; i < imax; i++)   mvbits[-i] = mvbits[i] = bits;
    }
    for(i = 0; i<256;i++){
         motion_search->yc_convert[i] = (uint8_t)(i*219/255+16);
    }
    
    motion_search->lambda_mode = QP2QUANT[AVC_MAX(0, (int)(p->quant-SHIFT_QP))];
    motion_search->lambda_motion = LAMBDA_FACTOR(motion_search->lambda_mode);
    return AMVENC_SUCCESS;
}

bool MVIntraDecisionABE(amvenc_motionsearch_t *motion_search, int *min_cost, uint8_t *cur, int pitch, bool ave)
{
    int j;
    uint8_t *out;
    int temp, SBE;
    //float ABE;
    bool intra = true;
    int out_value = 0;

    SBE = 0;
    /* top neighbor */
    out = cur - pitch;
    for (j = 0; j < 16; j++)
    {
        out_value = (uint8_t)motion_search->yc_convert[out[j]];
        temp = out_value - cur[j];
        SBE += ((temp >= 0) ? temp : -temp);
    }

    /* left neighbor */
    out = cur - 1;
    out -= pitch;
    cur -= pitch;
    for (j = 0; j < 16; j++)
    {
        out_value = (uint8_t)motion_search->yc_convert[*(out += pitch)];
        temp = out_value - *(cur += pitch);
        SBE += ((temp >= 0) ? temp : -temp);
    }

    /* compare mincost/384 and SBE/64 */
    SBE = SBE <<3; //ABE = SBE/64.0; //
    if (SBE >= *min_cost) //if( ABE*0.8 >= min_cost/384.0) //
    {
        intra = false; // no possibility of intra, just use inter
    }
    else
    {
        if (ave == true)
        {
            *min_cost = (*min_cost + (int)(SBE)) >> 1; // possibility of intra, averaging the cost
        }
        else
        {
            *min_cost = (int)(SBE);
        }
    }

    return intra;
}

static inline void CandidateSelection(amvenc_drv_t *p, int *mvx, int *mvy, int *num_can, int imb, int jmb, int *cmvx, int *cmvy , int mbnum)
{
    amvenc_motionsearch_t *motion_search = (amvenc_motionsearch_t*)(&p->motion_search);
    mvs_t *mot16x16 = motion_search->mot16x16;
    mvs_t *pmot = NULL;
    int mbwidth = p->src.mb_width;
    int mbheight = p->src.mb_height;
    int i, j, same, num1;

    /* this part is for predicted MV */
    int pmvA_x = 0, pmvA_y = 0, pmvB_x = 0, pmvB_y = 0, pmvC_x = 0, pmvC_y = 0;
    int availA = 0, availB = 0, availC = 0;

    *num_can = 0;

    if (p->PrevRefFrameNum != 0) // previous frame is an IDR frame
    {
        /* Spatio-Temporal Candidate (five candidates) */
        pmot = &mot16x16[mbnum]; /* same coordinate previous frame */
        mvx[(*num_can)] = pmot->x ;
        mvy[(*num_can)++] = pmot->y;

        if (imb > 0)  /*left neighbor current frame */
        {
            pmot = &mot16x16[mbnum-1];
            mvx[(*num_can)] = pmot->x ;
            mvy[(*num_can)++] = pmot->y;
        }
        if (jmb > 0)  /*upper neighbor current frame */
        {
            pmot = &mot16x16[mbnum-mbwidth];
            mvx[(*num_can)] = pmot->x ;
            mvy[(*num_can)++] = pmot->y;
        }
        if (imb < mbwidth - 1)  /*right neighbor previous frame */
        {
            pmot = &mot16x16[mbnum+1];
            mvx[(*num_can)] = pmot->x ;
            mvy[(*num_can)++] = pmot->y;
        }
        if (jmb < mbheight - 1)  /*bottom neighbor previous frame */
        {
            pmot = &mot16x16[mbnum+mbwidth];
            mvx[(*num_can)] = pmot->x ;
            mvy[(*num_can)++] = pmot->y;
        }

        /* get predicted MV */
        if (imb > 0)    /* get MV from left (A) neighbor either on current or previous frame */
        {
            availA = 1;
            pmot = &mot16x16[mbnum-1];
            pmvA_x = pmot->x;
            pmvA_y = pmot->y;
        }

        if (jmb > 0) /* get MV from top (B) neighbor either on current or previous frame */
        {
            availB = 1;
            pmot = &mot16x16[mbnum-mbwidth];
            pmvB_x = pmot->x;
            pmvB_y = pmot->y;

            availC = 1;

            if (imb < mbwidth - 1) /* get MV from top-right (C) neighbor of current frame */
            {
                pmot = &mot16x16[mbnum-mbwidth+1];
            }
            else /* get MV from top-left (D) neighbor of current frame */
            {
                pmot = &mot16x16[mbnum-mbwidth-1];
            }
            pmvC_x = pmot->x;
            pmvC_y = pmot->y;
        }
    }
    else  /* only Spatial Candidate (four candidates)*/
    {
        if (imb > 0)  /*left neighbor current frame */
        {
            pmot = &mot16x16[mbnum-1];
            mvx[(*num_can)] = pmot->x ;
            mvy[(*num_can)++] = pmot->y;

            if (jmb > 0)  /*upper-left neighbor current frame */
            {
                pmot = &mot16x16[mbnum-mbwidth-1];
                mvx[(*num_can)] = pmot->x ;
                mvy[(*num_can)++] = pmot->y;
            }
        }
        if (jmb > 0)  /*upper neighbor current frame */
        {
            pmot = &mot16x16[mbnum-mbwidth];
            mvx[(*num_can)] = pmot->x ;
            mvy[(*num_can)++] = pmot->y;

            if (imb < mbheight - 1)  /*upper-right neighbor current frame */
            {
                pmot = &mot16x16[mbnum-mbwidth+1];
                mvx[(*num_can)] = pmot->x ;
                mvy[(*num_can)++] = pmot->y;
            }
        }

        /* get predicted MV */
        if (imb > 0)    /* get MV from left (A) neighbor either on current or previous frame */
        {
            availA = 1;
            pmot = &mot16x16[mbnum-1];
            pmvA_x = pmot->x;
            pmvA_y = pmot->y;
        }

        if (jmb > 0) /* get MV from top (B) neighbor either on current or previous frame */
        {
            availB = 1;
            pmot = &mot16x16[mbnum-mbwidth];
            pmvB_x = pmot->x;
            pmvB_y = pmot->y;

            availC = 1;

            if (imb < mbwidth - 1) /* get MV from top-right (C) neighbor of current frame */
            {
                pmot = &mot16x16[mbnum-mbwidth+1];
            }
            else /* get MV from top-left (D) neighbor of current frame */
            {
                pmot = &mot16x16[mbnum-mbwidth-1];
            }
            pmvC_x = pmot->x;
            pmvC_y = pmot->y;
        }
    }

    /*  3/23/01, remove redundant candidate (possible k-mean) */
    num1 = *num_can;
    *num_can = 1;
    for (i = 1; i < num1; i++)
    {
        same = 0;
        j = 0;
        while (!same && j < *num_can)
        {
            if (mvx[i] == mvx[j] && mvy[i] == mvy[j])
                same = 1;
            j++;
        }
        if (!same)
        {
            mvx[*num_can] = mvx[i];
            mvy[*num_can] = mvy[i];
            (*num_can)++;
        }
    }

    if (num1 == 5 && *num_can == 1)
        *num_can = ALL_CAND_EQUAL; /* all are equal */

    /* calculate predicted MV */

    if (availA && !(availB || availC))
    {
        *cmvx = pmvA_x;
        *cmvy = pmvA_y;
    }
    else
    {
        *cmvx = AVC_MEDIAN(pmvA_x, pmvB_x, pmvC_x);
        *cmvy = AVC_MEDIAN(pmvA_y, pmvB_y, pmvC_y);
    }

    return ;
}


int FullSearch(amvenc_drv_t *p,  uint8_t *prev, uint8_t *cur, int *imin, int *jmin, int ilow, int ihigh, int jlow, int jhigh, int cmvx, int cmvy, int mbnum)
{
    amvenc_motionsearch_t *motion_search = (amvenc_motionsearch_t*)(&p->motion_search);
    int range = motion_search->mvRange;
    mvs_t *mot16x16 = motion_search->mot16x16;
    uint8_t *cand;
    int i, j, k, l;
    int d, dmin;
    int i0 = *imin; /* current position */
    int j0 = *jmin;
    int i_temp = (i0>>3)<<3;
    //int i_temp = i0;
    int lx = p->src.pix_width;
    int offset = i_temp + j0 * lx;
    int i_off = i0 - i_temp;
    int lambda_motion = motion_search->lambda_motion;
    uint8_t *mvbits = motion_search->mvbits;
    int mvshift = 2;
    int mvcost;
    int min_sad = 65535;

    cand = prev + offset;
    dmin = 65535;
    dmin = SAD_Macroblock(cur, cand, lx, i_off,dmin);
    mvcost = MV_COST(lambda_motion, mvshift, 0, 0, cmvx, cmvy);
    dmin += mvcost;
    /* perform spiral search */
    for (k = 1; k <= range; k++){
        i = i0 - k;
        j = j0 - k;
        i_temp = (i>>3)<<3;
        i_off = i - i_temp;
        //i_temp = i;
        cand = prev + i_temp + j * lx;
        for (l = 0; l < 8*k; l++){
            /* no need for boundary checking again */
            if (i >= ilow && i <= ihigh && j >= jlow && j <= jhigh){
                d = SAD_Macroblock(cur, cand, lx, i_off,dmin);
                mvcost = MV_COST(lambda_motion, mvshift, i - i0, j - j0, cmvx, cmvy);
                d +=  mvcost;
                if (d < dmin){
                    dmin = d;
                    *imin = i;
                    *jmin = j;
                    min_sad = d - mvcost; // for rate control
                }
            }

            if (l < (k << 1)){
                i++;
                //cand++;
                i_off++;
                if(i_off>=8){
                    i_off -= 8;
                    cand += 8;
                }		
            }else if (l < (k << 2)){
                j++;
                cand += lx;
            }else if (l < ((k << 2) + (k << 1))){
                i--;
                //cand--;
                i_off--;
                if(i_off<0){
                    i_off += 8;
                    cand -= 8;
                }
            }else{
                j--;
                cand -= lx;
            }
        }
    }
    mot16x16[mbnum].mad = min_sad>>8;
    return dmin;
}



/*************************************************************
    Function:   AVCMoveNeighborSAD
    Date:       3/27/01
    Purpose:    Move neighboring SAD around when center has shifted
*************************************************************/

static inline void AVCMoveNeighborSAD(int* dn, int new_loc)
{
    int tmp[9];
    tmp[0] = dn[0];
    tmp[1] = dn[1];
    tmp[2] = dn[2];
    tmp[3] = dn[3];
    tmp[4] = dn[4];
    tmp[5] = dn[5];
    tmp[6] = dn[6];
    tmp[7] = dn[7];
    tmp[8] = dn[8];
    dn[0] = dn[1] = dn[2] = dn[3] = dn[4] = dn[5] = dn[6] = dn[7] = dn[8] = 65536;

    switch (new_loc)
    {
        case 0:
            break;
        case 1:
            dn[4] = tmp[2];
            dn[5] = tmp[0];
            dn[6] = tmp[8];
            break;
        case 2:
            dn[4] = tmp[3];
            dn[5] = tmp[4];
            dn[6] = tmp[0];
            dn[7] = tmp[8];
            dn[8] = tmp[1];
            break;
        case 3:
            dn[6] = tmp[4];
            dn[7] = tmp[0];
            dn[8] = tmp[2];
            break;
        case 4:
            dn[1] = tmp[2];
            dn[2] = tmp[3];
            dn[6] = tmp[5];
            dn[7] = tmp[6];
            dn[8] = tmp[0];
            break;
        case 5:
            dn[1] = tmp[0];
            dn[2] = tmp[4];
            dn[8] = tmp[6];
            break;
        case 6:
            dn[1] = tmp[8];
            dn[2] = tmp[0];
            dn[3] = tmp[4];
            dn[4] = tmp[5];
            dn[8] = tmp[7];
            break;
        case 7:
            dn[2] = tmp[8];
            dn[3] = tmp[0];
            dn[4] = tmp[6];
            break;
        case 8:
            dn[2] = tmp[1];
            dn[3] = tmp[2];
            dn[4] = tmp[0];
            dn[5] = tmp[6];
            dn[6] = tmp[7];
            break;
    }
    dn[0] = tmp[new_loc];

    return ;
}

void UpdateMotionQP(amvenc_drv_t* p)
{
    p->motion_search.lambda_mode = QP2QUANT[AVC_MAX(0, (int)(p->quant-SHIFT_QP))];
    p->motion_search.lambda_motion = LAMBDA_FACTOR(p->motion_search.lambda_mode);
}

void MBMotionSearch(amvenc_drv_t *p, int i0, int j0, int mbnum, bool FS_en)
{
    amvenc_motionsearch_t *motion_search = (amvenc_motionsearch_t*)(&p->motion_search);
    uint8_t *ref, *cur, *cand;
    int width = p->src.pix_width;
    int height = p->src.pix_height;
    mvs_t *mot16x16 = motion_search->mot16x16;
    int range = motion_search->mvRange;
    int lx = p->src.pix_width;
    int i, j, imin, jmin, ilow, ihigh, jlow, jhigh;
    int d, dmin, dn[9];
    int k;
    int mvx[5], mvy[5];
    int num_can, center_again;
    int last_loc, new_loc = 0;
    int step, max_step = range >> 1;
    int next;
    int i_temp = (i0>>3)<<3;
    int i_off = i0 - i_temp;

    int cmvx, cmvy; /* estimated predicted MV */
    int lambda_motion = motion_search->lambda_motion;
    uint8_t *mvbits = motion_search->mvbits;
    int mvshift = 2;
    int mvcost;
    int min_sad = 65535;

    ref = p->ref_info.y; /* origin of actual frame */
    cur = p->src.pic[1].plane[0]+i0+j0*lx;

    /* we can make this part dynamic based on previous statistics */
    ilow = i0 - range;
    if (i0 - ilow > 2047) /* clip to conform with the standard */
    {
        ilow = i0 - 2047;
    }

    if (ilow < 0)  // change it from -15 to -13 because of 6-tap filter needs extra 2 lines.
    {
        ilow = 0;
    }
    
    ihigh = i0 + range - 1;
    if (ihigh - i0 > 2047) /* clip to conform with the standard */
    {
        ihigh = i0 + 2047;
    }

    if (ihigh > width - 16)
    {
        ihigh = width - 16;  // change from width-1 to width-3 for the same reason as above
    }

    jlow = j0 - range;
    if (j0 - jlow >(MAX_VMVR - 1)) /* clip to conform with the standard */
    {
        jlow = j0 - MAX_VMVR + 1;
    }
    if (jlow < 0)     // same reason as above
    {
        jlow = 0;
    }

    jhigh = j0 + range - 1;
    if (jhigh - j0 > (MAX_VMVR - 1)) /* clip to conform with the standard */
    {
        jhigh = j0 + MAX_VMVR - 1;
    }

    if (jhigh > height - 16) // same reason as above
    {
        jhigh = height - 16;
    }
    /* find initial motion vector & predicted MV*/
    CandidateSelection(p, mvx, mvy, &num_can, i0 >> 4, j0 >> 4, &cmvx, &cmvy,mbnum);

    imin = i0;
    jmin = j0; /* needed for fullsearch */

    /* for first row of MB, fullsearch can be used */
    if (FS_en)
    {
        dmin =  FullSearch(p, ref, cur, &imin, &jmin, ilow, ihigh, jlow, jhigh, cmvx, cmvy,mbnum);
    }
    else
    {   /*       fullsearch the top row to only upto (0,3) MB */
        /*       upto 30% complexity saving with the same complexity */
        if (p->PrevRefFrameNum == 0 && j0 == 0 && i0 <= 64)
        {
            dmin =  FullSearch(p, ref, cur, &imin, &jmin, ilow, ihigh, jlow, jhigh, cmvx, cmvy,mbnum);
        }
        else
        {
            /************** initialize candidate **************************/

            dmin = 65535;

            /* check if all are equal */
            if (num_can == ALL_CAND_EQUAL)
            {
                i = i0 + mvx[0];
                j = j0 + mvy[0];
                //i_temp = i;
                i_temp = (i>>3)<<3;
                i_off = i - i_temp;
                if (i >= ilow && i <= ihigh && j >= jlow && j <= jhigh)
                {
                    cand = ref + i_temp + j * lx;
                    d = SAD_Macroblock(cur, cand, lx, i_off,dmin);
                    mvcost = MV_COST(lambda_motion, mvshift, i - i0, j - j0, cmvx, cmvy);
                    d +=  mvcost;

                    if (d < dmin)
                    {
                        dmin = d;
                        imin = i;
                        jmin = j;
                        min_sad = d - mvcost; // for rate control
                    }
                }
            }
            else
            {
                /************** evaluate unique candidates **********************/
                for (k = 0; k < num_can; k++)
                {
                    i = i0 + mvx[k];
                    j = j0 + mvy[k];
                    //i_temp = i;
                    i_temp = (i>>3)<<3;
                    i_off = i - i_temp;
                    if (i >= ilow && i <= ihigh && j >= jlow && j <= jhigh)
                    {
                        cand = ref + i_temp + j * lx;
                        d = SAD_Macroblock(cur, cand, lx, i_off,dmin);
                        mvcost = MV_COST(lambda_motion, mvshift, i - i0, j - j0, cmvx, cmvy);
                        d +=  mvcost;

                        if (d < dmin)
                        {
                            dmin = d;
                            imin = i;
                            jmin = j;
                            min_sad = d - mvcost; // for rate control
                        }
                    }
                }
            }

            /******************* local refinement ***************************/
            center_again = 0;
            last_loc = new_loc = 0;
            step = 0;
            dn[0] = dmin;
            while (!center_again && step <= max_step)
            {

                AVCMoveNeighborSAD(dn, last_loc);

                center_again = 1;
                i = imin;
                j = jmin - 1;
                //i_temp = i;
                i_temp = (i>>3)<<3;
                i_off = i - i_temp;
                cand = ref + i_temp + j * lx;
                /*  starting from [0,-1] */
                /* spiral check one step at a time*/
                for (k = 2; k <= 8; k += 2)
                {
                    if (!tab_exclude[last_loc][k]) /* exclude last step computation */
                    {       /* not already computed */
                        if (i >= ilow && i <= ihigh && j >= jlow && j <= jhigh)
                        {
                            d = SAD_Macroblock(cur, cand, lx, i_off,dmin);
                            mvcost = MV_COST(lambda_motion, mvshift, i - i0, j - j0, cmvx, cmvy);
                            d += mvcost;

                            dn[k] = d; /* keep it for half pel use */

                            if (d < dmin)
                            {
                                dmin = d;
                                imin = i;
                                jmin = j;
                                center_again = 0;
                                new_loc = k;
                                min_sad = d - mvcost; // for rate control
                            }
                        }
                    }
                    if (k == 8)  /* end side search*/
                    {
                        if (!center_again)
                        {
                            k = -1; /* start diagonal search */
                            cand -= lx;
                            j--;
                        }
                    }
                    else
                    {
                        next = refine_next[k][0];
                        i += next;
                        //cand += next;
                        i_off += next;
                        if(i_off>=8){
                            i_off -= 8;
	                     cand += 8;
                        }else if(i_off<0){
                            i_off += 8;
	                     cand -= 8;
                        }
                        next = refine_next[k][1];
                        j += next;
                        cand += lx * next;
                    }
                }
                last_loc = new_loc;
                step ++;
            }
            if (!center_again)
                AVCMoveNeighborSAD(dn, last_loc);
        }
    }

    mot16x16[mbnum].sad = dmin;
    mot16x16[mbnum].x = (imin - i0);
    mot16x16[mbnum].y = (jmin - j0);
    mot16x16[mbnum].mad = min_sad>>8;
    return ;
}

