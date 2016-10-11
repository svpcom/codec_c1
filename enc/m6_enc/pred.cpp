/* ------------------------------------------------------------------
 * Copyright (C) 1998-2009 PacketVideo
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 * -------------------------------------------------------------------
 */
#include "pred.h"
#include <stdlib.h>

#define LOG_TAG "PRED_MODE"
#include <utils/Log.h>
#define TH_I4  0  /* threshold biasing toward I16 mode instead of I4 mode */
#define TH_Intra  0 /* threshold biasing toward INTER mode instead of intra mode */

#define FIXED_INTRAPRED_MODE  AML_I16
#define FIXED_I16_MODE  AML_I16_DC
#define FIXED_I4_MODE   AML_I4_Diagonal_Down_Left
#define FIXED_INTRA_CHROMA_MODE AML_IC_DC

#ifndef AML_MAX
#define AML_MAX(x,y) ((x)>(y)? (x):(y))
#endif
#ifndef AML_MIN
#define AML_MIN(x,y) ((x)<(y)? (x):(y))
#endif

#define CLIP_RESULT(x)      if((uint)x > 0xFF){ \
                 x = 0xFF & (~(x>>31));}


#define DISABLE_16x16
#define SHIFT_QP        12
#define  LAMBDA_ACCURACY_BITS         16
#define  LAMBDA_FACTOR(lambda)        ((int)((double)(1<<LAMBDA_ACCURACY_BITS)*lambda+0.5))
//#define  AMLNumI4PredMode 3
/*
Availability of the neighboring top-right block relative to the current block. */
const static int BlkTopRight_new[16] = {2, 2, 2, 3, 
										1, 1, 1, 3, 
										1, 1, 1, 3, 
										1, 1, 1, 3};

const static int QP2QUANT[40] =
{
    1, 1, 1, 1, 2, 2, 2, 2,
    3, 3, 3, 4, 4, 4, 5, 6,
    6, 7, 8, 9, 10, 11, 13, 14,
    16, 18, 20, 23, 25, 29, 32, 36,
    40, 45, 51, 57, 64, 72, 81, 91
};
/** from [blk8indx][blk4indx] to raster scan index */
const static int blkId_2blkXY[4][4] = {{0, 1, 4, 5}, {2, 3, 6, 7}, {8, 9, 12, 13}, {10, 11, 14, 15}};

void InitNeighborAvailability_iframe(amvenc_pred_mode_t *predMB,
					/*amvenc_drv_t* p*/ amvenc_curr_pred_mode_t* cur)
{	
	amvenc_neighbor_t* n = &cur->neighbor;

	int mb_x = n->mbx;
	int mb_y = n->mby;
	int mb_width = predMB->mb_width;
	int mb_height = predMB->mb_height;

	n->mbAvailA =0 ;
	n->mbAvailB =0 ;
	n->mbAvailC =0 ; 
	n->mbAvailD =0 ;

	cur->mbNum = mb_x + mb_y * mb_width;
	
	if(mb_x){
		n->mbAvailA = 1;
	}
	if(mb_y){
		n->mbAvailB = 1;
		n->mbAvailC = 1;
	}

	if((mb_y+1)%mb_width == 0){
		n->mbAvailC = 0;
	}
	n->mbAvailD = n->mbAvailA && n->mbAvailB;

	if(n->mbAvailA){
		cur->mbAddrA = cur->mbNum -1;
	}

	if(n->mbAvailB){
		cur->mbAddrB = cur->mbNum - mb_width;
	}

	if(n->mbAvailC){
		cur->mbAddrC = cur->mbNum - mb_width + 1;
	}

	if(n->mbAvailD){
		cur->mbAddrD = cur->mbNum - mb_width - 1;
	}

}

int InitMBIntraSearchModule( amvenc_drv_t *p)
{   
	amvenc_pred_mode_t *predMB = (amvenc_pred_mode_t *) (&p->intra_mode);
	uint totalMB;
	uint i;
	int *cost;

	//currMB = predMB->currMB;
	//predMB->mb_x = mb_x;
	//predMB->mb_y = mb_y;
	predMB->mb_width = p->src.mb_width;
	predMB->mb_height = p->src.mb_height;
	predMB->pitch = p->src.pix_width;
	predMB->height = p->src.pix_height;
	//currMB->CBP = 0;
	//predMB->lambda_mode = QP2QUANT[AML_MAX(0, (int)(p->quant-SHIFT_QP))];
	//predMB->lambda_motion = LAMBDA_FACTOR(predMB->lambda_mode);
	totalMB = predMB->mb_width * predMB->mb_height;

	//intramode->predMB = (amvenc_pred_mode_t*) malloc(sizeof(amvenc_pred_mode_t));
	//predMB->mbNum  = 0;
	predMB->mblock = (AMLMacroblock *)calloc(totalMB,sizeof(AMLMacroblock));
	//predMB->currMB = predMB->mblock;
	predMB->min_cost = (int *)calloc(totalMB,sizeof(int));
	cost = predMB->min_cost;
    
	if(NULL == predMB->mblock){
		ALOGE("%s failed\n", __func__);
		return -1;
	}
	return 0;
}

int CleanMBIntraSearchModule( amvenc_drv_t *p)
{  
	amvenc_pred_mode_t *predMB = (amvenc_pred_mode_t *) (&p->intra_mode);

	if(predMB->mblock){
		free( predMB->mblock);
		predMB->mblock = NULL;
	}

	if(predMB->min_cost){
		free( predMB->min_cost);
		predMB->min_cost = NULL;
	}
	return 0;
}

amvenc_curr_pred_mode_t *InitcurrMBStruct()
{
	amvenc_curr_pred_mode_t *curr;
	curr = (amvenc_curr_pred_mode_t *)calloc(1,sizeof(amvenc_curr_pred_mode_t));
	if(NULL == curr){
		ALOGE("%s failed\n", __func__);
	}    
	return curr;
}

int ClearcurrMBStruct(void *p)
{
	if(p)
		free( p);
    	p = NULL;
	return 0;
}


inline void intrapred_luma_16x16(amvenc_pred_mode_obj_t * encvid)
{

    amvenc_curr_pred_mode_t *video = encvid->mb_node;
    amvenc_pred_mode_t *common = encvid->mb_list;

    int *intraAvail = &video->neighbor.mbAvailA;

    int x_pos = (video->neighbor.mbx) << 4;
    int y_pos = (video->neighbor.mby) << 4;
    int pitch = common->pitch;
    int offset = y_pos * pitch + x_pos;

    uint8_t *curL = encvid->YCbCr[0] + offset; /* point to reconstructed frame */
    uint8_t *pred = video->pred_i16[AML_I16_Vertical];

    asm volatile(

    "ldr        r7,  [%[intraAvail]]            @r7: video->intraAvailA\n\t"
    "ldr        r8,  [%[intraAvail],  #4]       @r8: video->intraAvailB\n\t"

    "VMOV.I32   d2,  #0                         @d2: sum\n\t"

    "cmp        r8,  #0                         \n\t"
    "beq        3f                              \n\t"

    "sub        r9,  %[curL],  %[pitch]         @r9: top = curL - pitch;\n\t"
    "VLD1.32    {d0,  d1},  [r9]                @Q0: word4 word3 word2 word1\n\t"

    "MOV        r10,  #16                       \n\t"
    "1:                                         \n\t"
    "VST1.32    {d0,  d1},  [%[pred]]!          \n\t"
    "subs       r10,  r10,  #1                  \n\t"
    "bgt        1b                              \n\t"

    "VPADDL.U8  Q0,  Q0                         \n\t"
    "VPADD.U16  d0,  d0,  d1                    \n\t"
    "VPADDL.U16 d0,  d0                         \n\t"
    "VPADDL.U32 d0,  d0                         \n\t"
    "VADD.U32   d2,  d2,  d0                    \n\t"

    "cmp        r7,  #0                         \n\t"
    "bne        3f                              \n\t"
    "VMOV.I32   d0,  #8                         \n\t"
    "VADD.U32   d2,  d2,  d0                    \n\t"
    "VSHR.U32   d2,  d2,  #4                    \n\t"

    "3:                                         \n\t"
    "cmp        r7,  #0                         \n\t"
    "beq        6f                              \n\t"

    "sub        r9,  %[curL],  #1               @r9: left = curL - 1\n\t"

    "mov        r11,  #16                       \n\t"
    "4:                                         \n\t"
    "ldrb       r12, [r9]                       \n\t"
    "add        r9, r9, %[pitch]                \n\t"

    "VMOV.U32   d0[0],  r12                     \n\t"
    "VADD.U32   d2,  d2,  d0                    \n\t"
    "VDUP.I8    Q0,  r12                        \n\t"
    "@pred = video->pred_i16[AML_I16_Horizontal]\n\t"
    "VST1.32    {d0,  d1},  [%[pred]]!          \n\t" 
    "subs       r11,  r11,  #1                  \n\t"
    "bgt        4b                              \n\t"

    "cmp        r8,  #0                         \n\t"
    "beq        5f                              \n\t"
    "VMOV.I32   d0,  #16                        \n\t"
    "VADD.U32   d2,  d2,  d0                    \n\t"
    "VSHR.U32   d2,  d2,  #5                    \n\t"
    "b          6f                              \n\t"
    "5:                                         \n\t"
    "VMOV.I32   d0,  #8                         \n\t"
    "VADD.U32   d2,  d2,  d0                    \n\t"
    "VSHR.U32   d2,  d2,  #4                    \n\t"

    "6:                                         \n\t"
    "cmp        r7,  #0                         \n\t"
    "bne        7f                              \n\t"
    "cmp        r8,  #0                         \n\t"
    "bne        7f                              \n\t"
    "VMOV.I8    Q0,  #0x80                      \n\t"
    "b          8f                              \n\t"

    "7:                                         \n\t"
    "VDUP.I8    Q0,  d2[0]                      \n\t"

    "8:                                         \n\t"
    "mov        r10,  #16                       \n\t"

    "9:                                         \n\t"
    "@pred = encvid->pred_i16[AML_I16_DC]       \n\t"
    "VST1.32    {d0,  d1},  [%[pred]]!          \n\t"
    "subs       r10,  r10,  #1                  \n\t"
    "bgt        9b                              \n\t"

    "12:@end                                    \n\t"

        : [curL] "+r" (curL), [intraAvail] "+r" (intraAvail), [pred] "+r" (pred)
        : [pitch] "r" (pitch)
        : "cc", "memory",
          "r7", "r8", "r9", "r10", "r11", "r12",
          "q0", "q1"
    );
}
/* evaluate each prediction mode of I16 */
void find_cost_16x16( amvenc_pred_mode_obj_t * encvid,
        /*AMLEncObject *encvid,*/ uint8_t *orgY, int *min_cost)
{
    amvenc_curr_pred_mode_t *video = encvid->mb_node;
    amvenc_pred_mode_t *common = encvid->mb_list;
    //video->currMB = common->mblock + video->mbNum;
    AMLMacroblock *currMB = video->currMB;

    //AMLCommonObj *video = encvid->common;
    //AMLMacroblock *currMB = video->currMB;
    int cost;
    //int org_pitch = encvid->currInput->pitch;
    int org_pitch = common->pitch;

    /* evaluate vertical mode */
    //if (video->intraAvailB)
    if (video->neighbor.mbAvailB)
    {
        cost = pred_cost_i16(orgY, org_pitch, video->pred_i16[AML_I16_Vertical], *min_cost);
        if (cost < *min_cost)
        {
            *min_cost = cost;
            currMB->mbMode = AML_I16;
            currMB->mb_intra = 1;
            currMB->i16Mode = AML_I16_Vertical;
        }
    }


    /* evaluate horizontal mode */
    //if (video->intraAvailA)
    if (video->neighbor.mbAvailA)
    {
        cost = pred_cost_i16(orgY, org_pitch, video->pred_i16[AML_I16_Horizontal], *min_cost);
        if (cost < *min_cost)
        {
            *min_cost = cost;
            currMB->mbMode = AML_I16;
            currMB->mb_intra = 1;
            currMB->i16Mode = AML_I16_Horizontal;
        }
    }

    /* evaluate DC mode */
    cost = pred_cost_i16(orgY, org_pitch, video->pred_i16[AML_I16_DC], *min_cost);
    if (cost < *min_cost)
    {
        *min_cost = cost;
        currMB->mbMode = AML_I16;
        currMB->mb_intra = 1;
        currMB->i16Mode = AML_I16_DC;
    }


    return ;
}

/* perform searching for MB mode */
/* assuming that this is done inside the encoding loop,
no need to call InitNeighborAvailability */

void MBIntraSearch_iframe( amvenc_pred_mode_obj_t * encvid, unsigned char* input_addr )
{
    amvenc_curr_pred_mode_t *video = encvid->mb_node;
    amvenc_pred_mode_t *common = encvid->mb_list;
    //amvenc_pred_mode_t *predMB = (amvenc_pred_mode_t *) (&para->intra_mode);
    video->currMB = common->mblock + video->mbNum;
    AMLMacroblock *currMB = video->currMB;
    int min_cost;
    uint8_t *orgY;
    uint8_t *curL;
    int x_pos = (video->neighbor.mbx) << 4;
    int y_pos = (video->neighbor.mby) << 4;
    uint32_t *saved_inter;
    int j;
    AMLSliceType slice_type;
    int orgPitch = common->pitch;
    bool intra = true;
    static int count =0;
    //currMB->CBP = 0;
    video->slice_type = AML_I_SLICE;
    curL = encvid->YCbCr[0] + y_pos * orgPitch + x_pos;
    unsigned int *p = (unsigned int *)input_addr;
    
    //LOGV("MBIntraSearch start, video type is %d\n ",video->slice_type);	
    /* first do motion vector and variable block size search */
    //min_cost = common->min_cost[video->mbNum];
    min_cost = 0x7fffffff;

    /* now perform intra prediction search */
    /* need to add the check for encvid->intraSearch[video->mbNum] to skip intra
       if it's not worth checking. */

    /* i16 mode search */
    /* generate all the predictions */
#ifndef DISABLE_16x16       
    orgY = encvid->YCbCr[0] + y_pos * orgPitch + x_pos;
    intrapred_luma_16x16(encvid);

    /* evaluate them one by one */
    find_cost_16x16(encvid, orgY, &min_cost);
#else
    /* i4 mode search */
    mb_intra4x4_search(encvid, &min_cost);
#endif

#ifndef DISABLE_16x16       
    p[2]= currMB->i16Mode | currMB->i16Mode << 4 |
        currMB->i16Mode << 8 | currMB->i16Mode << 12 |
        currMB->i16Mode << 16| currMB->i16Mode << 20 |
        currMB->i16Mode << 24| currMB->i16Mode << 28;
    p[3] = p[2];
#else
    p[2]= currMB->i4Mode[10] | currMB->i4Mode[11] << 4 |
        currMB->i4Mode[14] << 8| currMB->i4Mode[15]<< 12 |
        currMB->i4Mode[8] << 16| currMB->i4Mode[9] << 20|
        currMB->i4Mode[12]<< 24 | currMB->i4Mode[13] << 28;

    p[3]= currMB->i4Mode[2] | currMB->i4Mode[3] << 4 |
        currMB->i4Mode[6] << 8| currMB->i4Mode[7]<< 12 |
        currMB->i4Mode[0] << 16| currMB->i4Mode[1] << 20|
        currMB->i4Mode[4]<< 24 | currMB->i4Mode[5] << 28;
#endif

    return ;
}

void mb_intra4x4_search(amvenc_pred_mode_obj_t * encvid, int *min_cost)
{
    amvenc_curr_pred_mode_t *video = encvid->mb_node;
    AMLMacroblock *currMB;
    amvenc_pred_mode_t *common = encvid->mb_list;
    //video->currMB = common->mblock + video->mbNum;
    currMB = video->currMB;
    //AMLCommonObj *video = encvid->common;
    //AMLMacroblock *currMB = video->currMB;
    //AMLPictureData *currPic = video->currPic;
    //AMLFrameIO *currInput = encvid->currInput;
    int pitch = common->pitch;
    int org_pitch = common->pitch;
    int offset;
    uint8_t *curL, *comp, *org4, *org8;
    int y = video->neighbor.mby << 4;
    int x = video->neighbor.mbx << 4;

    int b8, b4, cost4x4, blkidx;
    int cost = 0;
    int numcoef;
    int dummy = 0;
    int mb_intra = currMB->mb_intra; // save the original value

    offset = y * pitch + x;

    curL = encvid->YCbCr[0] + offset;
    org8 = encvid->YCbCr[0] + y * org_pitch + x;
    //video->pred_pitch = 4;

    cost = (int)(6.0 * encvid->lambda_mode + 0.4999);
    cost <<= 2;

    currMB->mb_intra = 1;  // temporary set this to one to enable the IDCT
    // operation inside dct_luma

    for (b8 = 0; b8 < 4; b8++)
    {
        comp = curL;
        org4 = org8;

        for (b4 = 0; b4 < 4; b4++)
        {
            blkidx = blkId_2blkXY[b8][b4];
            cost4x4 = blk_intra4x4_search_iframe(encvid, blkidx, comp, org4);
            cost += cost4x4;
            if (cost > *min_cost)
            {
                currMB->mb_intra = mb_intra; // restore the value
                return ;
            }

            if (b4&1)
            {
                comp += ((pitch << 2) - 4);
                org4 += ((org_pitch << 2) - 4);
            }
            else
            {
                comp += 4;
                org4 += 4;
            }
        }

        if (b8&1)
        {
            curL += ((pitch << 3) - 8);
            org8 += ((org_pitch << 3) - 8);
        }
        else
        {
            curL += 8;
            org8 += 8;
        }
    }

    currMB->mb_intra = mb_intra; // restore the value

    if (cost < *min_cost)
    {
        *min_cost = cost;
        currMB->mbMode = AML_I4;
        currMB->mb_intra = 1;
    }

    return ;
}

/* search for i4 mode for a 4x4 block */
int blk_intra4x4_search_iframe(amvenc_pred_mode_obj_t *encvid, int blkidx, uint8_t *cur, uint8_t *org)
{
    amvenc_curr_pred_mode_t *video = encvid->mb_node;
    AMLNeighborAvailability availability;
    amvenc_pred_mode_t *common = encvid->mb_list;
    //video->currMB = common->mblock + video->mbNum;
    AMLMacroblock *currMB = video->currMB;
    int pitch = common->pitch;
    uint8_t mode_avail[AMLNumI4PredMode];
    //uint8 mode_avail[AMLNumI4PredMode];
    uint32_t temp, DC;
    uint8_t *pred;
    int org_pitch = common->pitch;
    uint16_t min_cost, cost;
    //amvenc_neighbor_t *n = &video->neighbor;
    uint8_t P_A, P_B, P_C, P_D, P_I, P_J, P_K, P_L;
    uint32_t temp1, temp2;

    int ipmode, mostProbableMode;
    int fixedcost = 4 * encvid->lambda_mode;
    int min_sad = 0x7FFF;

    availability.left = true;
    availability.top = true;
    if (blkidx <= 3) /* top row block  (!block_y) */
    { /* check availability up */
        availability.top = video->neighbor.mbAvailB ;
    }
    if (!(blkidx&0x3)) /* left column block (!block_x)*/
    { /* check availability left */
        availability.left = video->neighbor.mbAvailA ;
    }

    if (availability.top == true)
    {
        temp = *(uint32_t*)(cur - pitch);
        P_A = temp & 0xFF;
        P_B = (temp >> 8) & 0xFF;
        P_C = (temp >> 16) & 0xFF;
        P_D = (temp >> 24) & 0xFF;
    }
    else
    {
        P_A = P_B = P_C = P_D = 128;
    }

    if (availability.left == true)
    {
        cur--;
        P_I = *cur;
        P_J = *(cur += pitch);
        P_K = *(cur += pitch);
        P_L = *(cur + pitch);
        cur -= (pitch << 1);
        cur++;
    }
    else
    {
        P_I = P_J = P_K = P_L = 128;
    }

    //===== INTRA PREDICTION FOR 4x4 BLOCK =====
    /* vertical */
    mode_avail[AML_I4_Vertical] = 0;
    if (availability.top)
    {
        mode_avail[AML_I4_Vertical] = 1;
        pred = video->pred_i4[AML_I4_Vertical];

        temp = (P_D << 24) | (P_C << 16) | (P_B << 8) | P_A ;
        *((uint32_t*)pred) =  temp; /* write 4 at a time */
        *((uint32_t*)(pred += 4)) =  temp;
        *((uint32_t*)(pred += 4)) =  temp;
        *((uint32_t*)(pred += 4)) =  temp;
    }
    /* horizontal */
    mode_avail[AML_I4_Horizontal] = 0;
    if (availability.left)
    {
        mode_avail[AML_I4_Horizontal] = 1;
        pred = video->pred_i4[AML_I4_Horizontal];

        temp = P_I | (P_I << 8);
        temp = temp | (temp << 16);
        *((uint32_t*)pred) = temp;
        temp = P_J | (P_J << 8);
        temp = temp | (temp << 16);
        *((uint32_t*)(pred += 4)) = temp;
        temp = P_K | (P_K << 8);
        temp = temp | (temp << 16);
        *((uint32_t*)(pred += 4)) = temp;
        temp = P_L | (P_L << 8);
        temp = temp | (temp << 16);
        *((uint32_t*)(pred += 4)) = temp;

    }
    /* DC */
    mode_avail[AML_I4_DC] = 1;
    pred = video->pred_i4[AML_I4_DC];
    if (availability.left)
    {
        DC = P_I + P_J + P_K + P_L;

        if (availability.top)
        {
            DC = (P_A + P_B + P_C + P_D + DC + 4) >> 3;
        }
        else
        {
            DC = (DC + 2) >> 2;

        }
    }
    else if (availability.top)
    {
        DC = (P_A + P_B + P_C + P_D + 2) >> 2;

    }
    else
    {
        DC = 128;
    }

    temp = DC | (DC << 8);
    temp = temp | (temp << 16);
    *((uint32_t*)pred) = temp;
    *((uint32_t*)(pred += 4)) = temp;
    *((uint32_t*)(pred += 4)) = temp;
    *((uint32_t*)(pred += 4)) = temp;

#if 0
    mode_avail[AML_I4_Horizontal_Up] = 0;
    /* Down-left */
    mode_avail[AML_I4_Diagonal_Down_Left] = 0;
    /* Down Right */
    mode_avail[AML_I4_Diagonal_Down_Right] = 0;
    /* Diagonal Vertical Right */
    mode_avail[AML_I4_Vertical_Right] = 0;
    /* Horizontal Down */
    mode_avail[AML_I4_Horizontal_Down] = 0;
    /* vertical left */
    mode_avail[AML_I4_Vertical_Left] = 0;
    //===== LOOP OVER ALL 4x4 INTRA PREDICTION MODES =====
    // can re-order the search here instead of going in order
#endif

    // find most probable mode
    video->mostProbableI4Mode[blkidx] = mostProbableMode = FindMostProbableI4Mode(encvid, blkidx);

    min_cost = 0xFFFF;

    for (ipmode = 0; ipmode < AMLNumI4PredMode; ipmode++)
    {
        if (mode_avail[ipmode] == true)
        {
            cost  = (ipmode == mostProbableMode) ? 0 : fixedcost;
            pred = video->pred_i4[ipmode];

            pred_cost_i4(org, org_pitch, pred, &cost);

            if (cost < min_cost)
            {
                currMB->i4Mode[blkidx] = (AMLIntra4x4PredMode)ipmode;
                min_cost   = cost;
                min_sad = cost - ((ipmode == mostProbableMode) ? 0 : fixedcost);
            }
        }
    }


    return min_cost;
}

int FindMostProbableI4Mode( amvenc_pred_mode_obj_t *encvid, int blkidx)
{
    int dcOnlyPredictionFlag;
    amvenc_curr_pred_mode_t *video = encvid->mb_node;
    AMLMacroblock *currMB = video->currMB;
    amvenc_pred_mode_t *common = encvid->mb_list;
    int intra4x4PredModeA = AML_I4_DC;
    int intra4x4PredModeB = AML_I4_DC;
    int predIntra4x4PredMode = AML_I4_DC;

    dcOnlyPredictionFlag = 0;
    if (blkidx&0x3)
    {
        intra4x4PredModeA = currMB->i4Mode[blkidx-1]; // block to the left
    }
    else /* for blk 0, 4, 8, 12 */
    {
        if (video->neighbor.mbAvailA)
        {
#if 0
            if (video->mblock[video->mbAddrA].mbMode == AML_I4)
            {
                intra4x4PredModeA = video->mblock[video->mbAddrA].i4Mode[blkidx + 3];
            }
            else
            {
                intra4x4PredModeA = AML_I4_DC;
            }
#else
            intra4x4PredModeA = common->mblock[video->mbAddrA].i4Mode[blkidx + 3];
#endif
        }
        else
        {
            dcOnlyPredictionFlag = 1;
            goto PRED_RESULT_READY;  // skip below
        }
    }

    if (blkidx >> 2)
    {
        intra4x4PredModeB = currMB->i4Mode[blkidx-4]; // block above
    }
    else /* block 0, 1, 2, 3 */
    {
        if (video->neighbor.mbAvailB)
        {
#if 0
            if (video->mblock[video->mbAddrB].mbMode == AML_I4)
            {
                intra4x4PredModeB = video->mblock[video->mbAddrB].i4Mode[blkidx+12];
            }
            else
            {
                intra4x4PredModeB = AML_I4_DC;
            }
#else
            intra4x4PredModeA = common->mblock[video->mbAddrB].i4Mode[blkidx + 12];
#endif
        }
        else
        {
            dcOnlyPredictionFlag = 1;
        }
    }

PRED_RESULT_READY:
    if (dcOnlyPredictionFlag)
    {
        intra4x4PredModeA = intra4x4PredModeB = AML_I4_DC;
    }

    predIntra4x4PredMode = AML_MIN(intra4x4PredModeA, intra4x4PredModeB);

    return predIntra4x4PredMode;
}
