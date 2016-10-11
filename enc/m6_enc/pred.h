#ifndef __AMLENC_PRED_H__
#define __AMLENC_PRED_H__
#include "mtvenclib.h"

typedef struct tagNeighborAvail
{
    int left;
    int top;    /* macroblock address of the current pixel, see below */
    int top_right;      /* x,y positions of current pixel relative to the macroblock mb_addr */
} AMLNeighborAvailability;

amvenc_curr_pred_mode_t *InitcurrMBStruct();
int ClearcurrMBStruct(void *p);

void MBIntraSearch_iframe( amvenc_pred_mode_obj_t * encvid,
                        unsigned char* input_addr );
void mb_intra4x4_search(amvenc_pred_mode_obj_t * encvid, int *min_cost);
int blk_intra4x4_search_iframe(amvenc_pred_mode_obj_t *encvid, int blkidx,
        uint8_t *cur, uint8_t *org);
int FindMostProbableI4Mode( amvenc_pred_mode_obj_t *encvid, int blkidx);
void find_cost_16x16( amvenc_pred_mode_obj_t * encvid,
        /*AVCEncObject *encvid,*/ uint8_t *orgY, int *min_cost);

/**
This function calculates the cost of a given I4 prediction mode.
\param "org"    "Pointer to the original block."
\param "org_pitch"  "Stride size of the original frame."
\param "pred"   "Pointer to the prediction block. (encvid->pred_i4)"
\param "cost"   "Pointer to the minimal cost (to be updated)."
\return "void"
*/
extern "C" void pred_cost_i4(uint8_t *org, int org_pitch, uint8_t *pred, uint16_t *cost);
extern "C" int pred_cost_i16(uint8_t *org, int org_pitch, uint8_t *pred, int min_cost);
//intra mode

#endif
