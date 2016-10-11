#ifndef __INTRA_SEARCH__
#define __INTRA_SEARCH__
#include <stdlib.h>
#include "mtvenclib.h"
#include "../intra_search/pred.h"

/******************************************************
 *
 * call before every frame
 * return an pointer to an struct used for MB intra search;
 *
******************************************************/
amvenc_pred_mode_obj_t *MBIntraSearch_prepare(amvenc_drv_t* p)
{
        amvenc_pred_mode_obj_t *encvid = NULL;
        amvenc_motionsearch_t   *motion_search = (amvenc_motionsearch_t*)(&p->motion_search);

        encvid = (amvenc_pred_mode_obj_t *) calloc(1, sizeof(amvenc_pred_mode_obj_t));
        if (encvid) {
                encvid->mb_list = &p->intra_mode;
                encvid->mb_node = InitcurrMBStruct();
                encvid->YCbCr[0]=p->src.pic[1].plane[0];
                encvid->YCbCr[1]=p->src.pic[1].plane[1];
                encvid->YCbCr[2]=p->src.pic[1].plane[2];
        }

        /*this may change*/
        encvid->lambda_mode     = motion_search->lambda_mode;
        encvid->lambda_motion   =motion_search->lambda_motion;


        return encvid;
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

#endif
