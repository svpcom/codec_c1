#define LOG_TAG "MTVENC_RC"
#include <utils/Log.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>

#include "mtvenclib.h"
#include "enc_define.h"
#include "rate_control.h"

#define RC_MAX_QUANT 40
#define RC_MIN_QUANT 0   //cap to 10 to prevent rate fluctuation    

#define MAD_MIN 1 /* handle the case of devision by zero in RC */

static double QP2Qstep(int QP)
{
    int i;
    double Qstep;
    static const double QP2QSTEP[6] = { 0.625, 0.6875, 0.8125, 0.875, 1.0, 1.125 };

    Qstep = QP2QSTEP[QP % 6];
    for (i = 0; i < (QP / 6); i++)
        Qstep *= 2;

    return Qstep;
}

/* convert from step size to QP */
static int Qstep2QP(double Qstep)
{
    int q_per = 0, q_rem = 0;

    if (Qstep < QP2Qstep(0))
        return 0;
    else if (Qstep > QP2Qstep(51))
        return 51;

    while (Qstep > QP2Qstep(5)){
        Qstep /= 2;
        q_per += 1;
    }

    if (Qstep <= (0.625 + 0.6875) / 2)
    {
        Qstep = 0.625;
        q_rem = 0;
    }
    else if (Qstep <= (0.6875 + 0.8125) / 2)
    {
        Qstep = 0.6875;
        q_rem = 1;
    }
    else if (Qstep <= (0.8125 + 0.875) / 2)
    {
        Qstep = 0.8125;
        q_rem = 2;
    }
    else if (Qstep <= (0.875 + 1.0) / 2)
    {
        Qstep = 0.875;
        q_rem = 3;
    }
    else if (Qstep <= (1.0 + 1.125) / 2)
    {
        Qstep = 1.0;
        q_rem = 4;
    }
    else
    {
        Qstep = 1.125;
        q_rem = 5;
    }

    return (q_per * 6 + q_rem);
}

static void updateRC_PostProc(MTVencRateControl *rateCtrl, MultiPass *pMP)
{
    if (rateCtrl->skip_next_frame > 0) /* skip next frame */
    {
        pMP->counter_BTsrc += 10 * rateCtrl->skip_next_frame;
    }
    else if (rateCtrl->skip_next_frame == -1) /* skip current frame */
    {
        pMP->counter_BTdst -= pMP->diff_counter;
        pMP->counter_BTsrc += 10;

        pMP->sum_mad -= pMP->mad;
        pMP->aver_mad = (pMP->aver_mad * pMP->encoded_frames - pMP->mad) / (pMP->encoded_frames - 1 + 0.0001);
        pMP->sum_QP  -= pMP->QP;
        pMP->encoded_frames --;
    }

    if (rateCtrl->VBV_fullness < rateCtrl->low_bound)
    {
        rateCtrl->VBV_fullness = rateCtrl->low_bound; // -rateCtrl->Bs/2;
        rateCtrl->TMN_W = rateCtrl->VBV_fullness - rateCtrl->low_bound;
        pMP->counter_BTsrc = pMP->counter_BTdst + (int)((float)(rateCtrl->Bs / 2 - rateCtrl->low_bound) / 2.0 / (pMP->target_bits_per_frame / 10));
    }
}

static void targetBitCalculation(amvenc_drv_t *p, MTVencRateControl *rateCtrl, MultiPass *pMP)
{
    float curr_mad;//, average_mad;
    int diff_counter_BTsrc, diff_counter_BTdst, prev_counter_diff, curr_counter_diff, bound;

    /* some stuff about frame dropping remained here to be done because pMP cannot be inserted into updateRateControl()*/
    updateRC_PostProc(rateCtrl, pMP);

    /* update pMP->counter_BTsrc and pMP->counter_BTdst to avoid interger overflow */
    if (pMP->counter_BTsrc > 1000 && pMP->counter_BTdst > 1000)
    {
        pMP->counter_BTsrc -= 1000;
        pMP->counter_BTdst -= 1000;
    }

    /* ---------------------------------------------------------------------------------------------------*/
    /* target calculation */
    curr_mad = (float)p->motion_search.totalSAD/p->src.mbsize;
    if (curr_mad < MAD_MIN) 
        curr_mad = MAD_MIN; /* MAD_MIN is defined as 1 in mp4def.h */
    diff_counter_BTsrc = diff_counter_BTdst = 0;
    pMP->diff_counter = 0;

    /*1.calculate average mad */
    pMP->sum_mad += curr_mad;
    if (pMP->encoded_frames >= 0) /* pMP->encoded_frames is set to -1 initially, so forget about the very first I frame */
        pMP->aver_mad = (pMP->aver_mad * pMP->encoded_frames + curr_mad) / (pMP->encoded_frames + 1);

    /* original verison */
    if (curr_mad > pMP->aver_mad*1.1){
        if (curr_mad / (pMP->aver_mad + 0.0001) > 2)
            diff_counter_BTdst = (int)(sqrt(curr_mad / (pMP->aver_mad + 0.0001)) * 10 + 0.4) - 10;
        else
           diff_counter_BTdst = (int)(curr_mad / (pMP->aver_mad + 0.0001) * 10 + 0.4) - 10;
    }else{ /* curr_mad <= average_mad*1.1 */
        diff_counter_BTsrc = 10 - (int)(sqrt(curr_mad / (pMP->aver_mad + 0.0001)) * 10 + 0.5);
    }

    /* actively fill in the possible gap */
    if (diff_counter_BTsrc == 0 && diff_counter_BTdst == 0 && curr_mad <= pMP->aver_mad*1.1 && pMP->counter_BTsrc < pMP->counter_BTdst)
        diff_counter_BTsrc = 1;

    /* if difference is too much, do clipping */
    /* First, set the upper bound for current bit allocation variance: 80% of available buffer */
    bound = (int)((rateCtrl->Bs / 2 - rateCtrl->VBV_fullness) * 0.6 / (pMP->target_bits_per_frame / 10)); /* rateCtrl->Bs */
    diff_counter_BTsrc =  AVC_MIN(diff_counter_BTsrc, bound);
    diff_counter_BTdst =  AVC_MIN(diff_counter_BTdst, bound);

    /* Second, set another upper bound for current bit allocation: 4-5*bitrate/framerate */
    bound = 50;

    diff_counter_BTsrc =  AVC_MIN(diff_counter_BTsrc, bound);
    diff_counter_BTdst =  AVC_MIN(diff_counter_BTdst, bound);


    /* Third, check the buffer */
    prev_counter_diff = pMP->counter_BTdst - pMP->counter_BTsrc;
    curr_counter_diff = prev_counter_diff + (diff_counter_BTdst - diff_counter_BTsrc);

    if (AVC_ABS(prev_counter_diff) >= rateCtrl->max_BitVariance_num || AVC_ABS(curr_counter_diff) >= rateCtrl->max_BitVariance_num)
    {
        if (curr_counter_diff > rateCtrl->max_BitVariance_num && diff_counter_BTdst)
        {
            diff_counter_BTdst = (rateCtrl->max_BitVariance_num - prev_counter_diff) + diff_counter_BTsrc;
            if (diff_counter_BTdst < 0) diff_counter_BTdst = 0;
        }

        else if (curr_counter_diff < -rateCtrl->max_BitVariance_num && diff_counter_BTsrc)
        {
            diff_counter_BTsrc = diff_counter_BTdst - (-rateCtrl->max_BitVariance_num - prev_counter_diff);
            if (diff_counter_BTsrc < 0) diff_counter_BTsrc = 0;
        }
    }

    /*3.diff_counter_BTsrc, diff_counter_BTdst ==> TMN_TH */
    rateCtrl->TMN_TH = (int)(pMP->target_bits_per_frame);
    pMP->diff_counter = 0;

    if (diff_counter_BTsrc)
    {
        rateCtrl->TMN_TH -= (int)(pMP->target_bits_per_frame * diff_counter_BTsrc * 0.1);
        pMP->diff_counter = -diff_counter_BTsrc;
    }
    else if (diff_counter_BTdst)
    {
        rateCtrl->TMN_TH += (int)(pMP->target_bits_per_frame * diff_counter_BTdst * 0.1);
        pMP->diff_counter = diff_counter_BTdst;
    }

    /*4.update pMP->counter_BTsrc, pMP->counter_BTdst */
    pMP->counter_BTsrc += diff_counter_BTsrc;
    pMP->counter_BTdst += diff_counter_BTdst;

    /*5.target bit calculation */
    rateCtrl->T = rateCtrl->TMN_TH - rateCtrl->TMN_W;
    return ;
}

void calculateQuantizer(amvenc_drv_t *p, MTVencRateControl *rateCtrl, MultiPass *pMP)
{
    int prev_actual_bits = 0, curr_target, i, j;
    float Qstep, prev_QP = 0.625;
    int curQP = 0;

    float curr_mad, prev_mad, curr_RD, prev_RD, average_mad, aver_QP;

    /* Mad based variable bit allocation */
    targetBitCalculation(p, rateCtrl, pMP);

    if (rateCtrl->T <= 0 || p->motion_search.totalSAD == 0)
    {
        if (rateCtrl->T < 0){
            rateCtrl->Qc = AVC_MIN(rateCtrl->Qc+1,rateCtrl->initQP);//RC_MAX_QUANT;
        }
        return;
    }
    /* current frame QP estimation */
    curr_target = rateCtrl->T;
    curr_mad = (float)p->motion_search.totalSAD/p->src.mbsize;
    if (curr_mad < MAD_MIN) 
        curr_mad = MAD_MIN; 
    curr_RD  = (float)curr_target / curr_mad;

    if(1)//if (rateCtrl->skip_next_frame == -1) // previous was skipped
    {
        i = pMP->framePos;
        prev_mad = pMP->pRDSamples[i].mad;
        prev_QP = pMP->pRDSamples[i].QP;
        prev_actual_bits = pMP->pRDSamples[i].actual_bits;
    }
#if 0
    else
    {
        /* Another version of search the optimal point */
        prev_mad = 0.0;
        i = 0;
        while (i < pMP->frameRange && prev_mad < 0.001) /* find first one with nonzero prev_mad */
        {
            prev_mad = pMP->pRDSamples[i].mad;
            i++;
        }

        if (i < pMP->frameRange)
        {
            prev_actual_bits = pMP->pRDSamples[i-1].actual_bits;

            for (j = i-1; i < pMP->frameRange; i++)
            {
                if (pMP->pRDSamples[i].mad != 0 &&
                        AVC_ABS(prev_mad - curr_mad) > AVC_ABS(pMP->pRDSamples[i].mad - curr_mad))
                {
                    prev_mad = pMP->pRDSamples[i].mad;
                    prev_actual_bits = pMP->pRDSamples[i].actual_bits;
                    j = i;
                }
            }
            prev_QP = QP2Qstep(pMP->pRDSamples[j].QP);
        }
    }
#endif

    // quadratic approximation
    if (prev_mad > 0.001) // only when prev_mad is greater than 0, otherwise keep using the same QP
    {
        prev_RD = (float)prev_actual_bits / prev_mad;
        if (prev_QP == 0.625) // added this to allow getting out of QP = 0 easily
        {
            Qstep = (int)(prev_RD / curr_RD + 0.5);
        }
        else
        {
            if (prev_RD / curr_RD > 0.5 && prev_RD / curr_RD < 2.0)
                Qstep = (int)(prev_QP * (sqrt(prev_RD / curr_RD) + prev_RD / curr_RD) / 2.0 + 0.9); /* Quadratic and linear approximation */
            else
                Qstep = (int)(prev_QP * (sqrt(prev_RD / curr_RD) + pow(prev_RD / curr_RD, 1.0 / 3.0)) / 2.0 + 0.9);
        }
        curQP = Qstep2QP(Qstep)+1;
        rateCtrl->Qc = AVC_MEDIAN(rateCtrl->Qc-2,curQP,rateCtrl->Qc+2);
        
        if (rateCtrl->Qc < RC_MIN_QUANT) rateCtrl->Qc = RC_MIN_QUANT;
        if (rateCtrl->Qc > RC_MAX_QUANT)    rateCtrl->Qc = RC_MAX_QUANT;
    }

    /* active bit resource protection */
    aver_QP = (pMP->encoded_frames == 0 ? 0 : pMP->sum_QP / (float)pMP->encoded_frames);
    average_mad = (pMP->encoded_frames == 0 ? 0 : pMP->sum_mad / (float)pMP->encoded_frames); /* this function is called from the scond encoded frame*/
    if (pMP->diff_counter == 0 &&
            ((float)rateCtrl->Qc <= aver_QP*1.1 || curr_mad <= average_mad*1.1) &&
            pMP->counter_BTsrc <= (pMP->counter_BTdst + (int)(pMP->framerate*1.0 + 0.5)))
    {
        rateCtrl->TMN_TH -= (int)(pMP->target_bits_per_frame / 10.0);
        rateCtrl->T = rateCtrl->TMN_TH - rateCtrl->TMN_W;
        pMP->counter_BTsrc++;
        pMP->diff_counter--;
    }

    return;
}

static void AVCSaveRDSamples(MultiPass *pMP)
{
    /* for pMP->pRDSamples */
    pMP->pRDSamples[pMP->framePos].QP    = pMP->QP;
    pMP->pRDSamples[pMP->framePos].actual_bits = pMP->actual_bits;
    pMP->pRDSamples[pMP->framePos].mad   = pMP->mad;
    pMP->pRDSamples[pMP->framePos].R_D = (float)pMP->actual_bits / (pMP->mad + 0.0001);
    return ;
}

static void updateRateControl(MTVencRateControl *rateCtrl, bool IDR)
{
    int  frame_bits;
    MultiPass *pMP = rateCtrl->pMP;

    /* BX rate contro\l */
    frame_bits = (int)(rateCtrl->bitRate / rateCtrl->frame_rate);
    rateCtrl->TMN_W += (rateCtrl->Rc - rateCtrl->TMN_TH);
    rateCtrl->VBV_fullness += (rateCtrl->Rc - frame_bits); //rateCtrl->Rp);

    rateCtrl->encoded_frames++;

    /* frame dropping */
    rateCtrl->skip_next_frame = 0;

    if ((rateCtrl->VBV_fullness > rateCtrl->Bs / 2) && IDR != true) /* skip the current frame */ /* rateCtrl->Bs */
    {
        rateCtrl->TMN_W -= (rateCtrl->Rc - rateCtrl->TMN_TH);
        rateCtrl->VBV_fullness -= rateCtrl->Rc;
        rateCtrl->skip_next_frame = -1;
    }
    else if ((float)(rateCtrl->VBV_fullness - rateCtrl->VBV_fullness_offset) > (rateCtrl->Bs / 2 - rateCtrl->VBV_fullness_offset)*0.95) /* skip next frame */
    {
        rateCtrl->VBV_fullness -= frame_bits;
        rateCtrl->skip_next_frame = 1;
        pMP->counter_BTsrc -= (int)((float)(rateCtrl->Bs / 2 - rateCtrl->low_bound) / 2.0 / (pMP->target_bits_per_frame / 10));
        /* BX_1, skip more than 1 frames  */
        while ((rateCtrl->VBV_fullness - rateCtrl->VBV_fullness_offset) > (rateCtrl->Bs / 2 - rateCtrl->VBV_fullness_offset)*0.95)
        {
            rateCtrl->VBV_fullness -= frame_bits;
            rateCtrl->skip_next_frame++;
            pMP->counter_BTsrc -= (int)((float)(rateCtrl->Bs / 2 - rateCtrl->low_bound) / 2.0 / (pMP->target_bits_per_frame / 10));
        }
        /* END BX_1 */
    }
}

AMVEnc_Status MTRCUpdateFrame(void *dev, void *rc, bool IDR, int* skip_num, int numFrameBits)
{
    amvenc_drv_t *p = (amvenc_drv_t *)dev;
    MTVencRateControl *rateCtrl = (MTVencRateControl *)rc;
    AMVEnc_Status status = AMVENC_SUCCESS;
    MultiPass *pMP = rateCtrl->pMP;
    int diff_BTCounter;

    *skip_num = 0;

    if (rateCtrl->rcEnable == true)
    {
        pMP->actual_bits = numFrameBits;
        pMP->mad = (float)p->motion_search.totalSAD/p->src.mbsize;

        AVCSaveRDSamples(pMP);

        pMP->encoded_frames++;

        pMP->sum_QP += pMP->QP;

        /* update pMP->counter_BTsrc, pMP->counter_BTdst */
        /* re-allocate the target bit again and then stop encoding */
        diff_BTCounter = (int)((float)(rateCtrl->TMN_TH - rateCtrl->TMN_W - pMP->actual_bits) /
                               (pMP->bitrate / (pMP->framerate + 0.0001) + 0.0001) / 0.1);
        if (diff_BTCounter >= 0)
            pMP->counter_BTsrc += diff_BTCounter; /* pMP->actual_bits is smaller */
        else
            pMP->counter_BTdst -= diff_BTCounter; /* pMP->actual_bits is bigger */

        rateCtrl->TMN_TH -= (int)((float)pMP->bitrate / (pMP->framerate + 0.0001) * (diff_BTCounter * 0.1));
        rateCtrl->T = pMP->target_bits = rateCtrl->TMN_TH - rateCtrl->TMN_W;
        pMP->diff_counter -= diff_BTCounter;

        rateCtrl->Rc = numFrameBits;  /* Total Bits for current frame */

        /* BX_RC */
        updateRateControl(rateCtrl, IDR);
        if (rateCtrl->skip_next_frame == -1) // skip current frame
        {
            status = AMVENC_SKIPPED_PICTURE;
        }
        *skip_num = rateCtrl->skip_next_frame;
    }
    return status;
}

AMVEnc_Status MTRCInitFrameQP(void *dev, void *rc, bool IDR, int bitrate, float frame_rate)
{
    amvenc_drv_t *p = (amvenc_drv_t *)dev;
    MTVencRateControl *rateCtrl = (MTVencRateControl *)rc;
    MultiPass *pMP = rateCtrl->pMP;

    if (rateCtrl->rcEnable == true)
    {
        /* frame layer rate control */
        if (rateCtrl->encoded_frames == 0)
        {
            p->quant= rateCtrl->Qc = rateCtrl->initQP;
        }
        else
        {
            calculateQuantizer(p, rateCtrl, pMP);
            p->quant = rateCtrl->Qc;
        }

        /* update pMP->framePos */
        if (++pMP->framePos == pMP->frameRange) 
            pMP->framePos = 0;

        if (rateCtrl->T == 0)
        {
            pMP->counter_BTdst = (int)(rateCtrl->frame_rate * 7.5 + 0.5); /* 0.75s time frame */
            pMP->counter_BTdst = AVC_MIN(pMP->counter_BTdst, (int)(rateCtrl->max_BitVariance_num / 2 * 0.40)); /* 0.75s time frame may go beyond VBV buffer if we set the buffer size smaller than 0.75s */
            pMP->counter_BTdst = AVC_MAX(pMP->counter_BTdst, (int)((rateCtrl->Bs / 2 - rateCtrl->VBV_fullness) * 0.30 / (rateCtrl->TMN_TH / 10.0) + 0.5)); /* At least 30% of VBV buffer size/2 */
            pMP->counter_BTdst = AVC_MIN(pMP->counter_BTdst, 20); /* Limit the target to be smaller than 3C */

            pMP->target_bits = rateCtrl->T = rateCtrl->TMN_TH = (int)(rateCtrl->TMN_TH * (1.0 + pMP->counter_BTdst * 0.1));
            pMP->diff_counter = pMP->counter_BTdst;
        }

        /* collect the necessary data: target bits, actual bits, mad and QP */
        pMP->target_bits = rateCtrl->T;
        pMP->QP  = p->quant;

        pMP->mad = (float)p->motion_search.totalSAD/p->src.mbsize;
        if (pMP->mad < MAD_MIN) 
            pMP->mad = MAD_MIN; /* MAD_MIN is defined as 1 in mp4def.h */

        pMP->bitrate = rateCtrl->bitRate; /* calculated in RCVopQPSetting */
        pMP->framerate = rateCtrl->frame_rate;
    } // rcEnable
    else
    {
        p->quant = rateCtrl->initQP;
    }
    return AMVENC_SUCCESS;
}

AMVEnc_Status MTRCUpdateBuffer(void *dev, void *rc, int frameInc, bool force_IDR)
{
    amvenc_drv_t *p = (amvenc_drv_t *)dev;
    MTVencRateControl *rateCtrl = (MTVencRateControl *)rc;
    int tmp;
    MultiPass *pMP = rateCtrl->pMP;

    if (rateCtrl->rcEnable == true)
    {
        if (frameInc > 1)
        {
            tmp = rateCtrl->bitsPerFrame * (frameInc - 1);
            rateCtrl->VBV_fullness -= tmp;
            pMP->counter_BTsrc += 10 * (frameInc - 1);

            /* Check buffer underflow */
            if (rateCtrl->VBV_fullness < rateCtrl->low_bound)
            {
                rateCtrl->VBV_fullness = rateCtrl->low_bound; 
                rateCtrl->TMN_W = rateCtrl->VBV_fullness - rateCtrl->low_bound;
                pMP->counter_BTsrc = pMP->counter_BTdst + (int)((float)(rateCtrl->Bs / 2 - rateCtrl->low_bound) / 2.0 / (pMP->target_bits_per_frame / 10));
            }
        }
    }
    return AMVENC_SUCCESS;
}

void MTCleanupRateControlModule(void *rc)
{
    MTVencRateControl *rateCtrl = (MTVencRateControl *)rc;
    if (rateCtrl->pMP)
        free(rateCtrl->pMP);
    rateCtrl->pMP = NULL;
    free(rateCtrl);
    return;
}

void* MTInitRateControlModule(bool rcEnable, int initQP, float frame_rate, int bitRate, int cpbSize)
{
    MTVencRateControl *rateCtrl = (MTVencRateControl*)calloc(1, sizeof(MTVencRateControl));

    if(!rateCtrl)
        return NULL;

    memset(rateCtrl,0,sizeof(MTVencRateControl));

    rateCtrl->rcEnable = rcEnable;
    rateCtrl->initQP = initQP;
    rateCtrl->frame_rate = frame_rate;
    rateCtrl->bitRate = bitRate;
    rateCtrl->cpbSize = cpbSize;
   
    if (rateCtrl->rcEnable == true)
    {
        rateCtrl->pMP = (MultiPass*)calloc(1, sizeof(MultiPass));
        if (!rateCtrl->pMP)
        {
            goto CLEANUP_RC;
        }
        memset(rateCtrl->pMP, 0, sizeof(MultiPass));
        rateCtrl->pMP->encoded_frames = -1; /* forget about the very first I frame */

        rateCtrl->pMP->frameRange = (int)(rateCtrl->frame_rate * 1.0); /* 1.0s time frame*/
        rateCtrl->pMP->frameRange = AVC_MAX(rateCtrl->pMP->frameRange, 5);
        rateCtrl->pMP->frameRange = AVC_MIN(rateCtrl->pMP->frameRange, 15);

        rateCtrl->pMP->framePos = -1;


        rateCtrl->bitsPerFrame = (int32)(rateCtrl->bitRate / rateCtrl->frame_rate);

        /* BX rate control */
        rateCtrl->skip_next_frame = 0; /* must be initialized */

        rateCtrl->Bs = rateCtrl->cpbSize;
        rateCtrl->TMN_W = 0;
        rateCtrl->VBV_fullness = (int)(rateCtrl->Bs * 0.5); /* rateCtrl->Bs */
        rateCtrl->encoded_frames = 0;

        rateCtrl->TMN_TH = rateCtrl->bitsPerFrame;

        rateCtrl->max_BitVariance_num = (int)((float)(rateCtrl->Bs - rateCtrl->VBV_fullness) / (rateCtrl->bitsPerFrame / 10.0)) - 5;
        if (rateCtrl->max_BitVariance_num < 0) 
            rateCtrl->max_BitVariance_num += 5;

        // Set the initial buffer fullness
        /* According to the spec, the initial buffer fullness needs to be set to 1/3 */
        rateCtrl->VBV_fullness = (int)(rateCtrl->Bs / 3.0 - rateCtrl->Bs / 2.0); /* the buffer range is [-Bs/2, Bs/2] */
        rateCtrl->pMP->counter_BTsrc = (int)((rateCtrl->Bs / 2.0 - rateCtrl->Bs / 3.0) / (rateCtrl->bitsPerFrame / 10.0));
        rateCtrl->TMN_W = (int)(rateCtrl->VBV_fullness + rateCtrl->pMP->counter_BTsrc * (rateCtrl->bitsPerFrame / 10.0));

        rateCtrl->low_bound = -rateCtrl->Bs / 2;
        rateCtrl->VBV_fullness_offset = 0;

        /* Setting the bitrate and framerate */
        rateCtrl->pMP->bitrate = rateCtrl->bitRate;
        rateCtrl->pMP->framerate = rateCtrl->frame_rate;
        rateCtrl->pMP->target_bits_per_frame = rateCtrl->pMP->bitrate / rateCtrl->pMP->framerate;

        rateCtrl->Qc = rateCtrl->initQP;
    }

    return (void*)rateCtrl;

CLEANUP_RC:
    MTCleanupRateControlModule((void*)rateCtrl);
    return NULL;
}

