#ifndef AMLOGIC_MT_RATECONTROL_
#define AMLOGIC_MT_RATECONTROL_

#include "mtvenclib.h"

typedef struct RDInfo_s
{
    int QP;
    int actual_bits;
    float mad;
    float R_D;
} RDInfo;

typedef struct MultiPass_s
{
    /* multipass rate control data */
    int target_bits;    /* target bits for current frame, = rc->T */
    int actual_bits;    /* actual bits for current frame obtained after encoding, = rc->Rc*/
    int QP;             /* quantization level for current frame, = rc->Qc*/
    float mad;          /* mad for current frame, = video->avgMAD*/
    int bitrate;        /* bitrate for current frame */
    float framerate;    /* framerate for current frame*/

    int encoded_frames;     /* counter for all encoded frames */

    /* Multiple frame prediction*/
    RDInfo pRDSamples[30];        /* pRDSamples[30], 30->30fps*/
    int framePos;               /* specific position in previous multiple frames*/
    int frameRange;             /* number of overall previous multiple frames */

    /* Bit allocation for scene change frames and high motion frames */
    float sum_mad;
    int counter_BTsrc;  /* BT = Bit Transfer, bit transfer from low motion frames or less complicatedly compressed frames */
    int counter_BTdst;  /* BT = Bit Transfer, bit transfer to scene change frames or high motion frames or more complicatedly compressed frames */
    float sum_QP;
    int diff_counter;   /* diff_counter = -diff_counter_BTdst, or diff_counter_BTsrc */

    /* For target bitrate or framerate update */
    float target_bits_per_frame;        /* = C = bitrate/framerate */
    float aver_mad;                     /* so-far average mad could replace sum_mad */
} MultiPass;

typedef struct MTVencRateControl_s
{
    bool rcEnable;  /* enable rate control, '1' on, '0' const QP */
    int initQP; /* initial QP */
    int bitRate;  /* target bit rate for the overall clip in bits/second*/
    float frame_rate; /* frame rate */
    int skip_next_frame;

    int32 cpbSize;  /* coded picture buffer size in bytes */

    /* this part comes from MPEG4 rate control */
    int Rc;     /*bits used for the current frame. It is the bit count obtained after encoding. */

    /*If the macroblock is intra coded, the original spatial pixel values are summed.*/
    int Qc;     /*quantization level used for the current frame. */
    int T;      /*target bit to be used for the current frame.*/
    int Bs; /*buffer size e.g., R/2 */

    int numFrameBits; /* keep track of number of bits of the current frame */
    int bitsPerFrame;

    /* BX rate control, something like TMN8 rate control*/

    MultiPass *pMP;

    int     TMN_W;
    int     TMN_TH;
    int     VBV_fullness;
    int     max_BitVariance_num; /* the number of the maximum bit variance within the given buffer with the unit of 10% of bitrate/framerate*/
    int     encoded_frames; /* counter for all encoded frames */
    int     low_bound;              /* bound for underflow detection, usually low_bound=-Bs/2, but could be changed in H.263 mode */
    int     VBV_fullness_offset;    /* offset of VBV_fullness, usually is zero, but can be changed in H.263 mode*/
    /* End BX */
} MTVencRateControl;

extern AMVEnc_Status MTRCUpdateFrame(void *dev, void *rc, bool IDR, int* skip_num, int numFrameBits);
extern AMVEnc_Status MTRCInitFrameQP(void *dev, void *rc, bool IDR, int bitrate, float frame_rate);
extern AMVEnc_Status MTRCUpdateBuffer(void *dev, void *rc, int frameInc, bool force_IDR);
extern void MTCleanupRateControlModule(void *rc);
extern void* MTInitRateControlModule(bool rcEnable, int initQP, float frame_rate, int bitRate, int cpbSize);

#endif