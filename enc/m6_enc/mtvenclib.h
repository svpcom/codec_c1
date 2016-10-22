#ifndef AML_VIDEO_ENCODER_MT_
#define AML_VIDEO_ENCODER_MT_

#include <utils/threads.h>
#include <semaphore.h>  
#include "enc_define.h"
#include "../intra_search/pred.h"

#define MTVENC_AVC_IOC_MAGIC  'E'

#define MTVENC_AVC_IOC_GET_ADDR			_IOW(MTVENC_AVC_IOC_MAGIC, 0x00, unsigned int)
#define MTVENC_AVC_IOC_INPUT_UPDATE		_IOW(MTVENC_AVC_IOC_MAGIC, 0x01, unsigned int)
#define MTVENC_AVC_IOC_GET_STATUS		_IOW(MTVENC_AVC_IOC_MAGIC, 0x02, unsigned int)
#define MTVENC_AVC_IOC_NEW_CMD			_IOW(MTVENC_AVC_IOC_MAGIC, 0x03, unsigned int)
#define MTVENC_AVC_IOC_GET_STAGE		_IOW(MTVENC_AVC_IOC_MAGIC, 0x04, unsigned int)
#define MTVENC_AVC_IOC_GET_OUTPUT_SIZE	_IOW(MTVENC_AVC_IOC_MAGIC, 0x05, unsigned int)
#define MTVENC_AVC_IOC_SET_QUANT 		_IOW(MTVENC_AVC_IOC_MAGIC, 0x06, unsigned int)
#define MTVENC_AVC_IOC_SET_ENCODER_WIDTH 		_IOW(MTVENC_AVC_IOC_MAGIC, 0x07, unsigned int)
#define MTVENC_AVC_IOC_SET_ENCODER_HEIGHT 		_IOW(MTVENC_AVC_IOC_MAGIC, 0x08, unsigned int)
#define MTVENC_AVC_IOC_CONFIG_INIT 				_IOW(MTVENC_AVC_IOC_MAGIC, 0x09, unsigned int)
#define MTVENC_AVC_IOC_FLUSH_CACHE 				_IOW(MTVENC_AVC_IOC_MAGIC, 0x0a, unsigned int)
#define MTVENC_AVC_IOC_FLUSH_DMA 				_IOW(MTVENC_AVC_IOC_MAGIC, 0x0b, unsigned int)
#define MTVENC_AVC_IOC_GET_BUFFINFO 				_IOW(MTVENC_AVC_IOC_MAGIC, 0x0c, unsigned int)

#define SAD_MAX   65536

#define RANGE 80 // -32 ~ 32

enum 
{ 
    P_8x8_UL, P_8x8_UR, P_8x8_DL, 
    P_8x8_DR, P_8x16_L, P_8x16_R,
    P_16x8_U, P_16x8_D, P_16x16_
};

enum
{
    P_8x8 = 1,
    P_16x8,
    P_8x16,
    P_16x16
}; 

typedef enum
{ 
    Slot_mode_idle = 0,
    Slot_mode_me,
    Slot_mode_I_fill,
    Slot_mode_P_fill
}SlotMode;

typedef struct 
{
    int frame; 
    uint8_t *plane[3];
}picture_t;

typedef struct
{
    int pix_width; 
    int pix_height;
	              
    int mb_width;   
    int mb_height;  
    int mbsize;
    
    bool nv21_data;
    picture_t pic[I_FRAME];
} yuv_t;

typedef struct  
{
    int x;
    int y;
    uint_t sad;
    int mad;
}mvs_t;

typedef struct  
{
    bool intra;
    unsigned mb_id;
    unsigned mb_type;
    int mvL0[16];
}mbinfo_t;

typedef struct{
    bool fullsearch_enable;

    int numIntraMB;
    int mvRange;

    int lambda_mode;
    int lambda_motion; 

    int totalSAD;

    uint8_t *mvbits_array;
    uint8_t *mvbits;
    mvs_t *mot16x16;
    int *min_cost;
    mbinfo_t* mb_array;
    uint8_t *yc_convert;    
}amvenc_motionsearch_t;

typedef struct{
    unsigned char* addr;
    unsigned size;
}amvenc_buff_t;



/**
Types of the macroblock and partition. PV Created.
@publishedAll
*/
    /* intra */

    /* inter for both P and B*/

/**
Mode of intra 4x4 prediction. Table 8-2
@publishedAll
*/

/**
Mode of intra 16x16 prediction. Table 8-3
@publishedAll
*/

/**
This slice type follows Table 7-3. The bottom 5 items may not needed.
@publishedAll
*/


/**
This structure contains macroblock related variables.
@publishedAll
*/








	/********* intra prediction scratch memory **********************/










typedef struct{
    pthread_t mThread;
    sem_t semdone;
    sem_t semstart;

    int start_mbx;
    int start_mby;
    int end_mbx;
    int end_mby;
    int x_step;
    int y_step;
    unsigned update_bytes;
    SlotMode mode;
    int ret;
    bool finish;
    amvenc_pred_mode_obj_t *mbObj;
}amvenc_slot_t;

typedef struct{
    unsigned char* buff;
    unsigned char* y;
    unsigned char* uv;
    unsigned width;
    unsigned height;
    unsigned pitch;
    unsigned size;
}amvenc_reference_t;

typedef struct
{
    int fd;
    bool IDRframe;
    bool mStart;
    bool mCancel;
    yuv_t src;

    unsigned enc_width;
    unsigned enc_height;
    unsigned quant;

    unsigned char ref_id;

    bool gotSPS;
    unsigned sps_len;
    bool gotPPS;
    unsigned pps_len;

    unsigned PrevRefFrameNum;

    unsigned total_encode_frame;
    unsigned total_encode_time;

    amvenc_slot_t slot[2];

    amvenc_buff_t mmap_buff;
    amvenc_buff_t input_buf;
    amvenc_buff_t ref_buf_y[2];
    amvenc_buff_t ref_buf_uv[2];
    amvenc_buff_t output_buf;

    amvenc_reference_t ref_info;
    amvenc_motionsearch_t motion_search;
    amvenc_pred_mode_t intra_mode;
}amvenc_drv_t;

// asm for fill buffer
extern void fill_i_buffer_spec_neon(unsigned char* cur_mb_y  ,unsigned char* cur_mb_u  ,unsigned char* cur_mb_v  ,unsigned short* input_addr);
extern void Y_ext_asm2(uint8_t *ydst, uint8_t *ysrc, uint_t stride);
extern void YV12_UV_ext_asm2(unsigned short *dst, uint8_t *usrc, uint8_t *vsrc, uint_t stride);
extern void NV21_UV_ext_asm2(unsigned short *dst, uint8_t *uvsrc, uint_t stride);
extern void Y_line_asm(uint8_t *ydst, uint8_t *ysrc, uint_t pix_stride, uint_t mb_stride);
extern void nv21_uvline_asm(uint8_t *dst, uint8_t *uvsrc, uint_t pix_stride, uint_t mb_stride);
extern void yv12_uvline_asm(uint8_t *dst, uint8_t *usrc, uint8_t *vsrc, uint_t pix_stride, uint_t mb_stride);
extern void pY_ext_asm(unsigned short *ydst, uint8_t *ysrc, uint8_t *yref, uint_t stride, int offset);
extern void NV21_pUV_ext_asm(unsigned short *dst, uint8_t *uvsrc, uint8_t *uvref, uint_t stride, int ref_x);
extern void YV12_pUV_ext_asm(unsigned short *dst, uint8_t *usrc, uint8_t *vsrc,uint8_t *uvref, uint_t stride, int offset);

//Motion Estimation
extern void UpdateMotionQP(amvenc_drv_t* p);
extern void MBMotionSearch(amvenc_drv_t *p, int i0, int j0, int mbnum, bool FS_en);
extern void CleanMotionSearchMoudle(amvenc_motionsearch_t *motion_search);
extern AMVEnc_Status InitMotionSearchModule(amvenc_drv_t *p);
extern bool MVIntraDecisionABE(amvenc_motionsearch_t *motion_search, int *min_cost, uint8_t *cur, int pitch, bool ave);

extern void* InitMTVEncode(int fd, unsigned enc_width, unsigned enc_height, unsigned quant, unsigned searchrange, bool fs_en);
extern AMVEnc_Status MTVEncodeInitFrame(void *dev, unsigned* yuv, AMVEncBufferType type, AMVEncFrameFmt fmt, bool IDRframe);
extern AMVEnc_Status MTVEncodeSPS_PPS(void *dev, unsigned char* outptr,int* datalen);
extern AMVEnc_Status MTVEncodeSlice(void *dev, unsigned char* outptr,int* datalen);
extern AMVEnc_Status MTVEncodeCommit(void* dev,  bool IDR);
extern void UnInitMTVEncode(void *dev);

extern int InitMBIntraSearchModule( amvenc_drv_t *p);
extern int CleanMBIntraSearchModule( amvenc_drv_t *p);

#endif
