
//#define LOG_NDEBUG 0
#define LOG_TAG "MTVENCLIB"
#include <utils/Log.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h> 
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <utils/threads.h>
#include <semaphore.h>  
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/poll.h>

#include "mtvenclib.h"
#include "intra_search.h"
#include "enc_define.h"

//#define DEBUG_TIME
//#define FAST_INTRA_SEARCH
//#pragma warning( disable : 4996 ) 
#ifdef DEBUG_TIME
static struct timeval start_test, end_test;
#endif

#define ENCODE_DONE_TIMEOUT 200

static int encode_poll(int fd, int timeout)
{    
    struct pollfd poll_fd[1];    
    poll_fd[0].fd = fd;    
    poll_fd[0].events = POLLIN |POLLERR;    
    return poll(poll_fd, 1, timeout);
}

const unsigned char picture_i_head[] = {0x5a ,0xa5,0x55,0xaa,0x00,0x00,0x00,0x00,
								0x13,0x88,0x10,0x80,0x11,0x18,0x12,0x20,
								0x00,0x02,0x09,0xff,0xff,0xff,0xff,0xff};

const unsigned char picture_p_head[] = {0x5a ,0xa5,0x55,0xaa,0x00,0x00,0x00,0x00,
								0x13,0x88,0x10,0x80,0x11,0x18,0x12,0x20,
								0x00,0x02,0x0f,0xff,0xff,0xff,0xff,0xff};							   
									 					   
inline void InitNeighborAvailability(amvenc_drv_t* p, amvenc_neighbor_t* n)
{
    //int mb_num = p->src.mb_width*mb_y+mb_x;
    //int mbAddrA = mb_num - 1;
    //int mbAddrB = mb_num - p->src.mb_width;
    //int mbAddrC = mb_num - p->src.mb_width + 1;
    //int mbAddrD = mb_num - p->src.mb_width - 1;
    n->mbAvailA =0 ;
    n->mbAvailB =0 ;
    n->mbAvailC =0 ; 
    n->mbAvailD =0 ;
    if(n->mbx){
        n->mbAvailA = 1;
    }
    if(n->mby){
        n->mbAvailB = 1;
    }
}
inline void mb_4x4_intra_search(amvenc_neighbor_t* n, unsigned char* input_addr)
{
    uint8_t iMode = HENC_DC_PRED;
    unsigned int value ;
    unsigned int *p = (unsigned int*)input_addr;
    /* evaluate DC mode */
    if((!n->mbAvailA)&&(!n->mbAvailB)){
        iMode = HENC_DC_PRED;
        goto Done;
    }
    /* evaluate vertical  mode */
    if(!n->mbAvailA){
        iMode = HENC_VERT_PRED ;
    }else{	
        /* evaluate horizontal mode */
       iMode = HENC_HOR_PRED ;
       goto Done;
    }

Done:	
    value = iMode|(iMode <<4)|(iMode <<8)|(iMode <<12)|(iMode <<16)|(iMode <<20)|(iMode <<24)|(iMode <<28) ;
    p[2] = value;
    p[3] = value;
}

inline void mb_16x16_intra_search(amvenc_neighbor_t* n, unsigned char* input_addr)
{
    uint8_t i16Mode = HENC_DC_PRED_16;
    /* evaluate DC mode */
    if((!n->mbAvailA)&&(!n->mbAvailB)){
        i16Mode = HENC_DC_PRED_16;
        goto Done;
    }
    /* evaluate vertical  mode */
    if(!n->mbAvailA){
        i16Mode = HENC_VERT_PRED_16 ;
    }else{	
    /* evaluate horizontal mode */
        i16Mode = HENC_HOR_PRED_16 ;
        goto Done;
    }

Done:	
    input_addr[17] = i16Mode ;
}

inline void chroma_intra_search(amvenc_neighbor_t* n, unsigned char* input_addr)
{
    uint8_t c_iMode = HENC_DC_PRED_8;
    /* evaluate DC mode */
    if((!n->mbAvailA)&&(!n->mbAvailB)){
        c_iMode = HENC_DC_PRED_8;
        goto Done;
    }
    /* evaluate vertical   mode */
    if(!n->mbAvailA){
        c_iMode = HENC_VERT_PRED_8   ;
    }else{
        /* evaluate horizontal mode */
        c_iMode = HENC_HOR_PRED_8 ;
    }
Done:	
    input_addr[16] = c_iMode ;
}

inline void fill_i_buffer_spec_default(amvenc_drv_t* para, unsigned char* input_addr, int mb_x, int mb_y)
{
    unsigned char*  p = input_addr;
    unsigned short* q = (unsigned short*)input_addr;
    uint8_t mb_type = HENC_MB_Type_I4MB ;
    amvenc_neighbor_t n;

    n.mbx = mb_x;
    n.mby = mb_y;

    memcpy(p , picture_i_head ,24 );
    p[18] = mb_type;
    p+= 24;
    memset(p,0,72);
	
    InitNeighborAvailability(para,&n);
    mb_4x4_intra_search(&n,input_addr);
    chroma_intra_search(&n,input_addr);	

    q[2] = mb_x;
    q[3] = mb_y;	
    q = (unsigned short*)(input_addr + 96);
    memset(q, 0, 768);
}

inline void fill_i_buffer_spec_nv21(amvenc_drv_t* para, unsigned char* input_addr,int mb_x,int mb_y)
{
    unsigned int stride = para->src.pix_width;
    unsigned char*  p = input_addr;
    unsigned short* q =  (unsigned short*)input_addr;
    uint8_t  *cur_mb_y, *cur_mb_uv;
    unsigned mb_y_offset, mb_uv_offset;
    uint8_t mb_type = HENC_MB_Type_I4MB;
    amvenc_neighbor_t n;

    n.mbx = mb_x;
    n.mby = mb_y;

    memcpy(p , (char*)picture_i_head ,24 );
    p[18] = mb_type;
    p+= 24;
    memset(p,0,72);

    InitNeighborAvailability(para,&n);
    mb_4x4_intra_search(&n,input_addr);
    chroma_intra_search(&n,input_addr);

    q[2] = mb_x;
    q[3] = mb_y;
    q = (unsigned short*)(input_addr + 96);
    mb_y_offset = (mb_x<<4) + (mb_y<<4) * stride ;
    mb_uv_offset = (mb_x<<4) + (mb_y<<3) * stride ;
    cur_mb_y = para->src.pic[1].plane[0] + mb_y_offset;
    cur_mb_uv = para->src.pic[1].plane[1] + mb_uv_offset;

    Y_ext_asm2((unsigned char*)q, cur_mb_y, stride);
    NV21_UV_ext_asm2( q, cur_mb_uv, stride);
}

inline void fill_i_buffer_spec_yv12(amvenc_drv_t* para, unsigned char* input_addr,int mb_x,int mb_y)
{
    unsigned int stride = para->src.pix_width;
    unsigned char*  p = input_addr;
    unsigned short* q =  (unsigned short*)input_addr;
    uint8_t  *cur_mb_y, *cur_mb_u, *cur_mb_v;
    unsigned mb_y_offset, mb_u_offset, mb_v_offset;
    uint8_t mb_type = HENC_MB_Type_I4MB;
    amvenc_neighbor_t n;

    n.mbx = mb_x;
    n.mby = mb_y;

    memcpy(p , (char*)picture_i_head ,24 );
    p[18] = mb_type;
    p+= 24;
    memset(p,0,72);

    InitNeighborAvailability(para,&n);
    mb_4x4_intra_search(&n,input_addr);
    chroma_intra_search(&n,input_addr);

    q[2] = mb_x;
    q[3] = mb_y;
    q = (unsigned short*)(input_addr + 96);

    mb_y_offset = (mb_x<<4) + (mb_y<<4) * stride ;
    mb_u_offset = (mb_x<<3) + (mb_y<<2) * stride ;
    mb_v_offset = (mb_x<<3) + (mb_y<<2) * stride ;
    cur_mb_y = para->src.pic[1].plane[0] + mb_y_offset;
    cur_mb_u = para->src.pic[1].plane[1] + mb_u_offset;
    cur_mb_v = para->src.pic[1].plane[2] + mb_v_offset;

    Y_ext_asm2((unsigned char*)q, cur_mb_y, stride);
    YV12_UV_ext_asm2(q, cur_mb_u, cur_mb_v, stride);
}

inline void fill_i_buffer_spec_line(amvenc_drv_t* para, unsigned char* input_addr,
        int mb_y, amvenc_pred_mode_obj_t *private_data)
{
    uint8_t *cur_mb_y = NULL;
    uint8_t *cur_mb_u = NULL;
    uint8_t *cur_mb_v = NULL;
    int mbx_max  = para->src.mb_width;
    unsigned int stride;
    uint8_t *input_mb = NULL;
    unsigned char*  p = input_addr;
    unsigned short* q = (unsigned short*)input_addr;
    //uint8_t mb_type = HENC_MB_Type_I4MB ;
    //amvenc_neighbor_t n;
    int i;

    input_mb = input_addr;
    stride = para->src.pix_width;
    cur_mb_y = para->src.pic[1].plane[0] + (mb_y<<4) * stride;
    if(para->src.nv21_data){
        cur_mb_u = para->src.pic[1].plane[1] + (mb_y<<3) * stride;
    }else{
        cur_mb_u = para->src.pic[1].plane[1] + (mb_y<<2) * stride;
        cur_mb_v = para->src.pic[1].plane[2] + (mb_y<<2) * stride;
    }
    //----head of the mb
    for (i = 0; i < mbx_max; i++)
    {
        q = (unsigned short *)input_mb;
        p = input_mb;

        memcpy(p , picture_i_head ,24 );
        //p[18] = mb_type;
        p+= 24;
        memset(p,0,72);
#ifdef FAST_INTRA_SEARCH
        cur->neighbor.mbx = i;
        cur->neighbor.mby = mb_y;
        InitNeighborAvailability(para,&cur->neighbor);
        mb_4x4_intra_search(&cur->neighbor,input_mb);
        chroma_intra_search(&cur->neighbor,input_mb);
#else
#if 1
        MBIntraSearch_full(private_data, i, mb_y, input_mb);
#else
        cur->neighbor.mbx = i;
        cur->neighbor.mby = mb_y;
        InitNeighborAvailability_iframe( common, cur);
        MBIntraSearch_iframe( private_data, input_mb );
        chroma_intra_search(&cur->neighbor,input_mb);
#endif
#endif

        q[2] = i;
        q[3] = mb_y;
        input_mb += 864;
    }
    //----head of the mb

    Y_line_asm(input_addr, cur_mb_y,stride,mbx_max);
    if(para->src.nv21_data)
        nv21_uvline_asm(input_addr, cur_mb_u,stride,mbx_max);
    else
        yv12_uvline_asm(input_addr, cur_mb_u,cur_mb_v,stride,mbx_max);
}

inline void fill_p_buffer_spec_nv21(amvenc_drv_t* para, unsigned char* input_addr,int mb_x,int mb_y, mbinfo_t* mb_info)
{
    unsigned int stride = para->src.pix_width;
    unsigned char*  p = input_addr;
    unsigned short* q =  (unsigned short*)input_addr;
    uint8_t* cur_mb_y, *cur_mb_uv;
    uint8_t* ref_mb_y, *ref_mb_uv;
    unsigned mb_y_offset, mb_uv_offset;
    int mvx, mvy;
    int ref_x,ref_y;
    int i = 16;
    short *temp = NULL;

    mvx = para->motion_search.mot16x16[mb_info->mb_id].x;
    mvy = para->motion_search.mot16x16[mb_info->mb_id].y;

    memcpy(p , (char*)picture_p_head ,24 );
    p[18] = 0xf;//mb_info->mb_type;
    p+= 24;
    memset(p,0,72);

    q[2] = mb_x;
    q[3] = mb_y;
    temp = (short*)q;
    while(i<48){
        temp[i++] = mvy<<2;
        temp[i++] = mvx<<2;
    }
    ref_x = (mb_x<<4)+mvx;
    ref_y = (mb_y<<4)+mvy;

    mb_y_offset = (mb_x<<4) + (mb_y<<4) * stride ;
    mb_uv_offset = (mb_x<<4)+ + (mb_y<<3) * stride ;
    cur_mb_y = para->src.pic[1].plane[0] + mb_y_offset;
    cur_mb_uv = para->src.pic[1].plane[1] + mb_uv_offset;
    mb_y_offset = mb_y_offset + mvx + (mvy * stride) ;
    mb_uv_offset = (ref_x&0xfffffffe)+ (ref_y/2) * stride ;
    ref_mb_y  = para->ref_info.y+ mb_y_offset;
    ref_mb_uv = para->ref_info.uv + mb_uv_offset;

    q = (unsigned short*)(input_addr + 96);
    pY_ext_asm(q, cur_mb_y, ref_mb_y, stride, ref_x&0x07);
    NV21_pUV_ext_asm(q, cur_mb_uv, ref_mb_uv, stride, ref_x&0x06);
}

inline void fill_p_buffer_spec_yv12(amvenc_drv_t* para, unsigned char* input_addr,int mb_x,int mb_y,mbinfo_t* mb_info)
{
    unsigned int stride = para->src.pix_width;
    unsigned char*  p = input_addr;
    unsigned short* q =  (unsigned short*)input_addr;
    uint8_t  *cur_mb_y, *cur_mb_u, *cur_mb_v;
    unsigned mb_y_offset, mb_u_offset, mb_v_offset;
    uint8_t* ref_mb_y, *ref_mb_u,*ref_mb_v;
    short mvx, mvy;
    int ref_x,ref_y;
    int i = 16;
    short * temp = NULL;

    mvx = para->motion_search.mot16x16[mb_info->mb_id].x;
    mvy = para->motion_search.mot16x16[mb_info->mb_id].y;

    memcpy(p , (char*)picture_p_head ,24 );
    p[18] = 0xf;//mb_info->mb_type;
    p+= 24;
    memset(p,0,72);

    q[2] = mb_x;
    q[3] = mb_y;
    temp = (short*)q;
    while(i<48){
        temp[i++] = mvy<<2;
        temp[i++] = mvx<<2;
    }
    ref_x = (mb_x<<4)+mvx;
    ref_y = (mb_y<<4)+mvy;

    mb_y_offset = (mb_x<<4) + (mb_y<<4) * stride ;
    mb_u_offset = (mb_x<<3) + (mb_y<<2) * stride ;
    mb_v_offset = (mb_x<<3) + (mb_y<<2) * stride ;
    cur_mb_y = para->src.pic[1].plane[0] + mb_y_offset;
    cur_mb_u = para->src.pic[1].plane[1] + mb_u_offset;
    cur_mb_v = para->src.pic[1].plane[2] + mb_v_offset;
    mb_y_offset = mb_y_offset + mvx + (mvy * stride) ;
    mb_u_offset = (ref_x&0xfffffffe)+ (ref_y/2) * stride ;
    ref_mb_y  = para->ref_info.y + mb_y_offset;
    ref_mb_u = para->ref_info.uv + mb_u_offset;

    q = (unsigned short*)(input_addr + 96);
    pY_ext_asm(q, cur_mb_y, ref_mb_y, stride, ref_x & 0x07);
    YV12_pUV_ext_asm(q, cur_mb_u, cur_mb_v, ref_mb_u, stride, ref_x & 0x06);
}

static int fill_I_buffer(amvenc_drv_t* p, int slot)
{
    amvenc_slot_t* cur_slot = (amvenc_slot_t*)&p->slot[slot];
    amvenc_motionsearch_t *motion_search = (amvenc_motionsearch_t*)(&p->motion_search);
    amvenc_pred_mode_obj_t *private_data = cur_slot->mbObj;
    int mb_y;
    uint8_t  *input_mb;
    int mb_stride = p->src.mb_width;
    int update_bytes = 0;
    int total_bytes = 0;
    int pitch = 0;
    update_bytes = cur_slot->update_bytes;
    total_bytes = 0;
    pitch = mb_stride*864;

    private_data = MBIntraSearch_prepare(p);

    for (mb_y = cur_slot->start_mby; mb_y <= cur_slot->end_mby; ){
	 if((!p->mStart)||(p->mCancel))
            break;
        input_mb = p->input_buf.addr + (mb_y*pitch);
        fill_i_buffer_spec_line(p, input_mb, mb_y, private_data);
        total_bytes += pitch;
        if(update_bytes>0){
            if(((total_bytes%update_bytes) == 0)&&(mb_y<cur_slot->end_mby)){
                //ALOGV("Slot:%d,currnet y:%d, end y: %d",slot,mb_y,cur_slot->end_mby);
                sem_post(&cur_slot->semdone);
                sem_wait(&(cur_slot->semstart));
            }
        }
        mb_y+=cur_slot->y_step;
    }	
    if((p->mCancel == false)&&(p->mStart == true)){
        cur_slot->finish = true;
        cur_slot->ret = 0;
        //ALOGV("Slot:%d finish, total bytes:%d",slot,total_bytes);
        sem_post(&cur_slot->semdone);
    }
    if (private_data)
            MBIntraSearch_clean(private_data);
    return 0;
}

static int fill_P_buffer(amvenc_drv_t* p, int slot)
{
    amvenc_slot_t* cur_slot = (amvenc_slot_t*)&p->slot[slot];
    int mb_y,mb_x;
    uint8_t  *input_mb;
    int mb_stride = p->src.mb_width;
    mbinfo_t* cur_mb = NULL;
    int update_bytes = 0;
    int total_bytes = 0;
    update_bytes = cur_slot->update_bytes;
    total_bytes = 0;
    for (mb_y = cur_slot->start_mby; mb_y <= cur_slot->end_mby; ){
	 if((!p->mStart)||(p->mCancel))
            break;
        for (mb_x = cur_slot->start_mbx; mb_x <= cur_slot->end_mbx; ){
	     if((!p->mStart)||(p->mCancel))
                break;
            input_mb = p->input_buf.addr + ((mb_y*mb_stride+mb_x)*864);
            cur_mb = p->motion_search.mb_array+(mb_y*mb_stride+mb_x);
            if(p->src.nv21_data)                        
                fill_p_buffer_spec_nv21(p, input_mb, mb_x, mb_y,cur_mb);
            else                        
                fill_p_buffer_spec_yv12(p, input_mb, mb_x, mb_y,cur_mb);
            total_bytes += 864;
            if(update_bytes>0){
                if(((total_bytes%update_bytes) == 0)&&(mb_y<cur_slot->end_mby)){
                    //ALOGV("Slot:%d,currnet y:%d, end y: %d",slot,mb_y,cur_slot->end_mby);
                    sem_post(&cur_slot->semdone);
                    sem_wait(&(cur_slot->semstart));
                }
            }
            mb_x+=cur_slot->x_step;
        }
        mb_y+=cur_slot->y_step;
    }	
    if((p->mCancel == false)&&(p->mStart == true)){
        cur_slot->finish = true;
        cur_slot->ret  = 0;
        //ALOGV("Slot:%d finish, total bytes:%d",slot,total_bytes);
        sem_post(&cur_slot->semdone);
    }
    return 0;
}

static int MotionEstimation(amvenc_drv_t* p, int slot)
{
    amvenc_slot_t* cur_slot = (amvenc_slot_t*)&p->slot[slot];
    amvenc_motionsearch_t *motion_search = (amvenc_motionsearch_t*)(&p->motion_search);
    int update_bytes = 0;
    int i, j, k;
    mvs_t *mot_mb_16x16, *mot16x16 = motion_search->mot16x16;
    int NumIntraSearch;
    int mbnum, offset;
    uint8_t *cur;
    int abe_cost;
    unsigned mv_uint32;
    int start_count = 5;

    mbnum = cur_slot->start_mby*p->src.mb_width;
    NumIntraSearch = 0; // to be intra searched in the encoding loop.
    offset = mbnum*256;
    update_bytes = cur_slot->update_bytes;
    cur = p->src.pic[1].plane[0];
    for (j = cur_slot->start_mby; j <= cur_slot->end_mby; j++){
        if((!p->mStart)||(p->mCancel))
            break;
        for (i = cur_slot->start_mbx; i <= cur_slot->end_mbx; i++){
            if((!p->mStart)||(p->mCancel))
                break;
            mot_mb_16x16 = mot16x16 + mbnum;
            //motion_search->mb_array[mbnum].intra = 0;
            motion_search->mb_array[mbnum].mb_id = mbnum;            
            cur = p->src.pic[1].plane[0] + offset;

            MBMotionSearch(p, i << 4, j << 4,mbnum,(motion_search->fullsearch_enable|((start_count>0)&&(p->PrevRefFrameNum == 0))));

            abe_cost = motion_search->min_cost[mbnum] = mot_mb_16x16->sad;

            /* set mbMode and MVs */
            //motion_search->mb_array[mbnum].mb_type= 0xf;//HENC_MB_Type_P16x16;
            
            //mv_uint32 = ((mot_mb_16x16->y) << 16) | ((mot_mb_16x16->x) & 0xffff);
            //for (k = 0; k < 32; k += 2)
            //    motion_search->mb_array[mbnum].mvL0[k>>1] = mv_uint32;
            
            /* make a decision whether it should be tested for intra or not */
            if (i != p->src.mb_width - 1 && j != p->src.mb_height - 1 && i != 0 && j != 0){
                if (true == MVIntraDecisionABE(motion_search,&abe_cost, cur, p->src.pix_width, true)){
                    NumIntraSearch++;
                    mot_mb_16x16->mad = abe_cost;
                }
                //else
                //    motion_search->mb_array[mbnum].intra = 1;
            }else{    /* INTRA update, use for prediction */
                //mot_mb_16x16[0].x = mot_mb_16x16[0].y = 0;
                //motion_search->mb_array[mbnum].intra = 1;
                //abe_cost = motion_search->min_cost[mbnum] = 0x7FFFFFFF;  /* max value for int */
                NumIntraSearch++ ;
            }
            //motion_search->mb_array[mbnum].intra = 0;
            mbnum ++;
            offset +=256;
            if(start_count>0)
                start_count --;
        } /* for i */
    } /* for j */
    if((p->mCancel == false)&&(p->mStart == true)){
        cur_slot->finish = true;
        cur_slot->ret  = NumIntraSearch;
        sem_post(&cur_slot->semdone);
    }
    return 0;
}

static void* _fillbuff_Thread(amvenc_drv_t* p, int slot)
{
    amvenc_slot_t* cur_slot = (amvenc_slot_t*)&p->slot[slot];
    while(1){
        if(p->mCancel ==false)
            sem_wait(&(cur_slot->semstart));
        if(p->mCancel){
            int sem_count = 0;
            sem_getvalue(&cur_slot->semdone,&sem_count);
            if(sem_count<0){
                sem_post(&cur_slot->semdone);
                usleep(50000);
            }
            break;
        }
        if(cur_slot->mode == Slot_mode_I_fill){
            fill_I_buffer(p, slot);
        }else if(cur_slot->mode == Slot_mode_P_fill)
            fill_P_buffer(p, slot);
        else if(cur_slot->mode == Slot_mode_me)
            MotionEstimation(p, slot);
        else if(cur_slot->mode != Slot_mode_idle)
            ALOGE("SLOT %d, mode is wrong!: %d",slot,cur_slot->mode);
    }

    return NULL;
}

static void* fillbuff_Thread_0(void *cookie)
{
    amvenc_drv_t *p = (amvenc_drv_t *)cookie;
    int slot = 0 ;
    prctl(PR_SET_NAME, (unsigned long)"MTVenc_SLOT0", 0, 0, 0);
    ALOGV("Thread slot %d is created \n" ,slot);	
    return _fillbuff_Thread(p,slot);
}

static void* fillbuff_Thread_1(void *cookie)
{
    amvenc_drv_t *p = (amvenc_drv_t *)cookie;
    int slot = 1 ;
    prctl(PR_SET_NAME, (unsigned long)"MTVenc_SLOT1", 0, 0, 0);
    ALOGV("Thread slot %d is created \n" ,slot);	
    return _fillbuff_Thread(p,slot);
}

static int init_yuv(yuv_t *src, unsigned* yuv, unsigned enc_width, unsigned enc_height , unsigned char* ref_buf_y,unsigned char* ref_buf_uv )
{
    int i;
    unsigned y = yuv[0];
    unsigned u = yuv[1];
    unsigned v = yuv[2];

    if((!y)||(!u))
        return -1;
    if(!v)
        src->nv21_data = true;
    else
        src->nv21_data = false;

    for (i = 0; i < I_FRAME; ++i) 
        src->pic[i].plane[0] = NULL;
    src->pix_width  = enc_width; 
    src->pix_height = enc_height; 
    src->mb_width   = src->pix_width>>4;  
    src->mb_height  = src->pix_height>>4; 
  
    src->pic[1].plane[0] = (unsigned char*)y;
    src->pic[1].plane[1] = (unsigned char*)u; 
    if(src->nv21_data == false)
        src->pic[1].plane[2] = (unsigned char*)v;	
		
    if((ref_buf_y == NULL)||(ref_buf_uv==NULL)){
        src->pic[0].plane[0] = src->pic[1].plane[0];
        src->pic[0].plane[1] = src->pic[1].plane[1];
    }else{
        src->pic[0].plane[0] = ref_buf_y;
        src->pic[0].plane[1] = ref_buf_uv;
        //if(src->nv21_data == false)
        //    src->pic[0].plane[2] = ref_buf + 0xf0000;
    }
    return 0;
}

static AMVEnc_Status start_ime(amvenc_drv_t* p, unsigned char* outptr,int* datalen)
{
    AMVEnc_Status ret = AMVENC_FAIL;
    unsigned cmd ,status;
    unsigned update_bytes = 864*p->src.mb_width, next_bytes;
    unsigned total_bytes = 0;
    unsigned offset[2];
    unsigned total_offset = 0, size = 0, retry = 0;
    unsigned addr_info[3];
    int addr_index;
    unsigned update_line = p->src.mb_height/4;
#ifdef DEBUG_TIME
    unsigned total_time = 0;
    gettimeofday(&start_test, NULL);
#endif

    //ioctl(p->fd, MTVENC_AVC_IOC_GET_ADDR, &addr_index);
    //addr_info[0] = addr_index == ENCODER_BUFFER_REF0?addr_index:ENCODER_BUFFER_REF1;
    //addr_info[1] = 0;
    //addr_info[2] = 0x170000;
    //ioctl(p->fd, MTVENC_AVC_IOC_FLUSH_DMA ,addr_info); 	 

    update_line = (p->src.mb_height%4)?(update_line+1):update_line;
    total_bytes = 864*p->src.mb_width*p->src.mb_height;
    update_bytes = update_bytes*update_line;
    p->slot[0].finish = false;
    p->slot[0].start_mbx = 0;
    p->slot[0].start_mby = 0;
    p->slot[0].x_step = 1;
    p->slot[0].y_step = 2;
    p->slot[0].end_mbx = p->src.mb_width - 1;
    p->slot[0].end_mby = (p->src.mb_height&1)?(p->src.mb_height - 1):(p->src.mb_height - 2);
    p->slot[0].update_bytes = update_bytes;
    p->slot[0].mode = Slot_mode_P_fill;
    p->slot[0].ret = -1;

    p->slot[1].finish = false;
    p->slot[1].start_mbx = 0;
    p->slot[1].start_mby = 1;
    p->slot[1].x_step = 1;
    p->slot[1].y_step = 2;
    p->slot[1].end_mbx = p->src.mb_width - 1;
    p->slot[1].end_mby = (p->src.mb_height&1)?(p->src.mb_height - 2):(p->src.mb_height - 1);
    p->slot[1].update_bytes = update_bytes;
    p->slot[1].mode = Slot_mode_P_fill;
    p->slot[1].ret = -1;

    p->mStart = true;
    sem_post(&(p->slot[0].semstart));
    sem_post(&(p->slot[1].semstart));
    offset[0] = offset[1] = 0 ;
    cmd = ENCODER_NON_IDR; 
    ioctl(p->fd, MTVENC_AVC_IOC_NEW_CMD, &cmd);
    while(p->mCancel == false){
        sem_wait(&(p->slot[0].semdone));
        sem_wait(&(p->slot[1].semdone));

        if((p->slot[0].finish)&&(p->slot[1].finish))
            total_offset = total_bytes;
        else
            total_offset += (update_bytes*2);
        if(total_offset>total_bytes)
            total_offset = total_bytes;

        if(total_offset>offset[0]){
            if(total_bytes == total_offset)
                total_offset = (total_offset+0xff)&0xffffff00;
            offset[1] = total_offset;
            addr_info[0] = ENCODER_BUFFER_INPUT;
            addr_info[1] = offset[0];
            addr_info[2] = offset[1];							
            ioctl(p->fd, MTVENC_AVC_IOC_FLUSH_CACHE ,addr_info);
            ioctl(p->fd, MTVENC_AVC_IOC_INPUT_UPDATE, &total_offset);
            offset[0] = total_offset;
            if(total_offset<total_bytes){
                sem_post(&(p->slot[0].semstart));
                sem_post(&(p->slot[1].semstart));
            }
        }

        if((p->slot[0].finish)&&(p->slot[1].finish)&&(total_offset >= total_bytes)){
            p->mStart = false;
            sem_init(&(p->slot[0].semdone), 0,0);
            sem_init(&(p->slot[1].semdone), 0,0);
            break;
        }
    }
    if(encode_poll(p->fd, ENCODE_DONE_TIMEOUT)<=0){
        ALOGD("start_ime: poll fail");
        return AMVENC_FAIL;
    }
    ioctl(p->fd, MTVENC_AVC_IOC_GET_STAGE, &status);	

    ret = AMVENC_FAIL;
    if(status == ENCODER_IDR_DONE){
        ioctl(p->fd, MTVENC_AVC_IOC_GET_OUTPUT_SIZE, &size);	
        if((size < p->output_buf.size)&&(size>0)){
            addr_info[0] = ENCODER_BUFFER_OUTPUT;
            addr_info[1] = 0 ;
            addr_info[2] = size ;
            ioctl(p->fd, MTVENC_AVC_IOC_FLUSH_DMA ,addr_info);
            memcpy(outptr,p->output_buf.addr,size);
            *datalen  = size;
            ret = AMVENC_PICTURE_READY;
            ALOGV("start_ime: done size: %d ",size);
        }
    }else{
        ALOGE("start_ime: encode timeout, status:%d",status);
        ret = AMVENC_FAIL;
    }
    if(ret == AMVENC_PICTURE_READY){
        p->PrevRefFrameNum++;
#ifdef DEBUG_TIME
        gettimeofday(&end_test, NULL);
        total_time = end_test.tv_sec - start_test.tv_sec;        
        total_time = total_time*1000000 + end_test.tv_usec -start_test.tv_usec;
        ALOGD("start_ime: need time: %d us",total_time);
        p->total_encode_frame++;
        p->total_encode_time +=total_time; 
#endif 
    }
    return ret;
}

static AMVEnc_Status start_intra(amvenc_drv_t* p, unsigned char* outptr,int* datalen)
{
    AMVEnc_Status ret = AMVENC_FAIL;
    unsigned cmd ,status;
    unsigned update_bytes = 864*p->src.mb_width ;
    unsigned total_bytes = 0;
    unsigned offset[2]; //0 :  pre_offset   1: current_offset
    unsigned total_offset = 0, size = 0, retry = 0, header_size = 0;
    unsigned addr_info[3];
    unsigned update_line = p->src.mb_height/4;
#ifdef DEBUG_TIME
    unsigned total_time = 0;
    gettimeofday(&start_test, NULL);
#endif
    update_line = (p->src.mb_height%4)?(update_line+1):update_line;
    total_bytes = 864*p->src.mb_width*p->src.mb_height;
    update_bytes = update_bytes*update_line;

    p->slot[0].finish = false;
    p->slot[0].start_mbx = 0;
    p->slot[0].start_mby = 0;
    p->slot[0].x_step = 1;
    p->slot[0].y_step = 2;
    p->slot[0].end_mbx = p->src.mb_width - 1;
    p->slot[0].end_mby = (p->src.mb_height&1)?(p->src.mb_height - 1):(p->src.mb_height - 2);
    p->slot[0].update_bytes = update_bytes;
    p->slot[0].mode = Slot_mode_I_fill;
    p->slot[0].ret = -1;

    p->slot[1].finish = false;
    p->slot[1].start_mbx = 0;
    p->slot[1].start_mby = 1;
    p->slot[1].x_step = 1;
    p->slot[1].y_step = 2;
    p->slot[1].end_mbx = p->src.mb_width - 1;
    p->slot[1].end_mby = (p->src.mb_height&1)?(p->src.mb_height - 2):(p->src.mb_height - 1);
    p->slot[1].update_bytes = update_bytes;
    p->slot[1].mode = Slot_mode_I_fill;
    p->slot[1].ret = -1;

    p->mStart = true;
    sem_post(&(p->slot[0].semstart));
    sem_post(&(p->slot[1].semstart));
    offset[0] = offset[1] = 0 ;
#if 0
    cmd = ENCODER_SEQUENCE; 
    ioctl(p->fd, MTVENC_AVC_IOC_NEW_CMD, &cmd);
    ioctl(p->fd, MTVENC_AVC_IOC_GET_STAGE, &status);	
    while((!p->mCancel)&&(status != ENCODER_PICTURE_DONE)){
        ioctl(p->fd, MTVENC_AVC_IOC_GET_STAGE, &status);	
        if((cmd == ENCODER_SEQUENCE)&&(status == ENCODER_SEQUENCE_DONE)){
            cmd = ENCODER_PICTURE ;
            ioctl(p->fd, MTVENC_AVC_IOC_NEW_CMD, &cmd);
        }
    }
    if((cmd == ENCODER_PICTURE)&&(status == ENCODER_PICTURE_DONE)){
        ioctl(p->fd, MTVENC_AVC_IOC_GET_OUTPUT_SIZE, &header_size);
        if(header_size >= p->output_buf.size)
            header_size = 21;
        cmd = ENCODER_IDR ;
        ioctl(p->fd, MTVENC_AVC_IOC_NEW_CMD, &cmd);
    }
#else
    header_size = 0;
    cmd = ENCODER_IDR ;
    ioctl(p->fd, MTVENC_AVC_IOC_NEW_CMD, &cmd);
#endif
    while(p->mCancel == false){
        sem_wait(&(p->slot[0].semdone));
        sem_wait(&(p->slot[1].semdone));

        if((p->slot[0].finish)&&(p->slot[1].finish))
            total_offset = total_bytes;
        else
            total_offset += (update_bytes*2);
        if(total_offset>total_bytes)
            total_offset = total_bytes;

        if(total_offset>offset[0]){
            if(total_bytes == total_offset)
                total_offset = (total_offset+0xff)&0xffffff00;
            offset[1] = total_offset;
            addr_info[0] = ENCODER_BUFFER_INPUT;
            addr_info[1] = offset[0];
            addr_info[2] = offset[1];							
            ioctl(p->fd, MTVENC_AVC_IOC_FLUSH_CACHE ,addr_info);
            ioctl(p->fd, MTVENC_AVC_IOC_INPUT_UPDATE, &total_offset);
            offset[0] = total_offset;
            if(total_offset<total_bytes){
                sem_post(&(p->slot[0].semstart));
                sem_post(&(p->slot[1].semstart));
            }
        }

        if((p->slot[0].finish)&&(p->slot[1].finish)&&(total_offset >= total_bytes)){
            p->mStart = false;
            sem_init(&(p->slot[0].semdone), 0,0);
            sem_init(&(p->slot[1].semdone), 0,0);
            break;
        }
    }
    if(encode_poll(p->fd, ENCODE_DONE_TIMEOUT)<=0){
        ALOGD("start_intra: poll fail");
        return AMVENC_FAIL;
    }
    ioctl(p->fd, MTVENC_AVC_IOC_GET_STAGE, &status);	
    ret = AMVENC_FAIL;
    if(status == ENCODER_IDR_DONE){
        ioctl(p->fd, MTVENC_AVC_IOC_GET_OUTPUT_SIZE, &size);	
        if((size < p->output_buf.size)&&(size>0)){
            addr_info[0] = ENCODER_BUFFER_OUTPUT;
            addr_info[1] = 0 ;
            addr_info[2] = size ;
            ioctl(p->fd, MTVENC_AVC_IOC_FLUSH_DMA ,addr_info);
            if(header_size>= size)
                header_size = 0;
            memcpy(outptr,p->output_buf.addr+header_size,size-header_size);
            *datalen  = size-header_size;
            ret = AMVENC_NEW_IDR;
            ALOGV("start_intra: done size: %d",size);
        }
    }else{
        ALOGE("start_intra: encode timeout, status:%d",status);
        ret = AMVENC_FAIL;
    }

    if(ret == AMVENC_NEW_IDR){
#ifdef DEBUG_TIME
        gettimeofday(&end_test, NULL);
        total_time = (end_test.tv_sec - start_test.tv_sec)*1000000 + end_test.tv_usec -start_test.tv_usec;
        p->total_encode_frame++;
        p->total_encode_time +=total_time;
        ALOGD("start_intra: need time: %d us",total_time);
#endif
    }
    return ret;
}

static int GetMTVencRefBuf(int fd)
{
    int ret = -1;
    int addr_index = 0;
    if(fd>=0){
        ioctl(fd, MTVENC_AVC_IOC_GET_ADDR, &addr_index);
        if(addr_index == 1){
            ret= 0;
        }else{
            ret = 1;
        }
    }
    return ret;
}

void* InitMTVEncode(int fd, unsigned enc_width, unsigned enc_height, unsigned quant, unsigned searchrange, bool fs_en)
{
    int addr_index = 0;
    int ret = 0;
    unsigned buff_info[20];
    unsigned mode = 1;

    amvenc_drv_t* p = (amvenc_drv_t*)calloc(1,sizeof(amvenc_drv_t));
    int i = 0;
    pthread_attr_t attr;

    if(!p){
        ALOGE("InitMTVEncode calloc fail");
        return NULL;
    }
    memset(p,0,sizeof(amvenc_drv_t));

    //p->fd= open(ENCODER_PATH, O_RDWR);
    p->fd = fd;
    if(p->fd < 0){
        ALOGE("InitMTVEncode open encode device fail");
        free(p);
        return NULL;
    }
    memset(buff_info,0,sizeof(buff_info));
    ret = ioctl(p->fd, MTVENC_AVC_IOC_GET_BUFFINFO,&buff_info[0]);
    if((ret)||(buff_info[0]==0)){
        ALOGE("InitMTVEncode -- old amvenc driver.  no buffer information, use defalut value!");
        buff_info[0] = 0x800000;
		
        buff_info[1] = 0;  // dct buff addr
        buff_info[2] = 0x300000;

        buff_info[3] = 0x300000; // ref buf 0  y addr
        buff_info[4] = 0xf0000;

        buff_info[5] = 0x3f0000; // ref buf 0  uv addr
        buff_info[6] = 0x80000;

        buff_info[7] = 0x480000; // ref buf 1  y addr
        buff_info[8] = 0xf0000;

        buff_info[9] = 0x570000; // ref buf 1  uv addr
        buff_info[10] = 0x80000;

        buff_info[11] = 0x700000; // output addr
        buff_info[12] = 0x100000;
    }

    p->mmap_buff.addr = (unsigned char*)mmap(0,buff_info[0], PROT_READ|PROT_WRITE , MAP_SHARED ,p->fd, 0);
    if (p->mmap_buff.addr == MAP_FAILED) {
        ALOGE("InitMTVEncode mmap fail");
        //close(p->fd);
        free(p);
        return NULL;
    }

    p->quant = quant;
    p->enc_width = enc_width;
    p->enc_height = enc_height;
    p->mmap_buff.size = buff_info[0];
	
    p->input_buf.addr = p->mmap_buff.addr+buff_info[1];
    p->input_buf.size = buff_info[3]-buff_info[1];
	
    p->ref_buf_y[0].addr = p->mmap_buff.addr +buff_info[3];
    p->ref_buf_y[0].size = buff_info[4];
    p->ref_buf_uv[0].addr = p->mmap_buff.addr +buff_info[5];
    p->ref_buf_uv[0].size = buff_info[6];

    p->ref_buf_y[1].addr = p->mmap_buff.addr +buff_info[7];
    p->ref_buf_y[1].size = buff_info[8];
    p->ref_buf_uv[1].addr = p->mmap_buff.addr +buff_info[9];
    p->ref_buf_uv[1].size = buff_info[10];
    p->output_buf.addr = p->mmap_buff.addr +buff_info[11] ;
    p->output_buf.size = buff_info[12];
    p->src.pix_width= enc_width;
    p->src.pix_height= enc_height;
    p->src.mb_width = enc_width>>4;
    p->src.mb_height= enc_height>>4;
    p->src.mbsize = p->src.mb_height*p->src.mb_width;
    p->sps_len = 0;
    p->gotSPS = false;
    p->pps_len = 0;
    p->gotPPS = false;

    p->ref_info.width = enc_width;
    p->ref_info.height = enc_height;
    p->ref_info.pitch = enc_width;
    p->ref_info.size = enc_width*enc_height*3/2;
    p->ref_info.y = NULL;//p->ref_info.buff;
    p->ref_info.uv = NULL;//p->ref_info.buff+enc_width*enc_height;

    ioctl(p->fd, MTVENC_AVC_IOC_SET_QUANT,&p->quant);
    ioctl(p->fd, MTVENC_AVC_IOC_GET_ADDR, &addr_index);
	
    ioctl(p->fd, MTVENC_AVC_IOC_SET_ENCODER_WIDTH ,&p->enc_width);
    ioctl(p->fd, MTVENC_AVC_IOC_SET_ENCODER_HEIGHT ,&p->enc_height);
    if((p->enc_width != enc_width)||(p->enc_height != enc_height)){
        ALOGE("InitMTVEncode --set encode size fail. set as %dx%d, but max size is %dx%d.",enc_width,enc_height,p->enc_width,p->enc_height);
        munmap(p->mmap_buff.addr ,p->mmap_buff.size);
        //close(p->fd);
        free(p);
        return NULL;
    }
    ioctl(p->fd, MTVENC_AVC_IOC_CONFIG_INIT,&mode);
    if(addr_index == 1){
        p->ref_id = 0;
    }else{
        p->ref_id = 1;
    }
    p->mCancel = false;
    p->mStart = false;
    p->total_encode_frame  = 0;
    p->total_encode_time = 0;
    p->motion_search.fullsearch_enable = fs_en;
    p->motion_search.mvRange = searchrange;
    if(InitMotionSearchModule(p) != AMVENC_SUCCESS){
        ALOGE("InitMotionSearchModule fail");
        munmap(p->mmap_buff.addr ,p->mmap_buff.size);
        //close(p->fd);
        free(p);
        return NULL;
    }

    if(InitMBIntraSearchModule(p) < 0){
        ALOGE("InitMBIntraSearchModule fail");
        CleanMotionSearchMoudle(&p->motion_search);
        munmap(p->mmap_buff.addr ,p->mmap_buff.size);
        free(p->ref_info.buff);
        //close(p->fd);
        free(p);
        return NULL;
    }

    typedef void* (*THREAD_FUN)(void*) ;
    THREAD_FUN thread_fun[2];
    void *dummy = NULL;

    for(i =0; i< 2 ; i++){
        sem_init(&(p->slot[i].semdone), 0,0);
        sem_init(&(p->slot[i].semstart), 0,0);
    }
    thread_fun[0] = fillbuff_Thread_0;
    thread_fun[1] = fillbuff_Thread_1;
    for(i = 0;i<2;i++){
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
        if(pthread_create(&(p->slot[i].mThread), &attr, thread_fun[i], p) !=0){
            pthread_attr_destroy(&attr);
            if(i>0){
                sem_post(&(p->slot[i-1].semstart));
                pthread_join(p->slot[i-1].mThread, &dummy);
            }
            sem_destroy(&(p->slot[0].semdone)); 
            sem_destroy(&(p->slot[1].semdone)); 
            sem_destroy(&(p->slot[0].semstart)); 
            sem_destroy(&(p->slot[1].semstart)); 
            ALOGE("InitMTVEncode create thread fail");
            CleanMotionSearchMoudle(&p->motion_search);
            CleanMBIntraSearchModule(p);
            munmap(p->mmap_buff.addr ,p->mmap_buff.size);
            //free(p->ref_info.buff);
            //close(p->fd);
            free(p);
            return NULL;
        }
        pthread_attr_destroy(&attr);
    }
    return (void *)p;
}

AMVEnc_Status MTVEncodeInitFrame(void *dev, unsigned *yuv, AMVEncBufferType type, AMVEncFrameFmt fmt,bool IDRframe)
{
    amvenc_drv_t* p = (amvenc_drv_t*)dev;
    AMVEnc_Status ret = AMVENC_FAIL;
    int i = 0;
    if((!p)||(!yuv))
        return ret;
    amvenc_motionsearch_t *motion_search = (amvenc_motionsearch_t*)(&p->motion_search);
#ifdef DEBUG_TIME
    unsigned total_time = 0;
    gettimeofday(&start_test, NULL);
#endif
    p->ref_id = GetMTVencRefBuf(p->fd);
	
    ALOGV("GetMTVencRefBuf id:%d",p->ref_id);

    init_yuv(&p->src, yuv,p->enc_width, p->enc_height, p->ref_buf_y[p->ref_id].addr,p->ref_buf_uv[p->ref_id].addr);
    p->IDRframe =IDRframe;

    if(p->IDRframe){
        p->PrevRefFrameNum = 0;
        for(i = 0; i<p->src.mb_width*p->src.mb_height;i++)
            p->motion_search.min_cost[i] = 0x7FFFFFFF;
        p->motion_search.totalSAD = 0;
        ret = AMVENC_NEW_IDR;
    }else{
        int NumIntraSearch = 0;
        p->ref_info.y = p->src.pic[0].plane[0];//p->ref_info.buff;
        p->ref_info.uv = p->src.pic[0].plane[1];//p->ref_info.buff+enc_width*enc_height;

        p->slot[0].finish = false;
        p->slot[0].start_mbx = 0;
        p->slot[0].start_mby = 0;
        p->slot[0].x_step = 1;
        p->slot[0].y_step = 1;
        p->slot[0].end_mbx = p->src.mb_width - 1;
        p->slot[0].end_mby = p->src.mb_height/2 - 1;
        p->slot[0].update_bytes = 0;
        p->slot[0].mode = Slot_mode_me;
        p->slot[0].ret = -1;

        p->slot[1].finish = false;
        p->slot[1].start_mbx = 0;
        p->slot[1].start_mby = p->src.mb_height/2;
        p->slot[1].x_step = 1;
        p->slot[1].y_step = 1;
        p->slot[1].end_mbx = p->src.mb_width - 1;
        p->slot[1].end_mby = p->src.mb_height - 1;
        p->slot[1].update_bytes = 0;
        p->slot[1].mode = Slot_mode_me;
        p->slot[1].ret = -1;
        p->mStart = true;

        sem_post(&(p->slot[0].semstart));
        sem_post(&(p->slot[1].semstart));

        sem_wait(&(p->slot[0].semdone));
        sem_wait(&(p->slot[1].semdone));
        p->mStart = false;
        ret = AMVENC_SUCCESS;
        if((p->slot[0].ret>0)&&(p->slot[1].ret>0)){
            NumIntraSearch = p->slot[0].ret+p->slot[1].ret;
            if (NumIntraSearch*100 > (45*p->src.mb_width*p->src.mb_height)){			
                ret = AMVENC_NEW_IDR;
                p->PrevRefFrameNum = 0;
                p->IDRframe = true;
            }
            p->motion_search.totalSAD = 0;
            for(i = 0; i<p->src.mb_width*p->src.mb_height;i++){
                if(p->IDRframe == true)
                    p->motion_search.min_cost[i] = 0x7FFFFFFF;
                p->motion_search.totalSAD += (p->motion_search.mot16x16[i].mad);
            }
        }
        //ALOGD("MTVEncodeInitFrame: NumIntraSearch:%d",NumIntraSearch);
    }
#ifdef DEBUG_TIME
    gettimeofday(&end_test, NULL);
    total_time = (end_test.tv_sec - start_test.tv_sec)*1000000 + end_test.tv_usec -start_test.tv_usec;
    p->total_encode_time +=total_time;
    ALOGD("MTVEncodeInitFrame: need time: %d us, ret:%d",total_time,ret);
#endif
    return ret;
}

AMVEnc_Status MTVEncodeSPS_PPS(void* dev, unsigned char* outptr,int* datalen)
{
    amvenc_drv_t* p = (amvenc_drv_t*)dev;
    AMVEnc_Status ret = AMVENC_FAIL;
    unsigned status;
    unsigned size = 0;
    unsigned control_info[3];

    control_info[0] = ENCODER_SEQUENCE; 
    ioctl(p->fd, MTVENC_AVC_IOC_NEW_CMD, &control_info[0]);
    ioctl(p->fd, MTVENC_AVC_IOC_GET_STAGE, &status);	
    while((status !=ENCODER_SEQUENCE_DONE)&&(p->mCancel == false)){
        usleep(100);
        ioctl(p->fd, MTVENC_AVC_IOC_GET_STAGE, &status);	
    }
    if((control_info[0] == ENCODER_SEQUENCE)&&(status == ENCODER_SEQUENCE_DONE)){
        ioctl(p->fd, MTVENC_AVC_IOC_GET_OUTPUT_SIZE, &size);	
        if((size < p->output_buf.size)&&(size>0)){
            p->gotSPS = true;
            p->sps_len = size;

            control_info[0] = ENCODER_PICTURE; 
            ioctl(p->fd, MTVENC_AVC_IOC_NEW_CMD, &control_info[0]);
            ioctl(p->fd, MTVENC_AVC_IOC_GET_STAGE, &status);	
            while((status !=ENCODER_PICTURE_DONE)&&(p->mCancel == false)){
                usleep(100);
                ioctl(p->fd, MTVENC_AVC_IOC_GET_STAGE, &status);	
            }
            if((control_info[0] == ENCODER_PICTURE)&&(status == ENCODER_PICTURE_DONE)){
                ioctl(p->fd, MTVENC_AVC_IOC_GET_OUTPUT_SIZE, &size);	
                if((size < p->output_buf.size)&&(size>0)){
                    p->gotPPS = true;
                    p->pps_len = size - p->sps_len;
                }
            }
        }
    }

    ret = AMVENC_FAIL;
    ALOGV("MTVEncodeSPS_PPS status:%d", status);
    if(status == ENCODER_PICTURE_DONE){
        if((p->gotPPS == true)&&(p->gotSPS == true)){
            control_info[0] = ENCODER_BUFFER_OUTPUT;
            control_info[1] = 0;
            control_info[2] = p->pps_len+p->sps_len ;
            ioctl(p->fd, MTVENC_AVC_IOC_FLUSH_DMA ,control_info);
            memcpy(outptr,p->output_buf.addr,p->pps_len+p->sps_len);
            *datalen  = p->pps_len+p->sps_len;
            ret = AMVENC_SUCCESS;
        }
    }
    return ret;
}

AMVEnc_Status MTVEncodeSlice(void* dev, unsigned char* outptr,int* datalen)
{
    amvenc_drv_t* p = (amvenc_drv_t*)dev;
    AMVEnc_Status ret = AMVENC_FAIL;
    if((!p)||(!outptr)||(!datalen))
        return ret;

    UpdateMotionQP(p);
    ioctl(p->fd, MTVENC_AVC_IOC_SET_QUANT,&p->quant);
    if(p->IDRframe){
        ret = start_intra(p,outptr,datalen);
    }else{
        ret = start_ime(p,outptr,datalen);
    }
    return ret;
}

AMVEnc_Status MTVEncodeCommit(void* dev,  bool IDR)
{
    amvenc_drv_t* p = (amvenc_drv_t*)dev;
    AMVEnc_Status ret = AMVENC_FAIL;
    //int status = 0;
    if(!p)
        return ret;
    //status = (IDR == true)?ENCODER_IDR:ENCODER_NON_IDR;
    //if(ioctl(p->fd, M8VENC_AVC_IOC_SUBMIT_ENCODE_DONE ,&status)== 0)
    //    ret = AMVENC_SUCCESS;
    return AMVENC_SUCCESS;
}

void UnInitMTVEncode(void* dev)
{
    amvenc_drv_t* p = (amvenc_drv_t*)dev;
    void *dummy = NULL;
    int i = 0;
    if(!p)
        return;

    p->mCancel = true;
    for(i = 0;i<2;i++){
        sem_post(&(p->slot[i].semstart));
        pthread_join(p->slot[i].mThread, &dummy);
        sem_destroy(&(p->slot[i].semdone)); 
        sem_destroy(&(p->slot[i].semstart)); 
    }

    CleanMotionSearchMoudle(&p->motion_search);
    CleanMBIntraSearchModule(p);

    if(p->mmap_buff.addr)
        munmap(p->mmap_buff.addr ,p->mmap_buff.size);
    //if(p->fd>=0)
    //    close(p->fd);

#ifdef DEBUG_TIME
    ALOGD("total_encode_frame: %d,  total_encode_time: %d ms",p->total_encode_frame,p->total_encode_time/1000);
#endif
    free(p);
    return;
}

