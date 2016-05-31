/*  ----------------------------------- DSP/BIOS Headers            */
#include <std.h>
#include <gbl.h>
#include <log.h>
#include <swi.h>
#include <sys.h>
#include <tsk.h>
#include <pool.h>

/*  ----------------------------------- DSP/BIOS LINK Headers       */
#include <failure.h>
#include <dsplink.h>
#include <platform.h>
#include <notify.h>
#include <bcache.h>
/*  ----------------------------------- Sample Headers              */
#include <canny_edge_config.h>
#include <task.h>

/* Buffer defines */
#define NUM_BUF_SIZES                    7 ///< Amount of pools to be configured
#define NUM_BUF_POOL0                    1 ///< Amount of buffers in the first pool
#define NUM_BUF_POOL1                    1 ///< Amount of buffers in the second pool
#define NUM_BUF_POOL2                    1 ///< Amount of buffers in the second pool
#define NUM_BUF_POOL3                    1 ///< Amount of buffers in the second pool
#define NUM_BUF_POOL4                    1 ///< Amount of buffers in the second pool
#define NUM_BUF_POOL5                    1 ///< Amount of buffers in the second pool
#define NUM_BUF_POOL6                    1 ///< Amount of buffers in the second pool
#define NUM_BUF_MAX                      1 ///< Maximum amount of buffers in pool

enum {
    canny_edge_INIT,                    ///< Initialization stage
    canny_edge_DELETE,                  ///< Shutdown step
    canny_edge_WRITEBACK,               ///< Simple write back program
    canny_edge_MAGNITUDE,               ///< Magnitude calculation x,y on DSP program
    canny_edge_DERIVATIVE               ///< Derivative calculation x,y on DSP program
};

Uint32 pool_sizes[] = {NUM_BUF_POOL0, NUM_BUF_POOL1, NUM_BUF_POOL2, NUM_BUF_POOL3, NUM_BUF_POOL4, NUM_BUF_POOL5, NUM_BUF_POOL6};
Void *dsp_buffers[NUM_BUF_SIZES][NUM_BUF_MAX];      ///< Buffer addresses on the DSP
Uint32 buffer_sizes[NUM_BUF_SIZES];                 ///< The buffer sizes


static Void Task_notify (Uint32 eventNo, Ptr arg, Ptr info) ;

Int Task_create (Task_TransferInfo ** infoPtr)
{
    Int status    = SYS_OK ;
    Task_TransferInfo * info = NULL ;

    /* Allocate Task_TransferInfo structure that will be initialized
     * and passed to other phases of the application */
    if (status == SYS_OK) 
	  {
        *infoPtr = MEM_calloc (DSPLINK_SEGID,
                               sizeof (Task_TransferInfo),
                               0) ; /* No alignment restriction */
        if (*infoPtr == NULL) 
		    {
            status = SYS_EALLOC ;
        }
        else 
		    {
            info = *infoPtr ;
        }
    }

    /* Fill up the transfer info structure */
    if (status == SYS_OK) 
	  {
        SEM_new (&(info->notifySemObj), 0) ;
    }

    /*
     *  Register notification for the event callback to get control and data
     *  buffer pointers from the GPP-side.
     */
    if (status == SYS_OK) 
	  {
        status = NOTIFY_register (ID_GPP,
                                  MPCSXFER_IPS_ID,
                                  MPCSXFER_IPS_EVENTNO,
                                  (FnNotifyCbck) Task_notify,
                                  info) ;
        if (status != SYS_OK) 
		    {
            return status;
        }
    }

    /*
     *  Send notification to the GPP-side that the application has completed its
     *  setup and is ready for further execution.
     */
    if (status == SYS_OK) 
	  {
        status = NOTIFY_notify (ID_GPP,
                                MPCSXFER_IPS_ID,
                                MPCSXFER_IPS_EVENTNO,
                                (Uint32) canny_edge_INIT);
        if (status != SYS_OK) 
		    {
            return status;
        }
    }

    /*
     *  Wait for the event callback from the GPP-side to post the semaphore
     *  indicating receipt of the data buffer pointers and sizes.
     */
    SEM_pend (&(info->notifySemObj), SYS_FOREVER) ;

    return status ;
}

Void Task_writeback(Void)
{
  Uint32 i;
  unsigned char *buf = (unsigned char *)dsp_buffers[0][0];

  /* Invalidate cache */
  BCACHE_inv (dsp_buffers[0][0], buffer_sizes[0], TRUE);

  /* Add 1 to each pixel */
  for(i = 0; i < buffer_sizes[0]; i++) {
    buf[i]++;
  }

  /* Write back and invalidate */
  BCACHE_wbInv(dsp_buffers[0][0], buffer_sizes[0], TRUE);

  /* Notify the result */
  NOTIFY_notify(ID_GPP, MPCSXFER_IPS_ID, MPCSXFER_IPS_EVENTNO, canny_edge_WRITEBACK);
}

Void Task_magnitude(Void)
{
  Uint32 r, c, pos, sq1, sq2;
  short int *delta_x = (short int *)dsp_buffers[2][0];
  short int *delta_y = (short int *)dsp_buffers[3][0];
  int *rows = (int *) dsp_buffers[4][0];
  int *cols = (int *) dsp_buffers[5][0];
  short int *magnitude = (short int *)dsp_buffers[6][0];

   /* Invalidate cache */
  BCACHE_inv (dsp_buffers[2][0], buffer_sizes[2], TRUE);
  BCACHE_inv (dsp_buffers[3][0], buffer_sizes[3], TRUE);
  BCACHE_inv (dsp_buffers[4][0], buffer_sizes[4], TRUE);
  BCACHE_inv (dsp_buffers[5][0], buffer_sizes[5], TRUE);

  for(r=0,pos=0; r<*rows; r++)
  {
    for(c=0; c<*cols; c++,pos++)
    {
        sq1 = (int)delta_x[pos] * (int)delta_x[pos];
        sq2 = (int)delta_y[pos] * (int)delta_y[pos];
        magnitude[pos] = (short) sq1+sq2; //(0.5 + sqrt((float)sq1 + (float)sq2));
    }
  }

  /* Write back and invalidate */
  BCACHE_wbInv(dsp_buffers[6][0], buffer_sizes[6], TRUE);

  /* Notify the result */
  NOTIFY_notify(ID_GPP, MPCSXFER_IPS_ID, MPCSXFER_IPS_EVENTNO, canny_edge_MAGNITUDE);
}

Void Task_derivative(Void)
{
  Uint32 r, c, pos;
  short int *smoothedim = (short int *)dsp_buffers[1][0];
  short int *delta_x = (short int *)dsp_buffers[2][0];
  short int *delta_y = (short int *)dsp_buffers[3][0];
  int *rows = (int *) dsp_buffers[4][0];
  int *cols = (int *) dsp_buffers[5][0];

  /* Invalidate cache */
  BCACHE_inv (dsp_buffers[1][0], buffer_sizes[1], TRUE);
  BCACHE_inv (dsp_buffers[4][0], buffer_sizes[4], TRUE); // row
  BCACHE_inv (dsp_buffers[5][0], buffer_sizes[5], TRUE); // col


  /* Calculate the X direction */
  for(r=0; r < *rows; r++){
    pos = r * (*cols);
    (delta_x)[pos] = smoothedim[pos+1] - smoothedim[pos];
    pos++;
    for(c=1;c<(*cols-1); c++, pos++){
      (delta_x)[pos] = smoothedim[pos+1] - smoothedim[pos-1];
    }
    (delta_x)[pos] = smoothedim[pos] - smoothedim[pos-1];
  }

  /* Calculate the Y direction */
  for(c=0; c < *cols; c++){
    pos = c;
    (delta_y)[pos] = smoothedim[pos+ *cols] - smoothedim[pos];
    pos += (*cols);
    for(r=1; r < (*rows-1); r++, pos += *cols){
      (delta_y)[pos] = smoothedim[pos+*cols] - smoothedim[pos - *cols];
    }
    (delta_y)[pos] = smoothedim[pos] - smoothedim[pos - *cols];
  }

  /* Write back and invalidate */
  BCACHE_wbInv(dsp_buffers[2][0], buffer_sizes[2], TRUE);
  BCACHE_wbInv(dsp_buffers[3][0], buffer_sizes[3], TRUE);

  /* Notify the GPP that DSP is done */
  NOTIFY_notify(ID_GPP, MPCSXFER_IPS_ID, MPCSXFER_IPS_EVENTNO, canny_edge_DERIVATIVE);

}

Int Task_execute (Task_TransferInfo * info)
{
  /* Wait for all tasks to be completed */
  SEM_pend (&(info->notifySemObj), SYS_FOREVER);

  return SYS_OK;
}

Int Task_delete (Task_TransferInfo * info)
{
    Int    status     = SYS_OK ;
    /*
     *  Unregister notification for the event callback used to get control and
     *  data buffer pointers from the GPP-side.
     */
    status = NOTIFY_unregister (ID_GPP,
                                MPCSXFER_IPS_ID,
                                MPCSXFER_IPS_EVENTNO,
                                (FnNotifyCbck) Task_notify,
                                info) ;

    /* Free the info structure */
    MEM_free (DSPLINK_SEGID,
              info,
              sizeof (Task_TransferInfo)) ;
    info = NULL ;

    return status ;
}


static Void Task_notify (Uint32 eventNo, Ptr arg, Ptr info)
{
    static Uint16 pool_cnt = 0;
    static Uint16 buffer_cnt = 0;
    static Uint8 got_address = 0;
    Task_TransferInfo * mpcsInfo = (Task_TransferInfo *) arg;
    (void) eventNo; // Avoid warning

    // Check if we received all the pool information
    if(pool_cnt < NUM_BUF_SIZES) {
      
      // We received an address
      if(got_address == 0) {
        dsp_buffers[pool_cnt][buffer_cnt] = info;
        got_address = 1;
      }

      // We received the buffer size
      else if(got_address == 1) {
        buffer_sizes[pool_cnt] = (int)info;
        got_address = 0;

        // Go to the next pool or next buffer
        if(buffer_cnt >= (pool_sizes[pool_cnt] - 1)) {
          pool_cnt++;
          buffer_cnt = 0;
        } else {
          buffer_cnt++;
        }
      }
    }

    // Check we if received the last buffer
    if(pool_cnt == NUM_BUF_SIZES) {
      SEM_post(&(mpcsInfo->notifySemObj));
      pool_cnt++;
    } 
    else if(pool_cnt > NUM_BUF_SIZES) { 

      // Execute a task
      if((Uint32)info == canny_edge_DELETE) {
        SEM_post(&(mpcsInfo->notifySemObj));
      }
      else if((Uint32)info == canny_edge_WRITEBACK) {
        Task_writeback();
      }
      else if((Uint32)info == canny_edge_MAGNITUDE) {
        Task_magnitude();
      }
      else if((Uint32)info == canny_edge_DERIVATIVE) {
        Task_derivative();
      }
    }   
}
