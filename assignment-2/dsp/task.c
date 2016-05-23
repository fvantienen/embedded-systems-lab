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
#define NUM_BUF_SIZES                    1 ///< Amount of pools to be configured
#define NUM_BUF_POOL0                    1 ///< Amount of buffers in the first pool
#define NUM_BUF_MAX                      1 ///< Maximum amount of buffers in pool

Uint16 pool_sizes[] = {NUM_BUF_POOL0, };
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
                                (Uint32) 0) ; /* No payload to be sent. */
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

Int Task_execute (Task_TransferInfo * info)
{
  int i;
  unsigned char *buf = (unsigned char *)dsp_buffers[0][0];

  //wait for semaphore
	  SEM_pend (&(info->notifySemObj), SYS_FOREVER);

	//invalidate cache
    BCACHE_inv (dsp_buffers[0][0], buffer_sizes[0], TRUE);

    for(i = 0; i < buffer_sizes[0]; i++) {
      buf[i]++;
    }

  // Write back and invalidate
    BCACHE_wbInv(dsp_buffers[0][0], buffer_sizes[0], TRUE);

	//notify the result
    NOTIFY_notify(ID_GPP, MPCSXFER_IPS_ID, MPCSXFER_IPS_EVENTNO, (Uint32)10);

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
    if(pool_cnt >= NUM_BUF_SIZES) {
      SEM_post(&(mpcsInfo->notifySemObj));
    }    
}
