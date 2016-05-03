/*  ----------------------------------- DSP/BIOS Headers            */
#include "matrixMultcfg.h"
#include <sys.h>
#include <sem.h>
#include <msgq.h>
#include <pool.h>

/*  ----------------------------------- DSP/BIOS LINK Headers       */
#include <dsplink.h>
#include <failure.h>

/*  ----------------------------------- Sample Headers              */
#include <tskMult.h>

/*  ----------------------------------- BSL Headers                 */
 
#ifdef __cplusplus
extern "C" {
#endif


/* FILEID is used by SET_FAILURE_REASON macro. */
#define FILEID  FID_APP_C

/* The size of the matrix */
Uint16 matrix_size;

/* Percentage to run */
Uint16 percentage;

/** ----------------------------------------------------------------------------
 *  @func   tskMessage
 *
 *  @desc   Task for TSK based TSKMULT application.
 */
static Int tskMult();

/** ============================================================================
 *  @func   atoi
 *
 *  @desc   Converts character string to integer value.
 *
 *  ============================================================================
 */
extern int atoi(const char* str);

/** ============================================================================
 *  @func   main
 *
 *  @desc   Entry function.
 *
 *  @modif  None
 *  ============================================================================
 */
Void main(Int argc, Char* argv [])
{
    /* Task handler for TSK_create */
    TSK_Handle tskMultTask;

#if !defined (DSP_BOOTMODE_NOBOOT)
    /* Get the size of the matrix to be calculated by the application */
    matrix_size = atoi(argv [0]);
    /* Get the percentage to calculate of this matrix */
    percentage = atoi(argv [1]);
    /* Initialize DSP/BIOS LINK. */
    DSPLINK_init();
#endif

    /* Creating task for TSKMULT application */
    tskMultTask = TSK_create(tskMult, NULL, 0);
    if (tskMultTask == NULL)
    {
        SET_FAILURE_REASON(SYS_EALLOC);
        LOG_printf(&trace, "Create TSKMULT: Failed.\n");
    }
}

/** ----------------------------------------------------------------------------
 *  @func   tskMult
 *
 *  @desc   Task for TSK based TSKMULT application.
 *
 *  @modif  None
 *  ----------------------------------------------------------------------------
 */
static Int tskMult()
{
    Int status = SYS_OK;
    TSKMULT_TransferInfo* info;

    /* Create Phase */
    status = TSKMULT_create(&info);

    /* Execute Phase */
    if (status == SYS_OK)
    {
        /* Start the execution phase. */
        status = TSKMULT_execute(info);
        if (status != SYS_OK)
        {
            SET_FAILURE_REASON(status);
        }
    }

    /* Delete Phase */
    status = TSKMULT_delete(info);
    if (status != SYS_OK)
    {
        SET_FAILURE_REASON(status);
    }
    return status;
}

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
