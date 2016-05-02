/** ============================================================================
 *  @file   tskMult.c
 *
 *  @path   
 *
 *  @desc   This is simple TSK based application that uses MSGQ. It receives
 *          and transmits messages from/to the GPP and runs the DSP
 *          application code (located in an external source file)
 *
 *  @ver    1.10
 */


/*  ----------------------------------- DSP/BIOS Headers            */
#include "matrixMultcfg.h"
#include <gbl.h>
#include <sys.h>
#include <sem.h>
#include <msgq.h>
#include <pool.h>

/*  ----------------------------------- DSP/BIOS LINK Headers       */
#include <dsplink.h>
#include <platform.h>
#include <failure.h>

/*  ----------------------------------- Sample Headers              */
#include <matrixMult_config.h>
#include <tskMult.h>

#ifdef __cplusplus
extern "C" {
#endif


/* FILEID is used by SET_FAILURE_REASON macro. */
#define FILEID  FID_APP_C

/* Place holder for the MSGQ name created on DSP */
Uint8 dspMsgQName[DSP_MAX_STRLEN];

/* The matrix size */
extern Uint16 matrix_size;


/** ============================================================================
 *  @func   TSKMULT_create
 *
 *  @desc   Create phase function for the TSKMULT application. Initializes
 *          the TSKMULT_TransferInfo structure with the information that will
 *          be used by the other phases of the application.
 *
 *  @modif  None.
 *  ============================================================================
 */
Int TSKMULT_create(TSKMULT_TransferInfo** infoPtr)
{
    Int status = SYS_OK;
    MSGQ_Attrs msgqAttrs = MSGQ_ATTRS;
    TSKMULT_TransferInfo* info = NULL;
    MSGQ_LocateAttrs syncLocateAttrs;

    /* Allocate TSKMULT_TransferInfo structure that will be initialized
     * and passed to other phases of the application */
    *infoPtr = MEM_calloc(DSPLINK_SEGID, sizeof(TSKMULT_TransferInfo), DSPLINK_BUF_ALIGN);
    if (*infoPtr == NULL)
    {
        status = SYS_EALLOC;
        SET_FAILURE_REASON(status);
    }
    else
    {
        info = *infoPtr;
        info->matrixSize = matrix_size;
        info->localMsgq = MSGQ_INVALIDMSGQ;
        info->locatedMsgq = MSGQ_INVALIDMSGQ;
    }

    if (status == SYS_OK)
    {
        /* Set the semaphore to a known state. */
        SEM_new(&(info->notifySemObj), 0);

        /* Fill in the attributes for this message queue. */
        msgqAttrs.notifyHandle = &(info->notifySemObj);
        msgqAttrs.pend = (MSGQ_Pend) SEM_pendBinary;
        msgqAttrs.post = (MSGQ_Post) SEM_postBinary;

        SYS_sprintf((Char *)dspMsgQName, "%s%d", DSP_MSGQNAME, GBL_getProcId());

        /* Creating message queue */
        status = MSGQ_open((String)dspMsgQName, &info->localMsgq, &msgqAttrs);
        if (status != SYS_OK)
        {
            SET_FAILURE_REASON(status);
        }
        else
        {
            /* Set the message queue that will receive any async. errors. */
            MSGQ_setErrorHandler(info->localMsgq, SAMPLE_POOL_ID);

            /* Synchronous locate.                           */
            /* Wait for the initial startup message from GPP */
            status = SYS_ENOTFOUND;
            while ((status == SYS_ENOTFOUND) || (status == SYS_ENODEV))
            {
                syncLocateAttrs.timeout = SYS_FOREVER;
                status = MSGQ_locate(GPP_MSGQNAME, &info->locatedMsgq, &syncLocateAttrs);
                if ((status == SYS_ENOTFOUND) || (status == SYS_ENODEV))
                {
                    TSK_sleep(1000);
                }
                else if(status != SYS_OK)
                {
#if !defined (LOG_COMPONENT)
                    LOG_printf(&trace, "MSGQ_locate (msgqOut) failed. Status = 0x%x\n", status);
#endif
                }
            }
        }
    }

    return status;
}


/** ============================================================================
 *  @func   TSKMULT_execute
 *
 *  @desc   Execute phase function for the TSKMULT application. Application
 *          receives a message, verifies the id and executes the DSP processing.
 *
 *  @modif  None.
 *  ============================================================================
 */
Int TSKMULT_execute(TSKMULT_TransferInfo* info)
{
    Int status = SYS_OK;
    ControlMsg *msg, *first_message, *ret_matrix;
    int i, j, k;

    /* Allocate the result message */
    status = MSGQ_alloc(SAMPLE_POOL_ID, (MSGQ_Msg*) &ret_matrix, APP_BUFFER_SIZE);
    if (status != SYS_OK)
    {
        SET_FAILURE_REASON(status);
        return status;
    }

    // Set the message information
    MSGQ_setMsgId((MSGQ_Msg) ret_matrix, 0x3);
    MSGQ_setSrcQueue((MSGQ_Msg) ret_matrix, info->localMsgq);

    do {
        /* Receive a message from the GPP */
        status = MSGQ_get(info->localMsgq,(MSGQ_Msg*) &msg, SYS_FOREVER);
        if (status != SYS_OK)
        {
            SET_FAILURE_REASON (status);
            return status;
        }

        /* Check if the message is an asynchronous error message */
        if (MSGQ_getMsgId((MSGQ_Msg) msg) == MSGQ_ASYNCERRORMSGID)
        {
#if !defined (LOG_COMPONENT)
            LOG_printf(&trace, "Transport error Type = %d",((MSGQ_AsyncErrorMsg *) msg)->errorType);
#endif
            /* Must free the message */
            MSGQ_free((MSGQ_Msg) msg);
            status = SYS_EBADIO;
            SET_FAILURE_REASON(status);
        }
        /* Got the first matrix */
        else if(MSGQ_getMsgId((MSGQ_Msg) msg) == 0x1)
        {
            first_message = msg;
        }
        /* Got the second matrix thus do the calculation */
        if (MSGQ_getMsgId((MSGQ_Msg) msg) == 0x2)
        {
            /* Do the matrix calculation */
            for (i = 0;i < matrix_size; i++)
            {
                for (j = 0; j < matrix_size; j++)
                {
                    ret_matrix->matrix[i][j]=0;
                    for(k=0; k < matrix_size; k++)
                        ret_matrix->matrix[i][j] = ret_matrix->matrix[i][j] + first_message->matrix[i][k] * msg->matrix[k][j];
                }
            }

            /* Send the message back to the GPP */
            status = MSGQ_put(info->locatedMsgq,(MSGQ_Msg) ret_matrix);
            if (status != SYS_OK)
            {
                MSGQ_free((MSGQ_Msg) ret_matrix);
                SET_FAILURE_REASON(status);
            }

            /* Must free the first and second message */
            MSGQ_free((MSGQ_Msg) first_message);
            MSGQ_free((MSGQ_Msg) msg);

            return status;
        }
    } while(status == SYS_OK);

    return status;
}


/** ============================================================================
 *  @func   TSKMULT_delete
 *
 *  @desc   Delete phase function for the TSKMULT application. It deallocates
 *          all the resources of allocated during create phase of the
 *          application.
 *
 *  @modif  None.
 *  ============================================================================
 */
Int TSKMULT_delete(TSKMULT_TransferInfo* info)
{
    Int status = SYS_OK;
    Int tmpStatus = SYS_OK;
    Bool freeStatus = FALSE;

    /* Release the located message queue */
    if (info->locatedMsgq != MSGQ_INVALIDMSGQ)
    {
        status = MSGQ_release(info->locatedMsgq);
        if (status != SYS_OK)
        {
            SET_FAILURE_REASON(status);
        }
    }

     /* Reset the error handler before deleting the MSGQ that receives */
     /* the error messages.                                            */
    MSGQ_setErrorHandler(MSGQ_INVALIDMSGQ, POOL_INVALIDID);

    /* Delete the message queue */
    if (info->localMsgq != MSGQ_INVALIDMSGQ)
    {
        tmpStatus = MSGQ_close(info->localMsgq);
        if ((status == SYS_OK) && (tmpStatus != SYS_OK))
        {
            status = tmpStatus;
            SET_FAILURE_REASON(status);
        }
    }

    /* Free the info structure */
    freeStatus = MEM_free(DSPLINK_SEGID, info, sizeof(TSKMULT_TransferInfo));
    if ((status == SYS_OK) && (freeStatus != TRUE))
    {
        status = SYS_EFREE;
        SET_FAILURE_REASON(status);
    }
    return status;
}


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
