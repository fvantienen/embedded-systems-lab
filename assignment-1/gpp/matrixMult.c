/** ============================================================================
 *  @file   matrixMult.c
 *
 *  @path
 *
 *  @desc   This is an application which 2 matrics to the DSP
 *          processor and receives the multiplciat back using
 *          DSP/BIOS LINK.
 *
*/
/*  ----------------------------------- DSP/BIOS Link                   */
#include <dsplink.h>

/*  ----------------------------------- DSP/BIOS LINK API               */
#include <proc.h>
#include <msgq.h>
#include <pool.h>

/*  ----------------------------------- Application Header              */
#include <matrixMult.h>
#include <system_os.h>

#include <stdio.h>


#if defined (__cplusplus)
extern "C"
{
#endif /* defined (__cplusplus) */

    /* Number of arguments specified to the DSP application. */
#define NUM_ARGS 1

    /* ID of the POOL used by matrixMult. */
#define SAMPLE_POOL_ID  0

    /*  Number of BUF pools in the entire memory pool */
#define NUMMSGPOOLS     4

    /* Number of messages in each BUF pool. */
#define NUMMSGINPOOL0   3
#define NUMMSGINPOOL1   2
#define NUMMSGINPOOL2   2
#define NUMMSGINPOOL3   4

    /* Matrix size */
#define MATRIX_SIZE 100

    /* Control message data structure. */
    /* Must contain a reserved space for the header */
    typedef struct ControlMsg
    {
        MSGQ_MsgHeader header;
        int matrix[MATRIX_SIZE][MATRIX_SIZE];
    } ControlMsg;

    /* Messaging buffer used by the application.
     * Note: This buffer must be aligned according to the alignment expected
     * by the device/platform. */
#define APP_BUFFER_SIZE DSPLINK_ALIGN (sizeof (ControlMsg), DSPLINK_BUF_ALIGN)

    /* Definitions required for the sample Message queue.
     * Using a Zero-copy based transport on the shared memory physical link. */
#if defined ZCPY_LINK
#define SAMPLEMQT_CTRLMSG_SIZE  ZCPYMQT_CTRLMSG_SIZE
    STATIC ZCPYMQT_Attrs mqtAttrs;
#endif

    /* Message sizes managed by the pool */
    STATIC Uint32 SampleBufSizes[NUMMSGPOOLS] =
    {
        APP_BUFFER_SIZE,
        SAMPLEMQT_CTRLMSG_SIZE,
        DSPLINK_ALIGN (sizeof(MSGQ_AsyncLocateMsg), DSPLINK_BUF_ALIGN),
        DSPLINK_ALIGN (sizeof(MSGQ_AsyncErrorMsg), DSPLINK_BUF_ALIGN)
    };

    /* Number of messages in each pool */
    STATIC Uint32 SampleNumBuffers[NUMMSGPOOLS] =
    {
        NUMMSGINPOOL0,
        NUMMSGINPOOL1,
        NUMMSGINPOOL2,
        NUMMSGINPOOL3
    };

    /* Definition of attributes for the pool based on physical link used by the transport */
#if defined ZCPY_LINK
    STATIC SMAPOOL_Attrs SamplePoolAttrs =
    {
        NUMMSGPOOLS,
        SampleBufSizes,
        SampleNumBuffers,
        TRUE   /* If allocating a buffer smaller than the POOL size, set this to FALSE */
    };
#endif

    /* Name of the first MSGQ on the GPP and on the DSP. */
    STATIC Char8 SampleGppMsgqName[DSP_MAX_STRLEN] = "GPPMSGQ1";
    STATIC Char8 SampleDspMsgqName[DSP_MAX_STRLEN] = "DSPMSGQ";

    /* Local GPP's and DSP's MSGQ Objects. */
    STATIC MSGQ_Queue SampleGppMsgq = (Uint32) MSGQ_INVALIDMSGQ;
    STATIC MSGQ_Queue SampleDspMsgq = (Uint32) MSGQ_INVALIDMSGQ;

    /* Place holder for the MSGQ name created on DSP */
    Char8 dspMsgqName[DSP_MAX_STRLEN];

    /* Extern declaration to the default DSP/BIOS LINK configuration structure. */
    extern LINKCFG_Object LINKCFG_config;

#if defined (VERIFY_DATA)
    /** ============================================================================
     *  @func   matrixMult_VerifyData
     *
     *  @desc   This function verifies the data-integrity of given message.
     *  ============================================================================
     */
    STATIC NORMAL_API DSP_STATUS matrixMult_VerifyData(ControlMsg *matrix1, ControlMsg *matrix2, ControlMsg *out, IN Uint32 matrixSize);
#endif


    /** ============================================================================
     *  @func   matrixMult_Create
     *
     *  @desc   This function allocates and initializes resources used by
     *          this application.
     *
     *  @modif  matrixMult_InpBufs , matrixMult_OutBufs
     *  ============================================================================
     */
    NORMAL_API DSP_STATUS matrixMult_Create(IN Char8* dspExecutable, IN Char8* strMatrixSize, IN Uint8 processorId)
    {
        DSP_STATUS status = DSP_SOK;
        Uint32 numArgs = NUM_ARGS;
        MSGQ_LocateAttrs syncLocateAttrs;
        Char8* args[NUM_ARGS];

        SYSTEM_0Print("Entered matrixMult_Create ()\n");

        /* Create and initialize the proc object. */
        status = PROC_setup(NULL);

        /* Attach the Dsp with which the transfers have to be done. */
        if (DSP_SUCCEEDED(status))
        {
            status = PROC_attach(processorId, NULL);
            if (DSP_FAILED(status))
            {
                SYSTEM_1Print("PROC_attach () failed. Status = [0x%x]\n", status);
            }
        }

        /* Open the pool. */
        if (DSP_SUCCEEDED(status))
        {
            status = POOL_open(POOL_makePoolId(processorId, SAMPLE_POOL_ID), &SamplePoolAttrs);
            if (DSP_FAILED(status))
            {
                SYSTEM_1Print("POOL_open () failed. Status = [0x%x]\n", status);
            }
        }
        else
        {
            SYSTEM_1Print("PROC_setup () failed. Status = [0x%x]\n", status);
        }

        /* Open the GPP's message queue */
        if (DSP_SUCCEEDED(status))
        {
            status = MSGQ_open(SampleGppMsgqName, &SampleGppMsgq, NULL);
            if (DSP_FAILED(status))
            {
                SYSTEM_1Print("MSGQ_open () failed. Status = [0x%x]\n", status);
            }
        }

        /* Set the message queue that will receive any async. errors */
        if (DSP_SUCCEEDED(status))
        {
            status = MSGQ_setErrorHandler(SampleGppMsgq, POOL_makePoolId(processorId, SAMPLE_POOL_ID));
            if (DSP_FAILED(status))
            {
                SYSTEM_1Print("MSGQ_setErrorHandler () failed. Status = [0x%x]\n", status);
            }
        }

        /* Load the executable on the DSP. */
        if (DSP_SUCCEEDED(status))
        {
            args [0] = strMatrixSize;
            {
                status = PROC_load(processorId, dspExecutable, numArgs, args);
            }
            if (DSP_FAILED(status))
            {
                SYSTEM_1Print("PROC_load () failed. Status = [0x%x]\n", status);
            }
        }

        /* Start execution on DSP. */
        if (DSP_SUCCEEDED(status))
        {
            status = PROC_start(processorId);
            if (DSP_FAILED(status))
            {
                SYSTEM_1Print("PROC_start () failed. Status = [0x%x]\n", status);
            }
        }

        /* Open the remote transport. */
        if (DSP_SUCCEEDED(status))
        {
            mqtAttrs.poolId = POOL_makePoolId(processorId, SAMPLE_POOL_ID);
            status = MSGQ_transportOpen(processorId, &mqtAttrs);
            if (DSP_FAILED(status))
            {
                SYSTEM_1Print("MSGQ_transportOpen () failed. Status = [0x%x]\n", status);
            }
        }

        /* Locate the DSP's message queue */
        /* At this point the DSP must open a message queue named "DSPMSGQ" */
        if (DSP_SUCCEEDED(status))
        {
            syncLocateAttrs.timeout = WAIT_FOREVER;
            status = DSP_ENOTFOUND;
            SYSTEM_2Sprint(dspMsgqName, "%s%d", (Uint32) SampleDspMsgqName, processorId);
            while ((status == DSP_ENOTFOUND) || (status == DSP_ENOTREADY))
            {
                status = MSGQ_locate(dspMsgqName, &SampleDspMsgq, &syncLocateAttrs);
                if ((status == DSP_ENOTFOUND) || (status == DSP_ENOTREADY))
                {
                    SYSTEM_Sleep(100000);
                }
                else if (DSP_FAILED(status))
                {
                    SYSTEM_1Print("MSGQ_locate () failed. Status = [0x%x]\n", status);
                }
            }
        }

        SYSTEM_0Print("Leaving matrixMult_Create ()\n");
        return status;
    }


    /** ============================================================================
     *  @func   matrixMult_Execute
     *
     *  @desc   This function implements the execute phase for this application.
     *
     *  @modif  None
     *  ============================================================================
     */
    NORMAL_API DSP_STATUS matrixMult_Execute(IN Uint32 matrixSize, Uint8 processorId)
    {
        DSP_STATUS  status = DSP_SOK;
        ControlMsg *in_matrix1, *in_matrix2, *out_matrix;
        int i, j;

        SYSTEM_0Print("Entered matrixMult_Execute ()\n");

        // Allocate the matrices
        status = MSGQ_alloc(SAMPLE_POOL_ID, APP_BUFFER_SIZE, (MSGQ_Msg*) &in_matrix1);
        if (DSP_FAILED(status)) {
            SYSTEM_0Print("Could not allocate first matrix!\n");
            return status;
        }

        status = MSGQ_alloc(SAMPLE_POOL_ID, APP_BUFFER_SIZE, (MSGQ_Msg*) &in_matrix2);
        if (DSP_FAILED(status)) {
            SYSTEM_0Print("Could not allocate second matrix!\n");
            return status;
        }

        // Setup first matrix
        MSGQ_setMsgId(in_matrix1, 0x1);
        for (i = 0; i < matrixSize; i++)
            for (j = 0; j < matrixSize; j++)
                in_matrix1->matrix[i][j] = i + j*2;

        // Setup second matrix
        MSGQ_setMsgId(in_matrix2, 0x2);
        for (i = 0; i < matrixSize; i++)
            for (j = 0; j < matrixSize; j++)
                in_matrix2->matrix[i][j] = i + j*3;

        /* Send the matrices */
        status = MSGQ_put(SampleDspMsgq, (MsgqMsg) in_matrix1);
        if (DSP_FAILED(status))
        {
            MSGQ_free((MsgqMsg) in_matrix1);
            SYSTEM_1Print("MSGQ_put () failed. Status = [0x%x]\n", status);
            return status;
        }

        status = MSGQ_put(SampleDspMsgq, (MsgqMsg) in_matrix2);
        if (DSP_FAILED(status))
        {
            MSGQ_free((MsgqMsg) in_matrix2);
            SYSTEM_1Print("MSGQ_put () failed. Status = [0x%x]\n", status);
            return status;
        }

#if defined (PROFILE)
        SYSTEM_GetStartTime();
#endif
        /* Receive the answer */
        status = MSGQ_get(SampleGppMsgq, WAIT_FOREVER, (MsgqMsg *) &out_matrix);
        if (DSP_FAILED(status))
        {
            SYSTEM_1Print("MSGQ_get () failed. Status = [0x%x]\n", status);
            return status;
        }

#if defined (PROFILE)
        SYSTEM_GetEndTime();
#endif

        /* Print the answer */
        SYSTEM_1Print("Answer(%d): \n", MSGQ_getMsgId((MSGQ_Msg) out_matrix));
        for (i = 0;i < matrixSize; i++)
        {
            for (j = 0; j < matrixSize; j++)
                SYSTEM_1Print("%d ", out_matrix->matrix[i][j]);
            SYSTEM_0Print("\n");
        }

#if defined (PROFILE)
        SYSTEM_GetProfileInfo(matrixSize);
#endif

#if defined (VERIFY_DATA)
        /* Verify correctness of data received. */
        status = matrixMult_VerifyData(in_matrix1, in_matrix2, out_matrix, matrixSize);
        if (DSP_FAILED(status))
        {
            SYSTEM_0Print("Verification succesfull!\n");
        }
        else
        {
            SYSTEM_0Print("Verification failed!\n");
        }
#endif

        MSGQ_free((MsgqMsg) out_matrix);
        SYSTEM_0Print("Leaving matrixMult_Execute ()\n");
        return status;
    }


    /** ============================================================================
     *  @func   matrixMult_Delete
     *
     *  @desc   This function releases resources allocated earlier by call to
     *          matrixMult_Create ().
     *          During cleanup, the allocated resources are being freed
     *          unconditionally. Actual applications may require stricter check
     *          against return values for robustness.
     *
     *  @modif  None
     *  ============================================================================
     */
    NORMAL_API Void matrixMult_Delete(Uint8 processorId)
    {
        DSP_STATUS status = DSP_SOK;
        DSP_STATUS tmpStatus = DSP_SOK;

        SYSTEM_0Print("Entered matrixMult_Delete ()\n");

        /* Release the remote message queue */
        status = MSGQ_release(SampleDspMsgq);
        if (DSP_FAILED(status))
        {
            SYSTEM_1Print("MSGQ_release () failed. Status = [0x%x]\n", status);
        }

        /* Close the remote transport */
        tmpStatus = MSGQ_transportClose(processorId);
        if (DSP_SUCCEEDED(status) && DSP_FAILED(tmpStatus))
        {
            status = tmpStatus;
            SYSTEM_1Print("MSGQ_transportClose () failed. Status = [0x%x]\n", status);
        }

        /* Stop execution on DSP. */
        tmpStatus = PROC_stop(processorId);
        if (DSP_SUCCEEDED(status) && DSP_FAILED(tmpStatus))
        {
            status = tmpStatus;
            SYSTEM_1Print("PROC_stop () failed. Status = [0x%x]\n", status);
        }

        /* Reset the error handler before deleting the MSGQ that receives */
        /* the error messages.                                            */
        tmpStatus = MSGQ_setErrorHandler(MSGQ_INVALIDMSGQ, MSGQ_INVALIDMSGQ);

        if (DSP_SUCCEEDED(status) && DSP_FAILED(tmpStatus))
        {
            status = tmpStatus;
            SYSTEM_1Print("MSGQ_setErrorHandler () failed. Status = [0x%x]\n", status);
        }

        /* Close the GPP's message queue */
        tmpStatus = MSGQ_close(SampleGppMsgq);
        if (DSP_SUCCEEDED(status) && DSP_FAILED(tmpStatus))
        {
            status = tmpStatus;
            SYSTEM_1Print("MSGQ_close () failed. Status = [0x%x]\n", status);
        }

        /* Close the pool */
        tmpStatus = POOL_close(POOL_makePoolId(processorId, SAMPLE_POOL_ID));
        if (DSP_SUCCEEDED(status) && DSP_FAILED(tmpStatus))
        {
            status = tmpStatus;
            SYSTEM_1Print("POOL_close () failed. Status = [0x%x]\n", status);
        }

        /* Detach from the processor */
        tmpStatus = PROC_detach(processorId);
        if (DSP_SUCCEEDED(status) && DSP_FAILED(tmpStatus))
        {
            status = tmpStatus;
            SYSTEM_1Print("PROC_detach () failed. Status = [0x%x]\n", status);
        }

        /* Destroy the PROC object. */
        tmpStatus = PROC_destroy();
        if (DSP_SUCCEEDED(status) && DSP_FAILED(tmpStatus))
        {
            status = tmpStatus;
            SYSTEM_1Print("PROC_destroy () failed. Status = [0x%x]\n", status);
        }

        SYSTEM_0Print("Leaving matrixMult_Delete ()\n");
    }


    /** ============================================================================
     *  @func   matrixMult_Main
     *
     *  @desc   Entry point for the application
     *
     *  @modif  None
     *  ============================================================================
     */
    NORMAL_API Void matrixMult_Main(IN Char8* dspExecutable, IN Char8* strMatrixSize, IN Char8* strProcessorId)
    {
        DSP_STATUS status = DSP_SOK;
        Uint32 matrixSize = 0;
        Uint8 processorId = 0;

        SYSTEM_0Print ("========== matrixMult: Matrix multiplication ==========\n");

        if ((dspExecutable != NULL) && (strMatrixSize != NULL))
        {
            matrixSize = SYSTEM_Atoi(strMatrixSize);

            if (matrixSize > 100 || matrixSize < 1)
            {
                status = DSP_EINVALIDARG;
                SYSTEM_2Print("ERROR! Invalid arguments specified for matrixMult application.\n Min matrix size is %d and max is %d\n", 1, 100);
            }
            else
            {
                processorId = SYSTEM_Atoi(strProcessorId);

                if (processorId >= MAX_DSPS)
                {
                    SYSTEM_1Print("== Error: Invalid processor id %d specified ==\n", processorId);
                    status = DSP_EFAIL;
                }
                /* Specify the dsp executable file name for message creation phase. */
                if (DSP_SUCCEEDED(status))
                {
                    status = matrixMult_Create(dspExecutable, strMatrixSize, processorId);

                    /* Execute the message execute phase. */
                    if (DSP_SUCCEEDED(status))
                    {
                        status = matrixMult_Execute(matrixSize, processorId);
                    }

                    /* Perform cleanup operation. */
                    matrixMult_Delete(processorId);
                }
            }
        }
        else
        {
            status = DSP_EINVALIDARG;
            SYSTEM_0Print("ERROR! Invalid arguments specified for matrixMult application\n");
        }
        SYSTEM_0Print ("====================================================\n");
    }

#if defined (VERIFY_DATA)
    /** ============================================================================
     *  @func   matrixMult_VerifyData
     *
     *  @desc   This function verifies the data-integrity of the matrix.
     *
     *  @modif  None
     *  ============================================================================
     */
    STATIC NORMAL_API DSP_STATUS matrixMult_VerifyData(ControlMsg *matrix1, ControlMsg *matrix2, ControlMsg *out, IN Uint32 matrixSize)
    {
        int i, j, k;
        DSP_STATUS status = DSP_SOK;

        for (i = 0;i < matrixSize; i++)
        {
            for (j = 0; j < matrixSize; j++)
            {
                for(k=0; k < matrixSize; k++)
                {
                    if(out->matrix[i][j] != (matrix1->matrix[i][k] * matrix2->matrix[k][j]))
                        status = DSP_EFAIL;
                }
            }
        }
        return status;
    }
#endif /* if defined (VERIFY_DATA) */


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
