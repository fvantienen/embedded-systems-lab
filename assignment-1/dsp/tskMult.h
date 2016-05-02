/** ============================================================================
 *  @file   tskMessage.h
 *
 *  @path   
 *
 *  @desc   This is simple TSK based application that uses MSGQ. It receives
 *          and transmits messages from/to the GPP and runs the DSP
 *          application code (located in an external source file)
 *
 *  @ver    1.10
 *  ============================================================================
 *  Copyright (c) Texas Instruments Incorporated 2002-2009
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *  ============================================================================
 */


#ifndef TSKMULT_
#define TSKMULT_


/*  ----------------------------------- DSP/BIOS Headers            */
#include <msgq.h>
#include <sem.h>


#ifdef __cplusplus
extern "C" {
#endif


/** ============================================================================
 *  @name   TSKMULT_TransferInfo
 *
 *  @desc   Structure used to keep various information needed by various phases
 *          of the application.
 *
 *  @field  matrixSize
 *              The matrix size
 *  @field  localMsgq
 *              Handle of opened message queue.
 *  @field  locatedMsgqHandle
 *              Handle to located message queue.
 *  @field  notifySemObj
 *              Semaphore used for message notification.
 *  ============================================================================
 */
typedef struct TSKMULT_TransferInfo_tag {
    Uint16     matrixSize;
    MSGQ_Queue localMsgq;
    MSGQ_Queue locatedMsgq;
    SEM_Obj    notifySemObj;
} TSKMULT_TransferInfo;


/** ============================================================================
 *  @func   TSKMULT_create
 *
 *  @desc   Create phase function of TSKMULT application.
 *
 *  @arg    transferInfo
 *              Information for transfer.
 *
 *  @ret    SYS_OK
 *              Successful operation.
 *          SYS_EBADIO
 *              Failure occured while doing IO.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    None
 *  ============================================================================
 */
Int TSKMULT_create(TSKMULT_TransferInfo** transferInfo);

/** ============================================================================
 *  @func   TSKMULT_execute
 *
 *  @desc   Excecute phase function of TSKMULT application.
 *
 *  @arg    transferInfo
 *              Information for transfer.
 *
 *  @ret    SYS_OK
 *              Successful operation.
 *          SYS_EBADIO
 *              Failure occured while doing IO.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    None
 *  ============================================================================
 */
Int TSKMULT_execute(TSKMULT_TransferInfo* transferInfo);

/** ============================================================================
 *  @func   TSKMULT_delete
 *
 *  @desc   Delete phase function of TSKMULT application.
 *
 *  @arg    transferInfo
 *              Information for transfer.
 *
 *  @ret    SYS_OK
 *              Successful operation.
 *          SYS_EBADIO
 *              Failure occured while doing IO.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    None
 *  ============================================================================
 */
Int TSKMULT_delete(TSKMULT_TransferInfo* transferInfo);


#ifdef __cplusplus
}
#endif /* extern "C" */


#endif /* TSKMULT_ */
