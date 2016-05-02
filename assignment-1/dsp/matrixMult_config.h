/** ============================================================================
 *  @file   matrixMult_config.h
 *
 *  @path   
 *
 *  @desc   Header file for MSGQ and POOL configurations for matrixMult.
 *
 *  @ver    1.10
 *  ============================================================================
 *  Copyright (c) Texas Instruments Incorporated 2002-2009
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *  ============================================================================
 */


#if !defined (MATRIXMULT_CONFIG_)
#define MATRIXMULT_CONFIG_

#if defined (__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */

/*  ----------------------------------- DSP/BIOS Headers            */
#include "matrixMultcfg.h"
#include <msgq.h>
#include <pool.h>

/*  ----------------------------------- DSP/BIOS LINK Headers       */
#include <dsplink.h>

#if defined (MSGQ_ZCPY_LINK)
#include <zcpy_mqt.h>
#include <sma_pool.h>
#endif /* if defined (MSGQ_ZCPY_LINK) */

/* Name of the MSGQ on the GPP and on the DSP. */
#define GPP_MSGQNAME        "GPPMSGQ1"
#define DSP_MSGQNAME        "DSPMSGQ"

/* ID of the POOL used by matrixMult. */
#define SAMPLE_POOL_ID      0

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

/* Number of pools configured in the system. */
#define NUM_POOLS          1

/* Number of local message queues */
#define NUM_MSG_QUEUES     1

/* Number of BUF pools in the entire memory pool */
#define NUM_MSG_POOLS      4

/* Number of messages in each BUF pool. */
#define NUM_MSG_IN_POOL0   3
#define NUM_MSG_IN_POOL1   2
#define NUM_MSG_IN_POOL2   2
#define NUM_MSG_IN_POOL3   4


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
#endif /* !defined (MATRIXMULT_CONFIG_) */
