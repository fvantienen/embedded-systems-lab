/** ============================================================================
 *  @file   matrixMult.h
 *
 *  @path
 *
 *  @desc   It multiplies a matrix by the use of a DSP
 *
 *  @ver    1.10
 *  ============================================================================
 *  Copyright (c) Texas Instruments Incorporated 2002-2009
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *  ============================================================================
 */


#if !defined (MATRIXMULT_H)
#define MATRIXMULT_H


#if defined (__cplusplus)
extern "C"
{
#endif /* defined (__cplusplus) */


    /** ============================================================================
     *  @func   matrixMult_Create
     *
     *  @desc   This function allocates and initializes resources used by
     *          this application.
     *
     *  @arg    dspExecutable
     *              DSP executable name.
     *  @arg    strNumIterations
     *              Number of iterations for which a message is transferred between
     *              GPP and DSP in string format.
     *  @arg    processorId
     *             Id of the DSP Processor.
     *
     *  @ret    DSP_SOK
     *              Operation successfully completed.
     *          DSP_EFAIL
     *              Resource allocation failed.
     *
     *  @enter  None
     *
     *  @leave  None
     *
     *  @see    matrixMult_Delete
     *  ============================================================================
     */
    NORMAL_API DSP_STATUS matrixMult_Create (IN Char8* dspExecutable, IN Char8* strNumIterations, IN Uint8 processorId);


    /** ============================================================================
     *  @func   matrixMult_Execute
     *
     *  @desc   This function implements the execute phase for this application.
     *
     *  @arg    numIterations
     *              Number of times to send the message to the DSP.
     *
     *  @arg    processorId
     *             Id of the DSP Processor.
     *
     *  @ret    DSP_SOK
     *              Operation successfully completed.
     *          DSP_EFAIL
     *              matrixMult execution failed.
     *
     *  @enter  None
     *
     *  @leave  None
     *
     *  @see    matrixMult_Delete , matrixMult_Create
     *  ============================================================================
     */
    NORMAL_API DSP_STATUS matrixMult_Execute(IN Uint32 numIterations, IN Uint8 processorId);


    /** ============================================================================
     *  @func   matrixMult_Delete
     *
     *  @desc   This function releases resources allocated earlier by call to
     *          matrixMult_Create ().
     *          During cleanup, the allocated resources are being freed
     *          unconditionally. Actual applications may require stricter check
     *          against return values for robustness.
     *
     *  @arg    processorId
     *             Id of the DSP Processor.
     *
     *  @ret    DSP_SOK
     *              Operation successfully completed.
     *          DSP_EFAIL
     *              Resource deallocation failed.
     *
     *  @enter  None
     *
     *  @leave  None
     *
     *  @see    matrixMult_Create
     *  ============================================================================
     */
    NORMAL_API Void matrixMult_Delete(IN Uint8 processorId);


    /** ============================================================================
     *  @func   matrixMult_Main
     *
     *  @desc   The OS independent driver function for the matrixMult sample
     *          application.
     *
     *  @arg    dspExecutable
     *              Name of the DSP executable file.
     *  @arg    strNumIterations
     *              Number of iterations a message is transferred between
     *              GPP and DSP in string format.
     *  @arg    strProcessorId
     *             Id of the DSP Processor in string format.
     *
     *
     *  @ret    None
     *
     *  @enter  None
     *
     *  @leave  None
     *
     *  @see    matrixMult_Create, matrixMult_Execute, matrixMult_Delete
     *  ============================================================================
     */
    NORMAL_API Void matrixMult_Main(IN Char8* dspExecutable, IN Char8* strNumIterations, IN Char8* strProcessorId);


#if defined (DA8XXGEM)
    /** ============================================================================
     *  @func   matrixMult_Main_DA8XX
     *
     *  @desc   The OS independent driver function for the matrixMult sample
     *          application.
     *
     *  @arg    dspExecutable
     *              Name of the DSP executable file.
     *  @arg    strNumIterations
     *              Number of iterations a message is transferred between
     *              GPP and DSP in string format.
     *  @arg    strProcessorId
     *             Id of the DSP Processor in string format.
     *  @arg    strDspAddr
     *              c_int00 address
     *  @arg    strShmAddr
     *              DSPLINK_shmBaseAddress address
     *  @arg    strArgsAddr
     *              .args section address
     *
     *  @ret    None
     *
     *  @enter  None
     *
     *  @leave  None
     *
     *  @see    matrixMult_Create, matrixMult_Execute, matrixMult_Delete
     *  ============================================================================
     */
    NORMAL_API Void matrixMult_Main_DA8XX(IN Char8* dspExecutable, IN Char8* strNumIterations, IN Char8* strProcessorId, IN Char8* strDspAddr, IN Char8* strShmAddr, IN Char8* strArgsAddr);
#endif

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */


#endif /* !defined (MATRIXMULT_H) */
