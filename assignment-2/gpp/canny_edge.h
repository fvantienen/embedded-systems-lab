#if !defined (canny_edge_H)
#define canny_edge_H

/*  ----------------------------------- DSP/BIOS Link                 */
#include <dsplink.h>

extern int gaussianPerc, derivativePerc, magnitudePerc;

/** ============================================================================
 *  @const  ID_PROCESSOR
 *
 *  @desc   The processor id of the processor being used.
 *  ============================================================================
 */
#define ID_PROCESSOR       0


/** ============================================================================
 *  @func   canny_edge_Create
 *
 *  @desc   This function allocates and initializes resources used by
 *          this application.
 *
 *  @arg    dspExecutable
 *              DSP executable name.
 *  @arg    strImage
 *              Path to the image that the canny edge must be applied to
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
 *  @see    canny_edge_Delete
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
canny_edge_Create (IN Char8 * dspExecutable,
                   IN Char8 * strImage,
                   IN Uint8   processorId) ;


/** ============================================================================
 *  @func   canny_edge_Execute
 *
 *  @desc   This function implements the execute phase for this application.
 *
 *  @arg    processorId
 *             Id of the DSP Processor. 
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              MESSAGE execution failed.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    canny_edge_Delete , canny_edge_Create
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
canny_edge_Execute (IN Uint8 processorId, IN Char8 * strImage) ;


/** ============================================================================
 *  @func   canny_edge_Delete
 *
 *  @desc   This function releases resources allocated earlier by call to
 *          canny_edge_Create ().
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
 *  @see    pool_notify_Create
 *  ============================================================================
 */
NORMAL_API
Void
canny_edge_Delete (IN Uint8 processorId) ;


/** ============================================================================
 *  @func   canny_edge_Main
 *
 *  @desc   The OS independent driver function for the canny edge detector
 *
 *  @arg    dspExecutable
 *              Name of the DSP executable file.
 *  @arg    strImage
 *              The PGM image that is used for canny edge
 *  @arg    strProcessorId
 *             ID of the DSP Processor in string format. 
 *
 *  @ret    None
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    canny_edge_Create, canny_edge_Execute, canny_edge_Delete
 *  ============================================================================
 */
NORMAL_API
Void
canny_edge_Main (IN Char8 * dspExecutable,
               IN Char8 * strBufferSize) ;


#endif /* !defined (canny_edge_H) */
