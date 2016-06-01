#include<stdlib.h>
#include<stdio.h>

#include <semaphore.h>
/*  ----------------------------------- DSP/BIOS Link                   */
#include <dsplink.h>

/*  ----------------------------------- DSP/BIOS LINK API               */
#include <proc.h>
#include <pool.h>
#include <mpcs.h>
#include <notify.h>
#if defined (DA8XXGEM)
#include <loaderdefs.h>
#endif


/*  ----------------------------------- Application Header              */
#include <canny_edge.h>
#include <math.h>
#include <string.h>
#include "pgm_io.h"
#include "hysteresis.h"
#include <arm_neon.h>


#if defined (__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */

/* Enable / Disable DSP/NEON */
//#define GAUSSIAN_DSP 1
//#define GUASSIAN_NEON 1
//#define DERIVATIVE_DSP 1
#define GAUSSIAN_DSP 1
/* Enable verbose printing by default */
#ifndef VERBOSE
#define VERBOSE 1
#endif
#define VPRINT if(VERBOSE) printf

/* Enable verify by default (makes it slower) */
#ifndef VERIFY
#define VERIFY 0
#endif

/* Pool and message defines */
#define SAMPLE_POOL_ID                   0 ///< Pool number used for data transfers
#define NUM_BUF_SIZES                    4 ///< Amount of pools to be configured
#define NUM_BUF_POOL0                    1 ///< Amount of buffers in the first pool
#define NUM_BUF_POOL1                    1 ///< Amount of buffers in the second pool
#define NUM_BUF_POOL2                    1 ///< Amount of buffers in the thrid pool
#define NUM_BUF_POOL3                    1 ///< Amount of buffers in the fourth pool
#define NUM_BUF_MAX                      1 ///< Maximum amount of buffers in pool
#define canny_edge_IPS_ID                0 ///< IPS ID used for sending notifications to the DPS
#define canny_edge_IPS_EVENTNO           5 ///< Event number used for notifications to the DSP

enum {
    canny_edge_INIT,                    ///< Initialization stage
    canny_edge_DELETE,                  ///< Shutdown step
    canny_edge_WRITEBACK,				///< Simple write back program
    canny_edge_GAUSSIAN,                ///< Calculate the Gaussian
    canny_edge_DERIVATIVE               ///< Calculate the derivatives
};

/* General variables */
sem_t sem;                                              ///< Semaphore used for synchronising events
unsigned char *canny_edge_image;                        ///< The canny edge input image
int canny_edge_rows, canny_edge_cols;                   ///< The canny edge input width and height
Uint32 pool_sizes[] = {NUM_BUF_POOL0, NUM_BUF_POOL1, NUM_BUF_POOL2, NUM_BUF_POOL3};   ///< The pool sizes
Uint32 buffer_sizes[NUM_BUF_SIZES];                     ///< The buffer sizes
Void *buffers[NUM_BUF_SIZES][NUM_BUF_MAX];              ///< The buffers
Void *dsp_buffers[NUM_BUF_SIZES][NUM_BUF_MAX];          ///< Buffer addresses on the DSP


/* Specific canny edge variables */
#define BOOSTBLURFACTOR 90.0
#define SIGMA 2.5
#define TLOW 0.5
#define THIGH 0.5

/* Used DSP functions */
STATIC Void canny_edge_Notify(Uint32 eventNo, Pvoid arg, Pvoid info);
STATIC Void canny_edge_Writeback(unsigned char *image, int rows, int cols, Uint8 processorId);
STATIC Void canny_edge_Gaussian(unsigned char *image, int rows, int cols, short int *smoothedim, Uint8 processorId);
STATIC Void canny_edge_Derivative(short int *smoothedim, int rows, int cols, short int *delta_x, short int *delta_y, Uint8 processorId);

/* Used neon functions */
STATIC void gaussian_smooth_neon(unsigned char *image, short int* smoothedim, Uint16 rows, Uint16 cols, float sigma);

/* Used GPP functions */
STATIC void gaussian_smooth(unsigned char *image, short int* smoothedim, int rows, int cols, float sigma);
STATIC void make_gaussian_kernel(float sigma, float **kernel, int *windowsize);
STATIC void derivative_x_y(short int *smoothedim, int rows, int cols, short int **delta_x, short int **delta_y);
STATIC void magnitude_x_y(short int *delta_x, short int *delta_y, int rows, int cols, short int *magnitude);
STATIC double angle_radians(double x, double y);


/** ============================================================================
 *  @func   canny_edge_Create
 *
 *  @desc   This function allocates and initializes resources used by
 *          this application.
 *
 *  @modif  None
 *  ============================================================================
 */
NORMAL_API DSP_STATUS canny_edge_Create (	IN Char8 * dspExecutable,
											IN Char8 * strImage,
											IN Uint8   processorId)
{
    DSP_STATUS      status     = DSP_SOK;
    SMAPOOL_Attrs   poolAttrs;
    Uint16          i, j;

    VPRINT("Entered canny_edge_Create ()\n") ;
    sem_init(&sem,0,0);

    /*
     *  Create and initialize the proc object.
     */
    status = PROC_setup (NULL) ;

    if (DSP_FAILED (status)) 
	{
        fprintf(stderr, "PROC_setup () failed. Status = [0x%x]\n", (int)status);
        return status;
    }

    /*
     *  Attach the Dsp with which the transfers have to be done.
     */
    status = PROC_attach (processorId, NULL) ;
    if (DSP_FAILED (status)) 
	{
        fprintf(stderr, "PROC_attach () failed. Status = [0x%x]\n", (int)status);
        return status;
    }

    /*
     *  Open the PGM image
     */
    VPRINT("Reading the image %s.\n", strImage);
    if(read_pgm_image(strImage, &canny_edge_image, &canny_edge_rows, &canny_edge_cols) == 0)
    {
        fprintf(stderr, "Error reading the input image, %s.\n", strImage);
        return DSP_EFAIL;
    }

    /* Set the buffer sizes based on image size */
    buffer_sizes[0] = DSPLINK_ALIGN(sizeof(unsigned char) * canny_edge_rows * canny_edge_cols, DSPLINK_BUF_ALIGN); //image
    buffer_sizes[1] = DSPLINK_ALIGN(sizeof(short int) * canny_edge_rows * canny_edge_cols, DSPLINK_BUF_ALIGN); //smoothedim
    buffer_sizes[2] = DSPLINK_ALIGN(sizeof(short int) * canny_edge_rows * canny_edge_cols, DSPLINK_BUF_ALIGN); //delta_x
    buffer_sizes[3] = DSPLINK_ALIGN(sizeof(short int) * canny_edge_rows * canny_edge_cols, DSPLINK_BUF_ALIGN); //delta_y

    /*
     *  Open the pool.
     */
    poolAttrs.bufSizes      = (Uint32 *) &buffer_sizes ;
    poolAttrs.numBuffers    = (Uint32 *) &pool_sizes ;
    poolAttrs.numBufPools   = NUM_BUF_SIZES ;
    poolAttrs.exactMatchReq = TRUE ;
    status = POOL_open (POOL_makePoolId(processorId, SAMPLE_POOL_ID), &poolAttrs) ;
    if (DSP_FAILED (status)) 
	{
        fprintf(stderr, "POOL_open () failed. Status = [0x%x]\n", (int)status);
        return status;
    }
    else {
        VPRINT("POOL_open () successfull!\n");
    }

    /*
     *  Go through all buffers to initialize them
     */
    for(i = 0; i < NUM_BUF_SIZES; i++) {
        for(j = 0; j < pool_sizes[i]; j++) {
            /* Allocate the buffer */
            status = POOL_alloc (POOL_makePoolId(processorId, SAMPLE_POOL_ID),
                                 (Void **) &buffers[i][j],
                                 buffer_sizes[i]) ;
            if (DSP_FAILED (status)) 
            {
                fprintf(stderr, "POOL_alloc() DataBuf failed. Status = [0x%x]\n", (int)status);
                return status;
            }

            /* Get the translated DSP address to be sent to the DSP. */
            status = POOL_translateAddr (
                                   POOL_makePoolId(processorId, SAMPLE_POOL_ID),
                                         &dsp_buffers[i][j],
                                         AddrType_Dsp,
                                         (Void *) buffers[i][j],
                                         AddrType_Usr) ;

            if (DSP_FAILED (status)) 
            {
                fprintf (stderr, "POOL_translateAddr () DataBuf failed. Status = [0x%x]\n", (int)status);
                return status;
            }
        }
    }

    /*
     *  Register for notification that the DSP-side application setup is
     *  complete.
     */
    status = NOTIFY_register (processorId,
                              canny_edge_IPS_ID,
                              canny_edge_IPS_EVENTNO,
                              (FnNotifyCbck) canny_edge_Notify,
                              0/* vladms XFER_SemPtr*/) ;
    if (DSP_FAILED (status)) 
	{
        fprintf(stderr, "NOTIFY_register () failed Status = [0x%x]\n", (int)status);
        return status;
    }

    /*
     *  Load the executable on the DSP.
     */
    status = PROC_load (processorId, dspExecutable, 0, NULL) ;
    if (DSP_FAILED (status)) {
        fprintf(stderr, "PROC_load () failed. Status = [0x%x]\n", (int)status);
        return status;
    }

    /*
     *  Start execution on DSP.
     */
    status = PROC_start (processorId) ;
    if (DSP_FAILED (status)) {
        fprintf(stderr, "PROC_start () failed. Status = [0x%x]\n", (int)status);
        return status;
    }

    /*
     *  Wait for the DSP-side application to indicate that it has completed its
     *  setup. The DSP-side application sends notification of the IPS event
     *  when it is ready to proceed with further execution of the application.
     */
    sem_wait(&sem);

    /*
     * Send the image cols and rows
     */
    status = NOTIFY_notify (processorId,
                            canny_edge_IPS_ID,
                            canny_edge_IPS_EVENTNO,
                            (Uint32) canny_edge_cols);
    if (DSP_FAILED (status)) 
    {
        fprintf(stderr, "NOTIFY_notify () DataBuf failed. Status = [0x%x]\n", (int)status);
        return status;
    }

    status = NOTIFY_notify (processorId,
                            canny_edge_IPS_ID,
                            canny_edge_IPS_EVENTNO,
                            (Uint32) canny_edge_rows);
    if (DSP_FAILED (status)) 
    {
        fprintf(stderr, "NOTIFY_notify () DataBuf failed. Status = [0x%x]\n", (int)status);
        return status;
    }


    /*
     *  Go through all buffers to initialize them on the DSP
     */
    for(i = 0; i < NUM_BUF_SIZES; i++) {
        for(j = 0; j < pool_sizes[i]; j++) {
            /* Send DSP address of the buffer to the DSP */
            status = NOTIFY_notify (processorId,
                                    canny_edge_IPS_ID,
                                    canny_edge_IPS_EVENTNO,
                                    (Uint32) dsp_buffers[i][j]);
            if (DSP_FAILED (status)) 
        	{
                fprintf(stderr, "NOTIFY_notify () DataBuf failed. Status = [0x%x]\n", (int)status);
                return status;
            }

            /* Send the size of the buffer to the DSP */
            status = NOTIFY_notify (processorId,
                                    canny_edge_IPS_ID,
                                    canny_edge_IPS_EVENTNO,
                                    (Uint32) buffer_sizes[i]);
            if (DSP_FAILED (status)) 
        	{
                fprintf(stderr, "NOTIFY_notify () DataBuf failed. Status = [0x%x]\n", (int)status);
                return status;
            }
        }
    }


    VPRINT("Leaving canny_edge_Create ()\n");
    return status;
}


#include <sys/time.h>

long long get_usec(void);

long long get_usec(void) 
{
  long long r;
  struct timeval t;
  gettimeofday(&t,NULL);
  r=t.tv_sec*1000000+t.tv_usec;
  return r;
}

/** ============================================================================
 *  @func   canny_edge_Execute
 *
 *  @desc   This function implements the execute phase for this application.
 *
 *  @modif  None
 *  ============================================================================
 */
NORMAL_API DSP_STATUS canny_edge_Execute (Uint8 processorId, IN Char8 * strImage)
{
    DSP_STATUS  status = DSP_SOK;
    long long start_time;
    unsigned char *image = (unsigned char *)buffers[0][0];
    short int *smoothedim = (short int *)buffers[1][0];
    short int *delta_x = (short int *)buffers[2][0];
    short int *delta_y = (short int *)buffers[3][0];
    short int *magnitude = (short int *)malloc(sizeof(short int) * canny_edge_rows * canny_edge_cols);
    unsigned char *nms = (unsigned char *)malloc(sizeof(unsigned char) * canny_edge_rows * canny_edge_cols);
    unsigned char *edge = (unsigned char *)malloc(sizeof(unsigned char) * canny_edge_rows * canny_edge_cols);
    char outfilename[128];    /* Name of the output "edge" image */

    VPRINT("Entered canny_edge_Execute ()\n");

    /* Copy the open image (since this is generated by PGM IO) */
    memcpy(image, canny_edge_image, buffer_sizes[0]);

    /* Start the timer */
    start_time = get_usec();

#if DO_WRITEBACK
    /* Do a writeback test */
    VPRINT(" Starting writeback\r\n");
    canny_edge_Writeback(image, canny_edge_rows, canny_edge_cols, processorId);
#endif

    /* Do the guassian smoothing */
    VPRINT(" Starting guassian smoothing\r\n");
#if GAUSSIAN_DSP
	canny_edge_Gaussian(image, canny_edge_rows, canny_edge_cols, smoothedim, processorId);
#elif GUASSIAN_NEON
    gaussian_smooth_neon(image, smoothedim, canny_edge_rows, canny_edge_cols, SIGMA);
#else
    gaussian_smooth(image, smoothedim, canny_edge_rows, canny_edge_cols, SIGMA);
#endif

    /* Calculate the derivatives */
    VPRINT(" Starting derivative x, y\r\n");
#if DERIVATIVE_DSP
    canny_edge_Derivative(smoothedim, canny_edge_rows, canny_edge_cols, delta_x, delta_y, processorId);
#else
    derivative_x_y(smoothedim, canny_edge_rows, canny_edge_cols, &delta_x, &delta_y);
#endif

    /* Compute the magnitude */
    VPRINT(" Starting magnitude x, y\r\n");
    magnitude_x_y(delta_x, delta_y, canny_edge_rows, canny_edge_cols, magnitude);

    /* Do the Non maximal suppression */
    VPRINT(" Starting non maximal suppression \r\n");
    non_max_supp(magnitude, delta_x, delta_y, canny_edge_rows, canny_edge_cols, nms);

    /* Apply the hysteresis */
    VPRINT(" Starting hysteresis \r\n");
    apply_hysteresis(magnitude, nms, canny_edge_rows, canny_edge_cols, TLOW, THIGH, edge);

    /* Stop the timer and return */
    printf("Canny edge took %lld us.\n", (get_usec() - start_time));

    /* Save the image */
    sprintf(outfilename, "%s_out.pgm", strImage);
    if(write_pgm_image(outfilename, edge, canny_edge_rows, canny_edge_cols, "", 255) == 0)
    {
        fprintf(stderr, "Error writing the edge image, %s.\n", outfilename);
        status = DSP_EFAIL;
    }

    return status;
}


/** ============================================================================
 *  @func   canny_edge_Delete
 *
 *  @desc   This function releases resources allocated earlier by call to
 *          canny_edge_Create ().
 *          During cleanup, the allocated resources are being freed
 *          unconditionally. Actual applications may require stricter check
 *          against return values for robustness.
 *
 *  @modif  None
 *  ============================================================================
 */
NORMAL_API Void canny_edge_Delete (Uint8 processorId)
{
    DSP_STATUS status    = DSP_SOK;
    Uint16 i, j;

	VPRINT("Entered canny_edge_Delete ()\n") ;

    /* Send DSP to stop */
    status = NOTIFY_notify (processorId,
                            canny_edge_IPS_ID,
                            canny_edge_IPS_EVENTNO,
                            (Uint32) canny_edge_DELETE);
    if (DSP_FAILED (status)) 
    {
        fprintf(stderr, "NOTIFY_notify () DataBuf failed. Status = [0x%x]\n", (int)status);
    }


    /* Free the canny edge image */
    free(canny_edge_image);

    /*
     *  Stop execution on DSP.
     */
    status = PROC_stop (processorId) ;
    if (DSP_FAILED (status)) {
        fprintf(stderr, "PROC_stop () failed. Status = [0x%x]\n", (int)status);
    }

    /*
     *  Unregister for notification of event registered earlier.
     */
    status = NOTIFY_unregister (processorId,
                                   canny_edge_IPS_ID,
                                   canny_edge_IPS_EVENTNO,
                                   (FnNotifyCbck) canny_edge_Notify,
                                   0/* vladms canny_edge_SemPtr*/) ;
    if (DSP_FAILED (status)) {
        fprintf(stderr, "NOTIFY_unregister () failed Status = [0x%x]\n", (int)status);
    }

    /*
     *  Free the memory allocated for the data buffer.
     */
    for(i = 0; i < NUM_BUF_SIZES; i++) {
        for(j = 0; j < pool_sizes[i]; j++) {
            status = POOL_free (POOL_makePoolId(processorId, SAMPLE_POOL_ID),
                                   (Void *) buffers[i][j],
                                   buffer_sizes[i]) ;
            if (DSP_FAILED (status)) {
                fprintf(stderr, "POOL_free () DataBuf failed. Status = [0x%x]\n", (int)status);
            }
        }
    }

    /*
     *  Close the pool
     */
    status = POOL_close (POOL_makePoolId(processorId, SAMPLE_POOL_ID)) ;
    if (DSP_FAILED (status)) {
        fprintf(stderr, "POOL_close () failed. Status = [0x%x]\n", (int)status);
    }

    /*
     *  Detach from the processor
     */
    status = PROC_detach  (processorId) ;
    if (DSP_FAILED (status)) {
        fprintf(stderr, "PROC_detach () failed. Status = [0x%x]\n", (int)status);
    }

    /*
     *  Destroy the PROC object.
     */
    status = PROC_destroy () ;
    if (DSP_FAILED (status)) {
        fprintf(stderr, "PROC_destroy () failed. Status = [0x%x]\n", (int)status);
    }

    VPRINT("Leaving canny_edge_Delete ()\n");
}


/** ============================================================================
 *  @func   canny_edge_Main
 *
 *  @desc   Entry point for the application
 *
 *  @modif  None
 *  ============================================================================
 */
NORMAL_API Void canny_edge_Main (IN Char8 * dspExecutable, IN Char8 * strImage)
{
    DSP_STATUS status       = DSP_SOK ;

	VPRINT("========== Application : canny_edge ==========\n");

    if (dspExecutable != NULL && strImage != NULL) 
	{
        /*
         *  Specify the dsp executable file name and the buffer size for
         *  canny_edge creation phase.
         */
        status = canny_edge_Create(dspExecutable, strImage, ID_PROCESSOR);

        if (DSP_SUCCEEDED(status)) 
		{
            status = canny_edge_Execute(ID_PROCESSOR, strImage);
        }

         canny_edge_Delete(ID_PROCESSOR);
    }
    else 
	{
        fprintf(stderr, "ERROR! Invalid arguments specified for canny_edge application\n");
    }

    VPRINT("====================================================\n");
}

/** ----------------------------------------------------------------------------
 *  @func   canny_edge_Notify
 *
 *  @desc   This function implements the event callback registered with the
 *          NOTIFY component to receive notification indicating that the DSP-
 *          side application has completed its setup phase.
 *
 *  @modif  None
 *  ----------------------------------------------------------------------------
 */
STATIC Void canny_edge_Notify (Uint32 eventNo, Pvoid arg, Pvoid info)
{
    VPRINT("Notification event: %lu, info: %8d \r\n", eventNo, (int)info);
    /* Post the semaphore for initialization. */
    if((int)info == canny_edge_INIT) 
	{
        sem_post(&sem);
    } else if((int)info == canny_edge_WRITEBACK) {
        sem_post(&sem);
    } else if((int)info == canny_edge_GAUSSIAN) {
        sem_post(&sem);
    } else if((int)info == canny_edge_DERIVATIVE) {
        sem_post(&sem);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// DSP ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/* Simple function which transmits the image and expects it back with each pixel +1 */
STATIC Void canny_edge_Writeback(unsigned char *image, int rows, int cols, Uint8 processorId)
{
#if VERIFY
    int i, status;
#endif

    /* Send the image */
    POOL_writeback (POOL_makePoolId(processorId, SAMPLE_POOL_ID),
                    image,
                    sizeof(unsigned char) * rows * cols);
    NOTIFY_notify (processorId, canny_edge_IPS_ID, canny_edge_IPS_EVENTNO, canny_edge_WRITEBACK);
    VPRINT("  Writeback send, waiting for response...\r\n");

    /* Wait for the response */
    sem_wait(&sem);

    /* Invalidate the result */
    POOL_invalidate(POOL_makePoolId(processorId, SAMPLE_POOL_ID),
                    image,
                    sizeof(unsigned char) * rows * cols);

    /* Check if the result is correct */
#if VERIFY
    status = DSP_SOK;
    for(i = 0; i < (rows * cols); i++) {
        canny_edge_image[i]++;
        if(image[i] != canny_edge_image[i]) {
            fprintf(stderr, "Got incorrect image back! Expected %d, Got %d (i: %d)\r\n", canny_edge_image[i], image[i], i);
            status = DSP_EFAIL;
        }
    }

    if(DSP_SUCCEEDED(status))
        VPRINT("Writeback was succesfull!\r\n");
#endif
}

STATIC Void canny_edge_Gaussian(unsigned char *image, int rows, int cols, short int *smoothedim, Uint8 processorId)
{
#if VERIFY
    int i, status;
#endif

    POOL_writeback (POOL_makePoolId(processorId, SAMPLE_POOL_ID),
                    image,
                    buffer_sizes[0]);

    /* Notify DSP */
    NOTIFY_notify (processorId, canny_edge_IPS_ID, canny_edge_IPS_EVENTNO, canny_edge_GAUSSIAN);
    VPRINT("  DSP_Gaussian send, waiting for response...\r\n");

     /* Wait for the response */
    sem_wait(&sem);

    /* Invalidate the result */
    POOL_invalidate(POOL_makePoolId(processorId, SAMPLE_POOL_ID),
                    smoothedim,
                    buffer_sizes[1]);
    /* Check if the result is correct */
}


STATIC Void canny_edge_Derivative(short int *smoothedim, int rows, int cols, short int *delta_x, short int *delta_y, Uint8 processorId)
{
#if VERIFY
    int i;
    int status = DSP_SOK;
    short int *verify_delta_x = (short int *) malloc(sizeof(short int) * rows * cols);
    short int *verify_delta_y = (short int *) malloc(sizeof(short int) * rows * cols);
#endif

    /* Send smoothedim */
    POOL_writeback (POOL_makePoolId(processorId, SAMPLE_POOL_ID),
                    smoothedim,
                    buffer_sizes[1]);

    /* Notify DSP */
    NOTIFY_notify (processorId, canny_edge_IPS_ID, canny_edge_IPS_EVENTNO, canny_edge_DERIVATIVE);
    VPRINT("  DSP_derivative_x_y send, waiting for response...\r\n");

     /* Wait for the response */
    sem_wait(&sem);

    /* Invalidate the result */
    POOL_invalidate(POOL_makePoolId(processorId, SAMPLE_POOL_ID),
                    delta_x,
                    buffer_sizes[2]);
    POOL_invalidate(POOL_makePoolId(processorId, SAMPLE_POOL_ID),
                    delta_y,
                    buffer_sizes[3]);

#if VERIFY
    /* verify with GPP function */
    derivative_x_y(smoothedim, rows, cols, &verify_delta_x, &verify_delta_y);
  
    /* Check for delta_x*/
    for(i = 0; i < rows*cols; i++) {
        if(delta_x[i] != verify_delta_x[i]) {
            fprintf(stderr, "Got incorrect delta_x result back! Expected %d, Got %d (i: %d)\r\n", verify_delta_x[i], delta_x[i], i);
            status = DSP_EFAIL;
        }
    }

    /* Check for delta_y*/
    for(i = 0; i < rows*cols; i++) {
        if(delta_y[i] != verify_delta_y[i]) {
            fprintf(stderr, "Got incorrect delta_y result back! Expected %d, Got %d (i: %d)\r\n", verify_delta_y[i], delta_y[i], i);
            status = DSP_EFAIL;
        }
    }

    /* Print if verify was succesfull */
    if(DSP_SUCCEEDED(status))
        VPRINT("Execution of canny_edge_Derivative was succesfull!\r\n");
    else
        fprintf(stderr, "Execution of canny_edge_Derivative was FAILED!\r\n");
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// NEON ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/* Guassian smooth on Neon */
STATIC void gaussian_smooth_neon(unsigned char *image, short int* smoothedim, Uint16 rows, Uint16 cols, float sigma)
{
    int windowsize;          
    float *tempim,        
          *kernel;            
    float *rows_image;                     /* Image for x-smoothing*/
    float *cols_image;                    /*  Image for y-smoothing*/
    float neon_kernel[17];               /* New kernel for neon */
    unsigned int neon_cols=cols+16;     /* Cols value for x-direction*/
    unsigned int neon_rows=rows+16;     /* Rows value for y-direction*/
    unsigned int i, k, a, c, r, j,t,b; /*Loop variant*/
    float32x4_t neon_pixel;       /* Four consecutive pixel values */
    float32x4_t neon_factor;      /* Four consecutive filter values */
    float32x4_t temp_dot;         /* The neon multiplication result store here */
    float dot= 0.0f;              /* The sum of pixel values */
    float Referkernel = 0.0f;      /* Intermediate sum of filter values considering boundary situation */
    float sum = 0.0f;             /* The sum of filter values */
    
    /****************************************************************************
    * Create a 1-dimensional gaussian smoothing kernel.
    ****************************************************************************/
    VPRINT("   Computing the gaussian smoothing kernel.\n");
    make_gaussian_kernel(sigma, &kernel, &windowsize);

    /****************************************************************************
    * Allocate a temporary buffer image and the smoothed image.
    ****************************************************************************/
    if((tempim = (float *) malloc(rows*cols* sizeof(float))) == NULL)
    {
        fprintf(stderr, "Error allocating the buffer image.\n");
        exit(1);
    }
    
    /****************************************************************************
    * Define the new kernel and referenced kernel in boundary case.
    ****************************************************************************/
    
    for (b = 0 ; b <= 16 ; b++)
    {
        if(b > 0 && b < 16 )
            neon_kernel[b] = kernel [b-1];
        else
            neon_kernel [b] = 0;    
    }
    
    for( a=8; a<=16; a++){
        Referkernel += neon_kernel[a];
    }
    /****************************************************************************
    * Blur in the x - direction.
    ****************************************************************************/
    VPRINT("   Bluring the image in the X-direction.\n");
    /* Allocate the memory to new image and set the boundary value as 0. */
    rows_image = (float*)malloc(neon_cols*rows*sizeof(float));
    for( i =0; i<rows; i++){
        memset(&rows_image[i*neon_cols],0,8*sizeof(float));

        for( k=0; k<cols;k++){
            rows_image[i*neon_cols+8+k] = (float)image[i*cols+k];
        }
        memset(&rows_image[i*neon_cols+8+cols],0,8*sizeof(float));
    }

    for(r=0; r<rows; r++){
        for( c=0; c<cols; c++){
            /* Assign different sum value according to pixel position */
            if(c==0){
                sum = Referkernel;
            }
            else if(c <=8){
                sum += neon_kernel[8-c];
            }
            else if(c>=cols-8){
                sum -=neon_kernel[cols-c+8];
            }
            /*Calculate the middle pixel value based on neon.*/
            temp_dot = vdupq_n_f32(0);
            for( j=0; j<=3; j++)
            {
                int e=0;
                if(j>=2)
                {
                    e=1;
                }
                //neon_pixel = vld1q_f32((float32_t const *)&rows_image[r*neon_cols+c+j*4+e]);
                //neon_factor = vld1q_f32((float32_t const *)&neon_kernel[j*4+e]);
                //temp_dot = vmlaq_f32(temp_dot, neon_pixel, neon_factor);
            }
            
    
            dot += vgetq_lane_f32(temp_dot, 0);
            dot += vgetq_lane_f32(temp_dot, 1);
            dot += vgetq_lane_f32(temp_dot, 2);
            dot += vgetq_lane_f32(temp_dot, 3);
            dot += rows_image[r*neon_cols+c+8] * neon_kernel[8];
            tempim[r*cols+c] = dot/sum;
            dot=0; 
        }
    }


    /****************************************************************************
    * Blur in the y - direction.
    ****************************************************************************/
    VPRINT("   Bluring the image in the Y-direction.\n");
    /* Allocate the memory to new image and set the boundary value as 0. The image is stored in unit of cols for convient y-direction smoothing*/
    cols_image = (float*)malloc(neon_rows*cols*sizeof(float));
    for( i =0; i<cols; i++){
        memset(&cols_image[i*neon_rows],0,8*sizeof(float));
        for( k=0; k<rows;k++){
            cols_image[i*neon_rows+8+k] = tempim[k*cols+i];
        }
        memset(&cols_image[i*neon_rows+8+rows],0,8*sizeof(float));
    }


    for(c=0; c<cols; c++){
        for( r=0; r<rows; r++){
            /* Assign different sum value according to pixel position */
            if(r==0){
                sum = Referkernel;
            }
            else if(r <=8){
                sum += neon_kernel[8-r];
            }
            else if(r>=rows-8){
                sum -=neon_kernel[rows-r+8];
            }
            /*Calculate the middle pixel value based on neon.*/
            temp_dot = vdupq_n_f32(0);
            for( j=0; j<=3; j++)
            {
                int e=0;
                if(j>=2)
                {
                    e=1;
                }
                neon_pixel = vld1q_f32((float32_t const *)&cols_image[c*neon_rows+r+j*4+e]);
                neon_factor = vld1q_f32((float32_t const *)&neon_kernel[j*4+e]);
                temp_dot = vmlaq_f32(temp_dot,neon_pixel,neon_factor);
            }
            
        
            dot += vgetq_lane_f32(temp_dot, 0);
            dot += vgetq_lane_f32(temp_dot, 1);
            dot += vgetq_lane_f32(temp_dot, 2);
            dot += vgetq_lane_f32(temp_dot, 3);
            dot += cols_image[c*neon_rows+r+8] * neon_kernel[8];
            smoothedim[r*cols+c] = (short int )(dot*BOOSTBLURFACTOR/sum + 0.5);
            dot=0; 
        }
    }
    free(rows_image);
    free(cols_image);
    free(tempim);
    free(kernel);
}

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// GPP ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* FUNCTION: angle_radians
* PURPOSE: This procedure computes the angle of a vector with components x and
* y. It returns this angle in radians with the answer being in the range
* 0 <= angle <2*PI.
*******************************************************************************/
STATIC double angle_radians(double x, double y)
{
    double xu, yu, ang;

    xu = fabs(x);
    yu = fabs(y);

    if((xu == 0) && (yu == 0)) return(0);

    ang = atan(yu/xu);

    if(x >= 0)
    {
        if(y >= 0) return(ang);
        else return(2*M_PI - ang);
    }
    else
    {
        if(y >= 0) return(M_PI - ang);
        else return(M_PI + ang);
    }
}

/*******************************************************************************
* PROCEDURE: magnitude_x_y
* PURPOSE: Compute the magnitude of the gradient. This is the square root of
* the sum of the squared derivative values.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
STATIC void magnitude_x_y(short int *delta_x, short int *delta_y, int rows, int cols,
                   short int *magnitude)
{
    int r, c, pos, sq1, sq2;

    for(r=0,pos=0; r<rows; r++)
    {
        for(c=0; c<cols; c++,pos++)
        {
            sq1 = (int)delta_x[pos] * (int)delta_x[pos];
            sq2 = (int)delta_y[pos] * (int)delta_y[pos];
            magnitude[pos] = (short)(0.5 + sqrt((float)sq1 + (float)sq2));
        }
    }
}


/*******************************************************************************
* PROCEDURE: derivative_x_y
* PURPOSE: Compute the first derivative of the image in both the x any y
* directions. The differential filters that are used are:
*
*                                          -1
*         dx =  -1 0 +1     and       dy =  0
*                                          +1
*
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
STATIC void derivative_x_y(short int *smoothedim, int rows, int cols,
        short int **delta_x, short int **delta_y)
{
   int r, c, pos;
   // Calculate the X direction
   for(r=0;r<rows;r++){
      pos = r * cols;
      (*delta_x)[pos] = smoothedim[pos+1] - smoothedim[pos];
      pos++;
      for(c=1;c<(cols-1);c++,pos++){
         (*delta_x)[pos] = smoothedim[pos+1] - smoothedim[pos-1];
      }
      (*delta_x)[pos] = smoothedim[pos] - smoothedim[pos-1];
   }

   // Calculate the Y direction
   for(c=0;c<cols;c++){
      pos = c;
      (*delta_y)[pos] = smoothedim[pos+cols] - smoothedim[pos];
      pos += cols;
      for(r=1;r<(rows-1);r++,pos+=cols){
         (*delta_y)[pos] = smoothedim[pos+cols] - smoothedim[pos-cols];
      }
      (*delta_y)[pos] = smoothedim[pos] - smoothedim[pos-cols];
   }
}

/*******************************************************************************
* PROCEDURE: gaussian_smooth
* PURPOSE: Blur an image with a gaussian filter.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
STATIC void gaussian_smooth(unsigned char *image, short int* smoothedim, int rows, int cols, float sigma)
{
    int r, c, rr, cc,     /* Counter variables. */
        windowsize,        /* Dimension of the gaussian kernel. */
        center;            /* Half of the windowsize. */
    float *tempim,        /* Buffer for separable filter gaussian smoothing. */
          *kernel,        /* A one dimensional gaussian kernel. */
          dot,            /* Dot product summing variable. */
          sum;            /* Sum of the kernel weights variable. */

    /****************************************************************************
    * Create a 1-dimensional gaussian smoothing kernel.
    ****************************************************************************/
    VPRINT("   Computing the gaussian smoothing kernel.\n");
    make_gaussian_kernel(sigma, &kernel, &windowsize);
    center = windowsize / 2;


    /****************************************************************************
    * Allocate a temporary buffer image
    ****************************************************************************/
    if((tempim = (float *) malloc(rows*cols* sizeof(float))) == NULL)
    {
        fprintf(stderr, "Error allocating the buffer image.\n");
        exit(1);
    }

    /****************************************************************************
    * Blur in the x - direction.
    ****************************************************************************/
    VPRINT("   Bluring the image in the X-direction.\n");
    for(r=0; r<rows; r++)
    {
        for(c=0; c<cols; c++)
        {
            dot = 0.0;
            sum = 0.0;
            for(cc=(-center); cc<=center; cc++)
            {
                if(((c+cc) >= 0) && ((c+cc) < cols))
                {
                    dot += (float)image[r*cols+(c+cc)] * kernel[center+cc];
                    sum += kernel[center+cc];
                }
            }
            tempim[r*cols+c] = dot/sum;
        }
    }

    /****************************************************************************
    * Blur in the y - direction.
    ****************************************************************************/
    VPRINT("   Bluring the image in the Y-direction.\n");
    for(c=0; c<cols; c++)
    {
        for(r=0; r<rows; r++)
        {
            sum = 0.0;
            dot = 0.0;
            for(rr=(-center); rr<=center; rr++)
            {
                if(((r+rr) >= 0) && ((r+rr) < rows))
                {
                    dot += tempim[(r+rr)*cols+c] * kernel[center+rr];
                    sum += kernel[center+rr];
                }
            }
            smoothedim[r*cols+c] = (short int)(dot*BOOSTBLURFACTOR/sum + 0.5);
        }
    }

    free(tempim);
    free(kernel);
}

/*******************************************************************************
* PROCEDURE: make_gaussian_kernel
* PURPOSE: Create a one dimensional gaussian kernel.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
STATIC void make_gaussian_kernel(float sigma, float **kernel, int *windowsize)
{
    int i, center;
    float x, fx, sum=0.0;

    *windowsize = 1 + 2 * ceil(2.5 * sigma);
    center = (*windowsize) / 2;

    if((*kernel = (float *) malloc((*windowsize)* sizeof(float))) == NULL)
    {
        fprintf(stderr, "Error callocing the gaussian kernel array.\n");
        exit(1);
    }

    for(i=0; i<(*windowsize); i++)
    {
        x = (float)(i - center);
        fx = pow(2.71828, -0.5*x*x/(sigma*sigma)) / (sigma * sqrt(6.2831853));
        (*kernel)[i] = fx;
        sum += fx;
    }

    for(i=0; i<(*windowsize); i++) (*kernel)[i] /= sum;

    if(VERBOSE)
    {
        printf("The filter coefficients are:\n");
        for(i=0; i<(*windowsize); i++)
            printf("kernel[%d] = %f\n", i, (*kernel)[i]);
    }
}


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
