/*  ----------------------------------- OS Specific Headers           */
#include <stdio.h>
#include <stdlib.h>

/*  ----------------------------------- DSP/BIOS Link                 */
#include <gpptypes.h>
#include <dsplink.h>
#include <errbase.h>

/*  ----------------------------------- Application Header            */
#include <system_os.h>
#include <helloDSP.h>


#if defined (__cplusplus)
extern "C"
{
#endif /* defined (__cplusplus) */


    /** ============================================================================
     *  @func   main
     *
     *  @desc   Entry point for the application
     *
     *  @modif  None
     *  ============================================================================
     */
    int main (int argc, char** argv)
    {
        Char8* dspExecutable = NULL;
        Char8* strNumIterations = NULL;
        Char8* strProcessorId = NULL;
        Uint8 processorId = 0;

        /*	long long _Fract value = atof("2.3");
        	printf("%k\n",value);	*/

        if ((argc != 4) && (argc!=3))
        {
            SYSTEM_1Print("Usage : %s <absolute path of DSP executable> <number of transfers> <DSP Processor Id>\n"
                          "For infinite transfers, use value of 0 for <number of transfers>\n"
                          "For DSP Processor Id,"
                          "\n\t use value of 0  if sample needs to be run on DSP 0 "
                          "\n\t use value of 1  if sample needs to be run on DSP 1"
                          "\n\t For single DSP configuration this is optional argument\n",
                          (int) argv[0]);
        }

        else
        {
            dspExecutable = argv[1];
            strNumIterations = argv[2];

            if (argc == 3)
            {
                strProcessorId = "0";
                processorId = 0;
            }
            else
            {
                strProcessorId = argv[3];
                processorId = atoi(argv[3]);
            }

            if (processorId < MAX_PROCESSORS)
            {
                helloDSP_Main(dspExecutable, strNumIterations, strProcessorId);
            }
        }

        return 0;
    }


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
