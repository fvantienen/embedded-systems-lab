/*  ----------------------------------- OS Specific Headers           */
#include <stdio.h>
#include <stdlib.h>

/*  ----------------------------------- DSP/BIOS Link                 */
#include <dsplink.h>

/*  ----------------------------------- Application Header            */
#include <canny_edge.h>

int gaussianPerc, derivativePerc, magnitudePerc;

/** ============================================================================
 *  @func   main
 *
 *  @desc   Entry point for the application
 *
 *  @modif  None
 *  ============================================================================
 */
int main(int argc, char **argv)
{
    Char8 *dspExecutable    = NULL;
    Char8 *strImage         = NULL;

    if (argc != 6) {
        printf("Usage : %s <absolute path of DSP executable> "
               "<Image path> <Gaussian percentage> <Derivative percentage> <Magnitude percentage>\n",
               argv [0]) ;
    } else {
        dspExecutable    = argv[1];
        strImage         = argv[2];
        gaussianPerc     = atoi(argv[3]);
        derivativePerc   = atoi(argv[4]);
        magnitudePerc    = atoi(argv[5]);

        canny_edge_Main(dspExecutable, strImage);
    }

    return 0 ;
}
