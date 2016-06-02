/*  ----------------------------------- OS Specific Headers           */
#include <stdio.h>
#include <stdlib.h>

/*  ----------------------------------- DSP/BIOS Link                 */
#include <dsplink.h>

/*  ----------------------------------- Application Header            */
#include <canny_edge.h>

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

    if (argc != 3) {
        printf("Usage : %s <absolute path of DSP executable> "
               "<Image path>\n",
               argv [0]) ;
    } else {
        dspExecutable    = argv[1];
        strImage         = argv[2];

        canny_edge_Main(dspExecutable, strImage);
    }

    return 0 ;
}
