//////////////////////////////////////////////////////////////////////////////////////
// 	README file -- Embedded Systems Laboratory -- Group 3							//
// 	Jing Wang 			4517938 													//
//  Yang Ma 			4508963 													//
//	Hongjia Wu 			4491785 													//
//	Chengxin Ma 		4499832 													//
//	Michel Jansen 		4064852 													//
//	Freek van Tienen	4094123 													//
//////////////////////////////////////////////////////////////////////////////////////

The following flags can be set in the file "GPP/canny_edge.c":

DO_WRITEBACK:
Do a simple write test to DSP and increment all pixels (1) or disable (0)

GAUSSIAN_PARALLEL: 
Enable (1) or disable (0) use of DSP in combination with GPP/NEON for gaussian smooth function.

GUASSIAN_NEON: 
Enable (1) or disable (0) use of NEON for gaussian smooth function

MAGNITUDE_PARALLEL: 
Enable (1) or disable (0) use of DSP in combination with GPP/NEON for magnitude function

MAGNITUDE_NEON: 
Enable (1) or disable (0) use of NEON for magnitude function

DERIVATIVE_PARALLEL:
Enable (1) or disable (0) use of DSP in combination with GPP/NEON for derivative function

DERIVATIVE_NEON:
Enable (1) or disable (0) use of NEON for derivative function

VERBOSE: Enable (1) or disable (0) verbose printing (Execution time will be longer!)

VERIFY: Enable (1) or disable (0) verification of the results (Execution time will be longer!)

//////////////////////////////////////////////////////////////////////////////////////
// Execute the program																//
//////////////////////////////////////////////////////////////////////////////////////

The workload can be divided by passing a percentage when executing the program 
(Example to execute the best case: ./canny_edge canny_edge.out pics/klomp.pgm 49 24 100)
The percentage indicates the amount of work done on the GPP/NEON (depending on the FUNCTION_NEON flag):
(Gaussian percentage) (Derivative percentage) (Magnitude percentage)

Best-case execution flags & percentages:
DO_WRITEBACK			0
GAUSSIAN_PARALLEL 		1
GUASSIAN_NEON 			1
MAGNITUDE_PARALLEL 		0
MAGNITUDE_NEON 			1
DERIVATIVE_PARALLEL 	1
DERIVATIVE_NEON 		1	
VERBOSE 				0
VERIFY 					0

Pass percentages
Klomp: 	49, 24, 100
Tiger: 	51, 46, 100
Square: 51, 20, 100


//////////////////////////////////////////////////////////////////////////////////////
// Compile the program																//
//////////////////////////////////////////////////////////////////////////////////////

In order to compile the program go to the folder and executed by typing "make".

"make dsp && make gpp && make send"