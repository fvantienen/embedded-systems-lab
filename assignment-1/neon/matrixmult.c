#include <stdio.h>
#include "Timer.h"
#include <arm_neon.h>

/* Matrix size */
int matrix_size;

void matMult(int16_t mat1[], int16_t mat2[], int32_t prod[matrix_size][matrix_size]);

int main(int argc, char** argv)
{
	int i, j;
	Timer neonTime;
    int16_t *mat1, *mat2;
    int32_t prod[sizeof(int32_t)*matrix_size][sizeof(int32_t)*matrix_size];

    /* Get argument size */
    matrix_size = atoi(argv[1]);
    if(matrix_size < 0 || matrix_size > 512)
    {
    	printf("Matrix size must be between 0 and 512.\n");
    	return -1;
    }

    /* Initialize timer */
    initTimer(&neonTime, "NEON Time");

    /* Allocate matrices */
    mat1 = malloc(matrix_size * matrix_size * sizeof(int16_t));
    mat2 = malloc(matrix_size * matrix_size * sizeof(int16_t));

    if (mat1 == NULL || mat2 == NULL) {
        printf("Out of memory\n");
    }

	/* Initialize matrices */
	for (i = 0; i < matrix_size; i++)
	{
		for (j = 0; j < matrix_size; j++)
		{
			mat1[i*matrix_size + j] = i+j*2;
		}
	}
	
	for(i = 0; i < matrix_size; i++)
	{
		for (j = 0; j < matrix_size; j++)
		{
			mat2[i*matrix_size + j] = i+j*3;
		}
	}

	/* Run the multiplication */
    startTimer(&neonTime);
	matMult(mat1,mat2,prod);
    stopTimer(&neonTime);
    printTimer(&neonTime);	

/*
	for (i = 0;i < matrix_size; i++)
	{
		printf("\n");
		for (j = 0; j < matrix_size; j++)
		{
			printf("\t%d ", prod[i][j]);
		}
	}
	
	printf("\nDone !!! \n");
}
*/
	return 0;
}


inline void MAC4 (int32x4_t *additive_value, int16x4_t *data1, int16x4_t *data2, int32x4_t *mac_output)
 {
	*mac_output = vmlal_s16(*additive_value,*data1, *data2);
 }

/* mat1 and mat2 defined in the external code are the matrix to be processed.  */

void matMult(int16_t mat1[], int16_t mat2[], int32_t prod[matrix_size][matrix_size])
{
	int output_size = 2 * matrix_size;
	int l,k;
	int16x4_t data1;
	int32x4_t mac_output[output_size/4];
	int32x4_t MAC_addvalue[output_size/4];
	int16x4_t constant_value;
	unsigned int index_input = 0;
	unsigned int transfer_index = 0 ;
	int32_t *pres_ver;

	/* Allocate output */
	pres_ver = malloc(output_size * output_size * sizeof(int32_t));
	for(l = 0 ; l < matrix_size/4; l++)
	{
	    MAC_addvalue[l] = vmovq_n_s32(0);
	}

	/* Perform the multiplication */
	for(l = 0; l < matrix_size*matrix_size; l++)
	{
	    constant_value = vmov_n_s16 (mat1[l]);
	    for(k = 0 ; k < matrix_size/4 ; k++)
	    {
	        data1 = vld1_s16 (&mat2[index_input]);
	        MAC4 (&MAC_addvalue[k], &constant_value, &data1,&mac_output[k]);
	        MAC_addvalue[k] = mac_output[k];
	        index_input +=4;
	    }

		index_input+=output_size-matrix_size;
	    if ((l + 1) % matrix_size == 0 )
	    {
	        index_input = 0;

	        for(k = 0 ; k < matrix_size/4 ; k++)
	        {
	            vst1q_s32(&pres_ver[transfer_index],MAC_addvalue[k]);
	            transfer_index +=4;
	        }

			transfer_index += output_size-matrix_size;
	        for(k = 0 ; k < matrix_size/4; k++)
	        {
	            MAC_addvalue[k] = vmovq_n_s32(0);
	        }
	    }
	}
}
