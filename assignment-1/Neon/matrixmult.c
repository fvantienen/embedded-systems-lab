#include <stdio.h>
#include "Timer.h"
#include <arm_neon.h>

int SIZE;

// void matMult(int mat1[SIZE][SIZE], int mat2[SIZE][SIZE], int prod[SIZE][SIZE]);

int main(int argc, char** argv)
{
	SIZE = atoi(argv[1]);

    Timer neonTime;
    initTimer(&neonTime, "NEON Time");
    int size_n = SIZE;
	int MAT_SIZE = 2* SIZE;
    int16_t *mat1, *mat2;
  	int32_t *pres,*pres_ver;
    int32_t prod[MAT_SIZE][MAT_SIZE], prod_ver[MAT_SIZE][MAT_SIZE];

    mat1 = malloc(SIZE * SIZE * sizeof(int16_t));
    mat2 = malloc(SIZE * SIZE * sizeof(int16_t));

    if (mat1 == NULL || mat2 == NULL) {
        printf("Out of memory\n");
    }

	//int mat1[SIZE*SIZE], mat2[SIZE*SIZE], prod[SIZE*SIZE];
	int i, j;
	
	for (i = 0; i < SIZE; i++)
	{
		for (j = 0; j < SIZE; j++)
		{
			mat1[i*SIZE + j] = i+j*2;
		}
	}
	
	for(i = 0; i < SIZE; i++)
	{
		for (j = 0; j < SIZE; j++)
		{
			mat2[i*SIZE + j] = i+j*3;
		}
	}

     startTimer(&neonTime);
	 matMult(mat1,mat2,prod);
     stopTimer(&neonTime);
     printTimer(&neonTime);	

/*
	for (i = 0;i < SIZE; i++)
	{
		printf("\n");
		for (j = 0; j < SIZE; j++)
		{
			printf("\t%d ", prod[i][j]);
		}
	}
	
	printf("\nDone !!! \n");
	return 0;
}
*/
}


inline void MAC4 (int32x4_t *additive_value, int16x4_t *data1, int16x4_t *data2, int32x4_t *mac_output)
 {
	*mac_output = vmlal_s16(*additive_value,*data1, *data2);
 }

/* mat1 and mat2 defined in the external code are the matrix to be processed.  */

void matMult(int mat1[SIZE*SIZE], int mat2[SIZE*SIZE], int prod[SIZE*SIZE])
{
	int size_n = SIZE;
	int MAT_SIZE = 2 * SIZE;
	int l,k;
	int16x4_t data1;
	int32x4_t mac_output[MAT_SIZE/4];
	int32x4_t MAC_addvalue[MAT_SIZE/4];
	int16x4_t constant_value;
	unsigned int index_input = 0;
	unsigned int transfer_index = 0 ;
	int32_t *pres_ver;
	pres_ver = malloc(MAT_SIZE * MAT_SIZE * sizeof(int32_t));
	for(l = 0 ; l < size_n/4; l++)
	{
	    MAC_addvalue[l] = vmovq_n_s32(0);
	}

	for(l = 0; l < size_n*size_n; l++)
	{
	    constant_value = vmov_n_s16 (mat1[l]);
	    for(k = 0 ; k < size_n/4 ; k++)
	    {
	        data1 = vld1_s16 (&mat2[index_input]);
	        MAC4 (&MAC_addvalue[k], &constant_value, &data1,&mac_output[k]);
	        MAC_addvalue[k] = mac_output[k];
	        index_input +=4;
	    }

	index_input+=MAT_SIZE-size_n;
	    if ((l + 1) % size_n == 0 )
	    {
	        index_input = 0;

	        for(k = 0 ; k < size_n/4 ; k++)
	        {
	            vst1q_s32(&pres_ver[transfer_index],MAC_addvalue[k]);
	            transfer_index +=4;
	        }
	transfer_index += MAT_SIZE-size_n;
	        for(k = 0 ; k < size_n/4; k++)
	        {
	            MAC_addvalue[k] = vmovq_n_s32(0);
	        }
	    }
	}
}
