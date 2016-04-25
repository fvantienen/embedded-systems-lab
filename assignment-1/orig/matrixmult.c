#include<stdio.h>
#include "Timer.h"

#define SIZE 16

void matMult(int mat1[SIZE][SIZE], int mat2[SIZE][SIZE], int prod[SIZE][SIZE]);

int main()
{
    Timer totalTime;
    initTimer(&totalTime, "Total Time");

	int mat1[SIZE][SIZE], mat2[SIZE][SIZE], prod[SIZE][SIZE];
	int i, j;
	
	for (i = 0;i < SIZE; i++)
	{
		for (j = 0; j < SIZE; j++)
		{
			mat1[i][j] = i+j*2;
		}
	}
	
	for(i = 0; i < SIZE; i++)
	{
		for (j = 0; j < SIZE; j++)
		{
			mat2[i][j] = i+j*3;
		}
	}

    startTimer(&totalTime);
	matMult(mat1,mat2,prod);
    stopTimer(&totalTime);
    printTimer(&totalTime);	

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

void matMult(int mat1[SIZE][SIZE], int mat2[SIZE][SIZE], int prod[SIZE][SIZE])
{
	int i, j, k;
	for (i = 0;i < SIZE; i++)
	{
		for (j = 0; j < SIZE; j++)
		{
			prod[i][j]=0;
			for(k=0;k<SIZE;k++)
				prod[i][j] = prod[i][j]+mat1[i][k] * mat2[k][j];
		}
	}
}
