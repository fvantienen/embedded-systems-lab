#include<stdio.h>
#include "Timer.h"

#define SIZE 100

void matMult(int mat1[SIZE][SIZE], int mat2[SIZE][SIZE], int prod[SIZE][SIZE], int matrixSize);

int main(int argc, char** argv)
{
    Timer totalTime;
    int matrixSize;
    int mat1[SIZE][SIZE], mat2[SIZE][SIZE], prod[SIZE][SIZE];
	int i, j;

    // Check argument size
    if (argc < 2) {
    	printf("You should give the matrix size as argument.\n");
    	return -1;
    }

    // Check matrix size
    matrixSize = atoi(argv[1]);
    if (matrixSize < 1 || matrixSize > 100) {
    	printf("Matrix should be between 1 and 100.\n");
    	return -1;
    }

    // Init timer
    initTimer(&totalTime, "Total Time");
	
	// Init matrices
	for (i = 0;i < matrixSize; i++)
	{
		for (j = 0; j < matrixSize; j++)
		{
			mat1[i][j] = i+j*2;
		}
	}

	for(i = 0; i < matrixSize; i++)
	{
		for (j = 0; j < matrixSize; j++)
		{
			mat2[i][j] = i+j*3;
		}
	}

    startTimer(&totalTime);
	matMult(mat1, mat2, prod, matrixSize);
    stopTimer(&totalTime);

	for (i = 0;i < matrixSize; i++)
	{
		printf("\n");
		for (j = 0; j < matrixSize; j++)
		{
			printf("\t%d ", prod[i][j]);
		}
	}
	
	printf("\n\n");
	printTimer(&totalTime);
	printf("Done !!! \n");
	return 0;
}

void matMult(int mat1[SIZE][SIZE], int mat2[SIZE][SIZE], int prod[SIZE][SIZE], int matrixSize)
{
	int i, j, k;
	for (i = 0;i < matrixSize; i++)
	{
		for (j = 0; j < matrixSize; j++)
		{
			prod[i][j]=0;
			for(k=0; k < matrixSize; k++)
				prod[i][j] = prod[i][j]+mat1[i][k] * mat2[k][j];
		}
	}
}
