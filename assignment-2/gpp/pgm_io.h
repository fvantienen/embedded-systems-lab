#if !defined (pgm_io_H)
#define pgm_io_H

/* Read PGM image */
int read_pgm_image(char *infilename, unsigned char **image, int *rows,
                   int *cols);

/* Write PGM image */
int write_pgm_image(char *outfilename, unsigned char *image, int rows,
                    int cols, char *comment, int maxval);


#endif /* !defined (pgm_io_H) */
