#if !defined (hysteresis_H)
#define hysteresis_H

/* Apply hysteresis */
void apply_hysteresis(short int *mag, unsigned char *nms, int rows, int cols,
                      float tlow, float thigh, unsigned char *edge);

/* Do a non maximum supression */
void non_max_supp(short *mag, short *gradx, short *grady, int nrows,
                  int ncols, unsigned char *result);


#endif /* !defined (hysteresis_H) */
