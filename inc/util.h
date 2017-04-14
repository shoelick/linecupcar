/*
 * util.h
 * Contains utilty functions and definitions
 */

#include "stddef.h"

extern const double HIGH_PASS[];
extern const double LOW_PASS[];
extern const double DERIVATIVE[];
extern const double BOXCAR_4[];

#define LINE_START 1
#define LINE_STOP -1

/*
 * Performs convolution data * kernel
 * Stores output in the array pointed to by result.
 */
void convolve(double *result, const double *data, size_t datalen, 
        const double *kernel, size_t kernellen);


/* 
 * Normalizes the pass function to be betwen 0 and 1.0.
 * Takes array of ints, produces array of doubles.
 */
void i_normalize(double *dest, int *data, size_t len);

/* 
 * Normalizes the pass function to be betwen 0 and 1.0.
 * Takes array of doubles, produces array of doubles.
 */
void d_normalize(double *dest, const double *data, const size_t len);

/*
 * Thresholding function
 * Prouduces integer arrays of thresholded data based on passed threshold.
 * sthreshold allows for signed thresholding.
 */
void threshold(int *dest, const double * const data, size_t n, double threshold);
void sthreshold(double *dest, const double * const data, size_t n, 
        double threshold); 

/*
 * Count the number of line blobs in the passed array.
 */
int count_lines(int const * const data, size_t len);

/*
 * Find leftmost line blob
 */
int find_left_line(int *data, size_t len);

/*
 * Find rightmost line blob
 */
int find_right_line(int *data, size_t len);

/* 
 * Wastes an unspecificed amount of time
 * todo: change to built in sleep function?
 */
void delay(int del);

void slopify(double *dest, const double * const data, const size_t n);
int center_average(int const * const data, size_t len);

int int_pow(int base, int exp);
