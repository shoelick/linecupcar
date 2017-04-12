/*
 * util.h
 * Contains utilty functions and definitions
 */

extern const double HIGH_PASS[];
extern const double LOW_PASS[];

/*
 * Performs convolution data * kernel
 * Stores output in the array pointed to by result.
 */
void convolve(const double *data, size_t datalen, const double *kernel, 
        size_t kernellen, double *result);

/* 
 * Normalizes the pass function to be betwen 0 and 1.0.
 * Takes array of ints, produces array of doubles.
 */
void i_normalize(double *dest, const int *data, const size_t len);

/* 
 * Normalizes the pass function to be betwen 0 and 1.0.
 * Takes array of doubles, produces array of doubles.
 */
void d_normalize(double *dest, const double *data, const size_t len);

/* 
 * Wastes an unspecificed amount of time
 * todo: change to built in sleep function?
 */
void delay(int del);

