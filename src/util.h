/*
 * util.h
 * Contains utilty functions and definitions
 */

const double HIGH_PASS[] = {-1.0, 0, 1.0} 
const double LOW_PASS[] = {1/3, 1/3, 1/3}

/*
 * Performs convolution data * kernel
 * Stores output in the array pointed to by result.
 */
void convolve(const double *data, size_t datalen, const double *kernel, 
        size_t kernellen, double *result);

/* 
 * Normalizes the pass function to be betwen 0 and 1.0.
 */
void normalize(const double *data, double *dest, const size_t len);

/* 
 * Wastes an unspecificed amount of time
 * todo: change to built in sleep function?
 */
void delay(int del);

