/* 
 * util.c
 * Utility functionality for signal processing and driving.
 */

#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <util.h>
#include "uart.h"

// kernels are delicious 
const double HIGH_PASS[] = {1.0, 0, -1.0};
const double DERIVATIVE[] = {1.0, -1.0};
const double DERIV2[] = {1.0, 2.0, -1.0};
const double LOW_PASS3[] = {1.0/3.0, 1.0/3.0, 1.0/3.0};
const double LOW_PASS5[] = {1.0/3.0, 1.0/3.0, 1.0/3.0, 1.0/3.0, 1.0/3.0};
const double BOXCAR_4[] = {1.0, 1.0, 1.0, 1.0};
const double BOXCAR_5[] = {1.0, 1.0, 1.0, 1.0, 1.0};
const double BOXCAR_7[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
const double TRI_5[] = {1.0,2.0,3.0,2.0,1.0};
const double GAUSS_SMOOTH_5[] = {1.0,2.0,3.0,2.0,1.0};
const double GAUSS_SMOOTH_7[] = {1.0,4.0,8.0,10.0,8.0,4.0,1.0};

/* 
 * Convolve data * kernel
 */
void convolve(double *result, const double *data, size_t datalen, 
        const double *kernel, size_t kernellen) {

    size_t n, kmin, kmax, k;

    // Loop should go up to datalen + kernellen - 1, but we're capping because 
    // of the size of the buffer
    for (n = 0; n < datalen; n++)
    {

        result[n] = 0;

        kmin = (n >= kernellen - 1) ? n - (kernellen - 1) : 0;
        kmax = (n < datalen - 1) ? n : datalen - 1;

        for (k = kmin; k <= kmax; k++)
        {
            result[n] += data[k] * kernel[n - k];
        }
    }
}

/*
 * Normalize the passed data to [0, 1.0]
 */
void i_normalize(double *dest, int *data, const size_t n) {

    int i = 0, maxind = 0;
    double scale;

    /* Find index of the max */
    for (i = 1; i < n; i++) {
        if (data[i] > data[maxind]) maxind = i;
    }

    /* Use max at index and save a calculation */
    scale = 1.0 / fabs(data[maxind]);

    /* Normalize */
    for (i = 0; i < n; i++) {
        dest[i] = data[i] * scale;
    }
}

/*
 * Normalize the passed data to [0, 1.0]
 * Uses the absolute value of any negative values
 */
void d_normalize(double *dest, const double *data, const size_t n) {

    int i = 0, maxind = 0;
    double scale;

    /* Find index of the max */
    for (i = 1; i < n; i++) {
        if (fabs(data[i]) > fabs(data[maxind])) maxind = i;
    }

    /* Use max at index and save a calculation */
    scale = 1.0 / fabs(data[maxind]);

    /* Normalize */
    for (i = 0; i < n; i++) {
        dest[i] = data[i] * scale;
    }
}

/*
 * Thresholds the double-array by the given threshold value. 
 * Puts 0s in the destination array where data[i] <= threshold, and 
 * 1s otherwise.
 * Chop off the first and last 5 spaces
 */
void threshold(int *dest, const double * const data, size_t n, double threshold) 
{
    int i;
    //int c = 10;
    for (i = 0; i < n; i++ ) {

        // Clip lower to avoid weird bias
        /*if (i < c) {
            dest[i] = 0;
        }*/
        if (data[i] <= 0) {
            // if neg
            dest[i] = (data[i] > -threshold) ? 0 : -1;
        } else {
            // if pos
            dest[i] = (data[i] < threshold) ? 0 : 1;
        }
    }
}

/*
 * Takes in the signed threshold'd data and counts the detected black line 
 * blobs.
 */
int center_average(int const * const data, size_t len) {

    int count = 0, width = 0, i;
    uint8_t found_line = 0;
    int pix_since_last_1 = 0;
    int index_start = 0;
    int index_of_last = 0;
    int newthresh = 30;
    int widththresh = 10;
    int sum = 0;
    for (i = 0; i < len; i++) {

        if (data[i] == 1) {

            if (!found_line) {
                //printu("Line start at %d\n\r", i);
                count += 1;
                width = 0;
                found_line = 1;
                index_start = i;
            } else {
                width += 1;
            }

            pix_since_last_1 = 0;
            index_of_last = i;
        } 
        else if (data[i] == 0 && found_line) {

            if (pix_since_last_1 > newthresh) {
                //printu("Line finished at %d\n\r", i); 

                if (width < widththresh) {
                    //printu("Line ignored with width %d\n\r", width); 
                    count--;
                }  else {

                    sum += (index_of_last + index_start)/2; 
                    //printu("Sum is now: %d\n\r", sum);
                }
                found_line = 0;
            } else {
                pix_since_last_1++;
            }
        }
    }

    if (found_line) {
        //printu("Assuming final one at %d\n\r", index_of_last); 
        sum += (index_of_last + index_start) /2;
        //printu("Sum is now: %d\n\r", sum);
    }

    return sum / count;
}

/*
 * Find largest blob of a given value in a thresholded set of data.
 * Used to find the right or left lines.
 * returns index of center if found, -1 if not
 */
int find_blob(const int const * data, const size_t len, const int val) {
  
    /* Boolean of whether we're currently looking at a blob */
    uint8_t found_blob = 0;

    /* Used to fight noise; allow spaces between blob vals */
    int pix_since_last = 0;
    int newthresh = 0;

    /* loop counter */
    int i;

    /* Indices of current blob in view */
    int index_start = 0;
    int index_of_last = 0;

    /* Indices of the blob we'll care about in the end */
    int index_start_max = 0;
    int index_of_end_max = 0;

    /* Minimum width of a blob to be considered */
    int widththresh = 0; // DO NOT CHANGE WITHOUT MAKING A BIG DEAL
    int width;

    int final = 0;

    //printu("Starting search of val %d\n\r", val);

    /* Iterate over every point in the line */
    for (i = 0; i < len; i++) {

        /* If we come across a [val]... */
        if (data[i] == val) {

            /* And it's the first one we've seen in a while ...*/
            if (!found_blob) {
                //printu("Blob start at %d\n\r", i);

                /* Make note we found one */
                found_blob = 1;

                /* Make note of where it started */
                index_start = i;
            } 

            /* Every time we see a val, it's fresh; take note of each one */
            pix_since_last = 0;
            index_of_last = i;
        } 
        /* Else if we've come across an index which doesn't have [val] but came
         * right after one.... */
        else if (data[i] != val && found_blob) {

            /* Check if it's been long enough since we've seen a [val] to record
             * the last blob we saw and move on */
            if (pix_since_last > newthresh) {
                //printu("Blob finished at %d\n\r", i); 

                /* Record blob width */
                width = index_of_last - index_start;

                /* Check if the completed blob was long enough to be kept */
                if (width > widththresh && \
                        index_of_end_max - index_start_max < width) {

                    index_start_max = index_start;
                    index_of_end_max = index_of_last;
                } else {
                    //printu("Line ignored with width %d\n\r", width); 
                }

                /* Don't count same blob twice nigga */
                index_of_last = 0; index_start = 0;

                /* no longer tracking a blob */
                found_blob = 0;

            } else {
                /* Still tracking the same blob, keep track of how long it's 
                 * been since we last saw [val]
                 */
                pix_since_last++;
            }
        }
    }

    /* See if we finished in the middle of a blob */
    if (found_blob) {

        /* Record blob width */
        width = index_of_last - index_start;

        /* Check if the completed blob is was long enough to be kept */
        if (width > widththresh && \
                index_of_end_max - index_start_max < width) {

            index_start_max = index_start;
            index_of_end_max = index_of_last;
            //printu("Sum is now: %d\n\r", sum);
        } else {
            //printu("Line ignored with width %d\n\r", width); 
        }
    }

    //printu("Final start: %d Final end: %d\n\r", index_start_max, index_of_end_max);

    /* Return average of start and end for center, or -1 if we didn't find one*/
    final = (index_of_end_max + index_start_max) / 2;
    return (final == 0) ? -1 : final;

}


/*
 * Takes in the signed threshold'd data and counts the detected black line 
 * blobs.
 */
int count_lines(int const * const data, size_t len) {

    int count = 0, width = 0, i;
    uint8_t found_line = 0;
    int pix_since_last_1 = 0;
    int index_of_last = 0;
    int newthresh = 35;
    int widththresh = 10;
    for (i = 0; i < len; i++) {

        if (data[i] == 1) {

            if (!found_line) {
                /*sprintf(str, "Line start at %d\n\r", i); 
                uart_put(str);*/
                count++;
                width = 0;
                found_line = 1;
            } else {
                width += i - index_of_last;
            }

            pix_since_last_1 = 0;
            index_of_last = i;
        } 
        else if (data[i] == 0 && found_line) {

            if (pix_since_last_1 > newthresh) {
                /*sprintf(str, "Line finished at %d\n\r", i); 
                uart_put(str);*/

                if (width < widththresh) {
                    /*sprintf(str, "Line ignored with width %d\n\r", width); 
                    uart_put(str);*/
                    count--;
                } 
                found_line = 0;
            } else {
                pix_since_last_1++;
            }
        }
    }

    return count;
}

/*
 * Waste time 
 */
void delay(int del){
    int i;
    for (i=0; i<del*50000; i++){
        // Do nothing
    }
}

/*
 * Simple Derivative by slope calculation
 */
void slopify(double *dest, const double * const data, const size_t n) {

    unsigned int i;

    for (i = 1; i < n; i++) {
        dest[i] = data[i] - data[i - 1];
    }
}

/*
 * Given a signal [-1.0, 1.0], amplify everything and clip at +/- 1.0
 */
void amplify(double *dest, const double *const data, const size_t n, int gain) {

    size_t i;
    double val;

    for (i = 0; i < n; i++) {
        val = gain * data[i];
        if (data[i] < 0) {
            // if neg
            dest[i] = (val > -1.0) ? val : -1.0;
        } else {
            // if pos
            dest[i] = (val < 1.0) ? val : 1.0;
        }
    }
}

/*
 * integer based exponentiation
 */
int int_pow(int base, int exp)
{
    int result = 1;
    while (exp)
    {
        if (exp & 1)
            result *= base;
        exp /= 2;
        base *= base;
    }
    return result;
}

double bound(double val, const double min, const double max) {

    if (val > max) {
        val = max; 
    }
    else if (val < min) {
        val = min;
    }

    return val;

}
