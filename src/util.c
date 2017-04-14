/* 
 * util.c
 * Utility functionality for signal processing and driving.
 */

#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <util.h>
#include "uart.h"

const double HIGH_PASS[] = {-1.0, 0, 1.0};
const double DERIVATIVE[] = {-1.0, 1.0};
const double LOW_PASS[] = {1.0/3.0, 1.0/3.0, 1.0/3.0};
const double BOXCAR_4[] = {1, 1, 1, 1};

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
    scale = 1.0 / data[maxind];

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
        dest[i] = fabs(data[i]) * scale;
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
    for (i = 0; i < n; i++ ) {
        if (i < 5 || i > n - 5 ) {
            dest[i] = 0;
        } else {
            dest[i] = (data[i] > threshold) ? 1 : 0;
        }
    }
}

/*
 * Signed threshold
 * Thresholds the double-array by the given threshold value. 
 * Puts 1s in the destination array where data[i] >= threshold, 
 * -1s in the dest array where data[i] <= -threshold, and 
 * 0s otherwise.
 */
void sthreshold(double *dest, const double * const data, size_t n, 
        double threshold)
{
    int i;
    for (i = 0; i < n; i++ ) {
        if (data[i] > threshold) dest[i] = 1;
        else if (data[i] < -1 * threshold) dest[i] = -1;
        else dest[i] = 0;
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
    int newthresh = 35;
    int widththresh = 10;
    int sum = 0;
    for (i = 0; i < len; i++) {

        if (data[i] == 1) {

            if (!found_line) {
                //sprintf(str, "Line start at %d\n\r", i);
                uart_put(str);
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
                //sprintf(str, "Line finished at %d\n\r", i); 
                uart_put(str);

                if (width < widththresh) {
                    //sprintf(str, "Line ignored with width %d\n\r", width); 
                    uart_put(str);
                    count--;
                }  else {

                    sum += (index_of_last + index_start)/2; 
                    //sprintf(str, "Sum is now: %d\n\r", sum);
                    uart_put(str);
                }
                found_line = 0;
            } else {
                pix_since_last_1++;
            }
        }
    }

    if (found_line) {
        //sprintf(str, "Assuming final one at %d\n\r", index_of_last); 
        uart_put(str);
        sum += (index_of_last + index_start) /2;
        //sprintf(str, "Sum is now: %d\n\r", sum);
        uart_put(str);
    }

    return sum / count;
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

    int i;

    dest[0] = 0;
    dest[1] = 0;
    dest[2] = 0;
    dest[3] = 0;
    dest[4] = 0;
    for (i = 5; i < n; i++) {
        dest[i] = fabs(data[i] - data[i - 1]);
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

