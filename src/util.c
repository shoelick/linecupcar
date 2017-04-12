/* 
 * util.c
 * Utility functionality for signal processing and driving.
 */

#include <stddef.h>
#include <stdio.h>
#include <util.h>

const double HIGH_PASS[] = {-1.0, 0, 1.0};
const double LOW_PASS[] = {1.0/3.0, 1.0/3.0, 1.0/3.0};

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
void i_normalize(double *dest, const int *data, const size_t n) {

    int i = 0, maxind = 0;
    double scale;

    /* Find index of the max */
    for (i = 1; i < n; i++) {
        if (data[i] > data[maxind]) maxind = i;
    }

    /* Use max at index and save a calculation */
    scale = 1.0 / data[maxind];
    printf("Found max at [%d]: %i, scaling by %f\n", maxind, data[maxind], 
            scale);

    /* Normalize */
    for (i = 0; i < n; i++) {
        dest[i] = data[i] * scale;
    }
}

/*
 * Normalize the passed data to [0, 1.0]
 */
void d_normalize(double *dest, const double *data, const size_t n) {

    int i = 0, maxind = 0;
    double scale;

    /* Find index of the max */
    for (i = 1; i < n; i++) {
        if (data[i] > data[maxind]) maxind = i;
    }

    /* Use max at index and save a calculation */
    scale = 1.0 / data[maxind];
    printf("Found max at [%d]: %f, scaling by %f\n", maxind, data[maxind], 
            scale);

    /* Normalize */
    for (i = 0; i < n; i++) {
        dest[i] = data[i] * scale;
    }
}

/*
 * Thresholds the double-array by the given threshold value. 
 * Puts 0s in the destination array where data[i] <= threshold, and 
 * 1s otherwise.
 */
void threshold(int *dest, const double const* data, size_t n, double threshold) 
{
    int i;
    for (i = 0; i < n; i++ ) {
        dest[i] = (data[i] > threshold) ? 1 : 0;
    }
}

/*
 * Signed threshold
 * Thresholds the double-array by the given threshold value. 
 * Puts 1s in the destination array where data[i] >= threshold, 
 * -1s in the dest array where data[i] <= -threshold, and 
 * 0s otherwise.
 */
void sthreshold(int *dest, const double const* data, size_t n, double threshold) 
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
int count_lines(int *data, size_t len) {

    int count, i;
    uint8_t found_line = 0;
    for (i = 0; i < len; i++) {

        /* End of line blob */
        if (data[i] == LINE_END) {

            /* If we're only catch the end of the line, increment */
            if (!found_line) {
                count++;
            } else {
                found_line = 0;
            }
        } 
        /* Start of line blob */
        else if (data[i] == LINE_START && !found_line) {
            found_line = 1;
            count++;
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
