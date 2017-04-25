
#include <math.h>
#include <stdlib.h>
#include "rollqueue.h"


int init_rollqueue(rollqueue *d, const size_t size) {

    /* Catch null pointer */
    if (d == NULL) {
        return ERR_ROLLQUEUE_NULLP;
    }

    d->len = size;
    d->data = calloc(size, sizeof(double));
    d->end = 0;

    return 0;
}

int add_data(rollqueue *d, const double val) {

    /* Catch null pointer */
    if (d == NULL) {
        return ERR_ROLLQUEUE_NULLP;
    }

    /* Add new data */
    d->data[d->end++] = val;

    /* Wrap around */
    if (d->end >= d->len) d->end = 0;

    return 0;
}

double get_average(const rollqueue const *d) {

    int sum = 0;

    for (int i = 0; i < d->len; i++) {
        sum += d->data[i];
    }

    return sum / d->len;

}

double get_ending_slope(const rollqueue const *d, const size_t n) {

    int sum = 0;
    size_t p = 0;

    /* Catch null pointer */
    if (d == NULL) {
        return ERR_ROLLQUEUE_NULLP;
    }

    /* Protect against overrunning buffer */
    if (n > d->len || n < 1) {
        return ERR_ROLLQUEUE_PARAM;
    }

    /* Start at the most recent end of the buffer */
    p = d->end - 1;

    for (int i = n; i > 0; i--) {
       
        /* 
         * Wrap around.
         * less than 1 because we use the slope of the two points before index p 
         */
        if (p < 1) {
            p = d->len - 1;
        }

        sum += (d->data[p] - d->data[p - 1]);

        p--;
    }

    return sum / n;
}
