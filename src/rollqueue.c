
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

    double sum = 0.0;

    for (int i = 0; i < d->len; i++) {
        sum += d->data[i];
    }

    return sum / d->len;

}

double get_ending_slope(const rollqueue const *d, const size_t n) {

    double sum = 0;
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
         */
       
        if (p == 0) {
            sum += d->data[p] - d->data[d->len-1];
        } else {
            sum += (d->data[p] - d->data[p - 1]);
        }

        if (p <= 0) {
            p = d->len - 1;
        }

        p--;
    }

    return sum / n;
}

double rollqueue_max(const rollqueue const *d) {

    int max = 0;

    for (int i = 0; i < d->len; i++) {

        if (d->data[max] < d->data[i]) max = i;

    }

    return d->data[max];
}
