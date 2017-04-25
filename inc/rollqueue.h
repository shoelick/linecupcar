/*
 * rollqueue.h
 * Author: Michael Shullick
 * Implements a running queue. Expected to be used to in a system with a
 * continuously changing stream of data. Provides operations on the recent 
 * history of that data.
 */

#define ERR_ROLLQUEUE_NULLP 1
#define ERR_ROLLQUEUE_PARAM 2

typedef struct rollqueue {
    size_t len;
    size_t end; /* Where the next piece of data will be stored */
    double *data;
} rollqueue;

/* 
 * Initialize the queue with the passed size.
 */
int init_rollqueue(rollqueue *d, const size_t size);

/*
 * Add a new piece of data to the queue. When the capacity of the queue is 
 * exceeded, the buffer wraps around and begins overwriting the oldest data.
 */
int add_data(rollqueue *d, const double val);

/* 
 * Get the running average of all data in the queue.
 */
double get_average(const rollqueue const *d);

/*
 * Get the average slope of the last n data points.
 */
double get_ending_slope(const rollqueue const *d, const size_t n);
