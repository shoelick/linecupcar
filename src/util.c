/* 
 * util.c
 * Utility functionality for signal processing and driving.
 */

/* 
 * Convolve data * kernel
 */
void convolve(const double *data, size_t datalen, const double *kernel, 
        size_t kernellen, double *result)
{
    size_t n, kmin, kmax, k;

    for (n = 0; n < datalen + kernellen - 1; n++)
    {

        Result[n] = 0;

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
void normalize(double *dest, const double *data, const size_t len) {

    int i = 0, maxind = 0;
    double scale;

    /* Find index of the max */
    for (int i = 1; i < n; i++) {
        if (data[i] > data[maxind]) maxind = i;
    }

    /* Use max at index and save a calculation */
    scale = 1.0 / data[maxind];

    /* Normalize */
    for (int i = 0; i < n; i++) {
        dest[i] = data[i] * scale;
    }
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
