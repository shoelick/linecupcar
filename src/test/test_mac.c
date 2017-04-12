
#include "stdio.h"
#include "../inc/util.h"
#include "test_mac.h"

int main() {

    int int_proc[360]; 
    double proc_data[360]; 
    double proc_data2[360]; 
    int max_ind;

    for (int i = 0; i < 360; i++) {
        int_proc[i] = test_data[i] * 1000.0;
    }

    printf("First 10 data points: \n");
    for (int i = 0; i < 10; i++) {
        printf("%d ", int_proc[i]);
    }
    printf("\n");

    i_normalize(proc_data, int_proc, 360);
    printf("Normalized: \n");    
    for (int i = 0; i < 10; i++) {
        printf("%f ", proc_data[i]);
    }    
    printf("\n");
    for (int i = 0; i < 10; i++) {
        printf("%f ", test_data[i]);
    }
    printf("\n");


    convolve(proc_data, test_data, 360, LOW_PASS, 3);
    printf("Convolved: \n");    
    for (int i = 0; i < 10; i++) {
        printf("%f ", proc_data[i]);
    }
    printf("\n");

    d_normalize(proc_data, proc_data, 360);
    printf("Normalized: \n");    
    for (int i = 0; i < 10; i++) {
        printf("%f ", proc_data[i]);
    }
    printf("\n");

    convolve(proc_data2, proc_data, 360, HIGH_PASS, 3);
    printf("Convolved high pass: \n");    
    for (int i = 0; i < 10; i++) {
        printf("%f ", proc_data2[i]);
    }
    printf("\n");

    d_normalize(proc_data, proc_data2, 360);
    printf("Normalized: \n");    
    for (int i = 0; i < 10; i++) {
        printf("%f ", proc_data[i]);
    }
    d_normalize(proc_data, proc_data, 360);
    

}
