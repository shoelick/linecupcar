#include "camera_driver.h"

void mvg_average(int *data, int size, int n) {
	int i, j, sum;
	int halfn = n / 2;
	for (i = halfn; i < size - halfn/2; i++) {
		sum = 0;
		for (j = i - halfn; j < i + halfn; j++) {
			sum += data[j];
		}
		data[i] = sum/n;
	}
}

