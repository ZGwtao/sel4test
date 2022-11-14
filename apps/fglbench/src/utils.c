#include "utils.h"

float 
calc_mean(int* array, int len)
{
    int i;
    float sum = 0;

    for(i = 0; i < len; ++i) {
        sum += array[i];    
    }

    return sum / len;
}

float
calc_stddev(int* array, int len, int mean)
{
    int i;
    float sum = 0;

    for(i = 0; i < len; ++i) {
        float diff = array[i] - mean;
        sum += diff * diff;
    }

    return sum / len;
}
