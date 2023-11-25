#include <stdio.h>

#define WHITESENSOR 1
#define BLACKSENSOR 0

float R2T(float r, int sensor);


#define SENSOR_OFFSET_WHITE -3.2
float R2T(float r, int sensor) {
    int nTempPts 11
    float rarray[] = {29400, 13000, 10920, 7565, 5080, 4680, 2400, 2080, 1530, 1105, 1100};
    float tarray[] = {32, 65, 74, 93, 113, 118, 157, 166, 184, 207, 213};

    float minr = rarray[nTempPts - 1];
    float maxr = rarray[0];
    float minT = tarray[0];
    float maxT = tarray[nTempPts - 1];
    float tval = -1.0;

    if (r > maxr) {
        return minT + SENSOR_OFFSET_WHITE;
    }
    if (r < minr) {
        return maxT + SENSOR_OFFSET_WHITE;
    }

    for (int i = 0; i < nTempPts; i++) {
        if (r >= rarray[i]) {
            float dTdR = (tarray[i] - tarray[i - 1]) / (rarray[i] - rarray[i - 1]);
            tval = tarray[i] + (r - rarray[i]) * dTdR + SENSOR_OFFSET_WHITE;
            break;
        }
    }

    return tval;
}

int main() {
    // Your existing code for main

    // EDGE TESTS
    // Test some edge cases for r value
    float rarray[] = {29400, 13000, 10920, 7565, 5080, 4680, 2400, 2080, 1530, 1105, 1100};
    float tarray[] = {32, 65, 74, 93, 113, 118, 157, 166, 184, 207, 213};

    float rmin = rarray[nTempPts - 1];
    float rmax = rarray[0];

    float dr = 0.05;
    float testrange = 5;

    printf("Near rmin:\n");
    for (float r = rmin - testrange; r <= rmin + testrange; r += dr) {
        printf("r: %.1f  t: %.1f\n", r, R2T(r, WHITESENSOR));
    }

    printf("Near rmax:\n");
    for (float r = rmax - testrange; r <= rmax + testrange; r += dr) {
        printf("r: %.1f  t: %.1f\n", r, R2T(r, WHITESENSOR));
    }

    return 0;
}
