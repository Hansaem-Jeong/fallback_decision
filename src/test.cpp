#include <stdio.h>
#include "BEV_image.h"
#include <time.h>

int main(void)
{
    double c[11] = { 0 };
    double t[288] = { 0 };
    double l[10] = { 0 };
    double a = 0;
    unsigned char b[275598] = { 0 };

    for(int i =0; i<=10000; ++i) {
        clock_t start_c = clock();
        BEV_image(c, t, l, a, b);
        printf("b : %d, time: %lf\n", b[0], (double)(clock()-start_c)/CLOCKS_PER_SEC);
        
    }


    return 0;
}
