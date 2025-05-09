#include <stdio.h> 
#include <stdint.h>


int main()
{
    int16_t a=32767;
    int16_t b=a+1;
    int16_t c = a-b;
    printf("%d %d %d",a,b,c);
}