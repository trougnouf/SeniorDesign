#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
int main()
{
	int16_t inints[] = {23.9, 52.1};
	unsigned char * msg = malloc(10);
	double angle = atan((double)inints[1]/(double)inints[0])*180/3.1428;
	sprintf(msg, "%lf.\t", angle);
	printf("%s", msg);
}
