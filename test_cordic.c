#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define pi		3.14159265358979323846	/* pi */
#define MY_PI 512

extern const int32_t cos_tb[1024];

void rotate_ccw(int32_t *v, int32_t a)
{
	int32_t x = v[0];
	int32_t y = v[1];

	v[0] = (x*cos_tb[a] 					- y*cos_tb[1023&(a+3*MY_PI/2)])/1024;
	v[1] = (x*cos_tb[1023&(a+3*MY_PI/2)] + y*cos_tb[a])/1024;
}

void rotate_cw(int32_t *v, int32_t a)
{
	int32_t x = v[0];
	int32_t y = v[1];

	v[0] = (x*cos_tb[a] 					+ y*cos_tb[1023&(a+3*MY_PI/2)])/1024;
	v[1] = (-x*cos_tb[1023&(a+3*MY_PI/2)] + y*cos_tb[a])/1024;
}

extern const int32_t cos_tb[1024];


int32_t getatan(int32_t *v)
{
	int32_t curang = MY_PI/4;
	int32_t sumang = 0;

	if(v[0] < 0) 
	{
		v[0] = -v[0];
		if(v[1] > 0) sumang = MY_PI/2;
		else sumang = 3*MY_PI/2;
	}else if(v[1] < 0) sumang = 2*MY_PI;
		
	while(v[1] && curang){
		
			printf("%d %d\n", curang, v[1]);
		
		if(v[1] > 0) {
			rotate_cw(v, curang);
			sumang += curang;
		}			
		else {
			rotate_ccw(v, curang);
			sumang -= curang;
		}		
		curang = curang/2;		
	}
	
	return sumang;	
}

double cord_atan(int32_t *v)
{
	const double AngTable[] = {45, 26.565, 14.036, 7.125, 3.576, 1.790, 0.895, 0.448, 0.224, 0.112};
	double SumAngle = 0; 
	int i = 0;
	int x, y, x1, y1;

	x = abs(v[0]);
	y = v[1];

	for(i = 0; i < 10; i++)
	{		
		printf("%.2f\n", SumAngle);
		
		x1 = x;
		y1 = y;
			
		if(y > 0){
			x = x1 + (y1 >> i); 
			y = y1 - (x1 >> i); 
			SumAngle = SumAngle + AngTable[i]; 
		}else{
			x = x1 - (y1 >> i); 
			y = y1 + (x1 >> i); 
			SumAngle = SumAngle - AngTable[i]; 
		}
		if(y == 0) break;
	}
	
	if(v[0] < 0) SumAngle = 180-SumAngle;		
	if(SumAngle < 0) SumAngle += 360;
	
	return  SumAngle;
}

int main(int argc, char *argv[])
{
	double 	a[2];
	int32_t v[2];
	double phi;

	
	a[0] = atof(argv[1]);
	a[1] = atof(argv[2]);
	
	phi = atan(a[1]/a[0]);
		
	printf("x=%.2f y=%.2f phi=%.2f\n", a[0], a[1], phi*180/pi);
	
	v[0] = (int)a[0];
	v[1] = (int)a[1];

	//printf("angle=%.2f\n", 180.0*getatan(v)/MY_PI);
	printf("angle=%.2f\n", cord_atan(v));
	
	/*
	printf("%d %d\n", v[0], v[1]);
	rotate_ccw(v, MY_PI/2);
	printf("%d %d\n", v[0], v[1]);
	*/
	
	
	return 0;
}
